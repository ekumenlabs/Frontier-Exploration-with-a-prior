import tqdm
from abc import abstractmethod
from copy import deepcopy
from itertools import tee
from typing import List
from math import hypot
import numpy as np

from FrontierExploration.preprocessing.grid.occupancy_grid import EMPTY, VIEWED, OccupancyGrid, is_cell_empty
from FrontierExploration.preprocessing.grid.raycasting import raycast_in_every_direction
from FrontierExploration.preprocessing.trajectory.astar import AStar


class TrajectorySolver(object):

    def __init__(self, grid: OccupancyGrid, start_cell: tuple):
        self._grid = grid
        self._cells_to_view = grid.get_empty_cells()
        self._original_empty_cells = deepcopy(self._cells_to_view)
        self._range_in_cells = 3
        self._start_cell = start_cell
        self._astar = AStar(grid=deepcopy(grid))
        self._visibility_grid = {}

    @abstractmethod
    def solve(self):
        "Method that solves given grid"

    def _resolve_trajectory(self, cells_to_evaluate):
        ret = []
        for cell_to_evaluate, next_cell_to_evaluate in pairwise(cells_to_evaluate):
            # run astar to find the path between current point and chosen frontier point
            ret.extend(self._astar.solve(start_cell=cell_to_evaluate, end_cell=next_cell_to_evaluate))
            # reset the astar algorithm
            self._astar.reset()
        return ret

    def _get_cells_to_evaluate(self):
        cell_to_evaluate = self._start_cell
        cells_to_evaluate = [self._start_cell]
        frontier_cells = []
        cells_to_explore = self.cells_to_view
        with tqdm.tqdm(total=cells_to_explore) as pbar:
            while self.cells_to_view > 1:
                last_cells_to_view = self.cells_to_view
                # check which cells you can view from the current cell
                frontier_cells.extend(self._get_frontier_cells(cell_to_evaluate))
                self._grid[cell_to_evaluate] = VIEWED
                self._cells_to_view = self._grid.get_empty_cells()

                # update the progress bar
                pbar.update(last_cells_to_view - self.cells_to_view)

                # pick the next cell to navigate to
                next_cell_to_evaluate = self._get_next_cell_to_evaluate(frontier_cells, cell_to_evaluate)
                if next_cell_to_evaluate is None:
                    break
                cells_to_evaluate.append(next_cell_to_evaluate)
                # with the freshly viewed cells, maybe some old frontier cells are no longer frontiers, revisit.
                [frontier_cells.remove(cell) for cell in frontier_cells if not self._is_frontier(cell)]
                cell_to_evaluate = next_cell_to_evaluate
                # mark the cell you're going to navigate as empty, so the subsequent calls don't crash
                self._grid[cell_to_evaluate] = EMPTY
                self._cells_to_view = self._grid.get_empty_cells()
        return cells_to_evaluate

    def _is_frontier(self, cell: tuple) -> bool:
        """
        Returns whether the cell is a frontier cell or not.
        """
        for cell in self._raycast_in_every_direction(cell=cell, available_cells=self._cells_to_view, frontier_eval=True):
            if is_cell_empty(empty_cells=self._cells_to_view, cell=cell):
                return True
        return False

    def _raycast_in_every_direction(self, cell: tuple, available_cells: np.ndarray, frontier_eval=False) -> List[tuple]:
        lidar_range = self._range_in_cells if not frontier_eval else self._range_in_cells
        return raycast_in_every_direction(available_cells=available_cells, start_cell=cell, range_in_cells=lidar_range, squarify=True)

    def _get_frontier_cells(self, cell_to_evaluate):
        frontier_cells = []
        # check which cells you can view from the current cell
        raycasted_cells = self._raycast_in_every_direction(
            cell=cell_to_evaluate, available_cells=self._cells_to_view)
        # mark them as viewed, and check whether they're frontiers or not (frontier == boundary between explored and unexplored)
        for cell in raycasted_cells:
            self._grid[tuple(cell)] = VIEWED
            if self._is_frontier(tuple(cell)):
                frontier_cells.append(tuple(cell))
        return frontier_cells

    def plot(self, *args, **kwargs):
        return self._grid.plot(*args, **kwargs)

    @property
    def cells_to_view(self) -> int:
        return len(self._cells_to_view)


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def paginate(lst: List[any], n: int):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def distance(a: tuple, b: tuple) -> int:
    return hypot(a[0] - b[0], a[1] - b[1])