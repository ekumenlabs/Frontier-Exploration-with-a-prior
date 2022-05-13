import pdb
import numpy as np
from grid.occupancy_grid import OccupancyGrid, is_cell_empty, VIEWED, EMPTY
import matplotlib.pyplot as plt
from grid.mock_grid import create_mock_grid
from preprocessing.grid.raycasting import raycast_in_every_direction
from typing import List
from copy import deepcopy
import tqdm
from trajectory.astar import AStar


def paginate(lst: List[any], n: int):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


class NaiveFrontierExploration(object):
    def __init__(self, grid: OccupancyGrid, start_cell: tuple):
        self._grid = grid
        self._cells_to_view = grid.get_empty_cells()
        self._range_in_cells = 9
        self._start_cell = start_cell
        self._astar = AStar(grid=deepcopy(grid))

    def solve(self) -> None:
        cell_to_evaluate = self._start_cell
        ret = []
        frontier_cells = []
        cells_to_explore = len(self._cells_to_view)
        with tqdm.tqdm(total=cells_to_explore) as pbar:
            while len(self._cells_to_view) > 1:
                raycasted_cells = self._raycast_in_every_direction(
                    cell=cell_to_evaluate, available_cells=self._cells_to_view)
                for cell in raycasted_cells:
                    self._grid[tuple(cell)] = VIEWED
                    if self._is_frontier(tuple(cell)):
                        frontier_cells.append(tuple(cell))
                pbar.update(len(raycasted_cells))
                self._grid[cell_to_evaluate] = VIEWED
                self._cells_to_view = self._grid.get_empty_cells()
                # TODO(Ramiro): pick the closest one instead of randomly picking one
                next_cell_to_evaluate = tuple(
                    frontier_cells[np.random.choice(range(len(frontier_cells)))])
                # run astar to find the path between current point and chosen frontier point
                ret.extend(self._astar.solve(
                    start_cell=cell_to_evaluate, end_cell=next_cell_to_evaluate))
                # reset the astar algorithm
                self._astar.reset()
                cell_to_evaluate = next_cell_to_evaluate
                self._grid[cell_to_evaluate] = EMPTY
                self._cells_to_view = self._grid.get_empty_cells()
                # with the freshly viewed cell, maybe some old frontier cells are no longer frontiers, revisit.
                for cell in frontier_cells:
                    if not self._is_frontier(cell):
                        frontier_cells.remove(cell)
        return ret

    # returns all frontier points from the grid
    def _is_frontier(self, cell: tuple) -> bool:
        for cell in self._raycast_in_every_direction(cell=cell, available_cells=self._cells_to_view, frontier_eval=True):
            if is_cell_empty(empty_cells=self._cells_to_view, cell=cell):
                return True
        return False

    def _raycast_in_every_direction(self, cell: tuple, available_cells: np.ndarray, frontier_eval=False) -> List[tuple]:
        lidar_range = self._range_in_cells if not frontier_eval else self._range_in_cells
        return raycast_in_every_direction(available_cells=available_cells, start_cell=cell, range_in_cells=lidar_range, squarify=True)

    def plot(self):
        return self._grid.plot()


if __name__ == "__main__":
    grid = create_mock_grid(num_x_cells=40, num_y_cells=40,
                            cell_size=0.1, occupancy_percentage=10)
    p = NaiveFrontierExploration(grid=grid, start_cell=(2, 5))
    cells_to_plot = p.solve()

    fig = p.plot()
    ax = fig.gca()
    pdb.set_trace()
    to_plot = np.asarray(cells_to_plot).T
    to_plot = np.add(to_plot, np.ones_like(to_plot) * 0.5)
    ax.plot(to_plot[0], to_plot[1],
            label="Naive frontier exploration trajectory")
    plt.show()
