from copy import deepcopy
from math import hypot
import matplotlib.pyplot as plt
import numpy as np
import tqdm 
from typing import List

from FrontierExploration.preprocessing.grid.raycasting import raycast_in_every_direction
from FrontierExploration.preprocessing.trajectory.astar import AStar
from FrontierExploration.preprocessing.grid.mock_grid import create_mock_grid
from FrontierExploration.preprocessing.grid.occupancy_grid import CELL_VALUES, OCCUPIED, OccupancyGrid, is_cell_empty, VIEWED, EMPTY


def paginate(lst: List[any], n: int):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def distance(a: tuple, b: tuple) -> int:
    return hypot(a[0] - b[0], a[1] - b[1])

class BlueprintAwareFrontierExploration(object):
    def __init__(self, grid: OccupancyGrid, start_cell: tuple):
        self._grid = grid
        self._cells_to_view = grid.get_empty_cells()
        self._original_empty_cells = deepcopy(self._cells_to_view)
        self._range_in_cells = 3
        self._start_cell = start_cell
        self._astar = AStar(grid=deepcopy(grid))
        self._visibility_grid = {}

    def solve(self) -> List[tuple]:
        """
        Explores the whole grid, starting from the start cell.
        This will return a list of every cell that was chosen as the frontier to explore, as well as the navigation between cells,
        leveraging an AStar algorithm for the latter.
        returns a list of tuples, that represents the trajectory the robot should take to explore the whole grid.
        """
        cell_to_evaluate = self._start_cell
        ret = []
        frontier_cells = [ self._start_cell ]
        cells_to_explore = len(self._cells_to_view)

        with tqdm.tqdm(total=cells_to_explore) as pbar:
            while len(frontier_cells) > 1:
                last_cells_to_view = len(self._cells_to_view)
                # check which cells you can view from the current cell
                raycasted_cells = self._raycast_in_every_direction(
                    cell=cell_to_evaluate, available_cells=self._original_empty_cells)
                # mark them as viewed, and check whether they're frontiers or not (frontier == boundary between explored and unexplored)
                for cell in raycasted_cells:
                    self._grid[tuple(cell)] = VIEWED
                    if self._is_frontier(tuple(cell)):
                        frontier_cells.append(tuple(cell))
                # update the progress bar
                self._grid[cell_to_evaluate] = VIEWED
                self._cells_to_view = self._grid.get_empty_cells()
                pbar.update(last_cells_to_view - len(self._cells_to_view))
                last_cells_to_view = len(self._cells_to_view)
                next_cell_to_evaluate = None
                best_visibility = None
                    
                self._raycast_some(cells=frontier_cells)
                for cell in frontier_cells:
                    if best_visibility is None or len(self._visibility_grid[cell] > best_visibility):
                        next_cell_to_evaluate = cell
                        best_visibility = len(self._visibility_grid[cell])

                    
                # run astar to find the path between current point and chosen frontier point
                ret.extend(self._astar.solve(
                    start_cell=cell_to_evaluate, end_cell=next_cell_to_evaluate))
                # reset the astar algorithm
                self._astar.reset()
                cell_to_evaluate = next_cell_to_evaluate
                # mark the cell you're going to navigate as empty, so the subsequent calls don't crash
                self._grid[cell_to_evaluate] = EMPTY
                self._cells_to_view = self._grid.get_empty_cells()
                # with the freshly viewed cells, maybe some old frontier cells are no longer frontiers, revisit.
                frontiers_to_delete = []
                for cell in frontier_cells:
                    if not self._is_frontier(cell):
                        frontiers_to_delete.append(cell)
                for cell in frontiers_to_delete:
                    frontier_cells.remove(cell)
        return ret

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

    def plot(self):
        return self._grid.plot()
    
    def _raycast_some(self, cells: List[tuple]) -> None:
        for cell in cells:
            self._visibility_grid[cell] = self._raycast_in_every_direction(cell=cell, available_cells=self._cells_to_view)
        

        


if __name__ == "__main__":
    grid = create_mock_grid(num_x_cells=30, num_y_cells=30,
                            cell_size=0.1, occupancy_percentage=10)
    p = BlueprintAwareFrontierExploration(grid=grid, start_cell=(2, 5))
    path = p.solve()

    fig = p.plot()
    ax = fig.gca()
    to_plot = np.asarray(path).T
    
    
    to_plot = np.add(to_plot, np.ones_like(to_plot) * 0.5)
    ax.plot(to_plot[0], to_plot[1],
            label="Blueprint aware frontier exploration trajectory")
    plt.legend(loc='best')
    print(f"The robot should take {len(path)} steps to explore the grid.")

    plt.show()
