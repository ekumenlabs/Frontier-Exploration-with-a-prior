from typing import List
import matplotlib.pyplot as plt
import numpy as np

from FrontierExploration.preprocessing.grid.mock_grid import create_mock_grid
from FrontierExploration.preprocessing.trajectory.trajectory_base import TrajectorySolver, distance

class NaiveFrontierExploration(TrajectorySolver):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def solve(self) -> List[tuple]:
        """
        Explores the whole grid, starting from the start cell.
        This will return a list of every cell that was chosen as the frontier to explore, as well as the navigation between cells,
        leveraging an AStar algorithm for the latter.
        returns a list of tuples, that represents the trajectory the robot should take to explore the whole grid.
        """
        cells_to_evaluate = self._get_cells_to_evaluate()
        ret = self._resolve_trajectory(cells_to_evaluate)     
        return ret


    def _get_next_cell_to_evaluate(self, frontier_cells, cell_to_evaluate):
        next_cell_to_evaluate = None
        best_distance = None
        for cell in frontier_cells:
            cell_distance = cell_distance = distance(cell, cell_to_evaluate)
            if best_distance is None or cell_distance < best_distance:
                next_cell_to_evaluate = cell
                best_distance = cell_distance
        return next_cell_to_evaluate


if __name__ == "__main__":
    grid = create_mock_grid(num_x_cells=30, num_y_cells=30,
                            cell_size=0.1, occupancy_percentage=10)
    p = NaiveFrontierExploration(grid=grid, start_cell=(2, 5))
    path = p.solve()

    fig = p.plot()
    ax = fig.gca()
    to_plot = np.asarray(path).T
    to_plot = np.add(to_plot, np.ones_like(to_plot) * 0.5)
    ax.plot(to_plot[0], to_plot[1],
            label="Naive frontier exploration trajectory")
    plt.legend(loc='best')
    print(f"The robot should take {len(path)} steps to explore the grid.")

    plt.show()
