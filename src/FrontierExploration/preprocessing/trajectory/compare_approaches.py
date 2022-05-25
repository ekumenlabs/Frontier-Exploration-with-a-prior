
from copy import deepcopy
from matplotlib import pyplot as plt
from preprocessing.grid.mock_grid import create_mock_grid
from preprocessing.trajectory.naive_approach import NaiveFrontierExploration
from preprocessing.trajectory.not_naive_approach import BlueprintAwareFrontierExploration
import numpy as np


if __name__ == "__main__":
    grid = create_mock_grid(num_x_cells=20, num_y_cells=30,
                            cell_size=0.1, occupancy_percentage=10)
    grid2 = deepcopy(grid)
    naive = NaiveFrontierExploration(grid=grid, start_cell=(2, 5))
    blueprint_aware = BlueprintAwareFrontierExploration(grid=grid2, start_cell=(2, 5))
    naive_path = naive.solve()
    blueprint_aware_path = blueprint_aware.solve()

    fig = grid.plot()
    ax = fig.gca()
    to_plot_naive = np.asarray(naive_path).T
    to_plot_naive = np.add(to_plot_naive, np.ones_like(to_plot_naive) * 0.5)
    ax.plot(to_plot_naive[0], to_plot_naive[1],
            label="Naive frontier exploration trajectory")
    plt.legend(loc='best')
    print(f"The robot should take {len(naive_path)} steps to explore the grid with the naive approach.")
    to_plot_blueprint_aware = np.asarray(blueprint_aware_path).T
    to_plot_blueprint_aware = np.add(to_plot_blueprint_aware, np.ones_like(to_plot_blueprint_aware) * 0.5)
    ax.plot(to_plot_blueprint_aware[0], to_plot_blueprint_aware[1],
            label="Blueprint aware frontier exploration trajectory")
    plt.legend(loc='best')
    print(f"The robot should take {len(blueprint_aware_path)} steps to explore the grid with the blueprint aware approach.")

    plt.show()
