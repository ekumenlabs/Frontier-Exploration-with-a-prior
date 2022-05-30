import numpy as np
import matplotlib.pyplot as plt

from FrontierExploration.preprocessing.grid.occupancy_grid import OccupancyGrid


def create_mock_grid(
    num_x_cells: int, num_y_cells: int, cell_size: float, occupancy_percentage: int
) -> OccupancyGrid:
    """
    Creates a mock grid with the given parameters. And fills it with the given occupancy percentage. (randomly)
    """
    assert occupancy_percentage >= 0 and occupancy_percentage <= 100
    assert cell_size > 0
    assert num_x_cells > 0
    assert num_y_cells > 0
    grid = OccupancyGrid(cell_size=cell_size, x_cells=num_x_cells, y_cells=num_y_cells)
    available_cells = np.asarray(
        [(x, y) for x in range(num_x_cells) for y in range(num_y_cells)]
    )

    np.random.seed(42)
    occupied_cells = np.random.choice(
        grid.get_x_cells() * grid.get_y_cells(),
        size=int(occupancy_percentage * grid.get_x_cells() * grid.get_y_cells() / 100),
        replace=False,
    )
    for (x, y) in available_cells[occupied_cells]:
        grid[x, y] = 100

    # fill borders:
    for x in range(grid.get_x_cells()):
        grid[x, 0] = 100
        grid[x, grid.get_y_cells() - 1] = 100
    for y in range(grid.get_y_cells()):
        grid[0, y] = 100
        grid[grid.get_x_cells() - 1, y] = 100
    return grid


# define main
def main():
    grid = create_mock_grid(
        num_x_cells=100, num_y_cells=100, cell_size=0.1, occupancy_percentage=5
    )
    grid.plot()
    plt.show()


if __name__ == "__main__":
    main()
