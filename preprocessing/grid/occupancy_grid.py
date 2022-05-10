from matplotlib import pyplot as plt
import numpy as np
from typing import *


class OccupancyGrid:
    def __init__(self, cell_size: float, x_cells: int, y_cells: int) -> None:
        self._cell_size = cell_size
        self._x_cells = x_cells
        self._y_cells = y_cells
        self._grid = np.zeros(
            (x_cells, y_cells), dtype=np.int8
        )  # grid containing ints between 0 and 100, representing % occupancy

    def get_cell_size(self) -> float:
        return self._cell_size

    def get_x_cells(self) -> int:
        return self._x_cells

    def get_y_cells(self) -> int:
        return self._y_cells

    def get_grid(self) -> np.ndarray:
        return self._grid

    # gets the occupancy percentage given X and Y coordinates
    def __getitem__(self, index: Tuple[int, int]) -> int:
        return self._grid[index]

    # sets the occupancy percentage given X and Y coordinates
    def __setitem__(self, index: Tuple[int, int], value: int) -> None:
        assert value >= 0 and value <= 100
        self._grid[index] = value

    def plot(self) -> plt.figure:
        # Create a new figure
        fig = plt.figure()

        ax = fig.gca()

        # Plot the grid
        ax.pcolormesh(self.get_grid().T, cmap="PuBu", edgecolor="k", lw=2)
        return fig
    def get_occupied_cells(self) -> np.ndarray:
        return np.transpose(np.nonzero(self._grid))
    
    def get_empty_cells(self) -> np.ndarray:
        return np.transpose(np.nonzero(self._grid == 0))

# this is provided as an external utility, so you can cache the occupied cells if you want

def is_cell_occupied(occupied_cells: np.ndarray, cell: np.ndarray) -> bool:
    return any(np.equal(occupied_cells, cell).all(1))

def is_cell_empty(empty_cells: np.ndarray, cell: np.ndarray) -> bool:
    return any(np.equal(empty_cells, cell).all(1))

