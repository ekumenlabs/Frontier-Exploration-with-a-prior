
from datetime import datetime
import pdb
from random import random
import numpy as np
from grid.occupancy_grid import OccupancyGrid, is_cell_empty, is_cell_occupied
import matplotlib.pyplot as plt
from grid.mock_grid import create_mock_grid
from preprocessing.grid.raycasting import raycast_in_every_direction
from typing import List
from copy import copy

def paginate(lst:List[any], n: int):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

class WaypointsPicker(object):
    def __init__(self, grid: OccupancyGrid):
        self._grid = grid
        self._original_available_cells = grid.get_empty_cells()
        self._viewed_grid = copy(grid)
        self._cells_to_view = grid.get_empty_cells()
        self._range_in_cells = 5
        self._rc_grid = None

    def _visible_grid(self) -> None:
        cell_to_evaluate = tuple(self._original_available_cells[np.random.choice(range(len(self._original_available_cells)))])
        ret = []
        while len(self._cells_to_view) >= len(self._original_available_cells) * 0.1:
            print(len(self._cells_to_view))
            visible_cells = self._raycast_in_every_direction(cell=cell_to_evaluate, available_cells=self._cells_to_view)
            if len(visible_cells) >= 10:
                ret.append(cell_to_evaluate)
                for cell in self._raycast_in_every_direction(cell=cell_to_evaluate, available_cells=self._original_available_cells):
                    self._viewed_grid[tuple(cell)] = 100
                self._viewed_grid[cell_to_evaluate] = 100
                self._cells_to_view = self._grid.get_empty_cells()
            cell_to_evaluate = tuple(self._cells_to_view[np.random.choice(range(len(self._cells_to_view)))])
        return ret

    def _raycasting_grid(self) -> None:
        time = datetime.now()
        cpu_cnt = 8
        chunk_size = len(self._original_available_cells) // cpu_cnt
        ret = []
        for index_to_process in paginate(range(len(self._original_available_cells)), chunk_size):
            ret.extend(self._raycast_some(index_to_process))
        print(f"Took {datetime.now() - time} to raycast")
        return ret

    def _raycast_some(self,indices_to_raycast : List[int] ) -> None:
        ret = []
        for index in indices_to_raycast:
            cell = self._original_available_cells[index]
            raycasted = self._raycast_in_every_direction(cell=cell,  available_cells=self._original_available_cells)
            for cell in raycasted:
                self._grid[tuple(cell)] = 50
                ret.append(cell)
        return ret
            


    def _raycast_in_every_direction(self, cell: tuple, available_cells: np.ndarray) -> List[tuple]:
        return raycast_in_every_direction(available_cells=available_cells, start_cell=cell, range_in_cells=self._range_in_cells, squarify=True)
    def plot(self):
        return self._grid.plot()


if __name__ == "__main__":
    grid = create_mock_grid(num_x_cells=50, num_y_cells=50, cell_size=0.1, occupancy_percentage=0)
    p = WaypointsPicker(grid=grid)
    cells_to_plot = p._raycasting_grid()

    fig = p.plot()
    ax = fig.gca()
    for cell in cells_to_plot:  
        ax.plot(cell[0] + 0.5, cell[1] + 0.5, 'rx')
    plt.show()
