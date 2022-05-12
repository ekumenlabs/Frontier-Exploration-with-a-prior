from math import floor, sqrt
import pdb
from grid.occupancy_grid import is_cell_empty, is_cell_empty
import functools

import numpy as np

num_rays = 70
ALLOWED_MOVEMENTS = [(np.cos(angle), np.sin(angle)) for angle in np.linspace(0, 2 * np.pi, num=num_rays, endpoint=False)]

def is_cell_in_array(cell: np.ndarray, array: np.ndarray) -> bool:
    return any(np.equal(array, cell).all(1))

# for a direction, return all cells that are visible from the given start cell.
def raycast_in_direction(start_cell: np.ndarray, direction: np.ndarray, available_cells: np.ndarray, range_in_cells:int) -> np.ndarray:
    assert is_cell_empty(empty_cells=available_cells, cell=start_cell)
    assert range_in_cells > 0
    # start from the middle of the cell
    start_cell = np.add(start_cell, [0.5, 0.5])
    ret = []
    for _ in range(range_in_cells):
        moved_cell = np.add(start_cell, direction)
        moved_cell_floor = np.floor(moved_cell, dtype=float).astype(int)
        if not is_cell_empty(empty_cells=available_cells, cell=moved_cell_floor):
            break
        if tuple(moved_cell_floor) not in ret:
            ret.append(tuple(moved_cell_floor))
        start_cell = moved_cell
    return ret


# return visible cells in all allowed directions. Allows for a "squarify", which is, return the largest square possible instead of the circle.
def raycast_in_every_direction(start_cell: np.ndarray, available_cells: np.ndarray, range_in_cells:int, squarify=False, outermost=False) -> np.ndarray:
    ret = []
    max_square_size = range_in_cells/ sqrt(2)
    for direction in ALLOWED_MOVEMENTS:
        visible_cells = raycast_in_direction(start_cell=start_cell, direction=direction, available_cells=available_cells, range_in_cells=range_in_cells)
        visible_cells_to_add = []
        if not squarify:
            visible_cells_to_add = visible_cells
            continue
        visible_cells_relative = [np.subtract(cell, start_cell) for cell in visible_cells]
        for idx, relative_cell in enumerate(visible_cells_relative):
            if abs(np.dot(relative_cell, [1, 0])) >= max_square_size or abs(np.dot(relative_cell, [0, 1])) >= max_square_size:
                continue
            visible_cells_to_add.append(visible_cells[idx])
        if(outermost and len(visible_cells_to_add) != 0):
            ret.append(visible_cells_to_add[-1])
        else:
            ret.extend(visible_cells_to_add)

    return np.unique(ret,axis=0)
                