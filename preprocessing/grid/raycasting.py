from grid.occupancy_grid import is_cell_empty, is_cell_empty
import functools

import numpy as np

ALLOWED_MOVEMENTS = [(np.cos(angle), np.sin(angle)) for angle in np.linspace(0, 2 * np.pi, num=16, endpoint=False)]

def is_cell_in_array(cell: np.ndarray, array: np.ndarray) -> bool:
    return any(np.equal(array, cell).all(1))

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

def raycast_in_every_direction(start_cell: np.ndarray, available_cells: np.ndarray, range_in_cells:int) -> np.ndarray:
    ret = []
    for direction in ALLOWED_MOVEMENTS:
        ret.extend(raycast_in_direction(start_cell=start_cell, direction=direction, available_cells=available_cells, range_in_cells=range_in_cells))
    return np.unique(ret,axis=0)




