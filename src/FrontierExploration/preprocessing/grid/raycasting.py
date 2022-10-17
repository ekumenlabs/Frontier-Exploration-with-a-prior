from math import sqrt
from multiprocessing import Event
from typing import Optional
from enum import Enum
import numpy as np
import geopandas as gpd
from shapely.geometry import Polygon, LineString, Point, MultiLineString

from tqdm import tqdm

from FrontierExploration.preprocessing.grid.occupancy_grid import is_cell_empty, is_cell_empty


UNKNOWN = -1
OCCUPIED = 100
SEEN = 50

num_rays = 70
ALLOWED_MOVEMENTS = [(np.cos(angle), np.sin(angle)) for angle in np.linspace(0, 2 * np.pi, num=num_rays, endpoint=False)]

def is_cell_in_array(cell: np.ndarray, array: np.ndarray) -> bool:
    return any(np.equal(array, cell).all(1))

# for a direction, return all cells that are visible from the given start cell.
def raycast_in_direction(start_cell: np.ndarray, direction: np.ndarray, available_cells: np.ndarray, range_in_cells:int, seen_cells: Optional[np.ndarray]) -> np.ndarray:
    # assert is_cell_empty(empty_cells=available_cells, cell=start_cell)
    assert range_in_cells > 0
    should_check_for_seen_cells = seen_cells is not None
    # start from the middle of the cell
    start_cell = np.add(start_cell, [0.5, 0.5])
    ret = []
    for _ in range(range_in_cells):
        moved_cell = np.add(start_cell, direction)
        moved_cell_floor = np.floor(moved_cell, dtype=float).astype(int)
        if not is_cell_empty(empty_cells=available_cells, cell=moved_cell_floor):
            break
        if tuple(moved_cell_floor) not in ret:
            if not should_check_for_seen_cells:
                ret.append(tuple(moved_cell_floor))
            elif not is_cell_in_array(moved_cell_floor, seen_cells):
                ret.append(tuple(moved_cell_floor))
        start_cell = moved_cell
    return ret


# return visible cells in all allowed directions. Allows for a "squarify", which is, return the largest square possible instead of the circle.
def raycast_in_every_direction(start_cell: np.ndarray, available_cells: np.ndarray, range_in_cells:int, squarify=False, outermost=False, seen_cells: Optional[np.ndarray]= None) -> np.ndarray:
    ret = []
    max_square_size = 1 + round(range_in_cells/ sqrt(2))
    for direction in ALLOWED_MOVEMENTS:
        visible_cells = raycast_in_direction(start_cell=start_cell, direction=direction, available_cells=available_cells, range_in_cells=range_in_cells, seen_cells=seen_cells)
        visible_cells_to_add = []
        visible_cells_relative = [np.subtract(cell, start_cell) for cell in visible_cells]
        if not squarify:
            visible_cells_to_add = visible_cells
        else:
            for idx, relative_cell in enumerate(visible_cells_relative):
                if abs(np.dot(relative_cell, [1, 0])) >= max_square_size or abs(np.dot(relative_cell, [0, 1])) >= max_square_size:
                    continue
                visible_cells_to_add.append(visible_cells[idx])
        if(len(visible_cells_to_add) == 0):
            continue
        if(outermost):
            ret.append(visible_cells_to_add[-1])
        else:
            ret.extend(visible_cells_to_add)

    return np.unique(ret,axis=0)


class BlockStatus(Enum):
    Seen = SEEN
    Occupied = OCCUPIED
    Unknown = UNKNOWN
    Raycasted = 125

class RayCast:
    def __init__(self, ray_range: float, start_x: float = 0, start_y: float = 0):
        self._circle_to_raycast = Point(start_x, start_y).buffer(ray_range)

    def run_on_polygons(self, seen_polygon: Polygon, seen_and_unknown_polygon: Polygon, stop_event: Optional[Event] = None)-> float:
        self._raycasted_circle = self._circle_to_raycast.intersection(seen_and_unknown_polygon).difference(seen_polygon)
        self._visibility = 100.0*(self._raycasted_circle.area / self._circle_to_raycast.area)

    @property
    def visibility(self) -> float:
        return self._visibility
    @property
    def raycasted_circle(self) -> Polygon:
        return self._raycasted_circle




