from math import sqrt
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
    def __init__(self, num_rays: int, ray_range: float, start_x: float = 0, start_y: float = 0):
        self.start_x = start_x
        self.start_y = start_y
        self._num_rays = num_rays
        self._ray_range = ray_range
        self.raycast_df = gpd.GeoDataFrame(geometry=[LineString([(start_x, start_y), movement]) for movement in self.directions])
        self.raycast_df["status"] = BlockStatus.Raycasted.value

    def run_on_df(self, layout_df: gpd.GeoDataFrame):
        for index in tqdm(layout_df[layout_df["status"] == BlockStatus.Occupied.value].index):
            self.intersect_with_polygon(layout_df['geometry'][index],layout_df['status'][index])

        for index in tqdm(layout_df[layout_df["status"] == BlockStatus.Seen.value].index):
            self.intersect_with_polygon(layout_df['geometry'][index], BlockStatus.Seen.value)
        
            
    def intersect_with_polygon(self, polygon: Polygon, block_status: BlockStatus):
        if block_status == BlockStatus.Occupied.value:
            self.raycast_df["geometry"] = self.raycast_df["geometry"].apply(self.get_line_to_intersection, args=(polygon,))
        if block_status == BlockStatus.Seen.value:
            poly_df = gpd.GeoDataFrame(geometry=[polygon])
            self.raycast_df["geometry"] = self.raycast_df.overlay(poly_df, how='difference')["geometry"]
            self.raycast_df.dropna(inplace=True)
    
    def get_line_to_intersection(self, line: LineString, polygon: Polygon):
        origin = self.origin_point
        if line.intersects(polygon):
            inter_line = line.difference(polygon)
            if isinstance(inter_line, MultiLineString):
                return inter_line[0]
            return inter_line
        return line        

    @property
    def visibility_percentage(self):
        return 100*self.visibility/self._ray_range
    
    @property
    def visibility(self):
        self.raycast_df.dropna(inplace=True)
        if self.raycast_df.empty:
            return 0
        self.raycast_df['length'] = self.raycast_df.apply(lambda row: row[0].length, axis=1)
        return self.raycast_df['length'].mean()
    
    @property
    def origin_point(self):
        return Point(self.start_x, self.start_y)
    
    @property
    def directions(self):
        return [(self._ray_range*np.cos(angle)+self.start_x, self._ray_range*np.sin(angle)+self.start_y) for angle in np.linspace(0, 2 * np.pi, num=self._num_rays, endpoint=False)]

    def plot(self, *args, **kwargs):
        self.raycast_df.plot( *args, **kwargs)