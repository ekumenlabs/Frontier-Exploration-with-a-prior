from ast import Mult
from audioop import getsample
from tracemalloc import start
from FrontierExploration.preprocessing.grid.raycasting import RayCast, BlockStatus
from shapely.geometry import Point
from FrontierExploration.preprocessing.layout.polygons import Square
import os
import geopandas as gpd
import numpy as np
import pandas as pd
from rasterio.features import shapes, rasterize
from shapely.geometry import Polygon, MultiPolygon
from shapely.affinity import affine_transform

import matplotlib.pyplot as plt

UNKNOWN = -1
OCCUPIED = 100
SEEN = 50

def get_polygons_from_shape(shape) -> MultiPolygon:
    ret = None
    for s in shape:
        if ret is None:
            ret = Polygon(*s[0]["coordinates"])
        try:
            shape = shape.union(Polygon(*s[0]["coordinates"]))
        except Exception as e:
            print(e)
            pass
    return ret

class VisibilityGrid():
    def __init__(self, layout: Polygon, square_size: float =0.05) -> None:
        self._square_size = square_size
        layout = affine_transform(layout, [1.0/square_size, 0, 0, 1.0/square_size, 0, 0])
        bounds = layout.bounds
        img_size = (int(bounds[3] - bounds[1]) + int(1 / square_size), int(bounds[2] - bounds[0]) + int(1 / square_size))
        self._layout_image : np.ndarray  = rasterize([layout], out_shape=img_size, fill=UNKNOWN, default_value=OCCUPIED)
        self._last_raycast = None
    
    def visibility(self, x: float, y: float) -> float:
        raycast = RayCast(72, 12, x ,y )
        raycast.run_on_df(self._occupancy_df)
        self._last_raycast = raycast
        return raycast.visibility_percentage

    def plot(self) -> None:
        plt.imshow(self._layout_image)

def main() -> None:
    files_dir = f"/home/ramiro/Frontier-Exploration-with-a-prior/Notebooks/files"
    file_dir = f"{files_dir}/small_house_clean.dxf"
    layout = gpd.read_file(file_dir)
    visibility_grid = VisibilityGrid(layout=layout,  square_size=1)
    return visibility_grid
    
if __name__ == "__main__":
    main()
