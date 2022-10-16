from multiprocessing import Event
from FrontierExploration.preprocessing.grid.raycasting import RayCast, OCCUPIED, UNKNOWN
import geopandas as gpd
import numpy as np
from rasterio.features import shapes, rasterize
from shapely.geometry import Polygon
from shapely.geometry import shape
from shapely.affinity import affine_transform

class VisibilityGrid():
    def __init__(self, layout: Polygon, square_size: float =0.05) -> None:
        self._square_size = square_size
        layout = affine_transform(geom=layout, matrix=[1.0/square_size, 0, 0, 1.0/square_size, 0, 0])
        bounds = layout.bounds
        img_size = (int(bounds[3] - bounds[1]) + int(1 / square_size), int(bounds[2] - bounds[0]) + int(1 / square_size))
        self._layout_image : np.ndarray  = rasterize([layout], out_shape=img_size, fill=UNKNOWN, default_value=OCCUPIED)
        self._occupancy_df = None
        self.update_layout()
        self._last_raycast = None

    def update_layout(self) -> None:
        shapess = shapes(self._layout_image, connectivity=8)
        geometries = []
        values = []
        for shapedict, value in shapess:
            geometries.append(shape(shapedict))
            values.append(value)
        self._occupancy_df = gpd.GeoDataFrame({"geometry":geometries, "status": values})
        self._occupancy_df["geometry"] = self._occupancy_df.simplify(tolerance=0.3, preserve_topology=False)

    
    def visibility(self, x: float, y: float, stop_event: Event) -> float:
        raycast = RayCast(16, 12 / self._square_size, x / self._square_size ,y / self._square_size )
        raycast.run_on_df(self._occupancy_df, stop_event=stop_event)
        self._last_raycast = raycast
        return raycast.visibility_percentage

    def plot(self) -> None:
        if self._last_raycast is not None:
            to_plot = self._last_raycast.raycast_df.append(self._occupancy_df)
            to_plot.plot(column="status")
        else:
            self._occupancy_df.plot(column="status")

def main() -> None:
    files_dir = f"/home/ramiro/Frontier-Exploration-with-a-prior/Notebooks/files"
    file_dir = f"{files_dir}/small_house_clean.dxf"
    layout = gpd.read_file(file_dir)
    visibility_grid = VisibilityGrid(layout=layout,  square_size=1)
    return visibility_grid
    
if __name__ == "__main__":
    main()
