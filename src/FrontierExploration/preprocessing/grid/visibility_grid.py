from tracemalloc import start
from FrontierExploration.preprocessing.grid.raycasting import RayCast, BlockStatus
from FrontierExploration.preprocessing.layout.polygons import Square
import os
import geopandas as gpd
import numpy as np
import pandas as pd

class VisibilityGrid():
    def __init__(self, layout: gpd.GeoDataFrame, square_size: float =1) -> None:
        self._square_size = square_size
        clean_layout = layout['geometry']
        xmin, ymin, xmax, ymax = clean_layout.total_bounds
        polygons_list = []
        for xstart in np.nditer(np.arange(int(xmin) - square_size, int(xmax), square_size)):
            for ystart in  np.nditer(np.arange(int(ymin) - square_size, int(ymax), square_size)):
                square = Square(square_size, xstart, ystart)
                if square.intersects(clean_layout.unary_union):
                    polygons_list.append(Square(square_size, xstart, ystart))
        
        layout_df = gpd.GeoDataFrame(geometry=polygons_list)
        layout_df["intersects"]= layout_df.intersects(clean_layout.unary_union)
        layout_df["status"]= BlockStatus.Occupied.value
        self._occupancy_df = layout_df
        self._last_raycast = None
    
    def visibility(self, x: float, y: float) -> float:
        raycast = RayCast(72, 12, x ,y )
        raycast.run_on_df(self._occupancy_df)
        self._last_raycast = raycast
        return raycast.visibility_percentage
    
    def set_or_create_cell(self, x: float, y:float , status: BlockStatus)->None:
        # revisit, does this need to be the center
        square = Square(side = self._square_size, start_x= x, start_y = y)
        if self._occupancy_df.intersects(square).any():
            self._occupancy_df.loc[self._occupancy_df.intersects(square), "status"] = status.value
            return
        row_to_append = {"geometry": square, "status": status.value}
        self._occupancy_df = self._occupancy_df.append(row_to_append, ignore_index=True)
    
    def set_occupied(self, x:float ,y:float) -> None:
        self.set_or_create_cell(x=x, y=y, status=BlockStatus.Occupied)

    def set_seen(self, x:float ,y:float) -> None:
        self.set_or_create_cell(x=x, y=y, status=BlockStatus.Seen)

    def plot(self) -> None:
        if self._last_raycast is not None:
            df_to_plot = self._last_raycast.raycast_df.append(self._occupancy_df[self._occupancy_df["status"] == BlockStatus.Occupied.value])
            df_to_plot.plot()
        else:
            self._occupancy_df[self._occupancy_df["status"] == BlockStatus.Occupied.value].plot()


def main() -> None:
    files_dir = f"/home/ramiro/Frontier-Exploration-with-a-prior/Notebooks/files"
    file_dir = f"{files_dir}/small_house_clean.dxf"
    layout = gpd.read_file(file_dir)
    raycast = RayCast(72, 12)
    visibility_grid = VisibilityGrid(layout=layout,  square_size=1)
    return visibility_grid
    
if __name__ == "__main__":
    main()
