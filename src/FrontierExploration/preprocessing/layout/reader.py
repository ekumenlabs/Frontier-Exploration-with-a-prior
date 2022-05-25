from typing import List
import ezdxf
import geopandas as gpd
import pandas as pd

from preprocessing.layout.polygons import Square
from preprocessing.grid.occupancy_grid import OccupancyGrid

class LayoutReader:
    def __init__(self):
        self.occupancy_gdf = None
        self.layout_polygon = None

    @staticmethod
    def clean_and_save(file_dir: str, output_file_dir: str, layers: List[str]):
      msp = ezdxf.readfile(file_dir).modelspace()
      clean_doc = ezdxf.new('R2010')
      clean_msp = clean_doc.modelspace()

      for layer in layers:
        for e in msp.query(f"*[layer=='{layer}']"):
          clean_msp.add_foreign_entity(e)
      clean_doc.saveas(output_file_dir)


class OccupancyDataFrame:
    def __init__(self, file_dir: str, square_size: int = 10):
        self.square_size = square_size
        polygons_list = []
        layout = gpd.read_file(file_dir)
        clean_layout = layout['geometry']
        xmin, ymin, xmax, ymax = clean_layout.total_bounds

        for xstart in range(int(xmin) - square_size, int(xmax), square_size):
            for ystart in range(int(ymin) - square_size, int(ymax), square_size):
                polygons_list.append(Square(square_size, xstart, ystart))

        self.layout_df = gpd.GeoDataFrame(geometry=polygons_list)
        self.layout_df["intersects"]= self.layout_df.intersects(clean_layout.unary_union)
        self.occupancy_df = self.layout_df.apply(self.get_center, axis=1, result_type='expand').pivot(index ='axis_x', columns ='axis_y', values='occuped')

    def plot(self, *args, **kwargs):
        self.layout_df.query("intersects == True").plot(*args, **kwargs)

    def get_occupancy_grid(self) -> OccupancyGrid:
        occupancy_grid = OccupancyGrid(self.square_size, self.occupancy_df.shape[0], self.occupancy_df.shape[1])
        for x in range(occupancy_grid.get_x_cells()):
            for y in range(occupancy_grid.get_y_cells()):
                occupancy_grid[x, y] = 100 if self.occupancy_df.iloc[x, y] else 0
        return occupancy_grid

    @staticmethod
    def get_center(item):
        xmin, ymin, xmax, ymax = item["geometry"].bounds
        return pd.Series([(xmax+xmin)/2, (ymax+ymin)/2, int(item["intersects"])], index=['axis_x', 'axis_y', 'occuped'])