from tqdm import tqdm
from typing import List, Optional, Tuple
import ezdxf
import geopandas as gpd
import pandas as pd
import numpy as np
from pcg_gazebo.simulation import SimulationModel, \
    add_custom_gazebo_resource_path
from pcg_gazebo.generators.creators import extrude
from pcg_gazebo.generators.shapes import random_rectangles, \
    random_rectangle, rectangle
from pcg_gazebo.generators import WorldGenerator
from shapely.geometry import Polygon, box as Box

from FrontierExploration.preprocessing.layout.polygons import Square
from enum import Enum
from FrontierExploration.preprocessing.grid.occupancy_grid import OccupancyGrid


class BlockStatus(Enum):
    Free = 0
    Occuped = 1
    Unknown = 2


class LayoutReader: 
    def __init__(self, file_name: str, file_extension: str,files_dir: str):
        self.files_dir = files_dir
        self.file_name = file_name
        self.file_extension = file_extension
        self.world_generator = WorldGenerator()
        self.layout = gpd.read_file(f"{files_dir}{file_name}.{file_extension}")
    
    def create_gazebo_model(
        self,
        output_file_dir: str,
        position_phase: Tuple[int, int] = (0, 0),
        wall_thickness: float = 0.1,
        wall_height: float = 10,
        show: bool = True
    ):
        # Create the wall model based on the extruded
        # boundaries of the polygon
        self.world_generator.world.name = self.file_name + '.world'
        walls_model_list = []
        counter = 0
        for polygon in tqdm(self.layout.unary_union.geoms):  
            box = Box(*polygon.bounds)
            model = extrude(
                polygon=polygon,
                thickness=wall_thickness,
                height=wall_height,
                pose=[box.centroid.x-position_phase[0], box.centroid.y-position_phase[1], wall_height / 2., 0, 0, 0],
                extrude_boundaries=False,
                color='xkcd')
            counter += 1
            model.name = self.file_name + str(counter) + '_wall'
            self.world_generator.world.add_model(
            tag=model.name,
            model=model)
        if show:
            self.world_generator.world.show()
        return self._export_world(output_file_dir)
    
    def _export_world(self, export_dir: str, create_path: bool = True) -> str:
        export_models_dir = f"{export_dir}/models"
        export_world_dir = f"{export_dir}/worlds"
        if create_path:
            add_custom_gazebo_resource_path(export_models_dir)
        full_world_filename = self.world_generator.export_world(
            output_dir=export_world_dir,
            filename=self.world_generator.world.name,
            models_output_dir=export_models_dir,
            with_default_sun=False,
            overwrite=True)
        return full_world_filename

    def _add_ground_plane(self):
        self.world_generator.world.add_model(
            tag='ground_plane',
            model=SimulationModel.from_gazebo_model('ground_plane'))

    def create_clean_dxf(self, output_file_dir: str, layers: List[str], entity_types: Optional[List[str]]=["*"]):
      msp = ezdxf.readfile(f"{self.files_dir}{self.file_name}.{self.file_extension}").modelspace()
      clean_doc = ezdxf.new('R2010')
      clean_msp = clean_doc.modelspace()

      for layer in layers:
        for entity_type in entity_types:
          for e in msp.query(f"{entity_type} [layer=='{layer}']"):
            clean_msp.add_foreign_entity(e)
      clean_doc.saveas(output_file_dir)


class OccupancyDataFrame:
    def __init__(self, file_dir: str, square_size: int = 10):
        self.square_size = square_size
        polygons_list = []
        layout = gpd.read_file(file_dir)
        clean_layout = layout['geometry']
        xmin, ymin, xmax, ymax = clean_layout.total_bounds
        for xstart in np.nditer(np.arange(int(xmin) - square_size, int(xmax), square_size)):
            for ystart in  np.nditer(np.arange(int(ymin) - square_size, int(ymax), square_size)):
                polygons_list.append(Square(square_size, xstart, ystart))

        self.layout_df = gpd.GeoDataFrame(geometry=polygons_list)
        self.layout_df["intersects"]= self.layout_df.intersects(clean_layout.unary_union)
        self.layout_df.loc[layout_df["intersects"] == True, "status"] = BlockStatus.Occuped.value
        self.layout_df.loc[layout_df["intersects"] == False, "status"] = BlockStatus.Unknown.value
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