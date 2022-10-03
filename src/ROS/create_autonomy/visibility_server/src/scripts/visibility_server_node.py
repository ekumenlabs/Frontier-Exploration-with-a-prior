#!/usr/bin/python3.8
from dataclasses import dataclass
from datetime import datetime
from multiprocessing import Lock
import geopandas as gpd
from FrontierExploration.preprocessing.grid.raycasting import SEEN, OCCUPIED, UNKNOWN
from FrontierExploration.preprocessing.grid.visibility_grid import VisibilityGrid
from FrontierExploration.preprocessing.plotting.live_plot_utils import LivePlotter
from nav_msgs.msg  import OccupancyGrid
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse
import numpy as np
import shapely
from math import isclose

import rospy

@dataclass(frozen=True)
class StartPosition:
    x: float
    y: float

class VisibilityServer(object):
    """
    Exposes a service that returns the visibility of a cell in map frame. 
    """
    OCCUPIED_THRESHOLD = 65 # [%]
    RADIUS_OF_INTEREST = 10 # [m]
    def __init__(self, grid: VisibilityGrid, start_pos: StartPosition):
        self._lock = Lock()
        self._grid = grid
        self._start_position = start_pos
        self._global_costmap_sub = rospy.Subscriber("/map", OccupancyGrid, self._occupancy_grid_cb)
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
        self._plotter = LivePlotter(self._grid, self._lock)


    def _occupancy_grid_cb(self, msg: OccupancyGrid) -> None:
        metadata = msg.info
        origin = metadata.origin
        origin_translation = np.array([origin.position.x, origin.position.y, origin.position.z])
        origin_x = origin_translation[0] + self._start_position.x
        origin_y = origin_translation[1] + self._start_position.y
        height = metadata.height
        width = metadata.width
        data = np.asarray(msg.data, np.int8).reshape(height, width)
        assert isclose(metadata.resolution, self._grid._square_size, rel_tol=1) , "Square size and maps resolution should match."

        idx_offset_x = int(origin_x // metadata.resolution)
        idx_offset_y = int(origin_y // metadata.resolution)
        # account for scans outside of the blueprint
        img_width = self._grid._layout_image.shape[0]
        img_height = self._grid._layout_image.shape[1]
        if idx_offset_x < 0:
            data = data[: , -idx_offset_x:]
            height += idx_offset_x
            idx_offset_x = 0


        if idx_offset_y < 0:
            data = data[-idx_offset_y:, :]
            width += idx_offset_y
            idx_offset_y = 0
        
        width = min(width, img_width)
        height = min(height, img_height)

        data = data[:width, :height]
        data[data> 0.65] = OCCUPIED
        data[np.logical_and(data<= 0.65, data!=UNKNOWN)] = 0
        data[data == UNKNOWN] = 1
        with self._lock:
            # TODO comment this logic
            start = datetime.now()
            slice_cpy = self._grid._layout_image[idx_offset_y:idx_offset_y+width, idx_offset_x:idx_offset_x+height].copy()
            slice_cpy = np.multiply(data, slice_cpy)
            slice_cpy[slice_cpy == 0] = SEEN
            slice_cpy[slice_cpy == SEEN * OCCUPIED] = OCCUPIED
            slice_cpy[slice_cpy == -OCCUPIED] = OCCUPIED
            slice_cpy[slice_cpy == OCCUPIED * OCCUPIED] = OCCUPIED
            self._grid._layout_image[idx_offset_y:idx_offset_y+width, idx_offset_x:idx_offset_x+height] = slice_cpy
            self._grid.update_layout()
            print(f"Took {datetime.now() - start} to update the map.")

    
    def _visibility_cb(self, req: VisibilityRequest) -> VisibilityResponse:
        """
        returns visibility of the cell corresponding to the given position in the map frame
        """
        response = VisibilityResponse()
        with self._lock:
            response.visibility =  int(self._grid.visibility(req.x_map_frame + self._start_position.x, req.y_map_frame + self._start_position.y))
        return response

    def run(self):
        """
        Main entrypoint for the visibility server node.
            """
        self._plotter.run()
        while not rospy.is_shutdown():
            rospy.sleep(1)
    def stop(self):
        self._plotter.stop()
        self._plotter = None


BASE_FILES_DIR = "/create_ws/Frontier-Exploration-with-a-prior/Notebooks/files"

    
if __name__ == '__main__':
    rospy.init_node('VisibilityServer', anonymous=False, log_level=rospy.DEBUG)
    # TODO expose parameters via parameter server

    file_dir = f"{BASE_FILES_DIR}/small_house_polygon_clean.dxf"
    layout = gpd.read_file(file_dir)

    bounds = layout.unary_union.bounds
    tf_to_zero = [1, 0, 0, 1, -bounds[0], -bounds[1]]

    start_pos_x = float(rospy.get_param("~start_x"))
    start_pos_y = float(rospy.get_param("~start_y"))

    start_pos = StartPosition(x=start_pos_x, y=start_pos_y)

    layout = layout["geometry"].apply(lambda x: shapely.affinity.affine_transform(x, tf_to_zero)).unary_union.buffer(0.2)
    visibility_grid = VisibilityGrid(layout=layout, square_size=0.05)
    try:
        visibility_server = VisibilityServer(grid=visibility_grid, start_pos=start_pos)
        visibility_server.run()
    except rospy.ROSInterruptException:
        visibility_server.stop()
