#!/usr/bin/python3.8
from datetime import datetime
from multiprocessing import Lock
import geopandas as gpd
from FrontierExploration.preprocessing.grid.visibility_grid import VisibilityGrid, SEEN, OCCUPIED, UNKNOWN
from FrontierExploration.preprocessing.plotting.live_plot_utils import LivePlotter
from nav_msgs.msg  import OccupancyGrid
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse
import numpy as np
import shapely
from math import isclose

import rospy

START_POS = (8, 27) # [m] in blueprints frame

class VisibilityServer(object):
    """
    Exposes a service that returns the visibility of a cell in map frame. 
    """
    OCCUPIED_THRESHOLD = 65 # [%]
    RADIUS_OF_INTEREST = 10 # [m]
    def __init__(self, grid: VisibilityGrid):
        self._lock = Lock()
        self._grid = grid
        self._robot_pose = None
        self._global_costmap_sub = rospy.Subscriber("/map", OccupancyGrid, self._occupancy_grid_cb)
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
        self._plotter = LivePlotter(self._grid, self._lock)


    def _subsample_data(self, data: np.ndarray, origin_x: float, origin_y:float, resolution:float, cells_to_keep:int) -> np.ndarray:
        robot_position_in_cells_x = int((self._robot_pose.x - origin_x) / resolution)
        robot_position_in_cells_y = int((self._robot_pose.y - origin_y) / resolution)
        lower_cell_bound_x = max(0, robot_position_in_cells_x - cells_to_keep)
        lower_cell_bound_y = max(0, robot_position_in_cells_y - cells_to_keep)
        upper_cell_bound_x = min(data.shape[0], robot_position_in_cells_x + cells_to_keep)
        upper_cell_bound_y = min(data.shape[1], robot_position_in_cells_y + cells_to_keep)
        return data[lower_cell_bound_x:upper_cell_bound_x][lower_cell_bound_y:upper_cell_bound_y]


    def _occupancy_grid_cb(self, msg: OccupancyGrid) -> None:
        metadata = msg.info
        origin = metadata.origin
        origin_translation = np.array([origin.position.x, origin.position.y, origin.position.z])
        origin_x = origin_translation[0] + START_POS[0]
        origin_y = origin_translation[1] + START_POS[1]
        height = metadata.height
        width = metadata.width
        data = np.asarray(msg.data, np.int8).reshape(height, width)
        assert isclose(metadata.resolution, self._grid._square_size, rel_tol=1) , "Square size and maps resolution should match."

        idx_offset_x = int(origin_x // metadata.resolution)
        idx_offset_y = int(origin_y // metadata.resolution)
        # account for scans outside of the blueprint
        if idx_offset_x < 0:
            data = data[: , -idx_offset_x:]
            height += idx_offset_x
            idx_offset_x = 0


        if idx_offset_y < 0:
            data = data[-idx_offset_y:, :]
            width += idx_offset_y
            idx_offset_y = 0
        data = data
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
            response.visibility =  int(self._grid.visibility(req.x_map_frame, req.y_map_frame))
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

    file_dir = f"{BASE_FILES_DIR}/small_house_clean.dxf"
    layout = gpd.read_file(file_dir)

    bounds = layout.unary_union.bounds
    tf_to_zero = [1, 0, 0, 1, -bounds[0], -bounds[1]]

    layout = layout["geometry"].apply(lambda x: shapely.affinity.affine_transform(x, tf_to_zero)).unary_union.buffer(0.3)
    visibility_grid = VisibilityGrid(layout=layout, square_size=0.05)
    try:
        visibility_server = VisibilityServer(grid=visibility_grid)
        visibility_server.run()
    except rospy.ROSInterruptException:
        visibility_server.stop()
