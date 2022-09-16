#!/usr/bin/python3.8
from datetime import datetime
from multiprocessing import Lock
import geopandas as gpd
from FrontierExploration.preprocessing.grid.visibility_grid import VisibilityGrid
from FrontierExploration.preprocessing.plotting.live_plot_utils import LivePlotter
from nav_msgs.msg  import OccupancyGrid
from tqdm import tqdm
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse
from tf2_msgs.msg import TFMessage
import numpy as np
from signal import signal
from geometry_msgs.msg import Point

import rospy


class VisibilityServer(object):
    """
    Exposes a service that returns the visibility of a cell in map frame. 
    """
    OCCUPIED_THRESHOLD = 65 # [%]
    RADIUS_OF_INTEREST = 10 # [m]
    def __init__(self, grid: VisibilityGrid):
        self._lock = Lock()
        self._grid = grid
        self._skip_maps = 20
        self._map_cnt = self._skip_maps - 1
        self._robot_pose = None
        self._robot_pose_sub = rospy.Subscriber("/robot_position_2d", Point, self._robot_pose_cb)
        self._global_costmap_sub = rospy.Subscriber("/map", OccupancyGrid, self._occupancy_grid_cb)
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
        self._plotter = LivePlotter(self._grid, self._lock)
    
    def _robot_pose_cb(self, msg:Point) -> None:
        self._robot_pose = Point(msg.x+10, msg.y+10, msg.z)



    def _subsample_data(self, data: np.ndarray, origin_x: float, origin_y:float, resolution:float, cells_to_keep:int) -> np.ndarray:
        robot_position_in_cells_x = int((self._robot_pose.x - origin_x) / resolution)
        robot_position_in_cells_y = int((self._robot_pose.y - origin_y) / resolution)

        lower_cell_bound_x = max(0, robot_position_in_cells_x - cells_to_keep)
        lower_cell_bound_y = max(0, robot_position_in_cells_y - cells_to_keep)
        upper_cell_bound_x = min(data.shape[0], robot_position_in_cells_x + cells_to_keep)
        upper_cell_bound_y = min(data.shape[1], robot_position_in_cells_y + cells_to_keep)
        print(self._robot_pose)
        # robot position_map = origin + cell / resolution

        return data[lower_cell_bound_x:upper_cell_bound_x][lower_cell_bound_y:upper_cell_bound_y]


    def _occupancy_grid_cb(self, msg: OccupancyGrid) -> None:
        if self._robot_pose is None:
            return
        self._map_cnt += 1
        self._map_cnt %= self._skip_maps
        if not self._map_cnt == 0:
            return
        metadata = msg.info
        origin = metadata.origin
        origin_translation = np.array([origin.position.x, origin.position.y, origin.position.z])
        origin_x = origin_translation[0] + 10
        origin_y = origin_translation[1] + 10
        height = metadata.height
        width = metadata.width
        data = np.asarray(msg.data, np.int8).reshape(width, height)
        # subsample data.        
        resolution = metadata.resolution
        cells_to_keep = int(self.RADIUS_OF_INTEREST / resolution) * 2
        if data.shape[0] > cells_to_keep or data.shape[1] > cells_to_keep:
            data = self._subsample_data(data=data, origin_x=origin_x, origin_y=origin_y,
                                        resolution=resolution, cells_to_keep=cells_to_keep)

        process_between =  int(self._grid._square_size / resolution)
        seen_indices = np.nonzero(np.logical_and(data < self.OCCUPIED_THRESHOLD, data != -1))
        occupied_indices = np.nonzero(data > self.OCCUPIED_THRESHOLD)

        # n X m celdas


        """
        A B C
        D E F
        G H I

        """
        # G = ORIGIN
        # H = ORIGIN + (resolution, 0)

         
        with self._lock:
            start = datetime.now()
            for x, y  in zip(seen_indices[0][::process_between], seen_indices[1][::process_between]):
                x_map_frame = origin_x + resolution * x
                y_map_frame = origin_y + resolution * y
                self._grid.set_seen(x_map_frame, y_map_frame)
            for x, y  in zip(occupied_indices[0][::process_between], occupied_indices[1][::process_between]):
                x_map_frame = origin_x + resolution * x
                y_map_frame = origin_y + resolution * y
                self._grid.set_occupied(x_map_frame, y_map_frame)
            print(f"Updating grids took {datetime.now() - start}")

    
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
    visibility_grid = VisibilityGrid(layout=layout, square_size=1)
    try:
        visibility_server = VisibilityServer(grid=visibility_grid)
        visibility_server.run()
    except rospy.ROSInterruptException:
        visibility_server.stop()
