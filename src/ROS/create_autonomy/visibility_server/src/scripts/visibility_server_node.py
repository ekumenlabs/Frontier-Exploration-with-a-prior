#!/usr/bin/python3.8
from multiprocessing import Lock
import pickle
import geopandas as gpd
from typing import Tuple
from FrontierExploration.preprocessing.grid.visibility_grid import VisibilityGrid
from FrontierExploration.preprocessing.plotting.live_plot_utils import LivePlotter
from nav_msgs.msg  import OccupancyGrid
from tqdm import tqdm
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse
import numpy as np

import rospy
from dataclasses import dataclass, field


class VisibilityServer(object):
    """
    Exposes a service that returns the visibility of a cell in map frame. 
    """
    OCCUPIED_THRESHOLD = 65 # [%]
    def __init__(self, grid: VisibilityGrid):
        self._lock = Lock()
        self._grid = grid
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
        self._global_costmap_sub = rospy.Subscriber("/map", OccupancyGrid, self._occupancy_grid_update_cb)
    
    def _occupancy_grid_update_cb(self, msg: OccupancyGrid) -> None:
        print("here!")
        
        metadata = msg.info
        origin = metadata.origin
        origin_translation = np.array([origin.position.x, origin.position.y, origin.position.z])
        origin_x = origin_translation[0]
        origin_y = origin_translation[1]
        height = metadata.height
        width = metadata.height
        data = np.asarray(msg.data, np.int8).reshape(width, height)
        resolution = metadata.resolution
        non_unknown_indices = np.nonzero(data == -1)
        occupied_indices = np.nonzero(data >= self.OCCUPIED_THRESHOLD)
        seen_indices = np.nonzero(np.logical_and(data < self.OCCUPIED_THRESHOLD, data != -1))
        with self._lock:
            # for x, y  in tqdm(zip(seen_indices[0], seen_indices[1])):
            #     x_map_frame = origin_x + resolution * x
            #     y_map_frame = origin_y + resolution * y
            #     self._grid.set_seen(x_map_frame, y_map_frame)

            for x, y  in tqdm(zip(occupied_indices[0], occupied_indices[1])):
                x_map_frame = origin_x + resolution * x
                y_map_frame = origin_y + resolution * y
                self._grid.set_occupied(x_map_frame, y_map_frame)

    
    
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
        _plotter = LivePlotter(self._grid)
        _plotter.run()
        while not rospy.is_shutdown():
            rospy.sleep(1)
        print("here")

BASE_FILES_DIR = "/create_ws/Frontier-Exploration-with-a-prior/Notebooks/files"

    
if __name__ == '__main__':
    rospy.init_node('VisibilityServer', anonymous=False, log_level=rospy.DEBUG)
    # TODO expose parameters via parameter server

    file_dir = f"{BASE_FILES_DIR}/small_house_clean.dxf"
    layout = gpd.read_file(file_dir)
    visibility_grid = VisibilityGrid(layout=layout)
    try:
        visibility_server = VisibilityServer(grid=visibility_grid)
        visibility_server.run()
    except rospy.ROSInterruptException:
        pass
    except:
        exit(0)
