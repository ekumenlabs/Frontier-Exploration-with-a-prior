#!/usr/bin/python3.8
from dataclasses import dataclass
from datetime import datetime
from multiprocessing import Event, Lock, Process
import os
import pickle
import geopandas as gpd
from FrontierExploration.preprocessing.grid.raycasting import SEEN, OCCUPIED, UNKNOWN
from FrontierExploration.preprocessing.grid.visibility_grid import VisibilityGrid
from FrontierExploration.preprocessing.plotting.live_plot_utils import LivePlotter
from nav_msgs.msg  import OccupancyGrid
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse
import numpy as np
import shapely
from math import isclose
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, TimeoutError

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
    def __init__(self, grid: VisibilityGrid, start_pos: StartPosition, plot: bool):
        self._lock = Lock()
        self._grid = grid
        self._start_position = start_pos
        self._global_costmap_sub = rospy.Subscriber("/map", OccupancyGrid, self._occupancy_grid_cb)
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
        self._plotter = LivePlotter(self._grid, self._lock) if plot else None


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
        start = datetime.now()
        response = VisibilityResponse()
        stop_visibility_evt = Event()
        with ThreadPoolExecutor(1) as pool:
            with self._lock:
                p = pool.submit(self._grid.visibility,x=req.x_map_frame + self._start_position.x, y=req.y_map_frame + self._start_position.y, stop_event=stop_visibility_evt)
                try:
                    visibility = int(p.result(timeout=0.5))
                except TimeoutError:
                    rospy.logwarn("Timedout getting visibility, returning 0.")
                    stop_visibility_evt.set()
                    visibility=0
        response.visibility = visibility
        print(f"Took {datetime.now() - start} to compute visibility")
        return response

    def run(self):
        """
        Main entrypoint for the visibility server node.
            """
        if self._plotter is not None:
            self._plotter.run()
        while not rospy.is_shutdown():
            rospy.sleep(1)
    def stop(self):
        if self._plotter is not None:
            self._plotter.stop()
        self._plotter = None


BASE_FILES_DIR = "/create_ws/Frontier-Exploration-with-a-prior/Notebooks/files"

    
if __name__ == '__main__':
    rospy.init_node('VisibilityServer', anonymous=False, log_level=rospy.DEBUG)

    start_pos_x = float(rospy.get_param("~start_x"))
    start_pos_y = float(rospy.get_param("~start_y"))
    polygon_path = rospy.get_param("~polygon_path")

    with open(polygon_path, "rb") as fp:
        polygon = pickle.load(fp).exterior.buffer(0.2)
    plot = rospy.get_param("~plot")

    start_pos = StartPosition(x=start_pos_x, y=start_pos_y)

    visibility_grid = VisibilityGrid(layout=polygon, square_size=0.05)
    try:
        visibility_server = VisibilityServer(grid=visibility_grid, start_pos=start_pos, plot=plot)
        visibility_server.run()
    except rospy.ROSInterruptException:
        visibility_server.stop()
