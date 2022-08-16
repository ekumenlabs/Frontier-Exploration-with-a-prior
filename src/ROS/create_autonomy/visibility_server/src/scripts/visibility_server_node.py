#!/usr/bin/python3.8
from typing import Any
import FrontierExploration.preprocessing.grid.occupancy_grid as grid
from FrontierExploration.preprocessing.grid.mock_grid import create_mock_grid
from math import hypot
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse

import rospy
# from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
# from threading import Lock
# from tf import TransformListener


class VisibilityServer(object):
    """Consumvisibility_serveres time elapsed and robot odometry to estimate battery consumption
    """
    def __init__(self, grid: grid.OccupancyGrid):
        self._grid = grid
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
    
    
    def _visibility_cb(self, req: VisibilityRequest) -> VisibilityResponse:
        """
        returns visibility of the cell corresponding to the given position in the map frame
        """
        coordinate = (int(req.x_map_frame / self._grid.get_cell_size()), int(req.y_map_frame / self._grid.get_cell_size()))
        response = VisibilityResponse()
        try:

            response.visibility =  grid[coordinate]
        except IndexError:
            response.visibility = 0
        return response

    def run(self):
        """
        Main entrypoint for the mocker.
        """
        pass
        while not rospy.is_shutdown():
            rospy.sleep(1)



    
if __name__ == '__main__':
    rospy.init_node('VisibilityServer', anonymous=False, log_level=rospy.DEBUG)
    try:
        grid = create_mock_grid(10, 10, 1, 100)
        visibility_server = VisibilityServer(grid=grid)
        visibility_server.run()
    except rospy.ROSInterruptException:
        pass
