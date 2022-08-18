#!/usr/bin/python3.8
from typing import Tuple
import FrontierExploration.preprocessing.grid.occupancy_grid as grid
from FrontierExploration.preprocessing.grid.mock_grid import create_mock_grid
from visibility_server.srv import Visibility, VisibilityRequest, VisibilityResponse

import rospy
from dataclasses import dataclass, field



@dataclass
class InitialState:
    initial_position: Tuple[int, int]
    # TODO (implement) support starting misaligned with the grid
    initial_rotation: Tuple [float, float, float, float] = field(default_factory=lambda: (1, 0, 0, 0))


class VisibilityServer(object):
    """Consumvisibility_serveres time elapsed and robot odometry to estimate battery consumption
    """
    def __init__(self, grid: grid.OccupancyGrid, initial_state: InitialState):
        self._grid = grid
        self._inital_state = initial_state
        self._service_server =  rospy.Service('visibility', Visibility, self._visibility_cb)
    
    
    def _visibility_cb(self, req: VisibilityRequest) -> VisibilityResponse:
        """
        returns visibility of the cell corresponding to the given position in the map frame
        """
        coordinate_map_frame = (int(req.x_map_frame / self._grid.get_cell_size()), int(req.y_map_frame / self._grid.get_cell_size()))
        # substract initial offset 
        # (go from map frame to grid frame)
        coordinate_grid_frame = (coordinate_map_frame[0] - self._inital_state.initial_position[0], coordinate_map_frame[1] - self._inital_state.initial_position[1])
        response = VisibilityResponse()
        try:
            response.visibility =  grid[coordinate_grid_frame]
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
    # TODO expose parameters via parameter server
    try:
        initial_state = InitialState(initial_position=(0, 0))
        grid = create_mock_grid(10, 10, 1, 100)
        visibility_server = VisibilityServer(grid=grid, initial_state=initial_state)
        visibility_server.run()
    except rospy.ROSInterruptException:
        pass
