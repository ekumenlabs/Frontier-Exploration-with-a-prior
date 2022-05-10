from matplotlib import pyplot as plt
from grid.raycasting import raycast_in_every_direction
from grid.mock_grid import create_mock_grid
import unittest


class TestRaycasting(unittest.TestCase):
    def testBasic(self):
        # should be completely empty but the borders
        grid = create_mock_grid(num_x_cells=10, num_y_cells=10, cell_size=0.1, occupancy_percentage=0)
        raycast = raycast_in_every_direction(start_cell=(5, 5), available_cells=grid.get_empty_cells(), range_in_cells=3)
        self.assertEqual(len(raycast), 36)
        for cell in raycast:
            grid[tuple(cell)] = 100

        # for debugging
        # grid.plot()
        # plt.show()




if __name__ == "__main__":
    unittest.main()