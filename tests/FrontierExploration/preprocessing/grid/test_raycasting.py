from datetime import datetime
import unittest
from matplotlib import pyplot as plt
import numpy as np

from FrontierExploration.preprocessing.grid.raycasting import raycast_in_every_direction
from FrontierExploration.preprocessing.grid.mock_grid import create_mock_grid


class TestRaycasting(unittest.TestCase):
    def testBasic(self):
        # should be completely empty but the borders
        grid = create_mock_grid(num_x_cells=100, num_y_cells=100, cell_size=0.1, occupancy_percentage=0)
        grid[(51, 51)] = 100
        start = datetime.now()
        seen_cells = np.asarray([[45, 52]])
        print(seen_cells.shape)
        # seen_cells=None
        raycast = raycast_in_every_direction(start_cell=(50, 50), available_cells=grid.get_empty_cells(), range_in_cells=9, squarify=False, outermost=False, seen_cells=seen_cells)
        print(f"took {datetime.now() - start}")
        # self.assertEqual(len(raycast), 36)
        for cell in raycast:
            grid[tuple(cell)] = 100

        # for debugging
        grid.plot()
        plt.show()




if __name__ == "__main__":
    unittest.main()