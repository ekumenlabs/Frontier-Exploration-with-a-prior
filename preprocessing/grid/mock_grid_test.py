import unittest
from mock_grid import create_mock_grid
import numpy as np

# define testcase
class TestMockGrid(unittest.TestCase):
    def testMockedGrid(self):
        for i in range(100):
            grid = create_mock_grid(cell_size=0.1, num_x_cells=10, num_y_cells=10, occupancy_percentage=i)
            occupied_cells = np.sum(grid.get_grid())
            # we should have at least occupancy_percentage% of the grid filled
            self.assertGreaterEqual(occupied_cells, int(grid.get_x_cells() * grid.get_y_cells() * i / 100.0))
            

if __name__ == "__main__":
    unittest.main()