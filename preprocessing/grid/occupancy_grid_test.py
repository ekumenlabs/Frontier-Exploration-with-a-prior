import unittest

import numpy as np
from grid.occupancy_grid import OccupancyGrid


class TestOccupancyGrid(unittest.TestCase):
    def testCreation(self):
        grid = OccupancyGrid(cell_size=0.1, x_cells=10, y_cells=10)
        self.assertEqual(grid.get_cell_size(), 0.1)
        self.assertEqual(grid.get_x_cells(), 10)
        self.assertEqual(grid.get_y_cells(), 10)
        self.assertTrue(
            np.array_equal(grid.get_grid(), np.zeros((10, 10), dtype=np.int8))
        )
    def testGetEmptyCells(self):
        grid = OccupancyGrid(cell_size=0.1, x_cells=10, y_cells=10)
        grid[(0, 0)] = 100
        grid[(4, 5)] = 100
        occupied_cells = grid.get_occupied_cells()
        empty_cells = grid.get_empty_cells()
        self.assertEqual(len(empty_cells), 100 - 2)
        self.assertEqual(len(occupied_cells), 2)
        self.assertFalse([0,0] in empty_cells.tolist())
        self.assertFalse([4, 5] in empty_cells.tolist())
        self.assertTrue([0, 0] in occupied_cells.tolist())
        self.assertTrue([4, 5] in occupied_cells.tolist())




if __name__ == "__main__":
    unittest.main()
