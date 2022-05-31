import unittest
import numpy as np

from FrontierExploration.preprocessing.grid.occupancy_grid import OccupancyGrid, is_cell_empty, is_cell_occupied


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
        self.assertTrue(is_cell_empty(empty_cells, (1, 0)))
        self.assertFalse(is_cell_empty(empty_cells, (0, 0)))
        self.assertFalse(is_cell_empty(empty_cells, (4, 5)))
        self.assertFalse(is_cell_occupied(occupied_cells, (1, 0)))
        self.assertTrue(is_cell_occupied(occupied_cells, (0, 0)))




if __name__ == "__main__":
    unittest.main()
