from __future__ import annotations
import functools
from heapq import heappush, heappop
from math import hypot
import pdb
from typing import Optional, List

from matplotlib import pyplot as plt
import numpy as np
from grid.occupancy_grid import OccupancyGrid, is_cell_empty, is_cell_occupied
from grid.raycasting import raycast_in_every_direction
from grid.mock_grid import create_mock_grid
from queue import PriorityQueue
from dataclasses import dataclass


@dataclass
class Node:
    position: tuple
    cost: int = 0
    heuristic: int = 0
    f: int = 0
    parent: Optional[Node] = None

    def __lt__(self, other):
        return self.f < other.f

# euclidean distance


def cost(a: tuple, b: tuple) -> int:
    return hypot(a[0] - b[0], a[1] - b[1])


class AStar(object):
    def __init__(self, grid: OccupancyGrid):
        self._grid = grid
        # same with stat and goal cell
        self._available_cells = grid.get_empty_cells()
        self._range_in_cells = 1
        self._open_set = PriorityQueue()
        self._closed_set = []

    def _is_cell_in_open_set(self, cell: tuple) -> bool:
        return cell in self._open_set.queue

    def reset(self) -> None:
        self._closed_set = []
        self._open_set = PriorityQueue()

    def solve(self, start_cell: tuple, end_cell: tuple) -> Optional[List[tuple]]:
        assert is_cell_empty(
            empty_cells=self._available_cells, cell=start_cell)
        assert is_cell_empty(
            empty_cells=self._available_cells, cell=end_cell)
        open_node: Node = Node(position=start_cell)
        self._open_set.put(open_node)
        while not self._open_set.empty():
            open_node: Node = self._open_set.get()
            if(open_node.position == end_cell):
                return self._reconstruct_path(open_node)
            self._closed_set.append(open_node.position)
            possible_children = [Node(position=tuple(pos), cost=open_node.cost + cost(pos, open_node.position), heuristic=cost(
                pos, end_cell), f=(cost(pos, open_node.position) + open_node.cost + 3 * cost(pos, end_cell)) + 1, parent=open_node) for pos in self._raycast_in_every_direction(cell=open_node.position)]
            for child in possible_children:
                if child.position not in self._closed_set:
                    self._open_set.put(child)

    @functools.lru_cache
    def _raycast_in_every_direction(self, cell: tuple) -> List[tuple]:
        return raycast_in_every_direction(available_cells=self._available_cells, start_cell=cell, range_in_cells=self._range_in_cells, squarify=True, outermost=True)

    @functools.lru_cache
    def _raycast(self, start_cell: tuple) -> list:
        raycast_in_every_direction(
            start_cell=start_cell, available_cells=self._available_cells, range_in_cells=self._range_in_cells)

    def _reconstruct_path(self, node: Node) -> Optional[List[tuple]]:
        path = []
        path.append(node.position)
        while node.parent is not None:
            node = node.parent
            path.append(node.position)
        path.reverse()
        return path


if __name__ == "__main__":
    grid = create_mock_grid(num_x_cells=100, num_y_cells=100,
                            cell_size=0.1, occupancy_percentage=5)
    star = AStar(grid)
    a = star.solve(start_cell=(5, 5), end_cell=(98, 89))
    fig = grid.plot()
    ax = fig.gca()
    for cell in a:
        ax.plot(cell[0] + 0.5, cell[1] + 0.5, 'rx')
    plt.show()
    print(a)
