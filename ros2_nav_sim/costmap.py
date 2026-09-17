"""A 2D occupancy-grid costmap, built from a small bundled ASCII map.

'.' = free cell, '#' = occupied cell. Cell (row, col) == (y, x) in grid
index space; converting to "world" coordinates just scales by
`resolution` (meters per cell).
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

from .messages import OccupancyGrid

FREE = 0
OCCUPIED = 100

# A small maze-like room: a corridor wall with a single gap near the top,
# and a second partial wall with a gap near the bottom, forcing the planner
# to weave rather than walking a straight diagonal line. Every row is the
# same width (10 columns), 10 rows tall.
BUNDLED_MAP_LINES: List[str] = [
    "..........",
    "..........",
    "..........",
    "####.#####",
    "..........",
    "..........",
    "..........",
    ".....####.",
    "..........",
    "..........",
]

DEFAULT_START = (1, 1)  # (row, col)
DEFAULT_GOAL = (8, 8)  # (row, col)


class Costmap:
    """A 2D occupancy grid. grid[row][col] is 0 (free) or 1 (occupied)."""

    def __init__(self, grid: List[List[int]], resolution: float = 1.0):
        if not grid or not grid[0]:
            raise ValueError("grid must be non-empty")
        width = len(grid[0])
        if any(len(row) != width for row in grid):
            raise ValueError("all grid rows must have the same width")
        self.grid = grid
        self.height = len(grid)
        self.width = width
        self.resolution = resolution

    @classmethod
    def from_ascii(cls, lines: Iterable[str], resolution: float = 1.0) -> "Costmap":
        grid = [[1 if ch == "#" else 0 for ch in line] for line in lines]
        return cls(grid, resolution=resolution)

    @classmethod
    def bundled_map(cls, resolution: float = 1.0) -> "Costmap":
        return cls.from_ascii(BUNDLED_MAP_LINES, resolution=resolution)

    def in_bounds(self, cell: Tuple[int, int]) -> bool:
        row, col = cell
        return 0 <= row < self.height and 0 <= col < self.width

    def is_occupied(self, cell: Tuple[int, int]) -> bool:
        row, col = cell
        return self.grid[row][col] != 0

    def is_free(self, cell: Tuple[int, int]) -> bool:
        return self.in_bounds(cell) and not self.is_occupied(cell)

    def neighbors4(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """4-connected free neighbors (up/down/left/right) of a cell."""
        row, col = cell
        candidates = [(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)]
        return [c for c in candidates if self.is_free(c)]

    def grid_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Cell (row, col) -> world (x, y) meters, using the cell center."""
        row, col = cell
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        return x, y

    def to_msg(self) -> OccupancyGrid:
        flat = [self.grid[r][c] * OCCUPIED for r in range(self.height) for c in range(self.width)]
        return OccupancyGrid(width=self.width, height=self.height, resolution=self.resolution, data=flat)
