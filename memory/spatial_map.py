"""
Spatial Map

Simple occupancy grid for tracking where the robot has been
and what's in the environment.
"""
import logging
import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass
from enum import IntEnum

logger = logging.getLogger(__name__)


class CellState(IntEnum):
    """State of a map cell"""
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2
    VISITED = 3


@dataclass
class MapPoint:
    """A point of interest on the map"""
    x: float
    y: float
    label: str
    point_type: str  # "obstacle", "landmark", "charger", "interesting"


class SpatialMap:
    """
    Simple 2D occupancy grid map.

    Tracks:
    - Where the robot has been (visited cells)
    - Obstacles encountered
    - Points of interest

    Grid coordinates are in millimeters from origin.
    Default: 5000x5000mm (5m x 5m) with 50mm resolution = 100x100 grid
    """

    def __init__(
        self,
        size_mm: Tuple[float, float] = (5000, 5000),
        resolution_mm: float = 50.0,
        origin_mm: Tuple[float, float] = (2500, 2500)
    ):
        self.size_mm = size_mm
        self.resolution = resolution_mm
        self.origin = origin_mm  # Robot starts at center

        # Grid dimensions
        self.width = int(size_mm[0] / resolution_mm)
        self.height = int(size_mm[1] / resolution_mm)

        # Initialize grid as unknown
        self.grid = np.full(
            (self.height, self.width),
            CellState.UNKNOWN,
            dtype=np.uint8
        )

        # Points of interest
        self.points: List[MapPoint] = []

        # Visit count for heatmap
        self.visit_count = np.zeros((self.height, self.width), dtype=np.uint16)

        logger.info(
            f"Created {self.width}x{self.height} map "
            f"({size_mm[0]}x{size_mm[1]}mm at {resolution_mm}mm resolution)"
        )

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (mm) to grid indices"""
        gx = int((x + self.origin[0]) / self.resolution)
        gy = int((y + self.origin[1]) / self.resolution)

        # Clamp to grid bounds
        gx = max(0, min(gx, self.width - 1))
        gy = max(0, min(gy, self.height - 1))

        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (mm)"""
        x = gx * self.resolution - self.origin[0]
        y = gy * self.resolution - self.origin[1]
        return x, y

    def mark_visited(self, x: float, y: float):
        """Mark a cell as visited"""
        gx, gy = self.world_to_grid(x, y)
        self.grid[gy, gx] = CellState.VISITED
        self.visit_count[gy, gx] += 1

    def mark_free(self, x: float, y: float):
        """Mark a cell as free (traversable)"""
        gx, gy = self.world_to_grid(x, y)
        if self.grid[gy, gx] != CellState.VISITED:
            self.grid[gy, gx] = CellState.FREE

    def mark_occupied(self, x: float, y: float):
        """Mark a cell as occupied (obstacle)"""
        gx, gy = self.world_to_grid(x, y)
        self.grid[gy, gx] = CellState.OCCUPIED

    def get_cell(self, x: float, y: float) -> CellState:
        """Get the state of a cell at world coordinates"""
        gx, gy = self.world_to_grid(x, y)
        return CellState(self.grid[gy, gx])

    def is_visited(self, x: float, y: float) -> bool:
        """Check if a location has been visited"""
        return self.get_cell(x, y) == CellState.VISITED

    def is_free(self, x: float, y: float) -> bool:
        """Check if a location is known to be free"""
        state = self.get_cell(x, y)
        return state in (CellState.FREE, CellState.VISITED)

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if a location is occupied"""
        return self.get_cell(x, y) == CellState.OCCUPIED

    def is_unknown(self, x: float, y: float) -> bool:
        """Check if a location is unexplored"""
        return self.get_cell(x, y) == CellState.UNKNOWN

    def add_point(self, x: float, y: float, label: str, point_type: str):
        """Add a point of interest"""
        self.points.append(MapPoint(x, y, label, point_type))
        logger.debug(f"Added point: {label} ({point_type}) at ({x}, {y})")

    def get_points_near(
        self,
        x: float,
        y: float,
        radius: float = 500.0
    ) -> List[MapPoint]:
        """Get points of interest within radius"""
        nearby = []
        for point in self.points:
            dist = np.sqrt((point.x - x)**2 + (point.y - y)**2)
            if dist <= radius:
                nearby.append(point)
        return nearby

    def find_nearest_unknown(
        self,
        x: float,
        y: float,
        min_distance: float = 100.0
    ) -> Optional[Tuple[float, float]]:
        """
        Find the nearest unexplored cell.

        Args:
            x, y: Current position
            min_distance: Minimum distance to search (mm)

        Returns:
            (x, y) of nearest unknown cell, or None if all explored
        """
        gx, gy = self.world_to_grid(x, y)

        # Spiral outward search
        max_radius = max(self.width, self.height)

        for r in range(int(min_distance / self.resolution), max_radius):
            # Check cells at this radius
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    # Only check cells on the perimeter
                    if abs(dx) != r and abs(dy) != r:
                        continue

                    cx, cy = gx + dx, gy + dy

                    # Bounds check
                    if cx < 0 or cx >= self.width or cy < 0 or cy >= self.height:
                        continue

                    if self.grid[cy, cx] == CellState.UNKNOWN:
                        return self.grid_to_world(cx, cy)

        return None

    def get_exploration_progress(self) -> float:
        """Get percentage of map explored (0-1)"""
        known_cells = np.sum(self.grid != CellState.UNKNOWN)
        total_cells = self.width * self.height
        return known_cells / total_cells

    def get_visited_percentage(self) -> float:
        """Get percentage of map actually visited (0-1)"""
        visited_cells = np.sum(self.grid == CellState.VISITED)
        total_cells = self.width * self.height
        return visited_cells / total_cells

    def get_summary(self) -> dict:
        """Get a summary of the map state"""
        unique, counts = np.unique(self.grid, return_counts=True)
        state_counts = dict(zip([CellState(u).name for u in unique], counts.tolist()))

        return {
            "size": f"{self.size_mm[0]}x{self.size_mm[1]}mm",
            "resolution": f"{self.resolution}mm",
            "cells": f"{self.width}x{self.height}",
            "exploration_progress": f"{self.get_exploration_progress()*100:.1f}%",
            "visited_percentage": f"{self.get_visited_percentage()*100:.1f}%",
            "cell_states": state_counts,
            "points_of_interest": len(self.points)
        }

    def to_ascii(self, robot_x: float = None, robot_y: float = None) -> str:
        """
        Generate ASCII representation of the map.

        Great for debugging and LLM context.
        """
        # Downsample for display
        display_width = min(50, self.width)
        display_height = min(30, self.height)

        x_scale = self.width / display_width
        y_scale = self.height / display_height

        lines = []
        for dy in range(display_height):
            line = ""
            for dx in range(display_width):
                gx = int(dx * x_scale)
                gy = int(dy * y_scale)

                # Check if robot is here
                if robot_x is not None and robot_y is not None:
                    rx, ry = self.world_to_grid(robot_x, robot_y)
                    if abs(gx - rx) < x_scale and abs(gy - ry) < y_scale:
                        line += "R"
                        continue

                cell = self.grid[gy, gx]
                if cell == CellState.UNKNOWN:
                    line += "."
                elif cell == CellState.FREE:
                    line += " "
                elif cell == CellState.OCCUPIED:
                    line += "#"
                elif cell == CellState.VISITED:
                    line += "o"

            lines.append(line)

        return "\n".join(lines)

    def save(self, filepath: str):
        """Save map to file"""
        np.savez(
            filepath,
            grid=self.grid,
            visit_count=self.visit_count,
            size_mm=self.size_mm,
            resolution=self.resolution,
            origin=self.origin
        )
        logger.info(f"Map saved to {filepath}")

    @classmethod
    def load(cls, filepath: str) -> 'SpatialMap':
        """Load map from file"""
        data = np.load(filepath)

        map_obj = cls(
            size_mm=tuple(data['size_mm']),
            resolution_mm=float(data['resolution']),
            origin_mm=tuple(data['origin'])
        )
        map_obj.grid = data['grid']
        map_obj.visit_count = data['visit_count']

        logger.info(f"Map loaded from {filepath}")
        return map_obj
