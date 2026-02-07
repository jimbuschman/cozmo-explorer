"""
Spatial Map

Simple occupancy grid for tracking where the robot has been
and what's in the environment.
"""
import logging
import math
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

    def update_from_sensor(
        self,
        robot_x: float,
        robot_y: float,
        robot_heading: float,
        sensor_angle_offset: float,
        distance_mm: float,
        max_range: float
    ):
        """
        Ray-trace a sensor reading onto the grid.

        Marks cells along the ray as FREE and the endpoint as OCCUPIED
        (if the reading is less than max_range).

        Args:
            robot_x, robot_y: Robot position in mm
            robot_heading: Robot heading in radians
            sensor_angle_offset: Sensor mounting angle offset in radians
            distance_mm: Measured distance in mm
            max_range: Maximum sensor range in mm
        """
        absolute_angle = robot_heading + sensor_angle_offset

        if distance_mm < max_range:
            # Obstacle detected - mark endpoint as OCCUPIED, ray as FREE
            obs_x = robot_x + distance_mm * math.cos(absolute_angle)
            obs_y = robot_y + distance_mm * math.sin(absolute_angle)
            self.mark_occupied(obs_x, obs_y)
            self.ray_mark_free(robot_x, robot_y, obs_x, obs_y)
        else:
            # No obstacle - mark entire ray as FREE
            end_x = robot_x + max_range * math.cos(absolute_angle)
            end_y = robot_y + max_range * math.sin(absolute_angle)
            self.ray_mark_free(robot_x, robot_y, end_x, end_y)

    def ray_mark_free(
        self,
        x1: float, y1: float,
        x2: float, y2: float
    ):
        """
        Mark grid cells along a ray as FREE using Bresenham's line algorithm.

        Cells already marked as VISITED are not overwritten.
        The endpoint cell is NOT marked (it may be OCCUPIED).
        """
        gx1, gy1 = self.world_to_grid(x1, y1)
        gx2, gy2 = self.world_to_grid(x2, y2)

        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx - dy

        cx, cy = gx1, gy1
        while True:
            # Don't mark the endpoint (it might be an obstacle)
            if cx == gx2 and cy == gy2:
                break
            # Mark as FREE if not already VISITED
            if 0 <= cx < self.width and 0 <= cy < self.height:
                if self.grid[cy, cx] != CellState.VISITED:
                    self.grid[cy, cx] = CellState.FREE
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    def find_nearest_frontier(
        self,
        x: float,
        y: float,
        min_distance: float = 100.0
    ) -> Optional[Tuple[float, float]]:
        """
        Find the nearest frontier cell (UNKNOWN cell adjacent to FREE/VISITED).

        Unlike find_nearest_unknown(), this only returns cells that are
        actually reachable - on the boundary between known and unknown space.

        Args:
            x, y: Current position in mm
            min_distance: Minimum search distance in mm

        Returns:
            (x, y) of nearest frontier cell, or None if none found
        """
        gx, gy = self.world_to_grid(x, y)
        max_radius = max(self.width, self.height)
        min_r = int(min_distance / self.resolution)

        for r in range(min_r, max_radius):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue

                    cx, cy = gx + dx, gy + dy
                    if cx < 0 or cx >= self.width or cy < 0 or cy >= self.height:
                        continue

                    if self.grid[cy, cx] != CellState.UNKNOWN:
                        continue

                    # Check if adjacent to FREE or VISITED
                    for nx, ny in [(cx-1, cy), (cx+1, cy), (cx, cy-1), (cx, cy+1)]:
                        if 0 <= nx < self.width and 0 <= ny < self.height:
                            if self.grid[ny, nx] in (CellState.FREE, CellState.VISITED):
                                return self.grid_to_world(cx, cy)

        return None

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
