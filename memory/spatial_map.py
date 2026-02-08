"""
Spatial Map

Simple occupancy grid for tracking where the robot has been
and what's in the environment.
"""
import logging
import math
import random
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
        min_distance: float = 100.0,
        exclude_angle: float = None,
        exclude_cone: float = math.radians(45)
    ) -> Optional[Tuple[float, float]]:
        """
        Find the nearest frontier cell (UNKNOWN cell adjacent to FREE/VISITED).

        Unlike find_nearest_unknown(), this only returns cells that are
        actually reachable - on the boundary between known and unknown space.

        Args:
            x, y: Current position in mm
            min_distance: Minimum search distance in mm
            exclude_angle: If set, skip frontiers within exclude_cone of this
                           angle (radians). Used to avoid turning back toward
                           a wall after escaping.
            exclude_cone: Half-width of exclusion cone in radians (default 45°)

        Returns:
            (x, y) of nearest frontier cell, or None if none found
        """
        gx, gy = self.world_to_grid(x, y)
        max_radius = max(self.width, self.height)
        min_r = int(min_distance / self.resolution)

        for r in range(min_r, max_radius):
            # Collect all frontiers at this radius, then pick randomly
            candidates = []
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
                    is_frontier = False
                    for nx, ny in [(cx-1, cy), (cx+1, cy), (cx, cy-1), (cx, cy+1)]:
                        if 0 <= nx < self.width and 0 <= ny < self.height:
                            if self.grid[ny, nx] in (CellState.FREE, CellState.VISITED):
                                is_frontier = True
                                break

                    if not is_frontier:
                        continue

                    # Filter by exclude_angle if set
                    if exclude_angle is not None:
                        wx, wy = self.grid_to_world(cx, cy)
                        angle_to = math.atan2(wy - y, wx - x)
                        diff = (angle_to - exclude_angle + math.pi) % (2 * math.pi) - math.pi
                        if abs(diff) < exclude_cone:
                            continue

                    candidates.append((cx, cy))

            if candidates:
                pick = random.choice(candidates)
                return self.grid_to_world(pick[0], pick[1])

        return None

    def find_best_frontier_cluster(
        self,
        x: float,
        y: float,
        min_cluster_size: int = 3,
        exclude_angle: float = None,
        exclude_cone: float = math.radians(45),
    ) -> Optional[Tuple[float, float]]:
        """
        Find the best frontier cluster to explore using information-gain heuristic.

        Instead of picking the nearest individual frontier cell, this:
        1. Finds all frontier cells (UNKNOWN adjacent to FREE/VISITED)
        2. Clusters them by connectivity (flood-fill)
        3. Scores each cluster: size / sqrt(distance) — big nearby clusters win
        4. Returns the nearest reachable point of the highest-scoring cluster

        This forces the robot to physically navigate to substantial unexplored
        regions instead of scanning tiny frontier fragments from a distance.

        Args:
            x, y: Current robot position in mm
            min_cluster_size: Ignore clusters smaller than this (noise)
            exclude_angle: If set, skip clusters whose nearest point falls
                           within exclude_cone of this angle (radians)
            exclude_cone: Half-width of exclusion cone in radians

        Returns:
            (x, y) of the nearest reachable point of the best cluster, or None
        """
        # Step 1: Find all frontier cells
        frontier_set = set()
        for gy in range(self.height):
            for gx in range(self.width):
                if self.grid[gy, gx] != CellState.UNKNOWN:
                    continue
                for nx, ny in [(gx - 1, gy), (gx + 1, gy), (gx, gy - 1), (gx, gy + 1)]:
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.grid[ny, nx] in (CellState.FREE, CellState.VISITED):
                            frontier_set.add((gx, gy))
                            break

        if not frontier_set:
            return None

        # Step 2: Cluster by connectivity (DFS flood-fill)
        clusters = []
        visited = set()
        for cell in frontier_set:
            if cell in visited:
                continue
            stack = [cell]
            visited.add(cell)
            cluster = []
            while stack:
                cx, cy = stack.pop()
                cluster.append((cx, cy))
                for nx, ny in [(cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)]:
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        stack.append((nx, ny))
            clusters.append(cluster)

        # Step 3: Score each cluster — size / sqrt(distance + 1)
        robot_gx, robot_gy = self.world_to_grid(x, y)
        scored = []

        for cluster in clusters:
            if len(cluster) < min_cluster_size:
                continue

            # Find nearest cell in cluster to robot
            min_dist_sq = float('inf')
            nearest_cell = cluster[0]
            for cx, cy in cluster:
                d_sq = (cx - robot_gx) ** 2 + (cy - robot_gy) ** 2
                if d_sq < min_dist_sq:
                    min_dist_sq = d_sq
                    nearest_cell = (cx, cy)

            min_dist = math.sqrt(min_dist_sq)

            # Exclude clusters in the escape exclusion cone
            if exclude_angle is not None:
                wx, wy = self.grid_to_world(nearest_cell[0], nearest_cell[1])
                angle_to = math.atan2(wy - y, wx - x)
                diff = (angle_to - exclude_angle + math.pi) % (2 * math.pi) - math.pi
                if abs(diff) < exclude_cone:
                    continue

            score = len(cluster) / math.sqrt(min_dist + 1)
            scored.append((score, nearest_cell, len(cluster)))

        if scored:
            scored.sort(key=lambda s: s[0], reverse=True)
            best_score, best_cell, best_size = scored[0]
            logger.debug(
                f"Frontier cluster: size={best_size} dist={math.sqrt((best_cell[0]-robot_gx)**2 + (best_cell[1]-robot_gy)**2):.0f} "
                f"score={best_score:.1f} (of {len(scored)} clusters)"
            )
            return self.grid_to_world(best_cell[0], best_cell[1])

        # All clusters filtered or too small — fall back to largest cluster
        if clusters:
            largest = max(clusters, key=len)
            min_dist_sq = float('inf')
            nearest = largest[0]
            for cx, cy in largest:
                d_sq = (cx - robot_gx) ** 2 + (cy - robot_gy) ** 2
                if d_sq < min_dist_sq:
                    min_dist_sq = d_sq
                    nearest = (cx, cy)
            return self.grid_to_world(nearest[0], nearest[1])

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
