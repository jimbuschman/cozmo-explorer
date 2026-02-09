"""
Map Renderer — converts occupancy grid to PNG image.

Uses numpy LUT for speed (<1ms render), Pillow for PNG encoding.
Color scheme:
  UNKNOWN (0): dark gray (30, 30, 30)
  FREE (1):    green (40, 120, 40)
  OCCUPIED (2): red (200, 50, 50)
  VISITED (3): blue (50, 50, 180)
  Robot:       yellow dot
"""
import io
import numpy as np
from PIL import Image
from typing import Optional, Tuple

# Color lookup table indexed by CellState value (0-3)
# Shape: (4, 3) — one RGB row per cell state
_COLOR_LUT = np.array([
    [30, 30, 30],     # UNKNOWN = 0
    [40, 120, 40],    # FREE = 1
    [200, 50, 50],    # OCCUPIED = 2
    [50, 50, 180],    # VISITED = 3
], dtype=np.uint8)

ROBOT_COLOR = (255, 220, 40)  # Yellow
SCALE = 4  # 100x100 grid -> 400x400 image


def render_map_png(
    grid: np.ndarray,
    robot_gx: Optional[int] = None,
    robot_gy: Optional[int] = None,
    scale: int = SCALE,
) -> bytes:
    """
    Render occupancy grid as a PNG image.

    Args:
        grid: numpy uint8 array (height x width), values 0-3
        robot_gx, robot_gy: robot position in grid coords (optional)
        scale: pixel scale factor (default 4x)

    Returns:
        PNG image as bytes
    """
    # LUT index: grid values -> RGB
    rgb = _COLOR_LUT[grid]  # (H, W, 3)

    # Draw robot position
    if robot_gx is not None and robot_gy is not None:
        h, w = grid.shape
        if 0 <= robot_gy < h and 0 <= robot_gx < w:
            # Mark robot cell and neighbors for visibility
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    ry, rx = robot_gy + dy, robot_gx + dx
                    if 0 <= ry < h and 0 <= rx < w:
                        rgb[ry, rx] = ROBOT_COLOR

    # Scale up
    img = Image.fromarray(rgb, mode='RGB')
    if scale != 1:
        new_size = (grid.shape[1] * scale, grid.shape[0] * scale)
        img = img.resize(new_size, Image.NEAREST)

    # Encode to PNG
    buf = io.BytesIO()
    img.save(buf, format='PNG', optimize=True)
    return buf.getvalue()
