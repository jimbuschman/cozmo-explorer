"""
World definitions: walls, obstacles, and preset rooms.

All coordinates in mm. Walls are line segments.
"""
from dataclasses import dataclass, field


@dataclass
class Wall:
    """A wall segment from (x1,y1) to (x2,y2)."""
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class World:
    """Collection of walls defining the environment."""
    walls: list = field(default_factory=list)
    name: str = "empty"
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_theta: float = 0.0  # radians


def box_room(width=1000, height=1000):
    """Open rectangular room."""
    w, h = width / 2, height / 2
    return World(
        walls=[
            Wall(-w, -h, w, -h),   # bottom
            Wall(w, -h, w, h),     # right
            Wall(w, h, -w, h),     # top
            Wall(-w, h, -w, -h),   # left
        ],
        name=f"box_{width}x{height}",
        spawn_x=0, spawn_y=0, spawn_theta=0,
    )


def corridor(width=300, length=2000):
    """Narrow corridor - tests zigzag escape from a wall at the end."""
    hw = width / 2
    hl = length / 2
    return World(
        walls=[
            Wall(-hl, -hw, hl, -hw),   # bottom wall
            Wall(hl, -hw, hl, hw),     # right end wall
            Wall(hl, hw, -hl, hw),     # top wall
            Wall(-hl, hw, -hl, -hw),   # left end wall
        ],
        name=f"corridor_{width}x{length}",
        spawn_x=-hl + 200, spawn_y=0, spawn_theta=0,
    )


def corner():
    """L-shaped corner - common failure case."""
    return World(
        walls=[
            # Main room (left side)
            Wall(-500, -500, 200, -500),   # bottom
            Wall(-500, 500, 200, 500),     # top
            Wall(-500, 500, -500, -500),   # left wall

            # Corner protrusion
            Wall(200, -500, 200, -100),    # right-bottom vertical
            Wall(200, -100, 800, -100),    # horizontal shelf
            Wall(800, -100, 800, 500),     # far right wall
            Wall(200, 500, 800, 500),      # top-right
        ],
        name="corner",
        spawn_x=-200, spawn_y=0, spawn_theta=0,
    )


def dead_end(width=400, depth=500):
    """Dead end pocket - requires reversing out entirely."""
    hw = width / 2
    return World(
        walls=[
            # Outer room
            Wall(-600, -400, 600, -400),
            Wall(600, -400, 600, 400),
            Wall(600, 400, -600, 400),
            Wall(-600, 400, -600, -400),

            # Dead end pocket at right side
            Wall(200, -hw, 200 + depth, -hw),    # bottom of pocket
            Wall(200 + depth, -hw, 200 + depth, hw),  # end wall
            Wall(200 + depth, hw, 200, hw),      # top of pocket
        ],
        name=f"dead_end_{width}x{depth}",
        spawn_x=350, spawn_y=0, spawn_theta=0,  # Start inside the pocket
    )


# All presets indexed by number key
PRESETS = {
    1: box_room,
    2: corridor,
    3: corner,
    4: dead_end,
}
