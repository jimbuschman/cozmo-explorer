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


def furnished_room():
    """
    Large open room (2500 x 2500 mm) with furniture-like obstacles.

    Simulates a real room: open space to drive, but things to bump into.

        +------------------------------------+
        |                                    |
        |    [couch]              [shelf]    |
        |                                    |
        |                                    |
        |          [table]                   |
        |          [     ]                   |
        |                                    |
        |                         [chair]    |
        |    [box]                           |
        |                                    |
        +------------------------------------+
    """
    w, h = 2500, 2500
    hw, hh = w / 2, h / 2
    walls = [
        # Outer walls
        Wall(-hw, -hh, hw, -hh),   # bottom
        Wall(hw, -hh, hw, hh),     # right
        Wall(hw, hh, -hw, hh),     # top
        Wall(-hw, hh, -hw, -hh),   # left

        # Couch (top-left, long rectangle)
        Wall(-900, 800, -400, 800),
        Wall(-400, 800, -400, 900),
        Wall(-400, 900, -900, 900),
        Wall(-900, 900, -900, 800),

        # Shelf (top-right, narrow rectangle against wall)
        Wall(600, 850, 1000, 850),
        Wall(1000, 850, 1000, 1050),
        Wall(1000, 1050, 600, 1050),
        Wall(600, 1050, 600, 850),

        # Table (center, larger square)
        Wall(-200, 100, 300, 100),
        Wall(300, 100, 300, 400),
        Wall(300, 400, -200, 400),
        Wall(-200, 400, -200, 100),

        # Chair (lower-right)
        Wall(600, -400, 800, -400),
        Wall(800, -400, 800, -200),
        Wall(800, -200, 600, -200),
        Wall(600, -200, 600, -400),

        # Box (lower-left, small)
        Wall(-900, -500, -700, -500),
        Wall(-700, -500, -700, -350),
        Wall(-700, -350, -900, -350),
        Wall(-900, -350, -900, -500),

        # Angled bookcase (mid-right, diagonal obstacle)
        Wall(700, 200, 900, 400),
        Wall(900, 400, 850, 450),
        Wall(850, 450, 650, 250),
        Wall(650, 250, 700, 200),
    ]

    return World(
        walls=walls,
        name="furnished_room_2.5x2.5",
        spawn_x=0, spawn_y=-200, spawn_theta=0,  # Start in center-ish
    )


def multi_room():
    """
    Multi-room environment (~3m x 3m) for full-stack simulation.

    Layout (top-down, Y increases upward):

        +-------+   +-------+-------+
        | Room C|   | Room B        |
        |       D3  D2      |       |
        +---+---+   +---+---+--+----+
            |  Hallway          |
        +---+---+   +-+--------+----+
        |       D1  D4|  Dead end   |
        | Room A|   | |  pocket     |
        |       |   | +------+------+
        +-------+   +-------+

    3 rooms + hallway + dead-end pocket + furniture obstacles.
    """
    walls = []

    # === Outer boundary (3000 x 3000 mm) ===
    # Bottom wall
    walls.append(Wall(0, 0, 3000, 0))
    # Right wall
    walls.append(Wall(3000, 0, 3000, 3000))
    # Top wall
    walls.append(Wall(3000, 3000, 0, 3000))
    # Left wall
    walls.append(Wall(0, 3000, 0, 0))

    # === Horizontal divider (separates top rooms from hallway) at y=1800 ===
    # Left section (Room A top / hallway bottom)
    walls.append(Wall(0, 1800, 500, 1800))
    # Gap = doorway D1 (500-700)
    walls.append(Wall(700, 1800, 1300, 1800))
    # Gap = doorway D4 to dead-end corridor (1300-1500)
    walls.append(Wall(1500, 1800, 3000, 1800))

    # === Horizontal divider (separates hallway from top rooms) at y=2200 ===
    walls.append(Wall(0, 2200, 500, 2200))
    # Gap = doorway D3 (500-700) into Room C
    walls.append(Wall(700, 2200, 1300, 2200))
    # Gap = doorway D2 (1300-1500) into Room B
    walls.append(Wall(1500, 2200, 3000, 2200))

    # === Room A (bottom-left, 0-1200 x 0-1800) ===
    # Right wall of Room A
    walls.append(Wall(1200, 0, 1200, 1800))

    # === Dead-end pocket (bottom-right area, 1200-3000 x 0-1800) ===
    # Inner pocket walls creating a U-shaped dead end
    # Left wall of pocket interior
    walls.append(Wall(1500, 0, 1500, 1200))
    # Top wall of pocket interior
    walls.append(Wall(1500, 1200, 2600, 1200))
    # This creates a pocket: enter from doorway D4 (y=1800, x=1300-1500),
    # go down into the space between x=1200-1500, then into the pocket

    # === Room C (top-left, 0-1200 x 2200-3000) ===
    # Right wall of Room C
    walls.append(Wall(1200, 2200, 1200, 3000))

    # === Room B (top-right, 1200-3000 x 2200-3000) ===
    # Vertical divider inside Room B (furniture-like obstacle)
    walls.append(Wall(2200, 2400, 2200, 2800))

    # === Furniture obstacles ===
    # Table in Room A (small box)
    walls.append(Wall(400, 600, 700, 600))
    walls.append(Wall(700, 600, 700, 900))
    walls.append(Wall(700, 900, 400, 900))
    walls.append(Wall(400, 900, 400, 600))

    # Pillar in hallway
    walls.append(Wall(1800, 1900, 1900, 1900))
    walls.append(Wall(1900, 1900, 1900, 2100))
    walls.append(Wall(1900, 2100, 1800, 2100))
    walls.append(Wall(1800, 2100, 1800, 1900))

    # L-shaped shelf in Room C
    walls.append(Wall(200, 2500, 200, 2800))
    walls.append(Wall(200, 2800, 600, 2800))

    return World(
        walls=walls,
        name="multi_room_3x3",
        spawn_x=600, spawn_y=1000, spawn_theta=0,  # Start in Room A
    )


# All presets indexed by number key
PRESETS = {
    1: box_room,
    2: corridor,
    3: corner,
    4: dead_end,
    5: multi_room,
    6: furnished_room,
}
