"""
Pygame top-down renderer for the 2D simulator.

Draws the robot, trailer, walls, sensor rays, and trail.
"""
import math

try:
    import pygame
except ImportError:
    pygame = None
    print("WARNING: pygame not installed. Run: pip install pygame")

from simulator.physics import ROBOT_WIDTH, ROBOT_LENGTH, TRAILER_WIDTH, TRAILER_LENGTH

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
DARK_GRAY = (40, 40, 40)
GRAY = (100, 100, 100)
RED = (255, 60, 60)
YELLOW = (255, 220, 50)
GREEN = (60, 220, 60)
BLUE = (60, 120, 255)
ORANGE = (255, 160, 40)
LIGHT_BLUE = (140, 200, 255)
TRAIL_COLOR = (80, 80, 180)
WALL_COLOR = (200, 200, 200)
MAP_FREE_COLOR = (40, 100, 40)
MAP_OCCUPIED_COLOR = (200, 50, 50)
MAP_VISITED_COLOR = (50, 50, 150)

# Danger/caution thresholds (mm) - should match WanderBehavior
DANGER_DISTANCE = 80
CAUTION_DISTANCE = 250


class Renderer:
    """Pygame-based top-down view of the simulated world."""

    def __init__(self, width=900, height=700, spatial_map=None):
        if pygame is None:
            raise RuntimeError("pygame not installed")

        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Cozmo Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 14)
        self.font_large = pygame.font.SysFont("consolas", 18, bold=True)
        self.width = width
        self.height = height

        # View transform: world coords -> screen coords
        self.scale = 0.4  # pixels per mm
        self.offset_x = width / 2
        self.offset_y = height / 2

        self.spatial_map = spatial_map

        self.paused = False
        self.speed_multiplier = 1.0
        self.status_text = ""

    def world_to_screen(self, wx, wy):
        """Convert world coordinates (mm) to screen pixels."""
        sx = int(wx * self.scale + self.offset_x)
        sy = int(-wy * self.scale + self.offset_y)  # Flip Y
        return sx, sy

    def _draw_occupancy_grid(self):
        """Draw the spatial map occupancy grid as a background overlay."""
        if self.spatial_map is None:
            return

        from memory.spatial_map import CellState

        grid = self.spatial_map.grid
        res = self.spatial_map.resolution
        pixel_size = max(1, int(res * self.scale))

        for gy in range(self.spatial_map.height):
            for gx in range(self.spatial_map.width):
                cell = grid[gy, gx]
                if cell == CellState.UNKNOWN:
                    continue

                wx, wy = self.spatial_map.grid_to_world(gx, gy)
                sx, sy = self.world_to_screen(wx, wy)

                if cell == CellState.FREE:
                    color = MAP_FREE_COLOR
                elif cell == CellState.OCCUPIED:
                    color = MAP_OCCUPIED_COLOR
                elif cell == CellState.VISITED:
                    color = MAP_VISITED_COLOR
                else:
                    continue

                rect = pygame.Rect(
                    sx - pixel_size // 2,
                    sy - pixel_size // 2,
                    pixel_size,
                    pixel_size,
                )
                pygame.draw.rect(self.screen, color, rect)

    def draw(self, sim_robot, maneuver_status=""):
        """Draw one frame."""
        self.screen.fill(DARK_GRAY)

        state = sim_robot.state
        world = sim_robot.world

        # Draw occupancy grid (background layer)
        self._draw_occupancy_grid()

        # Draw walls
        for wall in world.walls:
            p1 = self.world_to_screen(wall.x1, wall.y1)
            p2 = self.world_to_screen(wall.x2, wall.y2)
            pygame.draw.line(self.screen, WALL_COLOR, p1, p2, 3)

        # Draw trail
        for tx, ty in sim_robot._trail:
            sp = self.world_to_screen(tx, ty)
            pygame.draw.circle(self.screen, TRAIL_COLOR, sp, 2)

        # Draw sensor rays
        for ray in sim_robot.get_sensor_rays():
            start = self.world_to_screen(*ray['start'])
            end = self.world_to_screen(*ray['end'])
            dist = ray['distance']
            if dist < DANGER_DISTANCE:
                color = RED
            elif dist < CAUTION_DISTANCE:
                color = YELLOW
            else:
                color = GREEN
            pygame.draw.line(self.screen, color, start, end, 1)
            # Small dot at hit point
            pygame.draw.circle(self.screen, color, end, 3)

        # Draw trailer
        trailer_corners = state.get_trailer_corners()
        screen_corners = [self.world_to_screen(cx, cy) for cx, cy in trailer_corners]
        pygame.draw.polygon(self.screen, GRAY, screen_corners)
        pygame.draw.polygon(self.screen, WHITE, screen_corners, 2)

        # Draw hitch line
        hitch_screen = self.world_to_screen(state.hitch_x, state.hitch_y)
        robot_screen = self.world_to_screen(state.x, state.y)
        pygame.draw.line(self.screen, ORANGE, robot_screen, hitch_screen, 2)

        # Draw robot body
        robot_corners = state.get_robot_corners()
        screen_corners = [self.world_to_screen(cx, cy) for cx, cy in robot_corners]
        pygame.draw.polygon(self.screen, BLUE, screen_corners)
        pygame.draw.polygon(self.screen, WHITE, screen_corners, 2)

        # Draw direction arrow on robot
        arrow_len = ROBOT_LENGTH * 0.6 * self.scale
        center = self.world_to_screen(state.x, state.y)
        arrow_end = (
            int(center[0] + arrow_len * math.cos(state.theta)),
            int(center[1] - arrow_len * math.sin(state.theta)),  # Flip Y
        )
        pygame.draw.line(self.screen, WHITE, center, arrow_end, 3)

        # HUD text
        y_off = 10
        hud_lines = [
            f"World: {world.name}",
            f"Pos: ({state.x:.0f}, {state.y:.0f})  Heading: {math.degrees(state.theta):.1f} deg",
            f"Hitch angle: {math.degrees(state.trailer_phi):.1f} deg",
            f"Wheels: L={state.left_speed:.0f} R={state.right_speed:.0f} mm/s",
            f"Front: {sim_robot.sensors.get_front_obstacle_distance()} mm  "
            f"L: {sim_robot.sensors.ext_ultra_l_mm} mm  "
            f"R: {sim_robot.sensors.ext_ultra_r_mm} mm",
        ]
        if self.spatial_map:
            hud_lines.append(
                f"Map: {self.spatial_map.get_exploration_progress()*100:.1f}% explored  "
                f"{self.spatial_map.get_visited_percentage()*100:.1f}% visited"
            )
        if maneuver_status:
            hud_lines.append(f"Maneuver: {maneuver_status}")
        if self.paused:
            hud_lines.append("** PAUSED **")

        for line in hud_lines:
            surf = self.font.render(line, True, WHITE)
            self.screen.blit(surf, (10, y_off))
            y_off += 18

        # Controls help at bottom
        controls = "A=Auto Wander  Space=Pause  R=Reset  1-4=Worlds  Z=Zigzag  Esc=Quit"
        surf = self.font.render(controls, True, GRAY)
        self.screen.blit(surf, (10, self.height - 25))

        pygame.display.flip()

    def handle_events(self):
        """Process pygame events. Returns dict of actions."""
        actions = {}
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                actions['quit'] = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    actions['quit'] = True
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                    actions['toggle_pause'] = True
                elif event.key == pygame.K_r:
                    actions['reset'] = True
                elif event.key == pygame.K_z:
                    actions['zigzag'] = True
                elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    actions['world_preset'] = event.key - pygame.K_0
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.speed_multiplier = min(10.0, self.speed_multiplier * 2)
                    actions['speed_change'] = self.speed_multiplier
                elif event.key == pygame.K_MINUS:
                    self.speed_multiplier = max(0.25, self.speed_multiplier / 2)
                    actions['speed_change'] = self.speed_multiplier
                elif event.key == pygame.K_a:
                    actions['auto_wander'] = True
                # Manual drive with arrow keys
                elif event.key == pygame.K_UP:
                    actions['drive_forward'] = True
                elif event.key == pygame.K_DOWN:
                    actions['drive_backward'] = True
                elif event.key == pygame.K_LEFT:
                    actions['turn_left'] = True
                elif event.key == pygame.K_RIGHT:
                    actions['turn_right'] = True
            elif event.type == pygame.KEYUP:
                if event.key in (pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT):
                    actions['stop_drive'] = True
        return actions

    def tick(self, fps=60):
        """Limit frame rate."""
        self.clock.tick(fps)

    def quit(self):
        """Clean up pygame."""
        pygame.quit()
