"""
Simulated robot that provides the same interface as CozmoRobot.

Uses physics.py for kinematic updates and ray-casting for sensor simulation.
Duck-typed to match CozmoRobot's methods used by WanderBehavior: drive(), stop(),
turn(), escape_turn(), set_wheels(), arc_turn_left/right(), and sensors/pose.
"""
import asyncio
import math
import logging
import random

import config
from simulator.physics import RobotTrailerState, step as physics_step, TRACK_WIDTH
from simulator.world import World, Wall

logger = logging.getLogger(__name__)

# Sensor geometry (angles relative to robot heading)
SENSOR_CONFIGS = {
    'tof':          {'angle': 0.0,  'max_range': 2000},
    'ultra_center': {'angle': 0.0,  'max_range': 4000},
    'ultra_left':   {'angle': math.radians(15),  'max_range': 4000},
    'ultra_right':  {'angle': math.radians(-15), 'max_range': 4000},
}

# Sensor noise profiles (from real-world observation)
# Ultrasonics: noisy, ±3% gaussian + occasional dropout/spike
# ToF: more precise, ±1% gaussian
ULTRA_NOISE_PERCENT = 3.0   # Gaussian sigma as % of reading
ULTRA_DROPOUT_CHANCE = 0.02 # 2% chance of -1 (missed echo)
ULTRA_SPIKE_CHANCE = 0.01   # 1% chance of wild reading
TOF_NOISE_PERCENT = 1.0     # ToF is more accurate


def _add_ultrasonic_noise(distance_mm):
    """Add realistic noise to an ultrasonic reading."""
    if distance_mm <= 0:
        return distance_mm

    # Occasional dropout (missed echo) -> returns -1
    if random.random() < ULTRA_DROPOUT_CHANCE:
        return -1

    # Occasional spike (multipath/interference) -> random wild value
    if random.random() < ULTRA_SPIKE_CHANCE:
        return random.randint(50, 4000)

    # Gaussian noise proportional to distance
    sigma = distance_mm * ULTRA_NOISE_PERCENT / 100.0
    noisy = distance_mm + random.gauss(0, sigma)
    return max(1, int(round(noisy)))


def _add_tof_noise(distance_mm):
    """Add realistic noise to a ToF reading."""
    if distance_mm <= 0:
        return distance_mm

    sigma = distance_mm * TOF_NOISE_PERCENT / 100.0
    noisy = distance_mm + random.gauss(0, sigma)
    return max(1, int(round(noisy)))


class SimSensorData:
    """Sensor data matching CozmoRobot.SensorData interface."""

    def __init__(self):
        self.cliff_detected = False
        self.collision_detected = False
        self.is_picked_up = False
        self.is_on_charger = False
        self.battery_voltage = 4.0
        self.head_angle = 0.0
        self.lift_height = 0.0
        self.ext_tof_mm = -1
        self.ext_ultra_l_mm = -1
        self.ext_ultra_c_mm = -1
        self.ext_ultra_r_mm = -1
        self.ext_pitch = 0.0
        self.ext_roll = 0.0
        self.ext_yaw = 0.0
        self.ext_connected = True
        # Internal accel/gyro (ExperienceLogger reads these)
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        # External IMU raw data (ExperienceLogger reads these)
        self.ext_ax_g = 0.0
        self.ext_ay_g = 0.0
        self.ext_az_g = 0.0
        self.ext_gx_dps = 0.0
        self.ext_gy_dps = 0.0
        self.ext_gz_dps = 0.0
        self.ext_ts_ms = 0
        # Pose fields (synced from physics)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_angle = 0.0

    @staticmethod
    def _valid_distance(d):
        return 0 < d < 5000

    def get_front_obstacle_distance(self):
        distances = [d for d in [self.ext_tof_mm, self.ext_ultra_c_mm] if self._valid_distance(d)]
        return min(distances) if distances else -1

    def get_obstacle_distances(self):
        return {
            "front": self.get_front_obstacle_distance(),
            "left": self.ext_ultra_l_mm if self._valid_distance(self.ext_ultra_l_mm) else -1,
            "right": self.ext_ultra_r_mm if self._valid_distance(self.ext_ultra_r_mm) else -1,
        }


def _ray_cast(ox, oy, angle, walls, max_range):
    """
    Cast a ray from (ox, oy) at the given angle against wall segments.
    Returns distance to nearest intersection, or max_range if none.
    """
    dx = math.cos(angle)
    dy = math.sin(angle)
    best = max_range

    for wall in walls:
        dist = _ray_segment_intersect(ox, oy, dx, dy,
                                      wall.x1, wall.y1, wall.x2, wall.y2)
        if dist is not None and 0 < dist < best:
            best = dist

    return int(best)


def _ray_segment_intersect(ox, oy, dx, dy, x1, y1, x2, y2):
    """
    Ray-segment intersection.
    Ray: P = O + t*D  (t >= 0)
    Segment: Q = A + u*(B-A)  (0 <= u <= 1)
    """
    sx = x2 - x1
    sy = y2 - y1

    denom = dx * sy - dy * sx
    if abs(denom) < 1e-10:
        return None  # Parallel

    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom

    if t >= 0 and 0 <= u <= 1:
        return t
    return None


def _segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
    """Check if segment AB intersects segment CD."""
    sx1 = bx - ax
    sy1 = by - ay
    sx2 = dx - cx
    sy2 = dy - cy

    denom = sx1 * sy2 - sy1 * sx2
    if abs(denom) < 1e-10:
        return False

    s = ((cx - ax) * sy2 - (cy - ay) * sx2) / denom
    t = ((cx - ax) * sy1 - (cy - ay) * sx1) / denom

    return 0 <= s <= 1 and 0 <= t <= 1


def _body_hits_walls(corners, walls):
    """Check if any edge of a body polygon intersects any wall."""
    n = len(corners)
    for i in range(n):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % n]
        for wall in walls:
            if _segments_intersect(ax, ay, bx, by,
                                   wall.x1, wall.y1, wall.x2, wall.y2):
                return True
    return False


class SimPose:
    """Pose object matching CozmoRobot.pose interface."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0


class SimRobot:
    """
    Simulated robot with physics-based driving and ray-cast sensors.

    Provides the same async interface as CozmoRobot for WanderBehavior.
    """

    PHYSICS_HZ = 100   # Physics update rate
    SENSOR_HZ = 20     # Sensor update rate (matches ESP32 ~20Hz)

    def __init__(self, world: World):
        self.world = world
        self.state = RobotTrailerState(
            x=world.spawn_x,
            y=world.spawn_y,
            theta=world.spawn_theta,
        )
        self.sensors = SimSensorData()
        self.pose = SimPose()
        self.is_connected = True
        self._running = False
        self._physics_task = None
        self._trail = []  # Path history for rendering
        self._trail_interval = 5  # Record every N physics steps
        self._step_count = 0
        self._sensor_divider = max(1, self.PHYSICS_HZ // self.SENSOR_HZ)

        # Previous position for collision rollback
        self._prev_x = world.spawn_x
        self._prev_y = world.spawn_y
        self._prev_theta = world.spawn_theta
        self._prev_phi = 0.0

        # Escape flag (mirrors CozmoRobot)
        self._escape_in_progress = False
        self._audio_playing = False

    async def connect(self):
        """Connect (compatibility with CozmoRobot) - starts physics."""
        await self.start()
        return True

    async def disconnect(self):
        """Disconnect (compatibility with CozmoRobot) - stops physics."""
        await self.shutdown()

    async def set_head_angle(self, angle: float):
        """No-op: SimRobot has no head."""
        self.sensors.head_angle = angle

    async def set_lift_height(self, height: float):
        """No-op: SimRobot has no lift."""
        self.sensors.lift_height = height

    async def capture_image(self):
        """No-op: SimRobot has no camera. Returns None."""
        return None

    async def start(self):
        """Start the physics simulation loop."""
        self._running = True
        self._physics_task = asyncio.create_task(self._physics_loop())

    async def shutdown(self):
        """Stop the physics simulation."""
        self._running = False
        if self._physics_task:
            self._physics_task.cancel()
            try:
                await self._physics_task
            except asyncio.CancelledError:
                pass

    async def _physics_loop(self):
        """Run physics at PHYSICS_HZ, sensors at SENSOR_HZ, collision every tick."""
        dt = 1.0 / self.PHYSICS_HZ
        while self._running:
            # Save pre-step position for collision rollback
            self._prev_x = self.state.x
            self._prev_y = self.state.y
            self._prev_theta = self.state.theta
            self._prev_phi = self.state.trailer_phi

            physics_step(self.state, dt)
            self._check_collisions()
            self._update_pose()

            self._step_count += 1

            # Update sensors at ESP32 rate (~20Hz), not every physics tick
            if self._step_count % self._sensor_divider == 0:
                self._update_sensors()

            # Record trail
            if self._step_count % self._trail_interval == 0:
                self._trail.append((self.state.x, self.state.y))
                if len(self._trail) > 2000:
                    self._trail = self._trail[-1000:]

            await asyncio.sleep(dt)

    def _update_pose(self):
        """Sync pose from physics state."""
        self.pose.x = self.state.x
        self.pose.y = self.state.y
        self.pose.angle = self.state.theta

    def _check_collisions(self):
        """Check if robot or trailer body intersects any wall.

        On collision: revert position, stop wheels, set collision_detected.
        This simulates the real accelerometer spike -> emergency stop.
        """
        walls = self.world.walls
        hit = (_body_hits_walls(self.state.get_robot_corners(), walls) or
               _body_hits_walls(self.state.get_trailer_corners(), walls))

        if hit:
            # Revert to pre-collision position
            self.state.x = self._prev_x
            self.state.y = self._prev_y
            self.state.theta = self._prev_theta
            self.state.trailer_phi = self._prev_phi

            # Stop wheels and flag collision (same as real accelerometer handler)
            self.state.left_speed = 0.0
            self.state.right_speed = 0.0

            if not self._escape_in_progress:
                self.sensors.collision_detected = True
                logger.debug(f"COLLISION at ({self.state.x:.0f}, {self.state.y:.0f})")

    def _update_sensors(self):
        """Update simulated sensor readings via ray-casting (called at ~20Hz).

        Applies realistic noise: ultrasonics get ±3% gaussian + occasional
        dropouts/spikes, ToF gets ±1% gaussian. Matches real ESP32 pod behavior.
        """
        ox = self.state.x
        oy = self.state.y
        theta = self.state.theta
        walls = self.world.walls

        # ToF (more accurate)
        cfg = SENSOR_CONFIGS['tof']
        raw = _ray_cast(ox, oy, theta + cfg['angle'], walls, cfg['max_range'])
        self.sensors.ext_tof_mm = _add_tof_noise(raw)

        # Ultrasonic center
        cfg = SENSOR_CONFIGS['ultra_center']
        raw = _ray_cast(ox, oy, theta + cfg['angle'], walls, cfg['max_range'])
        self.sensors.ext_ultra_c_mm = _add_ultrasonic_noise(raw)

        # Ultrasonic left
        cfg = SENSOR_CONFIGS['ultra_left']
        raw = _ray_cast(ox, oy, theta + cfg['angle'], walls, cfg['max_range'])
        self.sensors.ext_ultra_l_mm = _add_ultrasonic_noise(raw)

        # Ultrasonic right
        cfg = SENSOR_CONFIGS['ultra_right']
        raw = _ray_cast(ox, oy, theta + cfg['angle'], walls, cfg['max_range'])
        self.sensors.ext_ultra_r_mm = _add_ultrasonic_noise(raw)

        # Yaw from robot heading (degrees, 0-360)
        self.sensors.ext_yaw = math.degrees(self.state.theta) % 360

    # === CozmoRobot-compatible interface ===

    async def set_wheels(self, left_speed: float, right_speed: float):
        """Set wheel speeds without blocking."""
        self.state.left_speed = left_speed
        self.state.right_speed = right_speed

    async def stop(self):
        """Stop all movement."""
        self.state.left_speed = 0.0
        self.state.right_speed = 0.0

    async def drive(self, speed: float, duration: float = None):
        """Drive forward/backward."""
        self.state.left_speed = speed
        self.state.right_speed = speed
        if duration:
            await asyncio.sleep(duration)
            await self.stop()

    async def turn(self, angle: float, speed: float = 30.0):
        """Turn via arc (trailer mode) - matches CozmoRobot.turn()."""
        ratio = config.TRAILER_ARC_RATIO
        arc_duration = abs(angle) / speed * 2.0
        if angle > 0:
            await self.arc_turn_left(speed, ratio, arc_duration)
        else:
            await self.arc_turn_right(speed, ratio, arc_duration)

    async def arc_turn_left(self, speed=50.0, ratio=0.5, duration=1.0):
        """Arc turn left - left wheel slower."""
        self.state.left_speed = speed * ratio
        self.state.right_speed = speed
        await asyncio.sleep(duration)
        await self.stop()

    async def arc_turn_right(self, speed=50.0, ratio=0.5, duration=1.0):
        """Arc turn right - right wheel slower."""
        self.state.left_speed = speed
        self.state.right_speed = speed * ratio
        await asyncio.sleep(duration)
        await self.stop()

    async def escape_turn(self, angle: float):
        """Arc turn at escape speed - matches CozmoRobot.escape_turn()."""
        escape_speed = config.ESCAPE_SPEED
        ratio = 0.3
        arc_duration = abs(angle) / 30.0 * 1.0
        if angle > 0:
            await self.arc_turn_left(escape_speed, ratio, arc_duration)
        else:
            await self.arc_turn_right(escape_speed, ratio, arc_duration)

    def reset(self, world: World = None):
        """Reset robot to spawn position, optionally with a new world."""
        if world:
            self.world = world
        self.state = RobotTrailerState(
            x=self.world.spawn_x,
            y=self.world.spawn_y,
            theta=self.world.spawn_theta,
        )
        self.pose.x = self.state.x
        self.pose.y = self.state.y
        self.pose.angle = self.state.theta
        self._prev_x = self.state.x
        self._prev_y = self.state.y
        self._prev_theta = self.state.theta
        self._prev_phi = 0.0
        self.sensors.collision_detected = False
        self._trail.clear()
        self._step_count = 0

    def get_sensor_rays(self):
        """Get ray endpoints for rendering (origin, hit_point, distance, max_range)."""
        ox = self.state.x
        oy = self.state.y
        theta = self.state.theta
        rays = []
        for name, cfg in SENSOR_CONFIGS.items():
            angle = theta + cfg['angle']
            dist = _ray_cast(ox, oy, angle, self.world.walls, cfg['max_range'])
            end_x = ox + dist * math.cos(angle)
            end_y = oy + dist * math.sin(angle)
            rays.append({
                'name': name,
                'start': (ox, oy),
                'end': (end_x, end_y),
                'distance': dist,
                'max_range': cfg['max_range'],
            })
        return rays
