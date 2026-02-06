"""
Cozmo Robot Interface

Wraps pycozmo to provide a clean, async-friendly interface
for controlling the Cozmo robot.
"""
import asyncio
import logging
from dataclasses import dataclass
from typing import Optional, Callable
from enum import Enum
import numpy as np

try:
    import pycozmo
except ImportError:
    pycozmo = None
    print("WARNING: pycozmo not installed. Running in simulation mode.")

from PIL import Image

import config

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """Connection state of the robot"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class SensorData:
    """Current sensor readings from the robot"""
    cliff_detected: bool = False
    collision_detected: bool = False  # Set when accelerometer spike detected
    is_picked_up: bool = False
    is_on_charger: bool = False
    battery_voltage: float = 0.0
    head_angle: float = 0.0  # radians
    lift_height: float = 0.0  # mm
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # Pose tracking
    pose_x: float = 0.0  # mm from start
    pose_y: float = 0.0  # mm from start
    pose_angle: float = 0.0  # radians
    # External sensor pod (ESP32)
    ext_tof_mm: int = -1
    ext_ultra_l_mm: int = -1
    ext_ultra_c_mm: int = -1
    ext_ultra_r_mm: int = -1
    ext_pitch: float = 0.0
    ext_roll: float = 0.0
    ext_yaw: float = 0.0
    # External IMU raw values
    ext_ax_g: float = 0.0  # Accelerometer X (g's)
    ext_ay_g: float = 0.0  # Accelerometer Y (g's)
    ext_az_g: float = 0.0  # Accelerometer Z (g's)
    ext_gx_dps: float = 0.0  # Gyroscope X (degrees/sec)
    ext_gy_dps: float = 0.0  # Gyroscope Y (degrees/sec)
    ext_gz_dps: float = 0.0  # Gyroscope Z (degrees/sec)
    ext_ts_ms: int = 0  # ESP32 timestamp
    ext_connected: bool = False

    @staticmethod
    def _valid_distance(d: int) -> bool:
        """Check if a distance reading is valid (not error/disconnected)."""
        # -1 = disconnected, 0 = error, 8191/65535 = ToF out-of-range/error
        return 0 < d < 5000

    def get_front_obstacle_distance(self) -> int:
        """Get closest obstacle distance from forward-facing sensors."""
        distances = [d for d in [self.ext_tof_mm, self.ext_ultra_c_mm] if self._valid_distance(d)]
        return min(distances) if distances else -1

    def get_obstacle_distances(self) -> dict:
        """Get all obstacle distances, -1 for invalid/disconnected."""
        return {
            "front": self.get_front_obstacle_distance(),
            "left": self.ext_ultra_l_mm if self._valid_distance(self.ext_ultra_l_mm) else -1,
            "right": self.ext_ultra_r_mm if self._valid_distance(self.ext_ultra_r_mm) else -1,
        }


@dataclass
class RobotPose:
    """Robot's position and orientation"""
    x: float = 0.0  # mm
    y: float = 0.0  # mm
    angle: float = 0.0  # radians

    def distance_to(self, other: 'RobotPose') -> float:
        """Calculate distance to another pose"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class CozmoRobot:
    """
    High-level interface to the Cozmo robot.

    Provides async methods for movement, sensing, and feedback.
    Handles connection management and state tracking.
    """

    def __init__(self):
        self._client: Optional[pycozmo.Client] = None
        self._state = RobotState.DISCONNECTED
        self._sensors = SensorData()
        self._pose = RobotPose()
        self._last_image: Optional[Image.Image] = None
        self._image_callback: Optional[Callable] = None
        self._connected_event = asyncio.Event()
        # For collision detection - track last horizontal accel
        self._last_accel_xy: Optional[float] = None

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def sensors(self) -> SensorData:
        return self._sensors

    @property
    def pose(self) -> RobotPose:
        return self._pose

    @property
    def is_connected(self) -> bool:
        return self._state == RobotState.CONNECTED

    @property
    def last_image(self) -> Optional[Image.Image]:
        return self._last_image

    async def connect(self, timeout: float = 30.0) -> bool:
        """
        Connect to Cozmo robot.

        Make sure your PC is connected to Cozmo's WiFi network first.
        """
        if pycozmo is None:
            logger.warning("pycozmo not available, running in simulation mode")
            self._state = RobotState.CONNECTED
            return True

        self._state = RobotState.CONNECTING
        logger.info("Connecting to Cozmo...")

        try:
            self._client = pycozmo.Client()
            self._client.start()
            self._client.connect()
            self._client.wait_for_robot()

            # Register event handlers
            self._setup_handlers()

            self._state = RobotState.CONNECTED
            self._connected_event.set()
            logger.info("Connected to Cozmo!")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to Cozmo: {e}")
            self._state = RobotState.ERROR
            return False

    def _setup_handlers(self):
        """Register handlers for robot events"""
        if self._client is None:
            return

        try:
            # Image handler
            if hasattr(pycozmo.event, 'EvtNewRawCameraImage'):
                self._client.add_handler(
                    pycozmo.event.EvtNewRawCameraImage,
                    self._on_camera_image
                )

            # Cliff detection
            if hasattr(pycozmo.event, 'EvtCliffDetectedChange'):
                self._client.add_handler(
                    pycozmo.event.EvtCliffDetectedChange,
                    self._on_cliff_detected
                )

            # Robot state updates - use protocol_encoder.RobotState for pose data
            # This gives us direct access to pose_x, pose_y, pose_angle_rad
            if hasattr(pycozmo, 'protocol_encoder') and hasattr(pycozmo.protocol_encoder, 'RobotState'):
                self._client.add_handler(
                    pycozmo.protocol_encoder.RobotState,
                    self._on_robot_state_packet
                )
                logger.info("Registered for RobotState packets (pose tracking)")

            # Also try the event-based handler as fallback
            if hasattr(pycozmo.event, 'EvtRobotStateUpdated'):
                self._client.add_handler(
                    pycozmo.event.EvtRobotStateUpdated,
                    self._on_state_updated
                )

        except Exception as e:
            logger.warning(f"Could not set up some event handlers: {e}")

    def _on_camera_image(self, cli, *args):
        """Handle new camera frame"""
        if args:
            self._last_image = args[0]
            if self._image_callback:
                self._image_callback(args[0])

    def _on_cliff_detected(self, cli, *args):
        """Handle cliff detection change"""
        if args:
            self._sensors.cliff_detected = args[0]
            if args[0]:
                logger.warning("Cliff detected!")

    def _on_robot_state_packet(self, cli, pkt):
        """Handle robot state updates from RobotState packets.

        This is the primary method for getting pose data from pycozmo.
        The packet has direct attributes: pose_x, pose_y, pose_z, pose_angle_rad
        """
        try:
            # Update sensor data from packet
            self._sensors.battery_voltage = getattr(pkt, 'battery_voltage', self._sensors.battery_voltage)
            self._sensors.head_angle = getattr(pkt, 'head_angle_rad', self._sensors.head_angle)
            self._sensors.lift_height = getattr(pkt, 'lift_height_mm', self._sensors.lift_height)

            # Update pose directly from packet attributes
            old_x, old_y = self._pose.x, self._pose.y
            self._pose.x = getattr(pkt, 'pose_x', self._pose.x)
            self._pose.y = getattr(pkt, 'pose_y', self._pose.y)
            self._pose.angle = getattr(pkt, 'pose_angle_rad', self._pose.angle)

            # Debug: log significant pose changes
            if abs(self._pose.x - old_x) > 1.0 or abs(self._pose.y - old_y) > 1.0:
                logger.debug(f"Pose updated: ({old_x:.1f}, {old_y:.1f}) -> ({self._pose.x:.1f}, {self._pose.y:.1f})")

            # Sync to sensors
            self._sensors.pose_x = self._pose.x
            self._sensors.pose_y = self._pose.y
            self._sensors.pose_angle = self._pose.angle

            # Cliff detection from raw sensor data (more reliable than event)
            # High values (>100) = sensor sees ground = safe
            # Low values = sensor sees nothing = cliff/edge
            cliff_data = getattr(pkt, 'cliff_data_raw', None)
            if cliff_data:
                # Cliff detected if NO sensor sees the ground (all readings low)
                was_cliff = self._sensors.cliff_detected
                self._sensors.cliff_detected = not any(r > 100 for r in cliff_data)

                # IMMEDIATE STOP on cliff detection - don't wait for behavior loop
                if self._sensors.cliff_detected and not was_cliff:
                    logger.warning(f"CLIFF DETECTED! Raw: {cliff_data} - Emergency stop!")
                    if self._client:
                        self._client.drive_wheels(0, 0)  # Immediate stop

            # Check pickup status
            self._sensors.is_picked_up = getattr(pkt, 'is_picked_up', False)
            self._sensors.is_on_charger = getattr(pkt, 'is_on_charger', False)

            # Accelerometer/gyro if available
            self._sensors.accel_x = getattr(pkt, 'accel_x', 0.0)
            self._sensors.accel_y = getattr(pkt, 'accel_y', 0.0)
            self._sensors.accel_z = getattr(pkt, 'accel_z', 0.0)
            self._sensors.gyro_x = getattr(pkt, 'gyro_x', 0.0)
            self._sensors.gyro_y = getattr(pkt, 'gyro_y', 0.0)
            self._sensors.gyro_z = getattr(pkt, 'gyro_z', 0.0)

            # Collision detection - check for sudden horizontal acceleration spike
            # This runs every ~30ms at packet level for fast response
            accel_xy = np.sqrt(self._sensors.accel_x**2 + self._sensors.accel_y**2)
            if self._last_accel_xy is not None:
                accel_delta = abs(accel_xy - self._last_accel_xy)
                if accel_delta > config.COLLISION_ACCEL_THRESHOLD:
                    self._sensors.collision_detected = True
                    logger.warning(f"COLLISION DETECTED! accel_delta={accel_delta:.1f} - Emergency stop!")
                    if self._client:
                        self._client.drive_wheels(0, 0)
            self._last_accel_xy = accel_xy

        except Exception as e:
            logger.debug(f"Error processing RobotState packet: {e}")

    def _on_state_updated(self, cli, *args):
        """Handle robot state update"""
        # pycozmo passes the state object - figure out what we got
        state = args[0] if args else cli

        try:
            # Update sensor data
            self._sensors.is_picked_up = getattr(state, 'is_picked_up', False)
            self._sensors.is_on_charger = getattr(state, 'is_on_charger', False)
            self._sensors.battery_voltage = getattr(state, 'battery_voltage', 0.0)
            self._sensors.head_angle = getattr(state, 'head_angle', 0.0)
            self._sensors.lift_height = getattr(state, 'lift_height', 0.0)

            # Update pose - check multiple possible attribute names
            pose = getattr(state, 'pose', None)
            if pose:
                self._pose.x = getattr(pose, 'x', 0.0)
                self._pose.y = getattr(pose, 'y', 0.0)
                self._pose.angle = getattr(pose, 'angle', 0.0)
            else:
                # Try getting pose directly from state
                self._pose.x = getattr(state, 'pose_x', self._pose.x)
                self._pose.y = getattr(state, 'pose_y', self._pose.y)
                self._pose.angle = getattr(state, 'pose_angle', self._pose.angle)

            self._sensors.pose_x = self._pose.x
            self._sensors.pose_y = self._pose.y
            self._sensors.pose_angle = self._pose.angle

        except Exception as e:
            logger.debug(f"State update parse error: {e}")

    def _on_robot_poked(self, cli, *args):
        """Handle robot poked event - useful for debugging pose"""
        logger.debug(f"Robot poked event: {args}")

    def _on_odometry_update(self, cli, *args):
        """Handle odometry update from pycozmo"""
        if args:
            odom = args[0]
            try:
                # Update pose from odometry
                self._pose.x = getattr(odom, 'x', self._pose.x)
                self._pose.y = getattr(odom, 'y', self._pose.y)
                self._pose.angle = getattr(odom, 'angle', getattr(odom, 'rotation', self._pose.angle))

                # Sync to sensors
                self._sensors.pose_x = self._pose.x
                self._sensors.pose_y = self._pose.y
                self._sensors.pose_angle = self._pose.angle
            except Exception as e:
                logger.debug(f"Odometry update error: {e}")

    def _update_pose_from_client(self):
        """Try to get pose directly from pycozmo client"""
        if self._client:
            try:
                # Try multiple ways to get pose from pycozmo
                # Method 1: client.pose attribute
                if hasattr(self._client, 'pose'):
                    pose = self._client.pose
                    if pose:
                        self._pose.x = getattr(pose, 'x', self._pose.x)
                        self._pose.y = getattr(pose, 'y', self._pose.y)
                        self._pose.angle = getattr(pose, 'angle', self._pose.angle)
                        return

                # Method 2: client._pose attribute (internal)
                if hasattr(self._client, '_pose'):
                    pose = self._client._pose
                    if pose:
                        self._pose.x = getattr(pose, 'x', self._pose.x)
                        self._pose.y = getattr(pose, 'y', self._pose.y)
                        self._pose.angle = getattr(pose, 'angle', self._pose.angle)
                        return

                # Method 3: Try getting robot state
                if hasattr(self._client, 'robot_state'):
                    state = self._client.robot_state
                    if state:
                        pose = getattr(state, 'pose', None)
                        if pose:
                            self._pose.x = getattr(pose, 'x', self._pose.x)
                            self._pose.y = getattr(pose, 'y', self._pose.y)
                            self._pose.angle = getattr(pose, 'angle', self._pose.angle)

            except Exception as e:
                logger.debug(f"Pose update from client failed: {e}")

    async def disconnect(self):
        """Disconnect from Cozmo"""
        if self._client:
            self._client.disconnect()
            self._client.stop()
            self._client = None
        self._state = RobotState.DISCONNECTED
        self._connected_event.clear()
        logger.info("Disconnected from Cozmo")

    # ==================== Movement ====================

    async def drive(self, speed: float, duration: float = None):
        """
        Drive forward (positive) or backward (negative).

        Args:
            speed: Speed in mm/s (-200 to 200)
            duration: Optional duration in seconds
        """
        if not self.is_connected:
            return

        if pycozmo and self._client:
            self._client.drive_wheels(
                lwheel_speed=speed,
                rwheel_speed=speed,
                duration=duration
            )
        else:
            logger.debug(f"SIM: drive speed={speed} duration={duration}")

    async def turn(self, angle: float, speed: float = 30.0):
        """
        Turn in place (or arc turn if TRAILER_MODE enabled).

        Args:
            angle: Angle in degrees (positive = left, negative = right)
            speed: Turn speed in deg/s (used as base speed for arc turns)
        """
        if not self.is_connected:
            return

        # If trailer mode is enabled, use arc turns instead of in-place turns
        if config.TRAILER_MODE:
            ratio = config.TRAILER_ARC_RATIO
            # Arc turns need more time than in-place turns for same angle
            # Roughly 2x duration for gentle arcs
            arc_duration = abs(angle) / speed * 2.0

            if angle > 0:
                await self.arc_turn_left(speed, ratio, arc_duration)
            else:
                await self.arc_turn_right(speed, ratio, arc_duration)
            return

        if pycozmo and self._client:
            # Convert to radians and use turn_in_place if available
            # Otherwise use differential drive
            rad_angle = np.radians(angle)
            duration = abs(angle) / speed

            if angle > 0:
                # Turn left
                self._client.drive_wheels(
                    lwheel_speed=-speed,
                    rwheel_speed=speed,
                    duration=duration
                )
            else:
                # Turn right
                self._client.drive_wheels(
                    lwheel_speed=speed,
                    rwheel_speed=-speed,
                    duration=duration
                )
        else:
            logger.debug(f"SIM: turn angle={angle} speed={speed}")

        await asyncio.sleep(abs(angle) / speed)

    async def arc_turn_left(self, speed: float = 50.0, ratio: float = 0.5, duration: float = 1.0):
        """
        Arc turn to the left (both wheels forward, left wheel slower).

        Good for trailer mode where in-place turns would jackknife.

        Args:
            speed: Outer wheel speed in mm/s
            ratio: Inner wheel speed ratio (0.5 = 50% of outer)
            duration: How long to turn in seconds
        """
        if not self.is_connected:
            return

        inner_speed = speed * ratio  # Left wheel (inner)
        outer_speed = speed          # Right wheel (outer)

        if pycozmo and self._client:
            self._client.drive_wheels(
                lwheel_speed=inner_speed,
                rwheel_speed=outer_speed,
                duration=duration
            )
        else:
            logger.debug(f"SIM: arc_turn_left speed={speed} ratio={ratio} duration={duration}")

        await asyncio.sleep(duration)

    async def arc_turn_right(self, speed: float = 50.0, ratio: float = 0.5, duration: float = 1.0):
        """
        Arc turn to the right (both wheels forward, right wheel slower).

        Good for trailer mode where in-place turns would jackknife.

        Args:
            speed: Outer wheel speed in mm/s
            ratio: Inner wheel speed ratio (0.5 = 50% of outer)
            duration: How long to turn in seconds
        """
        if not self.is_connected:
            return

        outer_speed = speed          # Left wheel (outer)
        inner_speed = speed * ratio  # Right wheel (inner)

        if pycozmo and self._client:
            self._client.drive_wheels(
                lwheel_speed=outer_speed,
                rwheel_speed=inner_speed,
                duration=duration
            )
        else:
            logger.debug(f"SIM: arc_turn_right speed={speed} ratio={ratio} duration={duration}")

        await asyncio.sleep(duration)

    async def reverse_arc_turn_left(self, speed: float = 50.0, ratio: float = 0.5, duration: float = 1.0):
        """
        Reverse arc turn to the left (both wheels backward, left wheel slower).

        Good for backing out with a trailer - turns the trailer to the right
        while backing up.

        Args:
            speed: Outer wheel speed in mm/s (will be negated for reverse)
            ratio: Inner wheel speed ratio (0.5 = 50% of outer)
            duration: How long to turn in seconds
        """
        if not self.is_connected:
            return

        inner_speed = -speed * ratio  # Left wheel (inner, reversed)
        outer_speed = -speed          # Right wheel (outer, reversed)

        if pycozmo and self._client:
            self._client.drive_wheels(
                lwheel_speed=inner_speed,
                rwheel_speed=outer_speed,
                duration=duration
            )
        else:
            logger.debug(f"SIM: reverse_arc_turn_left speed={speed} ratio={ratio} duration={duration}")

        await asyncio.sleep(duration)

    async def reverse_arc_turn_right(self, speed: float = 50.0, ratio: float = 0.5, duration: float = 1.0):
        """
        Reverse arc turn to the right (both wheels backward, right wheel slower).

        Good for backing out with a trailer - turns the trailer to the left
        while backing up.

        Args:
            speed: Outer wheel speed in mm/s (will be negated for reverse)
            ratio: Inner wheel speed ratio (0.5 = 50% of outer)
            duration: How long to turn in seconds
        """
        if not self.is_connected:
            return

        outer_speed = -speed          # Left wheel (outer, reversed)
        inner_speed = -speed * ratio  # Right wheel (inner, reversed)

        if pycozmo and self._client:
            self._client.drive_wheels(
                lwheel_speed=outer_speed,
                rwheel_speed=inner_speed,
                duration=duration
            )
        else:
            logger.debug(f"SIM: reverse_arc_turn_right speed={speed} ratio={ratio} duration={duration}")

        await asyncio.sleep(duration)

    async def stop(self):
        """Stop all movement immediately"""
        if pycozmo and self._client:
            self._client.drive_wheels(0, 0)
        else:
            logger.debug("SIM: stop")

    async def set_head_angle(self, angle: float):
        """
        Set head angle.

        Args:
            angle: Angle in radians (-0.44 to 0.78)
        """
        if pycozmo and self._client:
            self._client.set_head_angle(angle)
        else:
            logger.debug(f"SIM: head_angle={angle}")

    async def set_lift_height(self, height: float):
        """
        Set lift height.

        Args:
            height: Height ratio (0.0 = fully lowered, 1.0 = fully raised)
        """
        if pycozmo and self._client:
            # pycozmo expects millimeters (32mm min to 92mm max)
            height_mm = 32.0 + max(0.0, min(1.0, height)) * (92.0 - 32.0)
            self._client.set_lift_height(height_mm)
        else:
            logger.debug(f"SIM: lift_height={height}")

    # ==================== Camera ====================

    async def capture_image(self) -> Optional[Image.Image]:
        """
        Capture a fresh camera frame.

        Clears the previous frame and waits for a new one to arrive,
        ensuring the image reflects the current camera view (not a stale frame
        from before the lift was raised, etc.).

        Returns:
            PIL Image or None if not available
        """
        if pycozmo and self._client:
            # Enable camera if not already
            self._client.enable_camera(enable=True, color=True)
            # Clear stale frame so we wait for a fresh one
            self._last_image = None
            # Wait for a new frame to arrive (camera runs ~15fps, so ~70ms per frame)
            for _ in range(10):  # Up to 1 second
                await asyncio.sleep(0.1)
                if self._last_image is not None:
                    return self._last_image
            logger.warning("Timed out waiting for fresh camera frame")
            return self._last_image
        else:
            # Return a dummy image in simulation
            return Image.new('RGB', (320, 240), color='gray')

    def set_image_callback(self, callback: Callable):
        """Set callback for continuous image streaming"""
        self._image_callback = callback
        if pycozmo and self._client:
            self._client.enable_camera(enable=True, color=True)

    # ==================== Audio ====================

    async def set_volume(self, level: int = 50000):
        """
        Set Cozmo's speaker volume.

        Args:
            level: Volume level (0-65535, default 50000 ~= 75%)
        """
        if pycozmo and self._client:
            self._client.set_volume(level)
            logger.debug(f"Volume set to {level}")

    async def play_audio(self, wav_path: str, wait: bool = True):
        """
        Play a WAV file through Cozmo's speaker.

        Args:
            wav_path: Path to WAV file (must be 22kHz, 16-bit mono)
            wait: Wait for audio to complete before returning
        """
        if pycozmo and self._client:
            try:
                # Ensure volume is set
                self._client.set_volume(50000)

                # Play the audio
                self._client.play_audio(str(wav_path))
                logger.info(f"Playing audio: {wav_path}")

                # Wait for completion if requested
                if wait:
                    self._client.wait_for(pycozmo.event.EvtAudioCompleted)

            except Exception as e:
                logger.error(f"Failed to play audio: {e}")
                # Log to error file for debugging
                try:
                    from brain.personality import error_log
                    error_log.log_error("audio", f"Failed to play audio: {wav_path}", e)
                except:
                    pass  # Don't crash if error logging fails
        else:
            logger.debug(f"SIM: play_audio={wav_path}")

    # ==================== Lights ====================

    async def set_head_light(self, enable: bool):
        """
        Turn Cozmo's head LED (headlight) on or off.

        Args:
            enable: True to turn on, False to turn off
        """
        if pycozmo and self._client:
            self._client.set_head_light(enable)
            logger.debug(f"Head light: {'on' if enable else 'off'}")
        else:
            logger.debug(f"SIM: head_light={enable}")

    # ==================== Display ====================

    async def display_text(self, text: str, duration: float = 2.0):
        """Display text on Cozmo's face (limited)"""
        # pycozmo doesn't have direct text display
        # but we can play with the face
        logger.info(f"Display: {text}")

    async def set_expression(self, expression: str):
        """
        Set Cozmo's face expression.

        Args:
            expression: One of 'happy', 'sad', 'angry', 'surprised', 'neutral'
        """
        if pycozmo and self._client:
            # pycozmo uses procedural face generation
            # We'd need to map expressions to face parameters
            pass
        logger.debug(f"SIM: expression={expression}")

    async def play_animation(self, name: str):
        """Play a built-in animation by name"""
        if pycozmo and self._client:
            try:
                self._client.play_anim(name)
            except Exception as e:
                logger.warning(f"Failed to play animation {name}: {e}")
        else:
            logger.debug(f"SIM: animation={name}")

    # ==================== Context Manager ====================

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()


# Convenience function for quick testing
async def test_connection():
    """Quick test to verify Cozmo connection works"""
    async with CozmoRobot() as robot:
        if robot.is_connected:
            print("Connected to Cozmo!")
            print(f"Battery: {robot.sensors.battery_voltage}V")

            # Quick movement test
            await robot.drive(50, duration=1.0)
            await asyncio.sleep(1.0)
            await robot.turn(90)
            await robot.stop()

            print("Test complete!")
        else:
            print("Failed to connect")


if __name__ == "__main__":
    asyncio.run(test_connection())
