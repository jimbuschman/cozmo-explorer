"""
Cozmo Explorer - Main Entry Point

Mapping platform: Cozmo drives around building a spatial map.
Post-session, the LLM reviews the map and produces semantic annotations.
"""
import asyncio
import logging
import math
import os
import signal
import sys
from datetime import datetime
from typing import Optional

import config
from perception.external_sensors import ExternalSensorReader
from cozmo_interface.robot import CozmoRobot
from brain.mapper_state_machine import MapperStateMachine, MapperState
from brain.session_reviewer import SessionReviewer
from memory.spatial_map import SpatialMap
from memory.state_store import StateStore
from memory.map_annotations import MapAnnotationStore
from memory.experience_logger import ExperienceLogger
from control.manual_controller import ManualController

# Configure logging - both screen and file
log_format = '%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s'
log_datefmt = '%H:%M:%S'

logging.basicConfig(
    level=logging.DEBUG,
    format=log_format,
    datefmt=log_datefmt,
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(config.DATA_DIR / 'cozmo.log', mode='w', encoding='utf-8')
    ]
)

# Set console to INFO only
logging.getLogger().handlers[0].setLevel(logging.INFO)

logger = logging.getLogger(__name__)


class CozmoExplorer:
    """
    Main application class - mapping platform architecture.

    Cozmo's job: drive around, collect sensor data, build the spatial map.
    System's job: LLM reviews completed maps, produces semantic annotations.
    """

    def __init__(self):
        self.robot = CozmoRobot()
        self.spatial_map = SpatialMap()
        self.state_store = StateStore()
        self.mapper: Optional[MapperStateMachine] = None
        self.external_sensors: Optional[ExternalSensorReader] = None

        # LLM for post-session review
        self.llm = None
        self.session_reviewer: Optional[SessionReviewer] = None
        self.annotation_store: Optional[MapAnnotationStore] = None
        self.experience_logger: Optional[ExperienceLogger] = None

        # Manual control
        self.manual_controller: Optional[ManualController] = None
        self._manual_mode = False

        self._running = False
        self._start_time: Optional[datetime] = None

    async def initialize(self) -> bool:
        """Initialize all components."""
        logger.info("=" * 60)
        logger.info("  COZMO EXPLORER - Mapping Platform")
        logger.info("=" * 60)

        # Connect to state store
        logger.info("Connecting to state store...")
        self.state_store.connect()

        # Initialize experience logger
        logger.info("Initializing experience logger...")
        self.experience_logger = ExperienceLogger()
        self.experience_logger.connect()

        # Initialize annotation store
        self.annotation_store = MapAnnotationStore()
        self.annotation_store.connect()

        # Load previous map if exists
        map_file = config.DATA_DIR / "spatial_map.npz"
        if map_file.exists():
            try:
                self.spatial_map = SpatialMap.load(str(map_file))
                logger.info(f"Loaded previous spatial map: {self.spatial_map.get_visited_percentage()*100:.1f}% visited")
            except Exception as e:
                logger.warning(f"Could not load map: {e}")

        # Check LLM availability (for post-session review)
        if config.ENABLE_LLM:
            logger.info("Checking LLM availability...")
            try:
                from llm.client import LLMClient
                self.llm = LLMClient()
                await self.llm.connect()
                if await self.llm.health_check():
                    logger.info(f"LLM ready: {self.llm.model} (post-session review)")
                    self.session_reviewer = SessionReviewer(self.llm)
                else:
                    logger.warning("LLM not available - will skip post-session review")
                    self.llm = None
            except Exception as e:
                logger.warning(f"LLM init failed: {e} - will skip post-session review")
                self.llm = None
        else:
            logger.info("LLM disabled - no post-session review")

        # Connect to robot
        logger.info("Connecting to Cozmo...")
        logger.info("(Make sure PC is connected to Cozmo's WiFi)")
        connected = await self.robot.connect()

        if not connected:
            logger.error("Failed to connect to Cozmo!")
            return False

        logger.info("Robot connected!")

        # Connect external sensors (ESP32 pod)
        ext_mode = os.environ.get("EXT_SENSOR_MODE", config.EXT_SENSOR_MODE)
        ext_port = os.environ.get("EXT_SENSOR_PORT", config.EXT_SENSOR_PORT)
        ext_udp_port = int(os.environ.get("EXT_SENSOR_UDP_PORT", config.EXT_SENSOR_UDP_PORT))

        if ext_mode == "udp":
            logger.info(f"Connecting external sensors via WiFi UDP (port {ext_udp_port})...")
            self.external_sensors = ExternalSensorReader(
                mode="udp",
                udp_port=ext_udp_port
            )
        else:
            logger.info(f"Connecting external sensors via serial ({ext_port})...")
            self.external_sensors = ExternalSensorReader(
                mode="serial",
                port=ext_port,
                baud=config.EXT_SENSOR_BAUD
            )

        if self.external_sensors.start():
            logger.info("External sensors connected!")
            self.robot._sensors.ext_connected = True

            def on_ext_reading(reading):
                s = self.robot._sensors
                s.ext_tof_mm = reading.tof_mm
                s.ext_ultra_l_mm = reading.ultra_l_mm
                s.ext_ultra_c_mm = reading.ultra_c_mm
                s.ext_ultra_r_mm = reading.ultra_r_mm
                ax = reading.ax_g * config.IMU_ACCEL_X_SIGN
                ay = reading.ay_g * config.IMU_ACCEL_Y_SIGN
                az = reading.az_g * config.IMU_ACCEL_Z_SIGN
                s.ext_pitch = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi
                s.ext_roll = math.atan2(-ax, az) * 180.0 / math.pi
                s.ext_yaw = reading.yaw
                s.ext_ax_g = reading.ax_g
                s.ext_ay_g = reading.ay_g
                s.ext_az_g = reading.az_g
                s.ext_gx_dps = reading.gx_dps
                s.ext_gy_dps = reading.gy_dps
                s.ext_gz_dps = reading.gz_dps
                s.ext_ts_ms = reading.ts_ms

            self.external_sensors.set_callback(on_ext_reading)
        else:
            logger.warning("External sensors not available - reactive collision detection only")

        # Initialize mapper state machine
        self.mapper = MapperStateMachine(
            robot=self.robot,
            spatial_map=self.spatial_map,
            experience_logger=self.experience_logger,
            session_reviewer=self.session_reviewer,
            annotation_store=self.annotation_store,
        )

        # Initialize manual controller if enabled
        if config.MANUAL_CONTROL_ENABLED:
            logger.info("Manual control enabled - creating controller")
            self.manual_controller = ManualController(
                robot=self.robot,
                on_mode_change=self._on_manual_mode_change
            )
            self._manual_mode = True
        else:
            self._manual_mode = False

        logger.info("Initialization complete!")

        trailer_status = "with trailer" if config.TRAILER_MODE else "without trailer"
        control_status = "manual control" if config.MANUAL_CONTROL_ENABLED else "autonomous"
        logger.info(f"Mode: {control_status}, {trailer_status}")

        return True

    async def run(self):
        """Main run loop."""
        self._running = True
        self._start_time = datetime.now()

        # Start a new session
        session_id = self.state_store.start_session()
        logger.info(f"Started mapping session {session_id}")

        if self.experience_logger:
            self.experience_logger.set_session_id(session_id)

        try:
            # Set head to look forward
            await self.robot.set_head_angle(0.35)
            logger.info("Head positioned for mapping")

            # Start manual controller GUI if enabled
            if self.manual_controller:
                logger.info("Starting manual control GUI...")
                loop = asyncio.get_event_loop()
                self.manual_controller.start(loop)

                import threading
                gui_thread = threading.Thread(target=self.manual_controller.run_gui, daemon=True)
                gui_thread.start()
                logger.info("Manual control GUI running in background")

            # Start mapper in background (if not in manual mode)
            mapper_task = None
            if not self._manual_mode:
                mapper_task = asyncio.create_task(self.mapper.start())

            # Main monitoring loop
            last_status_time = datetime.now()
            while self._running:
                await asyncio.sleep(0.5)

                # Check if manual controller closed
                if self.manual_controller and not self.manual_controller.is_running:
                    logger.info("Manual controller closed - exiting")
                    break

                # Refresh pose from pycozmo
                self.robot._update_pose_from_client()

                # Start mapper if switching from manual to auto
                if not self._manual_mode and mapper_task is None:
                    logger.info("Starting mapper...")
                    mapper_task = asyncio.create_task(self.mapper.start())

                # Check if mapper finished
                if self.mapper and self.mapper.state == MapperState.DONE:
                    logger.info("Mapping complete!")
                    break

                # Periodic status (every 30 seconds)
                now = datetime.now()
                if (now - last_status_time).total_seconds() >= 30:
                    self._print_status()
                    last_status_time = now

        except asyncio.CancelledError:
            logger.info("Run loop cancelled")
        finally:
            await self._shutdown()

    async def _shutdown(self):
        """Clean shutdown."""
        logger.info("Shutting down...")

        # Stop manual controller
        if self.manual_controller:
            self.manual_controller.stop()

        # Stop mapper
        if self.mapper:
            await self.mapper.stop()

        # Save state
        duration = (datetime.now() - self._start_time).total_seconds() if self._start_time else 0

        self.state_store.save_robot_state(
            position={'x': self.robot.pose.x, 'y': self.robot.pose.y, 'angle': self.robot.pose.angle},
            battery=self.robot.sensors.battery_voltage,
            exploration_time=self.mapper.mapping_time if self.mapper else 0
        )

        self.state_store.end_session(
            duration=duration,
            distance=self.mapper.mapping_time * 50 if self.mapper else 0,
            areas=int(self.spatial_map.get_visited_percentage() * 100),
            summary=f"Mapped for {duration:.0f}s"
        )

        # Save map
        try:
            self.spatial_map.save(str(config.DATA_DIR / "spatial_map.npz"))
        except Exception as e:
            logger.error(f"Failed to save map: {e}")

        # Print final stats
        if self.mapper:
            status = self.mapper.get_status()
            logger.info(f"Final: {status['visited_pct']:.1f}% visited, "
                       f"{status['escapes']} escapes, {status['frontier_targets']} targets")

        if self.annotation_store:
            ann_count = self.annotation_store.count()
            if ann_count > 0:
                logger.info(f"Annotations: {ann_count} stored")

        # Stop external sensors
        if self.external_sensors:
            self.external_sensors.stop()

        # Disconnect
        await self.robot.disconnect()
        if self.llm:
            await self.llm.close()
        self.state_store.close()

        if self.experience_logger:
            self.experience_logger.close()
        if self.annotation_store:
            self.annotation_store.close()

        logger.info("Shutdown complete")

    def _on_manual_mode_change(self, is_manual: bool):
        """Handle switching between manual and auto modes."""
        self._manual_mode = is_manual
        if is_manual:
            logger.info("Switched to MANUAL mode")
            if self.mapper:
                self.mapper.pause()
        else:
            logger.info("Switched to AUTO mode")
            if self.mapper:
                self.mapper.resume()

    def _print_status(self):
        """Print current status."""
        if not self.mapper:
            return

        status = self.mapper.get_status()
        robot = self.robot

        logger.info("-" * 40)
        control_mode = "MANUAL" if self._manual_mode else "AUTO"
        trailer_status = "TRAILER" if config.TRAILER_MODE else ""
        logger.info(f"Mapper | Control: {control_mode} {trailer_status}")
        logger.info(f"State: {status['state']}")
        logger.info(f"Position: ({robot.pose.x:.0f}, {robot.pose.y:.0f})")
        logger.info(f"Battery: {robot.sensors.battery_voltage:.2f}V")
        logger.info(f"Map: {status['visited_pct']:.1f}% visited, {status['explored_pct']:.1f}% known")
        logger.info(f"Escapes: {status['escapes']} | Targets: {status['frontier_targets']}")

        if robot.sensors.ext_connected:
            s = robot.sensors
            dists = s.get_obstacle_distances()
            logger.info(f"Sensors: F={dists['front']}mm L={dists['left']}mm R={dists['right']}mm")
            logger.info(f"IMU: pitch={s.ext_pitch:.1f} roll={s.ext_roll:.1f} yaw={s.ext_yaw:.1f}")

        logger.info("-" * 40)

    def stop(self):
        """Request stop."""
        self._running = False


async def main():
    """Main entry point."""
    explorer = CozmoExplorer()

    def signal_handler(sig, frame):
        logger.info("Interrupt received, stopping...")
        explorer.stop()

    signal.signal(signal.SIGINT, signal_handler)

    if not await explorer.initialize():
        logger.error("Initialization failed!")
        return 1

    try:
        await explorer.run()
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
