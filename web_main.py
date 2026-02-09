"""
Cozmo Explorer — Web UI Entry Point

Starts a web server on port 8080 and opens the browser.
The user clicks "Start Mapping" in the dashboard to begin a session
(simulator or real robot). No immediate robot connection on launch.

Usage:
    python web_main.py
    python web_main.py --port 9090
"""
import argparse
import asyncio
import logging
import os
import signal
import sys
import time
import webbrowser
from datetime import datetime
from pathlib import Path

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
from memory.spatial_map import SpatialMap
from memory.experience_logger import ExperienceLogger
from memory.state_store import StateStore
from brain.mapper_state_machine import MapperStateMachine, MapperState
from web.server import WebServer

# Configure logging
log_format = '%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s'
log_datefmt = '%H:%M:%S'

logging.basicConfig(
    level=logging.DEBUG,
    format=log_format,
    datefmt=log_datefmt,
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(config.DATA_DIR / 'cozmo.log', mode='w', encoding='utf-8'),
    ]
)
logging.getLogger().handlers[0].setLevel(logging.INFO)

logger = logging.getLogger(__name__)


# Save the original asyncio.sleep before anything patches it
_original_sleep = asyncio.sleep


class WebSession:
    """
    Manages a single mapping session started from the web UI.

    Supports both simulator and real-robot modes. Each session creates
    its own robot, spatial_map, and mapper instances.
    """

    def __init__(self, web_server: WebServer):
        self.web = web_server
        self._running = False
        self._session_task = None

    async def start_session(self, mode: str, world: str, time_scale: float, duration: float):
        """Start a mapping session. Called from the web server's start handler."""
        if self._running:
            return
        self._running = True

        try:
            if mode == "sim":
                await self._run_sim_session(world, time_scale, duration)
            else:
                await self._run_real_session()
        except Exception as e:
            logger.error(f"Session error: {e}", exc_info=True)
        finally:
            self._running = False
            self.web.mode = "idle"
            self.web.set_components(robot=None, mapper=None, spatial_map=None)

    async def stop_session(self):
        """Stop the current session."""
        self._running = False

    async def _run_sim_session(self, world_name: str, time_scale: float, duration: float):
        """Run a simulator session — mirrors run_full_sim.run_room() logic."""
        from simulator.sim_robot import SimRobot
        from simulator.run_full_sim import WORLD_MAP, make_accelerated_sleep

        logger.info(f"Starting sim session: world={world_name} time_scale={time_scale}x duration={duration}s")

        # Build world
        world_fn = WORLD_MAP.get(world_name)
        if not world_fn:
            logger.error(f"Unknown world: {world_name}. Options: {list(WORLD_MAP.keys())}")
            return
        world = world_fn()

        # Create sim robot
        sim_robot = SimRobot(world)
        await sim_robot.start()

        # Load or create spatial map
        map_path = config.DATA_DIR / f"sim_map_web_{world_name}.npz"
        if map_path.exists():
            spatial_map = SpatialMap.load(str(map_path))
            logger.info(f"Loaded existing map: {spatial_map.get_visited_percentage()*100:.1f}% visited")
        else:
            spatial_map = SpatialMap()

        # Create state store + logger
        sim_db_path = config.DATA_DIR / "sim_state.db"
        state_store = StateStore(db_path=str(sim_db_path))
        state_store.connect()
        experience_logger = ExperienceLogger(db_path=str(sim_db_path))
        experience_logger.connect()

        session_id = state_store.start_session()
        experience_logger.set_session_id(session_id)
        experience_logger.set_room_id(f"web_{world_name}")

        # Create mapper
        mapper = MapperStateMachine(
            robot=sim_robot,
            spatial_map=spatial_map,
            experience_logger=experience_logger,
        )

        # Wire up web server
        self.web.set_components(robot=sim_robot, mapper=mapper, spatial_map=spatial_map)

        # Patch asyncio.sleep for time acceleration
        if time_scale != 1.0:
            asyncio.sleep = make_accelerated_sleep(time_scale)
            logger.info(f"Time acceleration: {time_scale}x")

        # Start mapper
        sm_task = asyncio.create_task(mapper.start())

        room_start = time.monotonic()
        try:
            while self._running:
                wall_now = time.monotonic()
                elapsed_sim = (wall_now - room_start) * time_scale

                if elapsed_sim >= duration:
                    logger.info(f"Duration reached: {elapsed_sim:.0f}s sim")
                    break

                if mapper.state == MapperState.DONE:
                    logger.info("Mapper reached DONE state")
                    break

                # Use real sleep for the monitoring loop
                await _original_sleep(1 / 30)

        finally:
            # Restore original sleep
            asyncio.sleep = _original_sleep

            # Stop mapper
            await mapper.stop()
            try:
                await asyncio.wait_for(sm_task, timeout=5.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                sm_task.cancel()

            await sim_robot.shutdown()

            # Save map
            spatial_map.save(str(map_path))

            # End session
            wall_seconds = time.monotonic() - room_start
            state_store.end_session(
                duration=wall_seconds,
                distance=0,
                areas=int(spatial_map.get_visited_percentage() * 100),
                summary=f"Web sim: {world_name}, {time_scale}x, {elapsed_sim:.0f}s sim"
            )

            experience_logger.close()
            state_store.close()

            logger.info(
                f"Sim session complete: {elapsed_sim:.0f}s sim, {wall_seconds:.1f}s wall, "
                f"{spatial_map.get_visited_percentage()*100:.1f}% visited"
            )

    async def _run_real_session(self):
        """Run a real-robot session — follows main.py CozmoExplorer pattern."""
        logger.info("Starting real robot session")

        # Import here to avoid pycozmo dependency when running sim-only
        from cozmo_interface.robot import CozmoRobot
        from perception.external_sensors import ExternalSensorReader
        from brain.session_reviewer import SessionReviewer
        from memory.map_annotations import MapAnnotationStore
        import math

        robot = CozmoRobot()
        spatial_map = SpatialMap()

        state_store = StateStore()
        state_store.connect()

        experience_logger = ExperienceLogger()
        experience_logger.connect()

        annotation_store = MapAnnotationStore()
        annotation_store.connect()

        # Load previous map
        map_file = config.DATA_DIR / "spatial_map.npz"
        if map_file.exists():
            try:
                spatial_map = SpatialMap.load(str(map_file))
                logger.info(f"Loaded previous map: {spatial_map.get_visited_percentage()*100:.1f}% visited")
            except Exception as e:
                logger.warning(f"Could not load map: {e}")

        # LLM (post-session review)
        session_reviewer = None
        llm = None
        if config.ENABLE_LLM:
            try:
                from llm.client import LLMClient
                llm = LLMClient()
                await llm.connect()
                if await llm.health_check():
                    session_reviewer = SessionReviewer(llm)
                    logger.info(f"LLM ready: {llm.model}")
            except Exception as e:
                logger.warning(f"LLM init failed: {e}")

        # Connect to robot
        logger.info("Connecting to Cozmo...")
        connected = await robot.connect()
        if not connected:
            logger.error("Failed to connect to Cozmo!")
            return

        logger.info("Robot connected!")

        # External sensors
        ext_sensors = None
        ext_mode = os.environ.get("EXT_SENSOR_MODE", config.EXT_SENSOR_MODE)
        ext_udp_port = int(os.environ.get("EXT_SENSOR_UDP_PORT", config.EXT_SENSOR_UDP_PORT))

        if ext_mode == "udp":
            ext_sensors = ExternalSensorReader(mode="udp", udp_port=ext_udp_port)
        else:
            ext_port = os.environ.get("EXT_SENSOR_PORT", config.EXT_SENSOR_PORT)
            ext_sensors = ExternalSensorReader(mode="serial", port=ext_port, baud=config.EXT_SENSOR_BAUD)

        if ext_sensors.start():
            logger.info("External sensors connected!")
            robot._sensors.ext_connected = True

            def on_ext_reading(reading):
                s = robot._sensors
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

            ext_sensors.set_callback(on_ext_reading)
        else:
            logger.warning("External sensors not available")

        # Create mapper
        mapper = MapperStateMachine(
            robot=robot,
            spatial_map=spatial_map,
            experience_logger=experience_logger,
            session_reviewer=session_reviewer,
            annotation_store=annotation_store,
        )

        # Wire up web server
        self.web.set_components(robot=robot, mapper=mapper, spatial_map=spatial_map)

        session_id = state_store.start_session()
        experience_logger.set_session_id(session_id)

        # Start
        await robot.set_head_angle(0.35)
        sm_task = asyncio.create_task(mapper.start())

        try:
            while self._running:
                await asyncio.sleep(0.5)
                robot._update_pose_from_client()
                if mapper.state == MapperState.DONE:
                    logger.info("Mapping complete!")
                    break
        finally:
            await mapper.stop()
            try:
                await asyncio.wait_for(sm_task, timeout=5.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                sm_task.cancel()

            # Save state
            state_store.save_robot_state(
                position={'x': robot.pose.x, 'y': robot.pose.y, 'angle': robot.pose.angle},
                battery=robot.sensors.battery_voltage,
                exploration_time=mapper.mapping_time,
            )
            state_store.end_session(
                duration=(time.time() - (self.web.start_time or time.time())),
                distance=mapper.mapping_time * 50,
                areas=int(spatial_map.get_visited_percentage() * 100),
                summary=f"Web real-robot session",
            )

            spatial_map.save(str(config.DATA_DIR / "spatial_map.npz"))

            if ext_sensors:
                ext_sensors.stop()
            await robot.disconnect()
            if llm:
                await llm.close()
            experience_logger.close()
            annotation_store.close()
            state_store.close()

            logger.info("Real robot session complete")


async def run(port: int):
    """Main async entry point."""
    web_server = WebServer(port=port)
    session = WebSession(web_server)

    # Wire callbacks
    web_server.on_start = session.start_session
    web_server.on_stop = session.stop_session
    web_server.on_pause = lambda: None  # Handled directly by server via mapper.pause/resume

    # Start web server
    await web_server.start()

    # Open browser
    webbrowser.open(f"http://localhost:{port}")
    logger.info(f"Dashboard: http://localhost:{port}")

    # Keep running until interrupted
    stop_event = asyncio.Event()

    def signal_handler(sig, frame):
        logger.info("Interrupt received, shutting down...")
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        await stop_event.wait()
    finally:
        await session.stop_session()
        await web_server.shutdown()

    logger.info("Shutdown complete")


def main():
    parser = argparse.ArgumentParser(description="Cozmo Explorer Web Dashboard")
    parser.add_argument("--port", type=int, default=config.WEB_PORT,
                        help=f"Web server port (default: {config.WEB_PORT})")
    args = parser.parse_args()
    asyncio.run(run(args.port))


if __name__ == "__main__":
    main()
