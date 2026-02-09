"""
Web server for Cozmo Explorer dashboard.

aiohttp-based, integrates into the existing asyncio event loop via AppRunner.
Provides REST API + SSE for live status updates + PNG map rendering.
"""
import asyncio
import json
import logging
import time
from collections import deque
from pathlib import Path
from typing import Optional

from aiohttp import web

from web.map_renderer import render_map_png

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"


class WebServer:
    """
    aiohttp web server for the Cozmo Explorer dashboard.

    Takes references to live system components (robot, mapper, spatial_map)
    and exposes their state via HTTP endpoints. Reads state directly from
    the objects (same process, no serialization overhead).
    """

    def __init__(self, port: int = 8080):
        self.port = port
        self._app: Optional[web.Application] = None
        self._runner: Optional[web.AppRunner] = None
        self._site: Optional[web.TCPSite] = None

        # References to live system components — set via set_components()
        self.robot = None
        self.mapper = None
        self.spatial_map = None

        # Session control — web_main.py sets these callbacks
        self.on_start = None   # async callable(mode, world, time_scale, duration)
        self.on_stop = None    # async callable()
        self.on_pause = None   # async callable()

        # Session state
        self.mode = "idle"        # "idle", "running", "paused"
        self.session_mode = None  # "real" or "sim"
        self.start_time = None    # wall-clock start

        # Log ring buffer (last 200 lines)
        self._log_lines: deque = deque(maxlen=200)
        self._log_handler: Optional[logging.Handler] = None

        # SSE clients
        self._sse_clients: list = []

        # Use unpatched sleep for SSE timing (sim patches asyncio.sleep)
        # Import the saved original from run_full_sim if available,
        # otherwise use current asyncio.sleep (which is fine for real-robot mode)
        try:
            from simulator.run_full_sim import _original_sleep
            self._real_sleep = _original_sleep
        except ImportError:
            self._real_sleep = asyncio.sleep

    def set_components(self, robot=None, mapper=None, spatial_map=None):
        """Set references to live system components."""
        if robot is not None:
            self.robot = robot
        if mapper is not None:
            self.mapper = mapper
        if spatial_map is not None:
            self.spatial_map = spatial_map

    async def start(self):
        """Start the web server (non-blocking, joins existing event loop)."""
        self._app = web.Application()
        self._setup_routes()
        self._install_log_handler()

        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, "0.0.0.0", self.port)
        await self._site.start()
        logger.info(f"Web server started on http://localhost:{self.port}")

    async def shutdown(self):
        """Stop the web server."""
        # Close SSE connections
        for queue in self._sse_clients:
            await queue.put(None)
        self._sse_clients.clear()

        if self._runner:
            await self._runner.cleanup()
        self._remove_log_handler()
        logger.info("Web server stopped")

    def _setup_routes(self):
        """Register all route handlers."""
        self._app.router.add_get("/", self._handle_index)
        self._app.router.add_get("/api/status", self._handle_status)
        self._app.router.add_get("/api/events", self._handle_sse)
        self._app.router.add_get("/api/map.png", self._handle_map_png)
        self._app.router.add_get("/api/log", self._handle_log)
        self._app.router.add_post("/api/start", self._handle_start)
        self._app.router.add_post("/api/stop", self._handle_stop)
        self._app.router.add_post("/api/pause", self._handle_pause)

    # ---- Log capture ----

    def _install_log_handler(self):
        """Capture log lines into the ring buffer for the dashboard."""
        handler = _RingBufferHandler(self._log_lines)
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s',
            datefmt='%H:%M:%S'
        ))
        logging.getLogger().addHandler(handler)
        self._log_handler = handler

    def _remove_log_handler(self):
        if self._log_handler:
            logging.getLogger().removeHandler(self._log_handler)
            self._log_handler = None

    # ---- Status snapshot ----

    def _build_status(self) -> dict:
        """Build a JSON-serializable status snapshot."""
        status = {
            "mode": self.mode,
            "session_mode": self.session_mode,
            "uptime": 0,
        }

        if self.start_time:
            status["uptime"] = time.time() - self.start_time

        # Mapper state
        if self.mapper:
            ms = self.mapper.get_status()
            status["mapper"] = ms
        else:
            status["mapper"] = None

        # Spatial map stats
        if self.spatial_map:
            status["map"] = {
                "explored_pct": self.spatial_map.get_exploration_progress() * 100,
                "visited_pct": self.spatial_map.get_visited_percentage() * 100,
            }
        else:
            status["map"] = None

        # Robot pose + sensors
        if self.robot:
            status["robot"] = {
                "x": round(self.robot.pose.x, 1),
                "y": round(self.robot.pose.y, 1),
                "angle_deg": round(self.robot.pose.angle * 57.2958, 1),
            }
            s = self.robot.sensors
            status["sensors"] = {
                "battery_v": round(s.battery_voltage, 2),
                "front_mm": s.get_front_obstacle_distance(),
                "left_mm": s.ext_ultra_l_mm if hasattr(s, 'ext_ultra_l_mm') else -1,
                "right_mm": s.ext_ultra_r_mm if hasattr(s, 'ext_ultra_r_mm') else -1,
                "pitch": round(s.ext_pitch, 1),
                "roll": round(s.ext_roll, 1),
                "connected": s.ext_connected,
            }
        else:
            status["robot"] = None
            status["sensors"] = None

        return status

    # ---- Route handlers ----

    async def _handle_index(self, request: web.Request) -> web.Response:
        """Serve the dashboard HTML."""
        html_path = STATIC_DIR / "index.html"
        if not html_path.exists():
            return web.Response(text="Dashboard not found", status=404)
        return web.FileResponse(html_path)

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Return current status as JSON."""
        return web.json_response(self._build_status())

    async def _handle_sse(self, request: web.Request) -> web.StreamResponse:
        """Server-Sent Events stream — pushes status at ~2Hz."""
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'text/event-stream',
                'Cache-Control': 'no-cache',
                'Connection': 'keep-alive',
                'X-Accel-Buffering': 'no',
            }
        )
        await response.prepare(request)

        queue: asyncio.Queue = asyncio.Queue()
        self._sse_clients.append(queue)

        # Start a background pusher for this client
        push_task = asyncio.ensure_future(self._sse_pusher(queue))

        try:
            while True:
                data = await queue.get()
                if data is None:
                    break
                await response.write(f"data: {data}\n\n".encode('utf-8'))
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            push_task.cancel()
            if queue in self._sse_clients:
                self._sse_clients.remove(queue)

        return response

    async def _sse_pusher(self, queue: asyncio.Queue):
        """Push status JSON into an SSE client queue at ~2Hz."""
        try:
            while True:
                status = self._build_status()
                await queue.put(json.dumps(status))
                await self._real_sleep(0.5)
        except asyncio.CancelledError:
            pass

    async def _handle_map_png(self, request: web.Request) -> web.Response:
        """Render and return the occupancy grid as PNG."""
        if self.spatial_map is None:
            return web.Response(text="No map available", status=404)

        robot_gx, robot_gy = None, None
        if self.robot:
            robot_gx, robot_gy = self.spatial_map.world_to_grid(
                self.robot.pose.x, self.robot.pose.y
            )

        png_bytes = render_map_png(
            self.spatial_map.grid,
            robot_gx=robot_gx,
            robot_gy=robot_gy,
        )

        return web.Response(
            body=png_bytes,
            content_type='image/png',
            headers={'Cache-Control': 'no-cache'},
        )

    async def _handle_log(self, request: web.Request) -> web.Response:
        """Return last N log lines as JSON array."""
        n = int(request.query.get('n', '50'))
        lines = list(self._log_lines)[-n:]
        return web.json_response(lines)

    async def _handle_start(self, request: web.Request) -> web.Response:
        """Start a mapping session."""
        if self.mode == "running":
            return web.json_response({"error": "Already running"}, status=400)

        try:
            body = await request.json()
        except Exception:
            body = {}

        mode = body.get("mode", "sim")        # "real" or "sim"
        world = body.get("world", "furnished_room")
        time_scale = float(body.get("time_scale", 10))
        duration = float(body.get("duration", 3600))

        if self.on_start:
            self.mode = "running"
            self.session_mode = mode
            self.start_time = time.time()
            # Fire and forget — the callback runs the session
            asyncio.ensure_future(self.on_start(mode, world, time_scale, duration))
            return web.json_response({"status": "started", "mode": mode, "world": world})
        else:
            return web.json_response({"error": "No start handler"}, status=500)

    async def _handle_stop(self, request: web.Request) -> web.Response:
        """Stop the current session."""
        if self.on_stop:
            await self.on_stop()
            self.mode = "idle"
            return web.json_response({"status": "stopped"})
        return web.json_response({"error": "No stop handler"}, status=500)

    async def _handle_pause(self, request: web.Request) -> web.Response:
        """Toggle pause on the mapper."""
        if self.mapper:
            if self.mapper.is_paused:
                self.mapper.resume()
                self.mode = "running"
                return web.json_response({"status": "resumed"})
            else:
                self.mapper.pause()
                self.mode = "paused"
                return web.json_response({"status": "paused"})
        return web.json_response({"error": "No mapper"}, status=400)


class _RingBufferHandler(logging.Handler):
    """Logging handler that appends formatted messages to a deque."""

    def __init__(self, buffer: deque):
        super().__init__()
        self._buffer = buffer

    def emit(self, record):
        try:
            msg = self.format(record)
            self._buffer.append(msg)
        except Exception:
            pass
