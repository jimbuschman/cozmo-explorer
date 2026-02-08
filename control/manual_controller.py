"""
Manual Controller for Cozmo

A tkinter-based GUI for manually controlling the Cozmo robot.
Uses the same robot interface as the autonomous system, so manual
driving generates training data for the learning system.

Keyboard Controls:
    Arrow Up/Down     - Forward/Backward
    Arrow Left/Right  - Turn left/right (arc turn in trailer mode)
    A/D               - Gentle arc turns
    Q/E               - Reverse arc turns
    Page Up/Down      - Head up/down
    R/F               - Lift up/down
    Space             - Stop
    M                 - Toggle Manual/Auto mode
    L                 - Start/stop logging
    Escape            - Exit
"""
import asyncio
import logging
import tkinter as tk
from tkinter import ttk
from typing import Optional, TYPE_CHECKING
from datetime import datetime

import config

if TYPE_CHECKING:
    from cozmo_interface.robot import CozmoRobot
    from memory.experience_logger import ExperienceLogger
    pass

logger = logging.getLogger(__name__)


class ManualController:
    """
    Tkinter-based manual control interface for Cozmo.

    Key features:
    - Direct keyboard control of movement
    - Live sensor display
    - Mode switching between manual and autonomous
    - Actions are logged for the learning system
    """

    # Control parameters
    DRIVE_SPEED = 50.0      # mm/s
    TURN_SPEED = 30.0       # deg/s
    ARC_SPEED = 50.0        # mm/s for arc turns
    HEAD_STEP = 0.1         # radians per key press
    LIFT_STEP = 0.1         # ratio per key press

    # Head angle limits (radians)
    HEAD_MIN = -0.44
    HEAD_MAX = 0.78

    def __init__(
        self,
        robot: "CozmoRobot",
        experience_logger: "ExperienceLogger" = None,
        rules_store: "LearnedRulesStore" = None,
        on_mode_change: callable = None
    ):
        """
        Initialize the manual controller.

        Args:
            robot: CozmoRobot instance to control
            experience_logger: Optional experience logger for training data
            rules_store: Optional rules store for displaying learning status
            on_mode_change: Callback when mode changes (manual/auto)
        """
        self.robot = robot
        self.experience_logger = experience_logger
        self.rules_store = rules_store
        self.on_mode_change = on_mode_change

        self._root: Optional[tk.Tk] = None
        self._running = False
        self._manual_mode = True
        self._logging_enabled = True
        self._current_head_angle = 0.0
        self._current_lift_height = 0.0

        # Track key states for continuous movement
        self._keys_pressed = set()

        # Sensor display labels
        self._labels = {}

        # Async loop reference
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def start(self, loop: asyncio.AbstractEventLoop):
        """
        Start the manual controller GUI.

        Args:
            loop: The asyncio event loop to use for robot commands
        """
        self._loop = loop
        self._running = True

        # Create main window
        self._root = tk.Tk()
        self._root.title("Cozmo Manual Control")
        self._root.geometry("500x600")
        self._root.configure(bg='#2b2b2b')

        # Build the UI
        self._build_ui()

        # Bind keyboard events
        self._bind_keys()

        # Start sensor update loop
        self._update_sensors()

        logger.info("Manual controller started")

    def _build_ui(self):
        """Build the tkinter UI"""
        root = self._root

        # Style configuration
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', background='#2b2b2b', foreground='white', font=('Consolas', 10))
        style.configure('TFrame', background='#2b2b2b')
        style.configure('Header.TLabel', font=('Consolas', 12, 'bold'))
        style.configure('Status.TLabel', font=('Consolas', 11))

        # === Mode Indicator ===
        mode_frame = ttk.Frame(root)
        mode_frame.pack(fill='x', padx=10, pady=5)

        self._labels['mode'] = ttk.Label(
            mode_frame,
            text="MANUAL MODE",
            style='Header.TLabel',
            foreground='#00ff00'
        )
        self._labels['mode'].pack(side='left')

        trailer_text = "TRAILER" if config.TRAILER_MODE else "NORMAL"
        trailer_color = '#ffaa00' if config.TRAILER_MODE else '#888888'
        self._labels['trailer'] = ttk.Label(
            mode_frame,
            text=trailer_text,
            style='Status.TLabel',
            foreground=trailer_color
        )
        self._labels['trailer'].pack(side='right')

        # === Battery & Pose ===
        info_frame = ttk.Frame(root)
        info_frame.pack(fill='x', padx=10, pady=5)

        self._labels['battery'] = ttk.Label(info_frame, text="Battery: --V", style='TLabel')
        self._labels['battery'].pack(side='left')

        self._labels['pose'] = ttk.Label(info_frame, text="Pose: (0, 0) 0.0", style='TLabel')
        self._labels['pose'].pack(side='right')

        # === Separator ===
        ttk.Separator(root, orient='horizontal').pack(fill='x', padx=10, pady=5)

        # === Obstacle Distances ===
        obstacle_frame = ttk.LabelFrame(root, text="Obstacle Distances (mm)")
        obstacle_frame.pack(fill='x', padx=10, pady=5)

        dist_inner = ttk.Frame(obstacle_frame)
        dist_inner.pack(fill='x', padx=5, pady=5)

        self._labels['dist_left'] = ttk.Label(dist_inner, text="L: --", style='TLabel', width=10)
        self._labels['dist_left'].pack(side='left')

        self._labels['dist_front'] = ttk.Label(dist_inner, text="F: --", style='TLabel', width=10)
        self._labels['dist_front'].pack(side='left', padx=20)

        self._labels['dist_right'] = ttk.Label(dist_inner, text="R: --", style='TLabel', width=10)
        self._labels['dist_right'].pack(side='left')

        # === IMU Data ===
        imu_frame = ttk.LabelFrame(root, text="IMU (degrees)")
        imu_frame.pack(fill='x', padx=10, pady=5)

        imu_inner = ttk.Frame(imu_frame)
        imu_inner.pack(fill='x', padx=5, pady=5)

        self._labels['pitch'] = ttk.Label(imu_inner, text="Pitch: --", style='TLabel', width=12)
        self._labels['pitch'].pack(side='left')

        self._labels['roll'] = ttk.Label(imu_inner, text="Roll: --", style='TLabel', width=12)
        self._labels['roll'].pack(side='left')

        self._labels['yaw'] = ttk.Label(imu_inner, text="Yaw: --", style='TLabel', width=12)
        self._labels['yaw'].pack(side='left')

        # === Learning Status ===
        learning_frame = ttk.LabelFrame(root, text="Learning System")
        learning_frame.pack(fill='x', padx=10, pady=5)

        learn_inner = ttk.Frame(learning_frame)
        learn_inner.pack(fill='x', padx=5, pady=5)

        self._labels['samples'] = ttk.Label(learn_inner, text="Samples: --", style='TLabel')
        self._labels['samples'].pack(side='left')

        self._labels['rules'] = ttk.Label(learn_inner, text="Rules: --", style='TLabel')
        self._labels['rules'].pack(side='right')

        logging_color = '#00ff00' if self._logging_enabled else '#ff0000'
        self._labels['logging'] = ttk.Label(
            learning_frame,
            text="Logging: ON" if self._logging_enabled else "Logging: OFF",
            style='TLabel',
            foreground=logging_color
        )
        self._labels['logging'].pack(padx=5, pady=2)

        # === Separator ===
        ttk.Separator(root, orient='horizontal').pack(fill='x', padx=10, pady=5)

        # === Controls Help ===
        help_frame = ttk.LabelFrame(root, text="Controls")
        help_frame.pack(fill='x', padx=10, pady=5)

        help_text = """
Arrow Keys    Forward/Back/Turn
A/D           Gentle Arc Turns
Q/E           Reverse Arc Turns
PgUp/PgDn     Head Up/Down
R/F           Lift Up/Down
Space         STOP
M             Toggle Mode
L             Toggle Logging
Esc           Exit
"""
        help_label = ttk.Label(help_frame, text=help_text, style='TLabel', justify='left')
        help_label.pack(padx=5, pady=5)

        # === Status Bar ===
        self._labels['status'] = ttk.Label(
            root,
            text="Ready - Use arrow keys to drive",
            style='Status.TLabel',
            foreground='#888888'
        )
        self._labels['status'].pack(side='bottom', fill='x', padx=10, pady=5)

    def _bind_keys(self):
        """Bind keyboard events"""
        root = self._root

        # Key press events
        root.bind('<KeyPress>', self._on_key_press)
        root.bind('<KeyRelease>', self._on_key_release)

        # Specific bindings for immediate response
        root.bind('<Up>', lambda e: self._drive_forward())
        root.bind('<Down>', lambda e: self._drive_backward())
        root.bind('<Left>', lambda e: self._turn_left())
        root.bind('<Right>', lambda e: self._turn_right())
        root.bind('<space>', lambda e: self._stop())
        root.bind('<Escape>', lambda e: self._exit())

        # Arc turns
        root.bind('a', lambda e: self._arc_left())
        root.bind('d', lambda e: self._arc_right())
        root.bind('q', lambda e: self._reverse_arc_left())
        root.bind('e', lambda e: self._reverse_arc_right())

        # Head and lift
        root.bind('<Prior>', lambda e: self._head_up())  # Page Up
        root.bind('<Next>', lambda e: self._head_down())  # Page Down
        root.bind('r', lambda e: self._lift_up())
        root.bind('f', lambda e: self._lift_down())

        # Mode toggle
        root.bind('m', lambda e: self._toggle_mode())
        root.bind('l', lambda e: self._toggle_logging())

        # Focus for key events
        root.focus_set()

    def _on_key_press(self, event):
        """Track key press state"""
        self._keys_pressed.add(event.keysym)

    def _on_key_release(self, event):
        """Track key release and stop if movement key released"""
        self._keys_pressed.discard(event.keysym)

        # Stop if no movement keys are pressed
        movement_keys = {'Up', 'Down', 'Left', 'Right', 'a', 'd', 'q', 'e'}
        if not self._keys_pressed & movement_keys:
            self._stop()

    def _run_async(self, coro):
        """Run an async coroutine from the GUI thread"""
        if self._loop and self._running:
            asyncio.run_coroutine_threadsafe(coro, self._loop)

    def _update_status(self, text: str):
        """Update the status bar"""
        if self._labels.get('status'):
            self._labels['status'].configure(text=text)

    def _log_manual_action(self, action_type: str, parameters: dict):
        """Log a manual action for the learning system"""
        if not self._logging_enabled or not self.experience_logger:
            return

        try:
            # Log sensor snapshot
            snapshot_id = self.experience_logger.log_sensor_snapshot_from_robot(self.robot)

            # Log the action
            parameters['manual'] = True
            parameters['trailer_mode'] = config.TRAILER_MODE
            self.experience_logger.log_action(
                action_type=f"manual_{action_type}",
                parameters=parameters,
                trigger="manual_control",
                context_snapshot_id=snapshot_id
            )
        except Exception as e:
            logger.debug(f"Failed to log manual action: {e}")

    # === Movement Commands ===

    def _drive_forward(self):
        if not self._manual_mode:
            return
        self._update_status("Driving forward...")
        self._log_manual_action("drive", {"direction": "forward", "speed": self.DRIVE_SPEED})
        self._run_async(self.robot.drive(self.DRIVE_SPEED))

    def _drive_backward(self):
        if not self._manual_mode:
            return
        self._update_status("Driving backward...")
        self._log_manual_action("drive", {"direction": "backward", "speed": self.DRIVE_SPEED})
        self._run_async(self.robot.drive(-self.DRIVE_SPEED))

    def _turn_left(self):
        if not self._manual_mode:
            return
        self._update_status("Turning left...")
        self._log_manual_action("turn", {"direction": "left", "angle": 15})
        # robot.turn() will automatically use arc turn if TRAILER_MODE is on
        self._run_async(self.robot.turn(15, self.TURN_SPEED))

    def _turn_right(self):
        if not self._manual_mode:
            return
        self._update_status("Turning right...")
        self._log_manual_action("turn", {"direction": "right", "angle": -15})
        self._run_async(self.robot.turn(-15, self.TURN_SPEED))

    def _arc_left(self):
        if not self._manual_mode:
            return
        ratio = config.TRAILER_ARC_RATIO
        self._update_status(f"Arc turn left (ratio={ratio})...")
        self._log_manual_action("arc_turn", {"direction": "left", "ratio": ratio})
        self._run_async(self.robot.arc_turn_left(self.ARC_SPEED, ratio, 0.5))

    def _arc_right(self):
        if not self._manual_mode:
            return
        ratio = config.TRAILER_ARC_RATIO
        self._update_status(f"Arc turn right (ratio={ratio})...")
        self._log_manual_action("arc_turn", {"direction": "right", "ratio": ratio})
        self._run_async(self.robot.arc_turn_right(self.ARC_SPEED, ratio, 0.5))

    def _reverse_arc_left(self):
        if not self._manual_mode:
            return
        ratio = config.TRAILER_ARC_RATIO
        self._update_status(f"Reverse arc left (ratio={ratio})...")
        self._log_manual_action("reverse_arc_turn", {"direction": "left", "ratio": ratio})
        self._run_async(self.robot.reverse_arc_turn_left(self.ARC_SPEED, ratio, 0.5))

    def _reverse_arc_right(self):
        if not self._manual_mode:
            return
        ratio = config.TRAILER_ARC_RATIO
        self._update_status(f"Reverse arc right (ratio={ratio})...")
        self._log_manual_action("reverse_arc_turn", {"direction": "right", "ratio": ratio})
        self._run_async(self.robot.reverse_arc_turn_right(self.ARC_SPEED, ratio, 0.5))

    def _stop(self):
        self._update_status("Stopped")
        self._run_async(self.robot.stop())

    # === Head and Lift ===

    def _head_up(self):
        if not self._manual_mode:
            return
        self._current_head_angle = min(self.HEAD_MAX, self._current_head_angle + self.HEAD_STEP)
        self._update_status(f"Head: {self._current_head_angle:.2f} rad")
        self._run_async(self.robot.set_head_angle(self._current_head_angle))

    def _head_down(self):
        if not self._manual_mode:
            return
        self._current_head_angle = max(self.HEAD_MIN, self._current_head_angle - self.HEAD_STEP)
        self._update_status(f"Head: {self._current_head_angle:.2f} rad")
        self._run_async(self.robot.set_head_angle(self._current_head_angle))

    def _lift_up(self):
        if not self._manual_mode:
            return
        self._current_lift_height = min(1.0, self._current_lift_height + self.LIFT_STEP)
        self._update_status(f"Lift: {self._current_lift_height:.1%}")
        self._run_async(self.robot.set_lift_height(self._current_lift_height))

    def _lift_down(self):
        if not self._manual_mode:
            return
        self._current_lift_height = max(0.0, self._current_lift_height - self.LIFT_STEP)
        self._update_status(f"Lift: {self._current_lift_height:.1%}")
        self._run_async(self.robot.set_lift_height(self._current_lift_height))

    # === Mode Switching ===

    def _toggle_mode(self):
        """Toggle between manual and autonomous mode"""
        self._manual_mode = not self._manual_mode

        if self._manual_mode:
            self._labels['mode'].configure(text="MANUAL MODE", foreground='#00ff00')
            self._update_status("Switched to MANUAL mode - use keyboard to drive")
        else:
            self._labels['mode'].configure(text="AUTO MODE", foreground='#0088ff')
            self._update_status("Switched to AUTO mode - robot drives itself")
            self._stop()  # Stop any current movement

        # Notify callback
        if self.on_mode_change:
            self.on_mode_change(self._manual_mode)

        logger.info(f"Mode changed to {'MANUAL' if self._manual_mode else 'AUTO'}")

    def _toggle_logging(self):
        """Toggle learning data logging"""
        self._logging_enabled = not self._logging_enabled

        if self._logging_enabled:
            self._labels['logging'].configure(text="Logging: ON", foreground='#00ff00')
            self._update_status("Logging ENABLED - actions will be recorded")
        else:
            self._labels['logging'].configure(text="Logging: OFF", foreground='#ff0000')
            self._update_status("Logging DISABLED - actions NOT recorded")

        logger.info(f"Logging {'enabled' if self._logging_enabled else 'disabled'}")

    def _exit(self):
        """Exit the controller"""
        self._running = False
        self._stop()
        if self._root:
            self._root.quit()

    # === Sensor Updates ===

    def _update_sensors(self):
        """Update sensor display (called periodically)"""
        if not self._running or not self._root:
            return

        try:
            sensors = self.robot.sensors
            pose = self.robot.pose

            # Battery (color coded)
            voltage = sensors.battery_voltage
            if voltage > 0:
                if voltage < config.LOW_BATTERY_VOLTAGE:
                    color = '#ff0000'
                elif voltage < config.LOW_BATTERY_VOLTAGE + 0.2:
                    color = '#ffaa00'
                else:
                    color = '#00ff00'
                self._labels['battery'].configure(text=f"Battery: {voltage:.2f}V", foreground=color)

            # Pose
            angle_deg = pose.angle * 57.2958  # rad to deg
            self._labels['pose'].configure(
                text=f"Pose: ({pose.x:.0f}, {pose.y:.0f}) {angle_deg:.1f}\u00b0"
            )

            # Obstacle distances
            if sensors.ext_connected:
                dists = sensors.get_obstacle_distances()
                self._labels['dist_left'].configure(text=f"L: {dists['left']}")
                self._labels['dist_front'].configure(text=f"F: {dists['front']}")
                self._labels['dist_right'].configure(text=f"R: {dists['right']}")

                # IMU
                self._labels['pitch'].configure(text=f"Pitch: {sensors.ext_pitch:.1f}")
                self._labels['roll'].configure(text=f"Roll: {sensors.ext_roll:.1f}")
                self._labels['yaw'].configure(text=f"Yaw: {sensors.ext_yaw:.1f}")
            else:
                self._labels['dist_left'].configure(text="L: N/A")
                self._labels['dist_front'].configure(text="F: N/A")
                self._labels['dist_right'].configure(text="R: N/A")
                self._labels['pitch'].configure(text="Pitch: N/A")
                self._labels['roll'].configure(text="Roll: N/A")
                self._labels['yaw'].configure(text="Yaw: N/A")

            # Learning status
            if self.experience_logger:
                try:
                    stats = self.experience_logger.get_recovery_statistics("escape_stall")
                    total_samples = stats.get('total', 0)
                    self._labels['samples'].configure(text=f"Samples: {total_samples}")
                except:
                    pass

            if self.rules_store:
                try:
                    rules = self.rules_store.get_all_rules()
                    active_count = sum(1 for r in rules if r.get('status') == 'active')
                    self._labels['rules'].configure(text=f"Rules: {active_count} active")
                except:
                    pass

        except Exception as e:
            logger.debug(f"Sensor update error: {e}")

        # Schedule next update
        if self._running and self._root:
            self._root.after(100, self._update_sensors)  # 10Hz update rate

    @property
    def is_manual_mode(self) -> bool:
        """Check if currently in manual mode"""
        return self._manual_mode

    @property
    def is_running(self) -> bool:
        """Check if controller is running"""
        return self._running

    def run_gui(self):
        """Run the tkinter main loop (blocking)"""
        if self._root:
            self._root.mainloop()

    def stop(self):
        """Stop the controller"""
        self._running = False
        self._stop()
        if self._root:
            self._root.quit()
            self._root.destroy()
            self._root = None
        logger.info("Manual controller stopped")
