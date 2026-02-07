"""
Experience Logger

Time-series logging of sensor data, actions, and outcomes for learning.
Stores data in SQLite tables for pattern analysis.
"""
import sqlite3
import logging
import json
import uuid
from typing import Optional, List, Dict, Any
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, asdict

import config

logger = logging.getLogger(__name__)


@dataclass
class SensorSnapshot:
    """Point-in-time sensor reading"""
    id: Optional[int] = None
    session_id: Optional[int] = None
    timestamp: Optional[datetime] = None
    # Pose
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_angle: float = 0.0
    # Internal sensors
    battery_voltage: float = 0.0
    cliff_detected: bool = False
    is_picked_up: bool = False
    is_on_charger: bool = False
    # Head and lift position
    head_angle: float = 0.0  # radians
    lift_height: float = 0.0  # mm
    # Accelerometer (internal)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    # Gyroscope (internal)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # External sensors (distances)
    ext_connected: bool = False
    ext_tof_mm: int = 0
    ext_ultra_l_mm: int = 0
    ext_ultra_c_mm: int = 0
    ext_ultra_r_mm: int = 0
    # External IMU (orientation)
    ext_pitch: float = 0.0
    ext_roll: float = 0.0
    ext_yaw: float = 0.0
    # External IMU (raw accelerometer in g's)
    ext_ax_g: float = 0.0
    ext_ay_g: float = 0.0
    ext_az_g: float = 0.0
    # External IMU (raw gyroscope in degrees/sec)
    ext_gx_dps: float = 0.0
    ext_gy_dps: float = 0.0
    ext_gz_dps: float = 0.0
    # External timestamp
    ext_ts_ms: int = 0
    # Image reference (path to captured image, if any)
    image_path: Optional[str] = None

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        d = asdict(self)
        if d.get('timestamp'):
            d['timestamp'] = d['timestamp'].isoformat()
        return d


@dataclass
class ActionEvent:
    """Record of an action taken"""
    id: Optional[int] = None
    session_id: Optional[int] = None
    timestamp: Optional[datetime] = None
    action_type: str = ""  # "escape_stall", "escape_cliff", "turn", "drive", etc.
    parameters: Optional[dict] = None  # e.g., {"angle": -90, "speed": 50}
    trigger: str = ""  # What triggered this action: "collision", "cliff", "random", "llm"
    context_snapshot_id: Optional[int] = None  # FK to sensor_snapshots


@dataclass
class OutcomeEvent:
    """Result of an action"""
    id: Optional[int] = None
    session_id: Optional[int] = None
    timestamp: Optional[datetime] = None
    action_event_id: Optional[int] = None  # FK to action_events
    outcome_type: str = ""  # "success", "collision", "stall", "cliff", "timeout"
    details: Optional[dict] = None  # Additional outcome data
    sensor_snapshot_id: Optional[int] = None  # FK to sensor_snapshots (post-action)


class ExperienceLogger:
    """
    SQLite-based logger for robot experiences.

    Records sensor snapshots, actions taken, and their outcomes
    for later pattern analysis and learning.
    """

    def __init__(self, db_path: str = None):
        self.db_path = Path(db_path or config.SQLITE_PATH)
        self._conn: Optional[sqlite3.Connection] = None
        self._session_id: Optional[int] = None
        self._room_id: Optional[str] = None

    def connect(self):
        """Connect to database and create tables"""
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self.db_path))
        self._conn.row_factory = sqlite3.Row
        self._create_tables()
        logger.info(f"ExperienceLogger connected: {self.db_path}")

    def _create_tables(self):
        """Create learning-specific tables if they don't exist"""
        cursor = self._conn.cursor()

        # Sensor snapshots - point-in-time sensor readings
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sensor_snapshots (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER,
                room_id TEXT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                pose_x REAL,
                pose_y REAL,
                pose_angle REAL,
                battery_voltage REAL,
                cliff_detected INTEGER,
                is_picked_up INTEGER,
                is_on_charger INTEGER,
                head_angle REAL,
                lift_height REAL,
                accel_x REAL,
                accel_y REAL,
                accel_z REAL,
                gyro_x REAL,
                gyro_y REAL,
                gyro_z REAL,
                ext_connected INTEGER,
                ext_tof_mm INTEGER,
                ext_ultra_l_mm INTEGER,
                ext_ultra_c_mm INTEGER,
                ext_ultra_r_mm INTEGER,
                ext_pitch REAL,
                ext_roll REAL,
                ext_yaw REAL,
                ext_ax_g REAL,
                ext_ay_g REAL,
                ext_az_g REAL,
                ext_gx_dps REAL,
                ext_gy_dps REAL,
                ext_gz_dps REAL,
                ext_ts_ms INTEGER,
                image_path TEXT
            )
        """)

        # Action events - what actions were taken
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS action_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER,
                room_id TEXT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                action_type TEXT,
                parameters TEXT,
                trigger TEXT,
                context_snapshot_id INTEGER,
                FOREIGN KEY (context_snapshot_id) REFERENCES sensor_snapshots(id)
            )
        """)

        # Outcome events - results of actions
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS outcome_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER,
                room_id TEXT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                action_event_id INTEGER,
                outcome_type TEXT,
                details TEXT,
                sensor_snapshot_id INTEGER,
                FOREIGN KEY (action_event_id) REFERENCES action_events(id),
                FOREIGN KEY (sensor_snapshot_id) REFERENCES sensor_snapshots(id)
            )
        """)

        # Migrate existing tables: add room_id column if missing
        for table in ('sensor_snapshots', 'action_events', 'outcome_events'):
            try:
                cursor.execute(f"ALTER TABLE {table} ADD COLUMN room_id TEXT")
            except sqlite3.OperationalError:
                pass  # Column already exists

        # Create indexes for common queries
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_sensor_snapshots_session
            ON sensor_snapshots(session_id)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_action_events_session
            ON action_events(session_id)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_action_events_type
            ON action_events(action_type)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_outcome_events_action
            ON outcome_events(action_event_id)
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_action_events_room
            ON action_events(room_id)
        """)

        self._conn.commit()

    def close(self):
        """Close database connection"""
        if self._conn:
            self._conn.close()
            self._conn = None

    def set_session_id(self, session_id: int):
        """Set the current session ID for logging"""
        self._session_id = session_id

    def set_room_id(self, room_id: str):
        """Set the current room ID for logging"""
        self._room_id = room_id

    # ==================== Sensor Snapshots ====================

    def log_sensor_snapshot(
        self,
        pose_x: float,
        pose_y: float,
        pose_angle: float,
        battery_voltage: float = 0.0,
        cliff_detected: bool = False,
        is_picked_up: bool = False,
        is_on_charger: bool = False,
        head_angle: float = 0.0,
        lift_height: float = 0.0,
        accel_x: float = 0.0,
        accel_y: float = 0.0,
        accel_z: float = 0.0,
        gyro_x: float = 0.0,
        gyro_y: float = 0.0,
        gyro_z: float = 0.0,
        ext_connected: bool = False,
        ext_tof_mm: int = 0,
        ext_ultra_l_mm: int = 0,
        ext_ultra_c_mm: int = 0,
        ext_ultra_r_mm: int = 0,
        ext_pitch: float = 0.0,
        ext_roll: float = 0.0,
        ext_yaw: float = 0.0,
        ext_ax_g: float = 0.0,
        ext_ay_g: float = 0.0,
        ext_az_g: float = 0.0,
        ext_gx_dps: float = 0.0,
        ext_gy_dps: float = 0.0,
        ext_gz_dps: float = 0.0,
        ext_ts_ms: int = 0,
        image_path: str = None
    ) -> int:
        """Log a sensor snapshot and return its ID"""
        if not self._conn:
            return -1

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT INTO sensor_snapshots (
                session_id, room_id, pose_x, pose_y, pose_angle,
                battery_voltage, cliff_detected, is_picked_up, is_on_charger,
                head_angle, lift_height,
                accel_x, accel_y, accel_z,
                gyro_x, gyro_y, gyro_z,
                ext_connected, ext_tof_mm, ext_ultra_l_mm, ext_ultra_c_mm, ext_ultra_r_mm,
                ext_pitch, ext_roll, ext_yaw,
                ext_ax_g, ext_ay_g, ext_az_g,
                ext_gx_dps, ext_gy_dps, ext_gz_dps,
                ext_ts_ms, image_path
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            self._session_id, self._room_id, pose_x, pose_y, pose_angle,
            battery_voltage, int(cliff_detected), int(is_picked_up), int(is_on_charger),
            head_angle, lift_height,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            int(ext_connected), ext_tof_mm, ext_ultra_l_mm, ext_ultra_c_mm, ext_ultra_r_mm,
            ext_pitch, ext_roll, ext_yaw,
            ext_ax_g, ext_ay_g, ext_az_g,
            ext_gx_dps, ext_gy_dps, ext_gz_dps,
            ext_ts_ms, image_path
        ))
        self._conn.commit()
        return cursor.lastrowid

    def log_sensor_snapshot_from_robot(self, robot, image_path: str = None) -> int:
        """Log a sensor snapshot from a CozmoRobot instance"""
        sensors = robot.sensors
        return self.log_sensor_snapshot(
            pose_x=robot.pose.x,
            pose_y=robot.pose.y,
            pose_angle=robot.pose.angle,
            battery_voltage=sensors.battery_voltage,
            cliff_detected=sensors.cliff_detected,
            is_picked_up=sensors.is_picked_up,
            is_on_charger=sensors.is_on_charger,
            head_angle=sensors.head_angle,
            lift_height=sensors.lift_height,
            accel_x=sensors.accel_x,
            accel_y=sensors.accel_y,
            accel_z=sensors.accel_z,
            gyro_x=sensors.gyro_x,
            gyro_y=sensors.gyro_y,
            gyro_z=sensors.gyro_z,
            ext_connected=sensors.ext_connected,
            ext_tof_mm=sensors.ext_tof_mm,
            ext_ultra_l_mm=sensors.ext_ultra_l_mm,
            ext_ultra_c_mm=sensors.ext_ultra_c_mm,
            ext_ultra_r_mm=sensors.ext_ultra_r_mm,
            ext_pitch=sensors.ext_pitch,
            ext_roll=sensors.ext_roll,
            ext_yaw=sensors.ext_yaw,
            ext_ax_g=getattr(sensors, 'ext_ax_g', 0.0),
            ext_ay_g=getattr(sensors, 'ext_ay_g', 0.0),
            ext_az_g=getattr(sensors, 'ext_az_g', 0.0),
            ext_gx_dps=getattr(sensors, 'ext_gx_dps', 0.0),
            ext_gy_dps=getattr(sensors, 'ext_gy_dps', 0.0),
            ext_gz_dps=getattr(sensors, 'ext_gz_dps', 0.0),
            ext_ts_ms=getattr(sensors, 'ext_ts_ms', 0),
            image_path=image_path
        )

    def get_sensor_snapshot(self, snapshot_id: int) -> Optional[SensorSnapshot]:
        """Get a sensor snapshot by ID"""
        if not self._conn:
            return None

        cursor = self._conn.cursor()
        cursor.execute("SELECT * FROM sensor_snapshots WHERE id = ?", (snapshot_id,))
        row = cursor.fetchone()

        if row:
            return SensorSnapshot(
                id=row['id'],
                session_id=row['session_id'],
                timestamp=datetime.fromisoformat(row['timestamp']) if row['timestamp'] else None,
                pose_x=row['pose_x'],
                pose_y=row['pose_y'],
                pose_angle=row['pose_angle'],
                battery_voltage=row['battery_voltage'],
                cliff_detected=bool(row['cliff_detected']),
                is_picked_up=bool(row['is_picked_up']),
                is_on_charger=bool(row['is_on_charger']),
                head_angle=row['head_angle'] if 'head_angle' in row.keys() else 0.0,
                lift_height=row['lift_height'] if 'lift_height' in row.keys() else 0.0,
                accel_x=row['accel_x'],
                accel_y=row['accel_y'],
                accel_z=row['accel_z'],
                gyro_x=row['gyro_x'] if 'gyro_x' in row.keys() else 0.0,
                gyro_y=row['gyro_y'] if 'gyro_y' in row.keys() else 0.0,
                gyro_z=row['gyro_z'] if 'gyro_z' in row.keys() else 0.0,
                ext_connected=bool(row['ext_connected']),
                ext_tof_mm=row['ext_tof_mm'],
                ext_ultra_l_mm=row['ext_ultra_l_mm'],
                ext_ultra_c_mm=row['ext_ultra_c_mm'],
                ext_ultra_r_mm=row['ext_ultra_r_mm'],
                ext_pitch=row['ext_pitch'],
                ext_roll=row['ext_roll'],
                ext_yaw=row['ext_yaw'],
                image_path=row['image_path'] if 'image_path' in row.keys() else None
            )
        return None

    # ==================== Action Events ====================

    def log_action(
        self,
        action_type: str,
        parameters: dict = None,
        trigger: str = "",
        context_snapshot_id: int = None
    ) -> int:
        """Log an action event and return its ID"""
        if not self._conn:
            return -1

        params_json = json.dumps(parameters) if parameters else None

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT INTO action_events (session_id, room_id, action_type, parameters, trigger, context_snapshot_id)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (self._session_id, self._room_id, action_type, params_json, trigger, context_snapshot_id))
        self._conn.commit()
        return cursor.lastrowid

    def get_actions_by_type(
        self,
        action_type: str,
        session_id: int = None,
        limit: int = 100
    ) -> List[ActionEvent]:
        """Get action events by type"""
        if not self._conn:
            return []

        query = "SELECT * FROM action_events WHERE action_type = ?"
        params = [action_type]

        if session_id is not None:
            query += " AND session_id = ?"
            params.append(session_id)

        query += " ORDER BY timestamp DESC LIMIT ?"
        params.append(limit)

        cursor = self._conn.cursor()
        cursor.execute(query, params)

        actions = []
        for row in cursor.fetchall():
            actions.append(ActionEvent(
                id=row['id'],
                session_id=row['session_id'],
                timestamp=datetime.fromisoformat(row['timestamp']) if row['timestamp'] else None,
                action_type=row['action_type'],
                parameters=json.loads(row['parameters']) if row['parameters'] else None,
                trigger=row['trigger'],
                context_snapshot_id=row['context_snapshot_id']
            ))
        return actions

    def get_recent_actions(self, limit: int = 50) -> List[ActionEvent]:
        """Get most recent actions across all types"""
        if not self._conn:
            return []

        cursor = self._conn.cursor()
        cursor.execute("""
            SELECT * FROM action_events
            ORDER BY timestamp DESC
            LIMIT ?
        """, (limit,))

        actions = []
        for row in cursor.fetchall():
            actions.append(ActionEvent(
                id=row['id'],
                session_id=row['session_id'],
                timestamp=datetime.fromisoformat(row['timestamp']) if row['timestamp'] else None,
                action_type=row['action_type'],
                parameters=json.loads(row['parameters']) if row['parameters'] else None,
                trigger=row['trigger'],
                context_snapshot_id=row['context_snapshot_id']
            ))
        return actions

    # ==================== Outcome Events ====================

    def log_outcome(
        self,
        action_event_id: int,
        outcome_type: str,
        details: dict = None,
        sensor_snapshot_id: int = None
    ) -> int:
        """Log an outcome event and return its ID"""
        if not self._conn:
            return -1

        details_json = json.dumps(details) if details else None

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT INTO outcome_events (session_id, room_id, action_event_id, outcome_type, details, sensor_snapshot_id)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (self._session_id, self._room_id, action_event_id, outcome_type, details_json, sensor_snapshot_id))
        self._conn.commit()
        return cursor.lastrowid

    def get_outcomes_for_action(self, action_event_id: int) -> List[OutcomeEvent]:
        """Get all outcomes for an action"""
        if not self._conn:
            return []

        cursor = self._conn.cursor()
        cursor.execute("""
            SELECT * FROM outcome_events WHERE action_event_id = ?
        """, (action_event_id,))

        outcomes = []
        for row in cursor.fetchall():
            outcomes.append(OutcomeEvent(
                id=row['id'],
                session_id=row['session_id'],
                timestamp=datetime.fromisoformat(row['timestamp']) if row['timestamp'] else None,
                action_event_id=row['action_event_id'],
                outcome_type=row['outcome_type'],
                details=json.loads(row['details']) if row['details'] else None,
                sensor_snapshot_id=row['sensor_snapshot_id']
            ))
        return outcomes

    # ==================== Analysis Queries ====================

    def get_action_outcome_pairs(
        self,
        action_type: str = None,
        outcome_type: str = None,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """
        Get action-outcome pairs for analysis.

        Returns list of dicts with action details, context, and outcomes.
        """
        if not self._conn:
            return []

        query = """
            SELECT
                a.id as action_id,
                a.action_type,
                a.parameters,
                a.trigger,
                a.timestamp as action_time,
                o.outcome_type,
                o.details as outcome_details,
                cs.pose_x, cs.pose_y, cs.pose_angle,
                cs.ext_ultra_l_mm, cs.ext_ultra_c_mm, cs.ext_ultra_r_mm,
                cs.ext_tof_mm, cs.ext_pitch, cs.ext_roll
            FROM action_events a
            LEFT JOIN outcome_events o ON a.id = o.action_event_id
            LEFT JOIN sensor_snapshots cs ON a.context_snapshot_id = cs.id
            WHERE 1=1
        """
        params = []

        if action_type:
            query += " AND a.action_type = ?"
            params.append(action_type)

        if outcome_type:
            query += " AND o.outcome_type = ?"
            params.append(outcome_type)

        query += " ORDER BY a.timestamp DESC LIMIT ?"
        params.append(limit)

        cursor = self._conn.cursor()
        cursor.execute(query, params)

        results = []
        for row in cursor.fetchall():
            results.append({
                'action_id': row['action_id'],
                'action_type': row['action_type'],
                'parameters': json.loads(row['parameters']) if row['parameters'] else None,
                'trigger': row['trigger'],
                'action_time': row['action_time'],
                'outcome_type': row['outcome_type'],
                'outcome_details': json.loads(row['outcome_details']) if row['outcome_details'] else None,
                'context': {
                    'pose_x': row['pose_x'],
                    'pose_y': row['pose_y'],
                    'pose_angle': row['pose_angle'],
                    'ultra_left': row['ext_ultra_l_mm'],
                    'ultra_center': row['ext_ultra_c_mm'],
                    'ultra_right': row['ext_ultra_r_mm'],
                    'tof': row['ext_tof_mm'],
                    'pitch': row['ext_pitch'],
                    'roll': row['ext_roll']
                }
            })
        return results

    def get_recovery_statistics(self, action_type: str = "escape_stall", room_id: str = None) -> Dict[str, Any]:
        """
        Get statistics on recovery actions (escape_stall, escape_cliff).

        Returns success rates and parameter analysis.
        Optionally filter by room_id for per-room stats.
        """
        if not self._conn:
            return {}

        cursor = self._conn.cursor()

        room_filter = ""
        room_params = []
        if room_id is not None:
            room_filter = " AND a.room_id = ?"
            room_params = [room_id]

        # Total actions of this type
        cursor.execute(f"""
            SELECT COUNT(*) as total FROM action_events a WHERE a.action_type = ?{room_filter}
        """, (action_type, *room_params))
        total = cursor.fetchone()['total']

        if total == 0:
            return {'total': 0, 'success_rate': 0, 'by_angle': {}}

        # Success rate
        cursor.execute(f"""
            SELECT COUNT(*) as success_count
            FROM action_events a
            JOIN outcome_events o ON a.id = o.action_event_id
            WHERE a.action_type = ? AND o.outcome_type = 'success'{room_filter}
        """, (action_type, *room_params))
        success_count = cursor.fetchone()['success_count']

        # Group by turn angle if available
        cursor.execute(f"""
            SELECT
                a.parameters,
                o.outcome_type,
                COUNT(*) as count
            FROM action_events a
            LEFT JOIN outcome_events o ON a.id = o.action_event_id
            WHERE a.action_type = ?{room_filter}
            GROUP BY a.parameters, o.outcome_type
        """, (action_type, *room_params))

        by_angle = {}
        for row in cursor.fetchall():
            params = json.loads(row['parameters']) if row['parameters'] else {}
            angle = params.get('angle', 'unknown')
            outcome = row['outcome_type'] or 'unknown'
            count = row['count']

            if angle not in by_angle:
                by_angle[angle] = {'total': 0, 'success': 0, 'collision': 0, 'stall': 0, 'other': 0}
            by_angle[angle]['total'] += count
            if outcome == 'success':
                by_angle[angle]['success'] += count
            elif outcome == 'collision':
                by_angle[angle]['collision'] += count
            elif outcome == 'stall':
                by_angle[angle]['stall'] += count
            else:
                by_angle[angle]['other'] += count

        return {
            'total': total,
            'success_count': success_count,
            'success_rate': success_count / total if total > 0 else 0,
            'by_angle': by_angle
        }

    def get_collision_precursors(self, lookback_seconds: float = 2.0) -> List[Dict[str, Any]]:
        """
        Get sensor readings that preceded collisions.

        Useful for learning to predict and avoid collisions.
        """
        if not self._conn:
            return []

        cursor = self._conn.cursor()

        # Find collision outcomes and their preceding sensor data
        cursor.execute("""
            SELECT
                o.action_event_id,
                o.timestamp as collision_time,
                a.context_snapshot_id,
                cs.*
            FROM outcome_events o
            JOIN action_events a ON o.action_event_id = a.id
            LEFT JOIN sensor_snapshots cs ON a.context_snapshot_id = cs.id
            WHERE o.outcome_type = 'collision'
            ORDER BY o.timestamp DESC
            LIMIT 50
        """)

        precursors = []
        for row in cursor.fetchall():
            precursors.append({
                'collision_time': row['collision_time'],
                'context': {
                    'pose_x': row['pose_x'],
                    'pose_y': row['pose_y'],
                    'pose_angle': row['pose_angle'],
                    'ultra_left': row['ext_ultra_l_mm'],
                    'ultra_center': row['ext_ultra_c_mm'],
                    'ultra_right': row['ext_ultra_r_mm'],
                    'tof': row['ext_tof_mm'],
                    'pitch': row['ext_pitch'],
                    'roll': row['ext_roll']
                }
            })

        return precursors

    def get_session_summary(self, session_id: int = None) -> Dict[str, Any]:
        """Get summary statistics for a session"""
        if not self._conn:
            return {}

        sid = session_id or self._session_id
        if not sid:
            return {}

        cursor = self._conn.cursor()

        # Count snapshots
        cursor.execute("""
            SELECT COUNT(*) as count FROM sensor_snapshots WHERE session_id = ?
        """, (sid,))
        snapshot_count = cursor.fetchone()['count']

        # Count actions by type
        cursor.execute("""
            SELECT action_type, COUNT(*) as count
            FROM action_events WHERE session_id = ?
            GROUP BY action_type
        """, (sid,))
        actions_by_type = {row['action_type']: row['count'] for row in cursor.fetchall()}

        # Count outcomes by type
        cursor.execute("""
            SELECT outcome_type, COUNT(*) as count
            FROM outcome_events WHERE session_id = ?
            GROUP BY outcome_type
        """, (sid,))
        outcomes_by_type = {row['outcome_type']: row['count'] for row in cursor.fetchall()}

        return {
            'session_id': sid,
            'snapshot_count': snapshot_count,
            'actions_by_type': actions_by_type,
            'outcomes_by_type': outcomes_by_type
        }

    # ==================== Phase 1 Summary ====================

    def get_phase1_summary(self) -> Dict[str, Any]:
        """
        Get summary of Phase 1 survival learning progress.

        This provides a quick overview of data collection and learning status
        for Phase 1 (Grounded Survival Learning).
        """
        if not self._conn:
            return {}

        cursor = self._conn.cursor()

        # Count snapshots
        cursor.execute("SELECT COUNT(*) as count FROM sensor_snapshots")
        snapshot_count = cursor.fetchone()['count']

        # Count actions
        cursor.execute("SELECT COUNT(*) as count FROM action_events")
        action_count = cursor.fetchone()['count']

        # Count outcomes
        cursor.execute("SELECT COUNT(*) as count FROM outcome_events")
        outcome_count = cursor.fetchone()['count']

        # Count images
        images_dir = config.DATA_DIR / "learning_images"
        image_count = 0
        if images_dir.exists():
            image_count = len(list(images_dir.glob("*.jpg")))

        # Get recovery statistics
        stall_stats = self.get_recovery_statistics("escape_stall")
        cliff_stats = self.get_recovery_statistics("escape_cliff")

        # Count collision precursors
        precursor_count = len(self.get_collision_precursors())

        # Get action breakdown
        cursor.execute("""
            SELECT action_type, COUNT(*) as count
            FROM action_events
            GROUP BY action_type
        """)
        actions_by_type = {row['action_type']: row['count'] for row in cursor.fetchall()}

        # Get outcome breakdown
        cursor.execute("""
            SELECT outcome_type, COUNT(*) as count
            FROM outcome_events
            GROUP BY outcome_type
        """)
        outcomes_by_type = {row['outcome_type']: row['count'] for row in cursor.fetchall()}

        return {
            'total_snapshots': snapshot_count,
            'total_actions': action_count,
            'total_outcomes': outcome_count,
            'images_captured': image_count,
            'recovery_stats': {
                'escape_stall': stall_stats,
                'escape_cliff': cliff_stats,
            },
            'collision_precursors': precursor_count,
            'actions_by_type': actions_by_type,
            'outcomes_by_type': outcomes_by_type,
            'phase1_ready': action_count >= 20,  # Minimum for learning analysis
        }

    # ==================== Context Manager ====================

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
