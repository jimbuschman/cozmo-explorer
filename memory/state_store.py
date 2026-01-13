"""
State Store

SQLite-based persistence for robot state across sessions.
"""
import sqlite3
import logging
import json
from typing import Optional, Any
from datetime import datetime
from pathlib import Path

import config

logger = logging.getLogger(__name__)


class StateStore:
    """
    Simple key-value store backed by SQLite.

    Persists:
    - Session information
    - Robot state between runs
    - Configuration overrides
    - Statistics and logs
    """

    def __init__(self, db_path: str = None):
        self.db_path = Path(db_path or config.SQLITE_PATH)
        self._conn: Optional[sqlite3.Connection] = None

    def connect(self):
        """Connect to database and create tables"""
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

        self._conn = sqlite3.connect(str(self.db_path))
        self._conn.row_factory = sqlite3.Row
        self._create_tables()

        logger.info(f"StateStore connected: {self.db_path}")

    def _create_tables(self):
        """Create database tables if they don't exist"""
        cursor = self._conn.cursor()

        # Key-value store for simple state
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS state (
                key TEXT PRIMARY KEY,
                value TEXT,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        # Session log
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                ended_at TIMESTAMP,
                duration_seconds REAL,
                distance_traveled REAL,
                areas_explored INTEGER,
                summary TEXT
            )
        """)

        # Event log
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                event_type TEXT,
                description TEXT,
                data TEXT,
                FOREIGN KEY (session_id) REFERENCES sessions(id)
            )
        """)

        self._conn.commit()

    def close(self):
        """Close database connection"""
        if self._conn:
            self._conn.close()
            self._conn = None

    # ==================== Key-Value State ====================

    def get(self, key: str, default: Any = None) -> Any:
        """Get a value from the state store"""
        if not self._conn:
            return default

        cursor = self._conn.cursor()
        cursor.execute("SELECT value FROM state WHERE key = ?", (key,))
        row = cursor.fetchone()

        if row is None:
            return default

        try:
            return json.loads(row['value'])
        except (json.JSONDecodeError, TypeError):
            return row['value']

    def set(self, key: str, value: Any):
        """Set a value in the state store"""
        if not self._conn:
            return

        value_str = json.dumps(value) if not isinstance(value, str) else value

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT OR REPLACE INTO state (key, value, updated_at)
            VALUES (?, ?, CURRENT_TIMESTAMP)
        """, (key, value_str))
        self._conn.commit()

    def delete(self, key: str):
        """Delete a key from the state store"""
        if not self._conn:
            return

        cursor = self._conn.cursor()
        cursor.execute("DELETE FROM state WHERE key = ?", (key,))
        self._conn.commit()

    # ==================== Sessions ====================

    def start_session(self) -> int:
        """Start a new exploration session"""
        if not self._conn:
            return -1

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT INTO sessions (started_at) VALUES (CURRENT_TIMESTAMP)
        """)
        self._conn.commit()

        session_id = cursor.lastrowid
        self.set('current_session_id', session_id)

        logger.info(f"Started session {session_id}")
        return session_id

    def end_session(
        self,
        duration: float = 0,
        distance: float = 0,
        areas: int = 0,
        summary: str = ""
    ):
        """End the current session"""
        if not self._conn:
            return

        session_id = self.get('current_session_id')
        if not session_id:
            return

        cursor = self._conn.cursor()
        cursor.execute("""
            UPDATE sessions
            SET ended_at = CURRENT_TIMESTAMP,
                duration_seconds = ?,
                distance_traveled = ?,
                areas_explored = ?,
                summary = ?
            WHERE id = ?
        """, (duration, distance, areas, summary, session_id))
        self._conn.commit()

        self.delete('current_session_id')
        logger.info(f"Ended session {session_id}")

    def get_session_history(self, limit: int = 10) -> list:
        """Get recent session history"""
        if not self._conn:
            return []

        cursor = self._conn.cursor()
        cursor.execute("""
            SELECT * FROM sessions
            ORDER BY started_at DESC
            LIMIT ?
        """, (limit,))

        return [dict(row) for row in cursor.fetchall()]

    # ==================== Events ====================

    def log_event(
        self,
        event_type: str,
        description: str,
        data: dict = None
    ):
        """Log an event"""
        if not self._conn:
            return

        session_id = self.get('current_session_id')
        data_str = json.dumps(data) if data else None

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT INTO events (session_id, event_type, description, data)
            VALUES (?, ?, ?, ?)
        """, (session_id, event_type, description, data_str))
        self._conn.commit()

    def get_events(
        self,
        session_id: int = None,
        event_type: str = None,
        limit: int = 100
    ) -> list:
        """Get events, optionally filtered"""
        if not self._conn:
            return []

        query = "SELECT * FROM events WHERE 1=1"
        params = []

        if session_id:
            query += " AND session_id = ?"
            params.append(session_id)

        if event_type:
            query += " AND event_type = ?"
            params.append(event_type)

        query += " ORDER BY timestamp DESC LIMIT ?"
        params.append(limit)

        cursor = self._conn.cursor()
        cursor.execute(query, params)

        events = []
        for row in cursor.fetchall():
            event = dict(row)
            if event['data']:
                try:
                    event['data'] = json.loads(event['data'])
                except json.JSONDecodeError:
                    pass
            events.append(event)

        return events

    # ==================== Robot State ====================

    def save_robot_state(
        self,
        position: dict,
        battery: float,
        exploration_time: float
    ):
        """Save current robot state for resume"""
        self.set('last_position', position)
        self.set('last_battery', battery)
        self.set('total_exploration_time', exploration_time)
        self.set('last_active', datetime.now().isoformat())

    def get_last_robot_state(self) -> Optional[dict]:
        """Get last known robot state"""
        position = self.get('last_position')
        if not position:
            return None

        return {
            'position': position,
            'battery': self.get('last_battery', 0),
            'exploration_time': self.get('total_exploration_time', 0),
            'last_active': self.get('last_active')
        }

    # ==================== Context Manager ====================

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
