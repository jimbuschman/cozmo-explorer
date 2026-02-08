"""
Map Annotations

SQLite storage for semantic annotations tied to map coordinates.
These are the persistent product of LLM session reviews - descriptions
of what the robot found, room types, landmarks, coverage gaps, etc.

Any robot or system can later query these annotations to understand
the mapped environment.
"""
import sqlite3
import json
import logging
from typing import List, Dict, Optional
from datetime import datetime
from pathlib import Path

import config

logger = logging.getLogger(__name__)


class MapAnnotationStore:
    """SQLite-backed storage for map annotations."""

    def __init__(self, db_path: str = None):
        self.db_path = db_path or str(config.SQLITE_PATH)
        self.conn: Optional[sqlite3.Connection] = None

    def connect(self):
        self.conn = sqlite3.connect(self.db_path)
        self.conn.row_factory = sqlite3.Row
        self._create_tables()

    def close(self):
        if self.conn:
            self.conn.close()
            self.conn = None

    def _create_tables(self):
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS map_annotations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER,
                timestamp TEXT NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                label TEXT NOT NULL,
                annotation_type TEXT NOT NULL,
                details TEXT,
                confidence REAL DEFAULT 1.0,
                source TEXT DEFAULT 'llm_review'
            )
        """)
        self.conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_annotations_type
            ON map_annotations(annotation_type)
        """)
        self.conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_annotations_coords
            ON map_annotations(x, y)
        """)
        self.conn.commit()

    def add_annotation(
        self,
        x: float,
        y: float,
        label: str,
        annotation_type: str,
        details: str = "",
        confidence: float = 1.0,
        session_id: int = None,
        source: str = "llm_review",
    ) -> int:
        """Add a semantic annotation to the map.

        Args:
            x, y: World coordinates in mm
            label: Short description (e.g., "doorway", "table leg", "wall corner")
            annotation_type: Category - "landmark", "room_type", "coverage_gap",
                           "obstacle", "doorway", "observation"
            details: Longer description or context
            confidence: 0.0-1.0 confidence score
            session_id: Session that produced this annotation
            source: "llm_review", "sensor", "manual"

        Returns:
            ID of the new annotation
        """
        cursor = self.conn.execute("""
            INSERT INTO map_annotations
            (session_id, timestamp, x, y, label, annotation_type, details, confidence, source)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            session_id,
            datetime.now().isoformat(),
            x, y, label, annotation_type, details, confidence, source,
        ))
        self.conn.commit()
        return cursor.lastrowid

    def get_annotations(
        self,
        annotation_type: str = None,
        near_x: float = None,
        near_y: float = None,
        radius: float = 500.0,
    ) -> List[Dict]:
        """Query annotations, optionally filtered by type and/or proximity."""
        query = "SELECT * FROM map_annotations WHERE 1=1"
        params = []

        if annotation_type:
            query += " AND annotation_type = ?"
            params.append(annotation_type)

        if near_x is not None and near_y is not None:
            # Simple bounding box filter (not true circle, but fast)
            query += " AND x BETWEEN ? AND ? AND y BETWEEN ? AND ?"
            params.extend([near_x - radius, near_x + radius,
                          near_y - radius, near_y + radius])

        query += " ORDER BY timestamp DESC"

        rows = self.conn.execute(query, params).fetchall()
        return [dict(row) for row in rows]

    def get_all_annotations(self) -> List[Dict]:
        """Get all annotations."""
        rows = self.conn.execute(
            "SELECT * FROM map_annotations ORDER BY timestamp DESC"
        ).fetchall()
        return [dict(row) for row in rows]

    def get_summary(self) -> Dict:
        """Get annotation counts by type."""
        rows = self.conn.execute("""
            SELECT annotation_type, COUNT(*) as count
            FROM map_annotations
            GROUP BY annotation_type
        """).fetchall()
        return {row['annotation_type']: row['count'] for row in rows}

    def count(self) -> int:
        row = self.conn.execute("SELECT COUNT(*) FROM map_annotations").fetchone()
        return row[0]
