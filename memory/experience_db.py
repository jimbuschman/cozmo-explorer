"""
Experience Database

Uses ChromaDB for semantic storage and retrieval of robot experiences.
"Have I seen something like this before?"
"""
import logging
from typing import Optional, List
from dataclasses import dataclass
from datetime import datetime
import json

try:
    import chromadb
    from chromadb.config import Settings
except ImportError:
    chromadb = None
    print("WARNING: chromadb not installed")

import config

logger = logging.getLogger(__name__)


@dataclass
class Experience:
    """A single experience/memory"""
    id: str
    description: str
    location_x: float
    location_y: float
    timestamp: datetime
    experience_type: str  # "observation", "obstacle", "object", "area"
    importance: float = 0.5  # 0-1
    metadata: dict = None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "description": self.description,
            "location_x": self.location_x,
            "location_y": self.location_y,
            "timestamp": self.timestamp.isoformat(),
            "experience_type": self.experience_type,
            "importance": self.importance,
            "metadata": self.metadata or {}
        }


class ExperienceDB:
    """
    Semantic experience storage using ChromaDB.

    Stores robot experiences with embeddings for similarity search:
    - "Have I seen this before?"
    - "What was near this location?"
    - "When did I encounter something similar?"
    """

    def __init__(self, persist_path: str = None):
        self.persist_path = persist_path or str(config.CHROMA_PATH)
        self._client = None
        self._collection = None

    async def connect(self):
        """Initialize ChromaDB"""
        if chromadb is None:
            logger.warning("ChromaDB not available, experience storage disabled")
            return False

        try:
            self._client = chromadb.PersistentClient(
                path=self.persist_path,
                settings=Settings(anonymized_telemetry=False)
            )

            self._collection = self._client.get_or_create_collection(
                name="robot_experiences",
                metadata={"description": "Robot exploration experiences"}
            )

            logger.info(
                f"ExperienceDB connected. "
                f"{self._collection.count()} existing experiences."
            )
            return True

        except Exception as e:
            logger.error(f"Failed to connect to ChromaDB: {e}")
            return False

    async def close(self):
        """Close ChromaDB (mainly for cleanup)"""
        # ChromaDB PersistentClient handles its own lifecycle
        self._collection = None
        self._client = None

    async def add_experience(self, experience: Experience) -> bool:
        """
        Add a new experience to the database.

        The experience description is automatically embedded for similarity search.
        """
        if self._collection is None:
            return False

        try:
            metadata = {
                "location_x": experience.location_x,
                "location_y": experience.location_y,
                "timestamp": experience.timestamp.isoformat(),
                "experience_type": experience.experience_type,
                "importance": experience.importance,
                **(experience.metadata or {})
            }

            self._collection.add(
                ids=[experience.id],
                documents=[experience.description],
                metadatas=[metadata]
            )

            logger.debug(f"Added experience: {experience.id}")
            return True

        except Exception as e:
            logger.error(f"Failed to add experience: {e}")
            return False

    async def find_similar(
        self,
        description: str,
        n_results: int = 5,
        threshold: float = None
    ) -> List[Experience]:
        """
        Find experiences similar to the given description.

        Args:
            description: Text to search for
            n_results: Maximum number of results
            threshold: Optional similarity threshold (0-1, higher = more similar)

        Returns:
            List of similar experiences, ordered by similarity
        """
        if self._collection is None:
            return []

        threshold = threshold or config.EXPERIENCE_RELEVANCE_THRESHOLD

        try:
            results = self._collection.query(
                query_texts=[description],
                n_results=n_results
            )

            experiences = []
            for i, doc_id in enumerate(results['ids'][0]):
                # Check distance/similarity
                distance = results['distances'][0][i] if results.get('distances') else 0
                # ChromaDB returns L2 distance, convert to similarity
                similarity = 1 / (1 + distance)

                if similarity < threshold:
                    continue

                metadata = results['metadatas'][0][i]
                exp = Experience(
                    id=doc_id,
                    description=results['documents'][0][i],
                    location_x=metadata.get('location_x', 0),
                    location_y=metadata.get('location_y', 0),
                    timestamp=datetime.fromisoformat(metadata.get('timestamp', datetime.now().isoformat())),
                    experience_type=metadata.get('experience_type', 'unknown'),
                    importance=metadata.get('importance', 0.5),
                    metadata=metadata
                )
                experiences.append(exp)

            return experiences

        except Exception as e:
            logger.error(f"Search failed: {e}")
            return []

    async def find_near_location(
        self,
        x: float,
        y: float,
        radius: float = 100.0,
        n_results: int = 10
    ) -> List[Experience]:
        """
        Find experiences near a specific location.

        Args:
            x, y: Location coordinates (mm)
            radius: Search radius (mm)
            n_results: Maximum results

        Returns:
            List of experiences within radius
        """
        if self._collection is None:
            return []

        try:
            # ChromaDB doesn't have native spatial queries,
            # so we fetch all and filter
            all_results = self._collection.get(
                include=["documents", "metadatas"]
            )

            experiences = []
            for i, doc_id in enumerate(all_results['ids']):
                metadata = all_results['metadatas'][i]
                loc_x = metadata.get('location_x', 0)
                loc_y = metadata.get('location_y', 0)

                # Calculate distance
                dist = ((loc_x - x)**2 + (loc_y - y)**2)**0.5

                if dist <= radius:
                    exp = Experience(
                        id=doc_id,
                        description=all_results['documents'][i],
                        location_x=loc_x,
                        location_y=loc_y,
                        timestamp=datetime.fromisoformat(
                            metadata.get('timestamp', datetime.now().isoformat())
                        ),
                        experience_type=metadata.get('experience_type', 'unknown'),
                        importance=metadata.get('importance', 0.5),
                        metadata=metadata
                    )
                    experiences.append((dist, exp))

            # Sort by distance and return
            experiences.sort(key=lambda x: x[0])
            return [exp for _, exp in experiences[:n_results]]

        except Exception as e:
            logger.error(f"Location search failed: {e}")
            return []

    async def get_recent(self, n: int = 10) -> List[Experience]:
        """Get the N most recent experiences"""
        if self._collection is None:
            return []

        try:
            all_results = self._collection.get(
                include=["documents", "metadatas"]
            )

            experiences = []
            for i, doc_id in enumerate(all_results['ids']):
                metadata = all_results['metadatas'][i]
                exp = Experience(
                    id=doc_id,
                    description=all_results['documents'][i],
                    location_x=metadata.get('location_x', 0),
                    location_y=metadata.get('location_y', 0),
                    timestamp=datetime.fromisoformat(
                        metadata.get('timestamp', datetime.now().isoformat())
                    ),
                    experience_type=metadata.get('experience_type', 'unknown'),
                    importance=metadata.get('importance', 0.5),
                    metadata=metadata
                )
                experiences.append(exp)

            # Sort by timestamp descending
            experiences.sort(key=lambda x: x.timestamp, reverse=True)
            return experiences[:n]

        except Exception as e:
            logger.error(f"Failed to get recent experiences: {e}")
            return []

    async def count(self) -> int:
        """Get total number of experiences"""
        if self._collection is None:
            return 0
        return self._collection.count()

    async def clear(self):
        """Clear all experiences (use with caution!)"""
        if self._client is None:
            return

        try:
            self._client.delete_collection("robot_experiences")
            self._collection = self._client.create_collection(
                name="robot_experiences",
                metadata={"description": "Robot exploration experiences"}
            )
            logger.info("Cleared all experiences")
        except Exception as e:
            logger.error(f"Failed to clear experiences: {e}")

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()
