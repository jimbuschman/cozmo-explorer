"""
Session Reviewer

Post-session LLM analysis of the completed map.
Reviews ASCII map + session stats and produces semantic annotations
about the environment (room type, landmarks, doorways, coverage gaps).
"""
import json
import logging
from typing import List, Dict, Optional, TYPE_CHECKING

from llm.review_prompts import MAP_REVIEW_SYSTEM_PROMPT, MAP_REVIEW_USER_PROMPT

if TYPE_CHECKING:
    from memory.spatial_map import SpatialMap

logger = logging.getLogger(__name__)


class SessionReviewer:
    """Reviews a completed mapping session using an LLM."""

    def __init__(self, llm_client):
        """
        Args:
            llm_client: LLMClient instance (must have _chat method)
        """
        self.llm_client = llm_client

    async def review_session(
        self,
        spatial_map: "SpatialMap",
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        escapes: int = 0,
        mapping_time: float = 0.0,
        images_captured: int = 0,
    ) -> List[Dict]:
        """
        Run LLM review on the completed map.

        Returns list of annotation dicts with keys:
            label, type, x, y, details
        """
        if not self.llm_client:
            logger.warning("No LLM client - skipping session review")
            return []

        # Build context
        ascii_map = spatial_map.to_ascii(robot_x=robot_x, robot_y=robot_y)
        visited_pct = spatial_map.get_visited_percentage() * 100
        explored_pct = spatial_map.get_exploration_progress() * 100

        user_prompt = MAP_REVIEW_USER_PROMPT.format(
            mapping_time=mapping_time,
            escapes=escapes,
            images_captured=images_captured,
            visited_pct=visited_pct,
            explored_pct=explored_pct,
            ascii_map=ascii_map,
        )

        try:
            # Send to LLM with system prompt
            full_prompt = MAP_REVIEW_SYSTEM_PROMPT + "\n\n" + user_prompt
            response = await self.llm_client._chat(full_prompt)

            if not response:
                logger.warning("Empty LLM response for session review")
                return []

            # Parse JSON from response
            annotations = self._parse_annotations(response)
            logger.info(f"Session review parsed {len(annotations)} annotations")
            return annotations

        except Exception as e:
            logger.error(f"Session review failed: {e}")
            return []

    def _parse_annotations(self, response: str) -> List[Dict]:
        """Parse LLM response into annotation dicts."""
        # Try to find JSON array in the response
        response = response.strip()

        # Look for JSON array
        start = response.find('[')
        end = response.rfind(']')
        if start >= 0 and end > start:
            json_str = response[start:end + 1]
            try:
                annotations = json.loads(json_str)
                if isinstance(annotations, list):
                    # Validate each annotation has required fields
                    valid = []
                    for ann in annotations:
                        if isinstance(ann, dict) and 'label' in ann and 'type' in ann:
                            valid.append({
                                'label': str(ann.get('label', '')),
                                'type': str(ann.get('type', 'observation')),
                                'x': float(ann.get('x', 0)),
                                'y': float(ann.get('y', 0)),
                                'details': str(ann.get('details', '')),
                            })
                    return valid
            except json.JSONDecodeError as e:
                logger.warning(f"Failed to parse review JSON: {e}")

        logger.warning(f"Could not find JSON array in review response")
        return []
