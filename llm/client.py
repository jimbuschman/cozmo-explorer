"""
LLM Client

Handles communication with the Ollama LLM for high-level decision making.
"""
import asyncio
import logging
import json
from typing import Optional
from datetime import datetime

import httpx

import config

logger = logging.getLogger(__name__)


class LLMClient:
    """
    Async client for Ollama LLM.

    Used to query the LLM for high-level guidance:
    - "What should I explore next?"
    - "I found something interesting, what is it?"
    - "I'm stuck, what should I do?"
    """

    def __init__(
        self,
        host: str = None,
        model: str = None,
        vision_model: str = None,
        timeout: float = None
    ):
        self.host = host or config.OLLAMA_HOST
        self.model = model or config.OLLAMA_MODEL
        self.vision_model = vision_model or "llava"  # Vision model for image analysis
        self.timeout = timeout or config.LLM_TIMEOUT

        self._client: Optional[httpx.AsyncClient] = None
        self._system_prompt = self._build_system_prompt()
        self._vision_prompt = self._build_vision_prompt()

    def _build_vision_prompt(self) -> str:
        return """You are the eyes of an autonomous Cozmo robot exploring a house.

Your job is to describe what you see in images from the robot's camera. Be concise but informative.

Focus on:
- Objects (furniture, items, obstacles)
- The environment (room type, floor, walls)
- Anything interesting or unusual
- Potential obstacles or hazards

Keep descriptions to 1-2 sentences. Be specific about what you see."""

    def _build_system_prompt(self) -> str:
        return """You are the decision-making brain of an autonomous Cozmo robot exploring a house.

Your role is to:
1. Analyze the robot's current state and sensor data
2. Decide what the robot should do next
3. Provide clear, actionable goals

The robot can:
- EXPLORE: Wander and discover new areas
- INVESTIGATE: Look closely at something interesting
- GO_TO: Navigate to a specific location (if it knows coordinates)
- IDLE: Wait and conserve energy

IMPORTANT - Cozmo battery levels:
- 4.5V+ = FULL battery, explore freely
- 4.0-4.5V = GOOD battery, keep exploring
- 3.7-4.0V = MEDIUM battery, can still explore
- 3.5-3.7V = LOW battery, consider charging soon
- Below 3.5V = CRITICAL, must charge

Guidelines:
- Be curious! Encourage exploration of unexplored areas
- If battery is above 3.7V, prioritize exploration over charging
- If the robot is stuck, suggest alternative approaches
- Keep responses concise and action-oriented

Respond with a clear goal statement. Examples:
- "Explore: Continue exploring to the north where you haven't been"
- "Investigate: That looks like a new object, take a closer look"
- "Idle: Battery is low, wait for it to charge before continuing"
"""

    async def connect(self):
        """Initialize the HTTP client"""
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=self.timeout)
        logger.info(f"LLM client ready: {self.host} using {self.model}")

    async def close(self):
        """Close the HTTP client"""
        if self._client:
            await self._client.aclose()
            self._client = None

    async def query_for_goal(self, state_summary: dict) -> Optional[str]:
        """
        Ask the LLM what the robot should do next.

        Args:
            state_summary: Dict with current robot state, position, sensors, etc.

        Returns:
            String with the LLM's suggested goal/action
        """
        if self._client is None:
            await self.connect()

        # Format state into a readable message
        user_message = self._format_state_message(state_summary)

        try:
            response = await self._chat(user_message)
            logger.info(f"LLM response: {response[:100]}...")
            return response
        except Exception as e:
            logger.error(f"LLM query failed: {e}")
            return None

    async def describe_image(self, image_base64: str, context: str = "") -> Optional[str]:
        """
        Ask LLaVA to describe what's in an image.

        Args:
            image_base64: Base64 encoded image
            context: Optional additional context

        Returns:
            Description of what's in the image
        """
        if self._client is None:
            await self.connect()

        prompt = "What do you see in this image? Be concise (1-2 sentences)."
        if context:
            prompt += f" Context: {context}"

        try:
            response = await self._vision_query(prompt, image_base64)
            logger.info(f"Vision response: {response[:100]}...")
            return response
        except Exception as e:
            logger.error(f"Image description failed: {e}")
            return None

    async def _vision_query(self, prompt: str, image_base64: str) -> str:
        """Send a vision query to LLaVA"""
        url = f"{self.host}/api/chat"

        payload = {
            "model": self.vision_model,
            "messages": [
                {"role": "system", "content": self._vision_prompt},
                {
                    "role": "user",
                    "content": prompt,
                    "images": [image_base64]
                }
            ],
            "stream": False
        }

        response = await self._client.post(url, json=payload, timeout=90.0)  # Vision takes longer
        response.raise_for_status()

        data = response.json()
        return data.get("message", {}).get("content", "")

    async def interpret_observation(
        self,
        observation: str,
        context: dict
    ) -> Optional[str]:
        """
        Ask the LLM to interpret an observation.

        Args:
            observation: Description of what was observed
            context: Additional context (position, recent history, etc.)

        Returns:
            LLM's interpretation
        """
        if self._client is None:
            await self.connect()

        message = f"""I observed: {observation}

Current context:
- Position: ({context.get('x', 0):.0f}, {context.get('y', 0):.0f})
- Time exploring: {context.get('exploration_time', 0):.0f}s
- Recent observations: {context.get('recent', [])}

What does this observation mean? Should I investigate further?"""

        try:
            response = await self._chat(message)
            return response
        except Exception as e:
            logger.error(f"Interpretation failed: {e}")
            return None

    def _format_state_message(self, state: dict) -> str:
        """Format state summary into a readable message for the LLM"""
        pos = state.get('position', {})
        sensors = state.get('sensors', {})
        ctx = state.get('context', {})

        message = f"""Current robot state:
- State: {state.get('state', 'UNKNOWN')}
- Position: ({pos.get('x', 0):.0f}, {pos.get('y', 0):.0f}) facing {pos.get('angle_degrees', 0):.0f} degrees
- Battery: {sensors.get('battery', 0):.2f}V {'(charging)' if sensors.get('on_charger') else ''}
- Cliff detected: {sensors.get('cliff_detected', False)}
- Time exploring: {ctx.get('exploration_time', 0):.0f} seconds
- Times stuck: {ctx.get('stuck_count', 0)}
- Current goal: {ctx.get('current_goal', 'None')}

Recent observations:
{self._format_observations(ctx.get('interesting_observations', []))}

What should I do next?"""
        return message

    def _format_observations(self, observations: list) -> str:
        """Format recent observations into readable text"""
        if not observations:
            return "- No recent observations"

        lines = []
        for obs in observations[-5:]:
            lines.append(f"- {obs}")
        return "\n".join(lines)

    async def _chat(self, message: str) -> str:
        """Send a chat message and get response"""
        url = f"{self.host}/api/chat"

        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": message}
            ],
            "stream": False
        }

        response = await self._client.post(url, json=payload)
        response.raise_for_status()

        data = response.json()
        return data.get("message", {}).get("content", "")

    async def _chat_with_image(self, message: str, image_base64: str) -> str:
        """Send a chat message with an image"""
        url = f"{self.host}/api/chat"

        payload = {
            "model": self.model,  # Should be a vision model like llava
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {
                    "role": "user",
                    "content": message,
                    "images": [image_base64]
                }
            ],
            "stream": False
        }

        response = await self._client.post(url, json=payload)
        response.raise_for_status()

        data = response.json()
        return data.get("message", {}).get("content", "")

    async def health_check(self) -> bool:
        """Check if Ollama is running and the model is available"""
        if self._client is None:
            await self.connect()

        try:
            # Check if Ollama is running
            response = await self._client.get(f"{self.host}/api/tags")
            response.raise_for_status()

            data = response.json()
            models = [m.get("name", "") for m in data.get("models", [])]

            # Check if our model is available
            model_available = any(self.model in m for m in models)

            if not model_available:
                logger.warning(
                    f"Model {self.model} not found. Available: {models}"
                )
                return False

            return True

        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return False

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


# Quick test
async def test_llm():
    """Test LLM connection and basic query"""
    async with LLMClient() as llm:
        if await llm.health_check():
            print("LLM is healthy!")

            response = await llm.query_for_goal({
                "state": "IDLE",
                "position": {"x": 0, "y": 0, "angle_degrees": 0},
                "sensors": {"battery": 4.0, "on_charger": False},
                "context": {"exploration_time": 0, "stuck_count": 0}
            })
            print(f"LLM says: {response}")
        else:
            print("LLM health check failed")


if __name__ == "__main__":
    asyncio.run(test_llm())
