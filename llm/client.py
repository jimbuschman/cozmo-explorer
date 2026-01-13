"""
LLM Client

Handles communication with the Ollama LLM for high-level decision making.
"""
import asyncio
import logging
import json
import re
from typing import Optional, Dict, Any
from datetime import datetime
from dataclasses import dataclass

import httpx

import config
from llm.prompts import DECISION_SYSTEM_PROMPT, VISION_SYSTEM_PROMPT
from memory.conversation_memory import ConversationMemory, MemoryItem, estimate_tokens

logger = logging.getLogger(__name__)


@dataclass
class LLMDecision:
    """Structured decision from the LLM"""
    action: str  # explore, investigate, go_to, idle
    target: Optional[str] = None
    reasoning: str = ""
    speak: Optional[str] = None
    raw_response: str = ""

    def is_valid(self) -> bool:
        return self.action in ("explore", "investigate", "go_to", "idle")


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
        self._system_prompt = DECISION_SYSTEM_PROMPT
        self._vision_prompt = VISION_SYSTEM_PROMPT

        # Conversation memory with token-budgeted pools
        self.memory = ConversationMemory(
            global_budget=config.TOKEN_BUDGET,
            summarizer=self._summarize_text
        )

        # Personality (will be set by main.py if using alien explorer mode)
        self.personality = None

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

    async def _summarize_text(self, text: str) -> Optional[str]:
        """Summarize text for memory compression"""
        if not text or len(text) < 100:
            return text

        try:
            prompt = f"""Summarize this exploration log in 2-3 sentences, preserving key discoveries and decisions:

{text}

Summary:"""
            response = await self._chat(prompt)
            return response.strip()
        except Exception as e:
            logger.error(f"Summarization failed: {e}")
            return None

    def set_personality(self, personality):
        """Set the personality module for alien explorer mode"""
        self.personality = personality
        if personality:
            # Set core identity from personality
            self.memory.set_core_memory(personality.get_identity_prompt())
            logger.info("Personality set: Alien Explorer mode activated")

    def add_observation_to_memory(self, observation: str):
        """Add a visual observation to conversation memory"""
        self.memory.add_observation(observation)

    def add_decision_to_memory(self, action: str, reasoning: str):
        """Add a decision to conversation memory"""
        self.memory.add_decision(action, reasoning)

    def add_recall_to_memory(self, memory_text: str, relevance: float = 1.0):
        """Add a recalled memory from ChromaDB"""
        self.memory.add_recall(memory_text, relevance)

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

    async def get_decision(self, context_prompt: str) -> LLMDecision:
        """
        Get a structured decision from the LLM with conversation memory.

        Args:
            context_prompt: Formatted context string from MemoryContext

        Returns:
            LLMDecision with parsed action, target, reasoning, and speak
        """
        if self._client is None:
            await self.connect()

        try:
            # Build full prompt with conversation memory
            full_prompt = self._build_decision_prompt(context_prompt)

            response = await self._chat(full_prompt)
            logger.info(f"LLM decision response: {response[:150]}...")

            decision = self._parse_decision_response(response)
            decision.raw_response = response

            # Store the decision in conversation memory
            if decision.is_valid():
                self.add_decision_to_memory(decision.action, decision.reasoning)

                # Log to personality journal if available
                if self.personality and decision.speak:
                    self.personality.log_thought(decision.speak)

            return decision

        except Exception as e:
            logger.error(f"Decision query failed: {e}")
            return LLMDecision(
                action="explore",
                reasoning=f"LLM query failed: {e}",
                raw_response=""
            )

    def _build_decision_prompt(self, context_prompt: str) -> str:
        """Build full prompt with conversation memory"""
        # Estimate tokens for the new context
        context_tokens = estimate_tokens(context_prompt)
        available_for_memory = config.TOKEN_BUDGET - context_tokens - ConversationMemory.OVERHEAD_TOKENS

        # Gather conversation memory
        memory_items = self.memory.gather_context(available_for_memory)
        memory_text = self.memory.format_for_prompt(memory_items)

        # Combine memory + current context
        parts = []

        if memory_text:
            parts.append(memory_text)

        parts.append(context_prompt)

        # Add personality session summary if available
        if self.personality:
            parts.append("")
            parts.append(self.personality.get_session_summary())

        return "\n\n".join(parts)

    def _parse_decision_response(self, response: str) -> LLMDecision:
        """
        Parse LLM response into a structured LLMDecision.

        Tries to extract JSON, falls back to keyword parsing if that fails.
        """
        # Try to find JSON in the response
        json_data = self._extract_json(response)

        if json_data:
            return LLMDecision(
                action=json_data.get("action", "explore").lower(),
                target=json_data.get("target"),
                reasoning=json_data.get("reasoning", ""),
                speak=json_data.get("speak")
            )

        # Fallback: keyword-based parsing
        logger.warning("Could not parse JSON from LLM response, using keyword fallback")
        return self._keyword_parse(response)

    def _extract_json(self, text: str) -> Optional[Dict[str, Any]]:
        """
        Extract JSON object from text that may contain other content.
        """
        # Try to find JSON block (with or without markdown code fences)
        patterns = [
            r'```json\s*(.*?)\s*```',  # Markdown JSON block
            r'```\s*(.*?)\s*```',       # Generic code block
            r'(\{[^{}]*"action"[^{}]*\})',  # JSON with action key
            r'(\{.*?\})',               # Any JSON object
        ]

        for pattern in patterns:
            matches = re.findall(pattern, text, re.DOTALL)
            for match in matches:
                try:
                    data = json.loads(match.strip())
                    if isinstance(data, dict) and "action" in data:
                        return data
                except json.JSONDecodeError:
                    continue

        # Try parsing the whole response as JSON
        try:
            data = json.loads(text.strip())
            if isinstance(data, dict):
                return data
        except json.JSONDecodeError:
            pass

        return None

    def _keyword_parse(self, response: str) -> LLMDecision:
        """
        Fallback keyword-based parsing when JSON extraction fails.
        """
        response_lower = response.lower()

        if "investigate" in response_lower or "look" in response_lower:
            action = "investigate"
        elif "go to" in response_lower or "navigate" in response_lower or "head" in response_lower:
            action = "go_to"
        elif "wait" in response_lower or "idle" in response_lower or "rest" in response_lower:
            action = "idle"
        else:
            action = "explore"

        # Try to extract a target from the response
        target = None
        target_patterns = [
            r'(?:investigate|look at|check out|go to|head toward)\s+(?:the\s+)?([^.!?\n]+)',
            r'(?:north|south|east|west)',
        ]
        for pattern in target_patterns:
            match = re.search(pattern, response_lower)
            if match:
                target = match.group(1) if match.lastindex else match.group(0)
                break

        return LLMDecision(
            action=action,
            target=target,
            reasoning=response[:200]  # Use first part of response as reasoning
        )

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
