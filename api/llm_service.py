from __future__ import annotations

import asyncio
import json
import logging
import re
from typing import Any, Dict, List, Optional, Tuple

import httpx

from api.command_manager import CommandManager
from api.config import settings
from api.destinations import DESTINATIONS
from api.ros_adapter import RosAdapter
from api.tools import TOOL_DEFINITIONS, execute_tool

logger = logging.getLogger("robot_control_api.llm")

SYSTEM_PROMPT = """You are the intelligent autonomous assistant for an office mobile robot operating in an indoor facility.
You assist human operators and visitors by providing robot telemetry, listing destinations, dispatching autonomous navigation, and canceling operations safely.

### Approved Destinations Registry:
The robot is strictly restricted to navigating between these approved locations:
- 'lobby': Lobby / Reception IN Lane (Security turnstiles)
- 'living_demo': Experience Centre: Living Demo Room (Smart home demonstration)
- 'hotel_exp': Experience Centre: Hotel Room (Hotel automation suite)
- 'hospital_exp': Experience Centre: Hospital Room (Healthcare automation suite)
- 'board_room': Board Room (Executive conference room)
- 'accounts_ws': Accounts Workstation (Finance department workstations)

### Critical Rules and Safety Protocols:
1. Destination Validation & Tools:
   - When the user asks to navigate, you MUST call the `navigate_to_location` tool with the corresponding `location_id`.
   - Never invent destination IDs or coordinates. The robot only accepts the 6 approved IDs above.
2. Ambiguity & Clarification:
   - If the user asks to go to an ambiguous location (e.g. "take me to the experience room", "take me to the demo"), DO NOT guess. You MUST ask for clarification because the Experience Centre contains three distinct rooms: Living Demo Room ('living_demo'), Hotel Room ('hotel_exp'), and Hospital Room ('hospital_exp').
3. Truthfulness on Navigation Progress:
   - NEVER state that the robot has arrived at a destination just because a navigation command was accepted or dispatched!
   - When a navigation command is accepted, inform the user that transit has begun and provide the command ID.
   - To know if the robot has actually arrived, query `get_command_status` or `get_robot_status`.
4. Single Active Command:
   - The robot can only execute one navigation command at a time. If navigation is already running, explain this to the user and suggest waiting or canceling before dispatching a new route.
5. Cancellation:
   - If the user asks to stop, halt, or cancel navigation, invoke `cancel_navigation`.
"""


class BaseLLMProvider:
    async def generate_response(
        self,
        messages: List[Dict[str, Any]],
        tools: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """
        Generates a model response given message history and tool schemas.
        Returns:
            {
                "content": Optional[str],
                "tool_calls": Optional[List[Dict[str, Any]]]
            }
        """
        raise NotImplementedError


class MockLLMProvider(BaseLLMProvider):
    """
    Deterministic offline provider for testing and environments without external LLM API keys.
    Understands intent from natural language and emits proper tool calls or responses.
    """

    def __init__(self, should_fail: bool = False, should_timeout: bool = False):
        self.should_fail = should_fail
        self.should_timeout = should_timeout

    async def generate_response(
        self,
        messages: List[Dict[str, Any]],
        tools: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        if self.should_timeout:
            await asyncio.sleep(0.5)
            raise TimeoutError("Simulated LLM provider timeout")

        if self.should_fail:
            raise RuntimeError("Simulated upstream LLM provider service failure (HTTP 502/503)")

        # Inspect the latest message
        last_msg = messages[-1]

        # If previous message was a tool result, synthesize a final human response
        if last_msg.get("role") == "tool":
            tool_content = last_msg.get("content", "{}")
            tool_name = last_msg.get("name", "")
            try:
                data = json.loads(tool_content)
            except Exception:
                data = {}

            if "error" in data:
                return {"content": f"I encountered an error executing that request: {data['error']}", "tool_calls": None}

            if tool_name == "get_robot_status":
                pose = data.get("pose")
                status = data.get("overall_status", "unknown")
                active = data.get("active_command")
                if pose:
                    ans = f"The robot is currently {status} at map coordinates ({pose['x']:.2f}, {pose['y']:.2f}) facing {pose['yaw']:.2f} rad."
                else:
                    ans = f"The robot status is {status}."
                if active:
                    ans += f" It is currently navigating to {active.get('destination_name')} (Command {active.get('command_id')}, remaining distance: {active.get('distance_remaining_m', 'unknown')}m)."
                return {"content": ans, "tool_calls": None}

            if tool_name == "list_locations":
                locs = data.get("approved_locations", [])
                lines = [f"- {l['name']} (ID: '{l['location_id']}'): {l['description']}" for l in locs]
                return {"content": "Here are the approved office locations I can navigate to:\n" + "\n".join(lines), "tool_calls": None}

            if tool_name == "navigate_to_location":
                cmd_id = data.get("command_id")
                dest = data.get("destination_name")
                return {
                    "content": f"Navigation to {dest} has been accepted and transit is now underway (Command ID: {cmd_id}). You can check my progress or cancel anytime.",
                    "tool_calls": None,
                }

            if tool_name == "get_command_status":
                cmd_id = data.get("command_id")
                st = data.get("status")
                dist = data.get("distance_remaining_m")
                return {
                    "content": f"Command {cmd_id} status is '{st}' with {dist if dist is not None else 0.0} meters remaining.",
                    "tool_calls": None,
                }

            if tool_name == "cancel_navigation":
                cmd_id = data.get("command_id")
                msg = data.get("message", "Navigation canceled.")
                return {
                    "content": f"Active navigation has been safely stopped. {msg} (Command ID: {cmd_id}).",
                    "tool_calls": None,
                }

            return {"content": f"Executed {tool_name} successfully: {data}", "tool_calls": None}

        # Otherwise, process user message
        text = str(last_msg.get("content", "")).lower().strip()

        # 1. Ambiguity detection
        if any(phrase in text for phrase in [
            "experience room",
            "experience centre",
            "experience center",
            "demo room",
            "to the experience",
            "take me to experience",
        ]) and not any(k in text for k in ["living", "hotel", "hospital"]):
            return {
                "content": "The Experience Centre has three distinct rooms: Living Demo Room ('living_demo'), Hotel Room ('hotel_exp'), and Hospital Room ('hospital_exp'). Which one would you like to visit?",
                "tool_calls": None,
            }

        # 2. Status / location queries
        if any(w in text for w in ["where are you", "robot status", "current status", "telemetry", "how are you", "your pose", "where is the robot"]):
            return {
                "content": None,
                "tool_calls": [
                    {
                        "id": "call_status_01",
                        "type": "function",
                        "function": {"name": "get_robot_status", "arguments": "{}"},
                    }
                ],
            }

        if any(w in text for w in [
            "where can you go",
            "where can you take",
            "list locations",
            "list the destinations",
            "destinations",
            "locations",
            "available destinations",
            "approved locations",
            "destinations list",
            "what rooms",
            "available rooms",
        ]):
            return {
                "content": None,
                "tool_calls": [
                    {
                        "id": "call_locs_01",
                        "type": "function",
                        "function": {"name": "list_locations", "arguments": "{}"},
                    }
                ],
            }

        # 3. Stop / Cancellation
        if any(w in text for w in ["stop", "cancel", "halt", "abort"]):
            return {
                "content": None,
                "tool_calls": [
                    {
                        "id": "call_cancel_01",
                        "type": "function",
                        "function": {"name": "cancel_navigation", "arguments": "{}"},
                    }
                ],
            }

        # 4. Command Status
        cmd_match = re.search(r"cmd_[a-f0-9]{8}", text)
        if cmd_match and ("status" in text or "progress" in text or "check" in text):
            return {
                "content": None,
                "tool_calls": [
                    {
                        "id": "call_cmd_st_01",
                        "type": "function",
                        "function": {"name": "get_command_status", "arguments": json.dumps({"command_id": cmd_match.group(0)})},
                    }
                ],
            }

        # 5. Navigation intent
        if any(w in text for w in ["navigate", "go to", "take me to", "drive to", "travel to", "head to"]):
            if "living" in text or "living demo" in text:
                loc_id = "living_demo"
            elif "hotel" in text:
                loc_id = "hotel_exp"
            elif "hospital" in text:
                loc_id = "hospital_exp"
            elif "board" in text or "conference" in text:
                loc_id = "board_room"
            elif "account" in text or "workstation" in text:
                loc_id = "accounts_ws"
            elif "lobby" in text or "reception" in text or "entrance" in text:
                loc_id = "lobby"
            else:
                # Extract requested destination or pass through for validation
                # e.g. "go to secret_room" -> location_id="secret_room"
                parts = text.split("to")
                loc_id = parts[-1].strip().replace(" ", "_") if len(parts) > 1 else "unknown"

            return {
                "content": None,
                "tool_calls": [
                    {
                        "id": "call_nav_01",
                        "type": "function",
                        "function": {
                            "name": "navigate_to_location",
                            "arguments": json.dumps({"location_id": loc_id}),
                        },
                    }
                ],
            }

        # 6. Conversational fallthrough
        return {
            "content": "Hello! I am your Autonomous Office Robot Assistant. I can take you to the Lobby, Living Demo Room, Hotel Room, Hospital Room, Board Room, or Accounts Workstations. How can I help you today?",
            "tool_calls": None,
        }


class OpenAICompatibleProvider(BaseLLMProvider):
    """Integrates with OpenAI, OpenRouter, Ollama, vLLM, or generic OpenAI-compatible APIs."""

    def __init__(
        self,
        api_key: str,
        model: str,
        base_url: Optional[str] = None,
        timeout: float = 15.0,
    ):
        self.api_key = api_key
        self.model = model
        self.base_url = (base_url or "https://api.openai.com/v1").rstrip("/")
        self.timeout = timeout

    async def generate_response(
        self,
        messages: List[Dict[str, Any]],
        tools: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        payload = {
            "model": self.model,
            "messages": messages,
            "tools": tools,
            "tool_choice": "auto",
        }

        async with httpx.AsyncClient(timeout=self.timeout) as client:
            resp = None
            for attempt in range(3):
                resp = await client.post(
                    f"{self.base_url}/chat/completions",
                    headers=headers,
                    json=payload,
                )
                # Retry on transient server busy/high demand spikes (HTTP 503 or 429)
                if resp.status_code in (503, 429) and attempt < 2:
                    await asyncio.sleep(1.5 * (attempt + 1))
                    continue
                break

            if resp is None or resp.status_code != 200:
                error_body = resp.text if resp else "No response"
                raise RuntimeError(
                    f"LLM Provider API error (HTTP {resp.status_code if resp else 'N/A'}): {error_body}"
                )

            data = resp.json()
            choice = data["choices"][0]["message"]
            raw_content = choice.get("content")
            if raw_content:
                cleaned_content = re.sub(r"<thought>[\s\S]*?</thought>", "", raw_content).strip()
                if cleaned_content:
                    raw_content = cleaned_content

            return {
                "content": raw_content,
                "tool_calls": choice.get("tool_calls"),
            }


class LLMService:
    """
    Central orchestration service managing the LLM provider, conversation context,
    tool execution loop, and safe argument validation.
    """

    def __init__(
        self,
        ros_adapter: RosAdapter,
        command_manager: CommandManager,
        provider: Optional[BaseLLMProvider] = None,
    ):
        self.ros_adapter = ros_adapter
        self.command_manager = command_manager
        self._custom_provider = provider
        self._conversations: Dict[str, List[Dict[str, Any]]] = {}

    def get_provider(self) -> BaseLLMProvider:
        if self._custom_provider:
            return self._custom_provider

        from api.config import load_env_file
        load_env_file()

        prov = settings.llm_provider
        if prov == "mock":
            return MockLLMProvider()

        if prov in ("openai", "openrouter", "vllm", "ollama", "gemini"):
            key = settings.llm_api_key
            if not key and prov not in ("ollama", "vllm"):
                raise ValueError(
                    f"LLM provider '{prov}' selected, but no API key was configured in LLM_API_KEY, OPENAI_API_KEY, or GEMINI_API_KEY."
                )
            base_url = settings.llm_base_url
            if prov == "gemini" and not base_url:
                base_url = "https://generativelanguage.googleapis.com/v1beta/openai"

            return OpenAICompatibleProvider(
                api_key=key or "dummy_key",
                model=settings.llm_model,
                base_url=base_url,
                timeout=settings.llm_timeout_seconds,
            )

        # Fallback to Mock if unspecified or unknown
        logger.warning(f"Unrecognized LLM provider '{prov}', defaulting to MockLLMProvider.")
        return MockLLMProvider()

    async def chat(
        self,
        message: str,
        conversation_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Handles an incoming user chat message, executes any requested allowlisted tools,
        and returns the assistant's human-readable response and tool execution audit trail.
        """
        conv_id = conversation_id or "default"
        if conv_id not in self._conversations:
            self._conversations[conv_id] = [
                {"role": "system", "content": SYSTEM_PROMPT}
            ]

        history = self._conversations[conv_id]
        history.append({"role": "user", "content": message.strip()})

        provider = self.get_provider()
        executed_tool_records: List[Dict[str, Any]] = []
        active_cmd_id: Optional[str] = None

        # Maximum 3 tool calling loops to avoid runaway recursion
        for _ in range(3):
            step_result = await provider.generate_response(
                messages=history,
                tools=TOOL_DEFINITIONS,
            )

            content = step_result.get("content")
            raw_tool_calls = step_result.get("tool_calls")

            # If the model produced a natural-language response without tool calls, we are done
            if not raw_tool_calls:
                history.append({"role": "assistant", "content": content or ""})
                return {
                    "response": content or "",
                    "command_id": active_cmd_id,
                    "tool_calls": executed_tool_records,
                    "conversation_id": conv_id,
                }

            # Add assistant's tool-call request to history
            history.append({
                "role": "assistant",
                "content": content,
                "tool_calls": raw_tool_calls,
            })

            # Execute allowlisted tool calls server-side
            for tc in raw_tool_calls:
                fn = tc.get("function", {})
                t_name = fn.get("name", "")
                raw_args = fn.get("arguments", "{}")
                if isinstance(raw_args, str):
                    try:
                        args = json.loads(raw_args)
                    except Exception:
                        args = {}
                else:
                    args = raw_args or {}

                # Execute with strict validation
                result = execute_tool(
                    tool_name=t_name,
                    arguments=args,
                    ros_adapter=self.ros_adapter,
                    command_manager=self.command_manager,
                )

                if "command_id" in result:
                    active_cmd_id = result["command_id"]

                executed_tool_records.append({
                    "tool_name": t_name,
                    "arguments": args,
                    "result": result,
                })

                # Append tool result to history for next model turn
                history.append({
                    "role": "tool",
                    "tool_call_id": tc.get("id", f"call_{t_name}"),
                    "name": t_name,
                    "content": json.dumps(result),
                })

        # Final pass after tool executions
        final_step = await provider.generate_response(messages=history, tools=TOOL_DEFINITIONS)
        final_content = final_step.get("content") or "Operations completed."
        history.append({"role": "assistant", "content": final_content})

        return {
            "response": final_content,
            "command_id": active_cmd_id,
            "tool_calls": executed_tool_records,
            "conversation_id": conv_id,
        }
