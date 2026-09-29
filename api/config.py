from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def load_env_file(env_file_path: Optional[str] = None) -> None:
    """Loads environment variables from a .env file if present."""
    if env_file_path:
        p = Path(env_file_path)
    else:
        # Search current working directory and project root
        candidates = [Path(".env"), Path("/workspace/gazebo_four_wheel_robot/.env")]
        p = next((c for c in candidates if c.is_file()), None)

    if not p or not p.exists():
        return

    try:
        with open(p, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k = k.strip()
                v = v.strip().strip("'\"")
                if k:
                    os.environ[k] = v
    except Exception:
        pass


# Automatically attempt to load .env on import
load_env_file()


class Settings:
    @property
    def llm_provider(self) -> str:
        """Provider name: 'mock', 'openai', 'gemini', 'anthropic', etc."""
        return os.environ.get("LLM_PROVIDER", "mock").strip().lower()

    @property
    def llm_model(self) -> str:
        """Model identifier (e.g. 'gpt-4o-mini', 'gemini-1.5-flash', 'claude-3-5-sonnet-20241022')."""
        provider = self.llm_provider
        default_model = {
            "openai": "gpt-4o-mini",
            "gemini": "gemini-1.5-flash",
            "anthropic": "claude-3-5-sonnet-20241022",
            "mock": "mock-robot-llm-v1",
        }.get(provider, "mock-robot-llm-v1")
        return os.environ.get("LLM_MODEL", default_model).strip()

    @property
    def llm_api_key(self) -> Optional[str]:
        """Reads API key from LLM_API_KEY, OPENAI_API_KEY, GEMINI_API_KEY, or ANTHROPIC_API_KEY."""
        return (
            os.environ.get("LLM_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or os.environ.get("GEMINI_API_KEY")
            or os.environ.get("ANTHROPIC_API_KEY")
        )

    @property
    def llm_base_url(self) -> Optional[str]:
        """Custom API base URL (useful for local models, Ollama, OpenRouter, vLLM)."""
        return os.environ.get("LLM_BASE_URL")

    @property
    def llm_timeout_seconds(self) -> float:
        """Timeout for external LLM HTTP calls."""
        try:
            return float(os.environ.get("LLM_TIMEOUT_SECONDS", "25.0"))
        except ValueError:
            return 25.0


settings = Settings()
