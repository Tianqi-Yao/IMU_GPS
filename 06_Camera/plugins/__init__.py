"""
Plugin framework for camera frame sources and frame processors.

FrameSource  — manages a camera device (open/close/get_frame).
               Registered with @register_plugin.

FrameProcessor — stateless image transform applied on top of a running
                 FrameSource stream.  No camera management; just process().
                 Registered with @register_processor.

The "available_plugins" list sent to the browser merges both types.
Switching to a FrameProcessor leaves the camera stream running.
Switching to a FrameSource restarts the stream with the new source.
"""

from __future__ import annotations

import abc
import importlib
import logging
from pathlib import Path

import numpy as np

__all__ = [
    "FrameSource",   "register_plugin",    "get_plugin",    "list_plugins",
    "FrameProcessor","register_processor", "get_processor", "list_processors",
    "is_processor",
]

logger = logging.getLogger(__name__)


# ── FrameSource ABC ─────────────────────────────────────────────────────────

class FrameSource(abc.ABC):
    """Abstract base class for camera frame providers (plugin interface)."""

    PLUGIN_NAME: str = ""
    PLUGIN_LABEL: str = ""
    PLUGIN_DESCRIPTION: str = ""

    @classmethod
    def config_schema(cls) -> list[dict]:
        """Declare configurable params: [{"key", "type", "default", "label"}, ...]"""
        return []

    @abc.abstractmethod
    def open(self) -> None:
        """Initialize camera resources."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release all camera resources."""

    @abc.abstractmethod
    def get_frame(self) -> np.ndarray | None:
        """Return latest BGR frame or None."""


# ── Plugin Registry ─────────────────────────────────────────────────────────

_plugin_registry: dict[str, type[FrameSource]] = {}


def register_plugin(cls: type[FrameSource]) -> type[FrameSource]:
    """Decorator: register a FrameSource subclass."""
    name = cls.PLUGIN_NAME
    if not name:
        raise ValueError(f"{cls.__name__} has no PLUGIN_NAME")
    _plugin_registry[name] = cls
    logger.info("Plugin registered: '%s' (%s)", name, cls.PLUGIN_LABEL)
    return cls


def get_plugin(name: str) -> type[FrameSource]:
    """Look up a registered plugin by name."""
    if name not in _plugin_registry:
        raise KeyError(
            f"Unknown plugin: '{name}'. Available: {list(_plugin_registry.keys())}"
        )
    return _plugin_registry[name]


def list_plugins() -> list[dict]:
    """Return metadata for all registered sources and processors (combined)."""
    sources = [
        {
            "name": c.PLUGIN_NAME,
            "label": c.PLUGIN_LABEL,
            "description": c.PLUGIN_DESCRIPTION,
            "config_schema": c.config_schema(),
            "is_processor": False,
        }
        for c in _plugin_registry.values()
    ]
    return sources + list_processors()


# ── FrameProcessor ABC ──────────────────────────────────────────────────────

class FrameProcessor(abc.ABC):
    """
    Stateless (or stateful) image transform applied on top of a FrameSource.

    Unlike FrameSource, a FrameProcessor does NOT manage the camera device.
    It receives a raw BGR frame and returns a processed BGR frame.
    Switching processors never restarts the camera stream.
    """

    PROCESSOR_NAME: str = ""
    PROCESSOR_LABEL: str = ""
    PROCESSOR_DESCRIPTION: str = ""

    @classmethod
    def config_schema(cls) -> list[dict]:
        """Declare configurable params: [{"key", "type", "default", "label"}, ...]"""
        return []

    def reconfigure(self, **kwargs) -> None:
        """Apply updated config without recreating the processor (optional override)."""

    @abc.abstractmethod
    def process(self, frame: np.ndarray) -> np.ndarray:
        """Transform frame in-place or return a new BGR array."""


# ── Processor Registry ───────────────────────────────────────────────────────

_processor_registry: dict[str, type[FrameProcessor]] = {}


def register_processor(cls: type[FrameProcessor]) -> type[FrameProcessor]:
    """Decorator: register a FrameProcessor subclass."""
    name = cls.PROCESSOR_NAME
    if not name:
        raise ValueError(f"{cls.__name__} has no PROCESSOR_NAME")
    _processor_registry[name] = cls
    logger.info("Processor registered: '%s' (%s)", name, cls.PROCESSOR_LABEL)
    return cls


def get_processor(name: str) -> type[FrameProcessor]:
    """Look up a registered processor by name."""
    if name not in _processor_registry:
        raise KeyError(
            f"Unknown processor: '{name}'. Available: {list(_processor_registry.keys())}"
        )
    return _processor_registry[name]


def list_processors() -> list[dict]:
    """Return metadata for all registered processors."""
    return [
        {
            "name": c.PROCESSOR_NAME,
            "label": c.PROCESSOR_LABEL,
            "description": c.PROCESSOR_DESCRIPTION,
            "config_schema": c.config_schema(),
            "is_processor": True,
        }
        for c in _processor_registry.values()
    ]


def is_processor(name: str) -> bool:
    """Return True if name is a registered FrameProcessor (not a FrameSource)."""
    return name in _processor_registry


# ── Auto-discovery ──────────────────────────────────────────────────────────

def _auto_discover() -> None:
    """Import all .py files in this directory to trigger @register_plugin."""
    pkg_dir = Path(__file__).parent
    for py_file in sorted(pkg_dir.glob("*.py")):
        if py_file.name.startswith("_"):
            continue
        module_name = f"{__name__}.{py_file.stem}"
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            logger.warning("Failed to load plugin '%s': %s", py_file.stem, exc)


_auto_discover()
