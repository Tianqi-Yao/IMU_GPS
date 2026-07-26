"""common/ports.py — HTTP -> WebSocket port derivation.

Repo-wide convention: every bridge exposes a static-file HTTP server on a
base port and a WebSocket broadcast server on that same port + 1. Bridges
should call derive_ws_port() instead of hardcoding "+ 1" so the convention
is expressed in exactly one place.
"""


def derive_ws_port(http_port: int) -> int:
    """Return the WebSocket port for a given HTTP port."""
    return http_port + 1
