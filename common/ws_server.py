"""common/ws_server.py — shared WebSocket broadcast server.

Framework-only: manages the set of connected clients, a broadcast(payload)
primitive, and dispatches inbound client messages to a caller-supplied
callback. No module-specific message schemas (imu_frame, joystick, cmd_start,
...) belong here — those are decoded/dispatched by each bridge's own
on_client_message callback.
"""

import asyncio
import inspect
import json
import logging
from typing import Any, Awaitable, Callable, Optional, Union

import websockets
from websockets.exceptions import ConnectionClosed

logger = logging.getLogger(__name__)

OnClientMessage = Callable[[Any, str], Union[Optional[str], Awaitable[Optional[str]]]]
OnClientConnect = Callable[[Any], Union[None, Awaitable[None]]]


class BroadcastWsServer:
    """Manages a set of WebSocket clients and broadcasts JSON payloads to them."""

    SEND_TIMEOUT_S = 5.0

    def __init__(
        self,
        host: str,
        port: int,
        on_client_message: Optional[OnClientMessage] = None,
        on_client_connect: Optional[OnClientConnect] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._on_client_message = on_client_message
        self._on_client_connect = on_client_connect
        self._clients: set = set()
        self._lock = asyncio.Lock()

    @property
    def client_count(self) -> int:
        return len(self._clients)

    async def broadcast(self, payload: dict) -> None:
        """Serialize `payload` as JSON and send it concurrently to every
        connected client. A slow/stalled client (bounded by SEND_TIMEOUT_S)
        cannot delay delivery to the rest.
        """
        message = json.dumps(payload)
        async with self._lock:
            clients = set(self._clients)
        if not clients:
            return

        async def _send(ws) -> Optional[Any]:
            try:
                await asyncio.wait_for(ws.send(message), timeout=self.SEND_TIMEOUT_S)
                return None
            except Exception:
                return ws

        results = await asyncio.gather(*(_send(ws) for ws in clients))
        dead = {ws for ws in results if ws is not None}
        if dead:
            async with self._lock:
                self._clients.difference_update(dead)

    async def _handle_client(self, websocket) -> None:
        async with self._lock:
            self._clients.add(websocket)
        try:
            if self._on_client_connect is not None:
                try:
                    result = self._on_client_connect(websocket)
                    if inspect.isawaitable(result):
                        await result
                except ConnectionClosed:
                    raise
                except Exception:
                    logger.warning("on_client_connect handler failed", exc_info=True)
            async for raw in websocket:
                if self._on_client_message is None:
                    continue
                try:
                    result = self._on_client_message(websocket, raw)
                    if inspect.isawaitable(result):
                        result = await result
                    if result:
                        await websocket.send(result)
                except ConnectionClosed:
                    raise
                except Exception:
                    logger.warning("on_client_message handler failed", exc_info=True)
        except ConnectionClosed:
            pass
        finally:
            async with self._lock:
                self._clients.discard(websocket)

    async def serve(self) -> None:
        """Start the WebSocket server on host:port and block forever."""
        async with websockets.serve(self._handle_client, self._host, self._port):
            await asyncio.Future()
