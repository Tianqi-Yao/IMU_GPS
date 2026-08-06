"""common/http_server.py — shared static-file HTTP server.

Framework-only: serves a directory of static files on a background thread.
Two narrow extension points exist because two consumer modules need them,
but neither carries business logic — both are pure HTTP plumbing hooks:
  - index_transform: rewrite index.html bytes before sending (04_Robot uses
    this to inject data-max-linear/data-max-angular attributes)
  - extra_routes: serve a handful of extra non-static paths (05_AutoNav's
    GET /export_csv)
"""

import logging
import socketserver
import threading
from http.server import SimpleHTTPRequestHandler
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

IndexTransform = Callable[[bytes], bytes]
RouteHandler = Callable[[], Tuple[bytes, str]]


def _make_handler(
    directory: str,
    index_transform: Optional[IndexTransform],
    extra_routes: Optional[Dict[str, RouteHandler]],
    no_cache: bool,
):
    class _Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=directory, **kwargs)

        def do_GET(self):
            if extra_routes and self.path in extra_routes:
                body, content_type = extra_routes[self.path]()
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if index_transform is not None and self.path in ("/", "/index.html"):
                index_path = Path(directory) / "index.html"
                if index_path.exists():
                    body = index_transform(index_path.read_bytes())
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
            super().do_GET()

        def end_headers(self):
            if no_cache:
                self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def log_message(self, format, *args):
            pass

    return _Handler


class StaticFileServer:
    """Serves `directory` over HTTP on `port`, in a background daemon thread."""

    def __init__(
        self,
        directory: Path,
        port: int,
        host: str = "0.0.0.0",
        index_transform: Optional[IndexTransform] = None,
        extra_routes: Optional[Dict[str, RouteHandler]] = None,
        no_cache: bool = False,
        threaded: bool = False,
    ) -> None:
        self._directory = str(directory)
        self._port = port
        self._host = host
        self._threaded = threaded
        self._handler_cls = _make_handler(self._directory, index_transform, extra_routes, no_cache)
        self._httpd: Optional[socketserver.TCPServer] = None

    def start(self) -> None:
        threading.Thread(target=self._run, name="http-server", daemon=True).start()

    def _run(self) -> None:
        server_cls = socketserver.ThreadingTCPServer if self._threaded else socketserver.TCPServer
        server_cls.allow_reuse_address = True
        try:
            with server_cls((self._host, self._port), self._handler_cls) as httpd:
                self._httpd = httpd
                httpd.serve_forever()
        except Exception:
            logger.error("HTTP server failed to start on port %d", self._port, exc_info=True)
