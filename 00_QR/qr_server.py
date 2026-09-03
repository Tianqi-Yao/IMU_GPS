"""qr_server.py — LAN QR code launcher page for the IMU_GPS bridge stack.

Serves a single static HTML page (generated once at startup, then served
from memory) with WiFi-join and per-module access QR codes. No WebSocket
server, no external image library required (uses qrcode's SvgPathImage
factory so Pillow is not needed).

Input contract: none besides config.py (QR_PORT, WIFI_*, per-module *_WS_PORT).
Output contract: GET / -> HTML page with embedded SVG QR codes.
Run: python qr_server.py
"""

import html
import io
import logging
import socketserver
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler
from pathlib import Path

import qrcode
import qrcode.image.svg

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

DEFAULT_QR_PORT      = _cfg.QR_PORT if _cfg else 8700
DEFAULT_WIFI_SSID    = _cfg.WIFI_SSID if _cfg else ""
DEFAULT_WIFI_PASS    = _cfg.WIFI_PASSWORD if _cfg else ""
DEFAULT_WIFI_AUTH    = _cfg.WIFI_AUTH_TYPE if _cfg else "WPA"

MODULES = [
    ("01 IMU",     _cfg.IMU_WS_PORT if _cfg else 8765,     "IMU Attitude Monitor"),
    ("02 RTK",     _cfg.RTK_WS_PORT if _cfg else 8775,     "RTK GPS Map"),
    ("03 Nav",     _cfg.NAV_WS_PORT if _cfg else 8785,     "Navigation (IMU+RTK passthrough)"),
    ("04 Robot",   _cfg.ROBOT_WS_PORT if _cfg else 8888,   "Robot Controller"),
    ("05 AutoNav", _cfg.AUTONAV_WS_PORT if _cfg else 8805, "Autonomous Navigation"),
    ("06 Camera",  _cfg.CAM_WS_PORT if _cfg else 8815,     "Camera Stream"),
    ("07 SysMon",  _cfg.SYSMON_WS_PORT if _cfg else 8825,  "System Health Monitor"),
]


def get_local_ip() -> str:
    import socket

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def _wifi_escape(value: str) -> str:
    for ch in ("\\", ";", ",", '"', ":"):
        value = value.replace(ch, "\\" + ch)
    return value


def make_wifi_qr_payload(ssid: str, password: str, auth_type: str) -> str:
    auth = "nopass" if not password else auth_type
    payload = f"WIFI:T:{auth};S:{_wifi_escape(ssid)};"
    if auth != "nopass":
        payload += f"P:{_wifi_escape(password)};"
    payload += ";"
    return payload


def make_qr_svg(data: str) -> str:
    qr = qrcode.QRCode(image_factory=qrcode.image.svg.SvgPathImage, box_size=5, border=2)
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image()
    buf = io.BytesIO()
    img.save(buf)
    return buf.getvalue().decode("utf-8")


def build_page(ip: str) -> str:
    wifi_card = ""
    if DEFAULT_WIFI_SSID:
        wifi_payload = make_wifi_qr_payload(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS, DEFAULT_WIFI_AUTH)
        wifi_svg = make_qr_svg(wifi_payload)
        wifi_card = f"""
        <div class="card wifi-card">
          <h2>WiFi</h2>
          <div class="qr">{wifi_svg}</div>
          <div class="label">{html.escape(DEFAULT_WIFI_SSID)}</div>
        </div>"""

    module_cards = []
    for name, port, label in MODULES:
        url = f"http://{ip}:{port}"
        svg = make_qr_svg(url)
        module_cards.append(f"""
        <div class="card">
          <h2>{html.escape(name)}</h2>
          <div class="qr">{svg}</div>
          <div class="label">{html.escape(label)}</div>
          <a class="url" href="{url}" target="_blank">{url}</a>
        </div>""")

    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IMU_GPS Launcher</title>
<style>
  body {{ margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
         background: #f0f4f0; color: #1a3a1a; }}
  header {{ background: #1a3a1a; color: #e8f5e8; padding: 24px; text-align: center; }}
  header h1 {{ margin: 0; font-size: 1.5rem; }}
  .grid {{ display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px; padding: 24px; }}
  .card {{ background: #fff; border-radius: 12px; padding: 16px; text-align: center;
           box-shadow: 0 1px 4px rgba(0,0,0,0.1); }}
  .card h2 {{ margin: 0 0 8px; font-size: 1.1rem; }}
  .qr {{ width: 100%; max-width: 220px; margin: 0 auto; }}
  .qr svg {{ width: 100%; height: auto; }}
  .label {{ margin-top: 8px; color: #446644; font-size: 0.9rem; }}
  .url {{ display: block; margin-top: 8px; color: #1a5a1a; text-decoration: none; word-break: break-all; }}
  .url:hover {{ text-decoration: underline; }}
  @media (max-width: 680px) {{ .grid {{ grid-template-columns: repeat(2, 1fr); }} }}
  @media (max-width: 420px) {{ .grid {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<header><h1>IMU_GPS — LAN Launcher</h1></header>
<div class="grid">{wifi_card}{"".join(module_cards)}</div>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    _page: bytes = b""

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(self._page)))
            self.end_headers()
            self.wfile.write(self._page)
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def main() -> None:
    ip = get_local_ip()
    _Handler._page = build_page(ip).encode("utf-8")

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", DEFAULT_QR_PORT), _Handler) as httpd:
        url = f"http://{ip}:{DEFAULT_QR_PORT}"
        logger.info("QR Launcher: %s", url)
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            logger.info("Stopped")


if __name__ == "__main__":
    main()
