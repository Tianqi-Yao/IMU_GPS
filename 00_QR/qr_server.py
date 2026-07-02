"""
qr_server.py — Serve a LAN QR code launcher page.

Opens http://<LAN_IP>:8700 and shows 6 QR codes for modules 01~06.
Phone on the same LAN scans a code to open the corresponding page.

Dependency: pip install qrcode   (no Pillow required)

Usage:
    python 00_QR/qr_server.py
"""

import io
import socket
import socketserver
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler

import qrcode
import qrcode.image.svg

import rootutils
ROOT = rootutils.setup_root(__file__, indicator=".git", pythonpath=True)
try:
    import config as _cfg
except ImportError:
    _cfg = None

QR_PORT = _cfg.QR_PORT if _cfg else 8700

MODULES = [
    ("01 IMU",      _cfg.IMU_WS_PORT     if _cfg else 8765, "IMU Attitude Monitor"),
    ("02 RTK",      _cfg.RTK_WS_PORT     if _cfg else 8775, "RTK GPS Map"),
    ("03 Nav",      _cfg.NAV_WS_PORT     if _cfg else 8785, "Navigation (Fused)"),
    ("04 Robot",    _cfg.ROBOT_WS_PORT   if _cfg else 8888, "Robot Controller"),
    ("05 AutoNav",  _cfg.AUTONAV_WS_PORT if _cfg else 8805, "Autonomous Navigation"),
    ("06 Camera",   _cfg.CAM_WS_PORT     if _cfg else 8815, "Camera Stream"),
]


def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def make_qr_svg(url: str) -> str:
    factory = qrcode.image.svg.SvgPathImage
    qr = qrcode.QRCode(box_size=5, border=2)
    qr.add_data(url)
    qr.make(fit=True)
    img = qr.make_image(image_factory=factory)
    buf = io.BytesIO()
    img.save(buf)
    return buf.getvalue().decode("utf-8")


def build_page(ip: str) -> str:
    cards = ""
    for name, port, desc in MODULES:
        url = f"http://{ip}:{port}"
        svg = make_qr_svg(url)
        cards += f"""
        <div class="card">
          <div class="mod-name">{name}</div>
          <div class="mod-desc">{desc}</div>
          <div class="qr">{svg}</div>
          <a class="url" href="{url}" target="_blank">{url}</a>
        </div>"""

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>IMU_GPS Launcher</title>
  <style>
    body {{font-family:system-ui,sans-serif;margin:0;background:#f0f4f0;color:#1a2e1a}}
    header {{background:#1a3a1a;color:#e8f5e8;padding:18px 24px}}
    header h1 {{margin:0;font-size:1.3rem}}
    header p  {{margin:6px 0 0;font-size:0.95rem;opacity:.8}}
    .grid {{display:grid;grid-template-columns:repeat(3,1fr);gap:18px;
            padding:24px;max-width:900px;margin:0 auto}}
    @media(max-width:680px){{.grid{{grid-template-columns:repeat(2,1fr)}}}}
    @media(max-width:420px){{.grid{{grid-template-columns:1fr}}}}
    .card {{background:#fff;border-radius:12px;padding:16px;
            box-shadow:0 2px 8px rgba(0,0,0,.08);text-align:center}}
    .mod-name {{font-weight:700;font-size:1.05rem;margin-bottom:2px}}
    .mod-desc {{font-size:0.8rem;color:#5a7a5a;margin-bottom:10px}}
    .qr svg   {{width:100%;max-width:180px;height:auto}}
    .url      {{display:block;margin-top:8px;font-size:0.78rem;
                color:#2b6cb0;word-break:break-all;text-decoration:none}}
    .url:hover{{text-decoration:underline}}
  </style>
</head>
<body>
  <header>
    <h1>IMU_GPS Control Panel</h1>
    <p>LAN IP: <strong>{ip}</strong> — scan to open on phone</p>
  </header>
  <div class="grid">{cards}</div>
</body>
</html>"""


class _Handler(BaseHTTPRequestHandler):
    _page: bytes = b""

    def do_GET(self):
        if self.path not in ("/", "/index.html"):
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(self._page)))
        self.end_headers()
        self.wfile.write(self._page)

    def log_message(self, fmt, *args):
        pass  # suppress per-request logs


if __name__ == "__main__":
    ip = get_local_ip()
    _Handler._page = build_page(ip).encode("utf-8")  # build once, serve forever

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", QR_PORT), _Handler) as httpd:
        url = f"http://{ip}:{QR_PORT}"
        print(f"QR Launcher: {url}")
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
        httpd.serve_forever()
