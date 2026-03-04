# IMU_GPS — Farm Robot IMU & GPS Data Acquisition

Real-time BNO085 + ESP32-C3 IMU data acquisition with a Web 3D visualization interface.

---

## Repository Structure

```
IMU_GPS/
├── 00_IMU/
│   ├── bno085_esp32c3/
│   │   └── bno085_esp32c3.ino   # ESP32-C3 Arduino firmware
│   ├── serial_bridge.py          # Python serial → WebSocket bridge
│   ├── requirements.txt          # pyserial>=3.5, websockets>=12.0
│   └── web_static/
│       ├── index.html            # Visualization page
│       ├── imu_visualizer.js     # Three.js r160 3D renderer + WebSocket client
│       └── style.css             # Dark theme styles
├── 02_RTK/
│   └── rtk_reader.py            # RTK GPS serial reader
└── config.py                    # Shared configuration
```

---

## Hardware Wiring (ESP32-C3)

| Signal | GPIO |
|--------|------|
| MOSI   | 1    |
| MISO   | 6    |
| SCK    | 7    |
| CS     | 0    |
| INT    | 5    |
| RST    | 2    |
| BOOT   | 9 (hold 3 s to save calibration) |

---

## Quick Start

### 1. Flash Firmware

Open `00_IMU/bno085_esp32c3/bno085_esp32c3.ino` in Arduino IDE, select **ESP32C3 Dev Module**, and upload.

Required library (install via Arduino Library Manager):
- **Adafruit BNO08x** (not the SparkFun variant)

### 2. Start the Bridge Server

```bash
cd 00_IMU
pip install -r requirements.txt
python serial_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
```

CLI arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `/dev/ttyACM0` | ESP32-C3 serial device |
| `--baud` | `921600` | Serial baud rate |
| `--ws-port` | `8765` | HTTP static file server port; WebSocket uses `ws-port + 1` (8766) |

### 3. Open the Visualizer

Desktop browser: `http://localhost:8765`

Mobile browser (LAN): `http://<host-ip>:8765`

---

## Web Visualizer Features

### Data Panel

Displays live sensor data:
- **Rotation Vector** — magnetometer-fused quaternion
- **Heading** — bearing angle + compass direction (derived from Game Rotation Vector)
- **Euler Angles** — roll / pitch / yaw (computed by Python)
- **Accelerometer**, **Linear Accel**, **Gravity**
- **Gyroscope**, **Game Rotation Vector** (no magnetometer), **Magnetometer**
- Step count, calibration level, data rate (Hz)

### Toolbar Buttons

| Button | Description |
|--------|-------------|
| Input box + **Set Heading** | Manually enter a bearing (0–359°) to label the current IMU direction as that angle |
| **Clear North** | Remove the north calibration offset and restore the raw heading |
| **Reset View** | Return the 3D camera to its default position |
| **Lock Yaw** | Toggle: strip pitch & roll, keep only yaw — useful for observing horizontal bearing |
| **Pause / Resume** | Toggle: freeze / resume live data updates |

---

## Architecture

```
ESP32-C3 (BNO085 via SPI)
    │  Serial JSON frames @ 921600 baud
    ▼
serial_bridge.py
    ├── Computes Euler angles (roll / pitch / yaw)
    ├── Serves web_static/ over HTTP → :8765
    └── Broadcasts over WebSocket → :8766
            │
            ▼
    Browser — imu_visualizer.js
        ├── Three.js 3D rendering (SLERP smoothing)
        ├── Compass HUD (Canvas 2D)
        └── Toolbar controls (northOffset / lockYaw / pause)
```

**Coordinate frame**: BNO085 uses a Z-up right-hand frame; Three.js uses Y-up right-hand. The bridge applies a frame-correction quaternion `(-√2/2, 0, 0, √2/2)` (−90° rotation around X) via `premultiply`.

---

## Sensor Calibration

1. On power-up the sensor automatically loads the last saved calibration (DCD).
2. Move and rotate the device until all axes reach **High (3)** calibration (green badge in the data panel).
3. Hold **BOOT (GPIO 9) for 3 seconds** to persist the current calibration to the sensor's non-volatile storage.

---

## Dependencies

| Dependency | Version |
|------------|---------|
| Python | ≥ 3.10 |
| pyserial | ≥ 3.5 |
| websockets | ≥ 12.0 |
| Three.js | r160 (CDN) |
| Arduino IDE | 2.x |
| Adafruit BNO08x | latest |

---

## RTK Map Visualizer (New)

RTK module files:

```
02_RTK/
├── rtk_reader.py      # Emlid RS+ NMEA parser (GGA/RMC)
├── rtk_bridge.py      # Python RTK -> WebSocket + HTTP static UI
└── web_static/
    ├── index.html
    ├── rtk_visualizer.js
    └── style.css
```

Run:

```bash
cd 02_RTK
python rtk_bridge.py --ws-port 8775 --hz 5 --open-browser
```

Open: `http://localhost:8775`

Notes:
- If RTK does not provide valid `lat/lon`, UI falls back to `38.9412928598587, -92.31884600793728`.
- CSV (or tab-delimited) waypoint import is supported; minimum columns are `lat, lon`; `tolerance_m, max_speed` are optional.
- Track segments are color-coded by progress:
  - Green: inside current waypoint tolerance (reached)
  - Orange: approaching target (within 2x tolerance)
  - Red: off path
- Driving logs are recorded in-browser and can be exported as CSV.
