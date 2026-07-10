"""
config.py — Global hyperparameter configuration

All default parameters for modules 00~06 are defined here.
CLI arguments can still override these values at runtime.
Edit this file and restart the relevant module to apply changes.
"""

# ══════════════════════════════════════════════════════════════════════════════
# 00_QR  — LAN QR code launcher page
# ══════════════════════════════════════════════════════════════════════════════
QR_PORT = 8700   # HTTP port for the QR launcher page

# WiFi QR code (phone scans to auto-join network). Set WIFI_SSID to enable;
# leave empty to hide the WiFi card on the launcher page.
WIFI_SSID      = ""      # WiFi network name, e.g. "MyHomeWiFi"
WIFI_PASSWORD  = ""      # WiFi password (leave "" for WIFI_AUTH_TYPE = "nopass")
WIFI_AUTH_TYPE = "WPA"   # "WPA" (covers WPA/WPA2/WPA3), "WEP", or "nopass"

# ══════════════════════════════════════════════════════════════════════════════
# 01_IMU  — BNO085 serial → WebSocket bridge
# ══════════════════════════════════════════════════════════════════════════════
# ls /dev/cu.*
IMU_SERIAL_PORT   = "/dev/cu.usbmodem1201"   # Serial device (Mac: cu.usbmodem*, Linux: /dev/ttyACM0)
IMU_BAUD          = 921600                   # Serial baud rate
IMU_WS_PORT       = 8765                     # HTTP port; WebSocket = IMU_WS_PORT + 1
IMU_NORTH_OFFSET  = 0.0                      # North heading offset (degrees) for yaw calibration


# ══════════════════════════════════════════════════════════════════════════════
# 02_RTK  — RTK GPS serial → WebSocket bridge
# ══════════════════════════════════════════════════════════════════════════════

RTK_SERIAL_PORT   = "/dev/cu.usbmodem1101"  # RTK receiver serial port (Linux: /dev/ttyACM1)
RTK_BAUD          = 115200                     # Serial baud rate (NMEA standard)
RTK_WS_PORT       = 8775                     # HTTP port; WebSocket = RTK_WS_PORT + 1
RTK_HZ            = 5.0                      # Broadcast rate (Hz)
RTK_DEFAULT_LAT   = 38.9412928598587         # Fallback latitude when no GPS fix
RTK_DEFAULT_LON   = -92.31884600793728       # Fallback longitude when no GPS fix


# ══════════════════════════════════════════════════════════════════════════════
# 03_Nav  — IMU + RTK fused navigation
# ══════════════════════════════════════════════════════════════════════════════

NAV_WS_PORT           = 8785                      # HTTP port; WebSocket = NAV_WS_PORT + 1
NAV_IMU_WS            = "ws://localhost:8766"     # imu_bridge WebSocket URL
NAV_RTK_WS            = "ws://localhost:8776"     # rtk_bridge WebSocket URL
NAV_HZ                = 10.0                      # Navigation loop broadcast rate (Hz)
NAV_REACH_TOLERANCE_M = 0.5                       # Waypoint arrival tolerance (meters)


# ══════════════════════════════════════════════════════════════════════════════
# 04_Robot  — Robot serial control bridge
# ══════════════════════════════════════════════════════════════════════════════

ROBOT_WS_PORT          = 8888                      # HTTP port; WebSocket = ROBOT_WS_PORT + 1
ROBOT_SERIAL_PORT      = "/dev/cu.usbmodem1301"   # Feather M4 serial port (Linux: /dev/ttyACM0)
ROBOT_SERIAL_BAUD      = 115200                    # Serial baud rate
ROBOT_SERIAL_TIMEOUT   = 0.05                      # Serial read timeout (seconds)
ROBOT_MAX_LINEAR       = 1.0                       # Max linear velocity (m/s)
ROBOT_MAX_ANGULAR      = 1.0                       # Max angular velocity (rad/s)
ROBOT_MAX_LINEAR_ACCEL  = 1.5                      # Max linear acceleration (m/s^2); limits joystick ramp rate
ROBOT_MAX_ANGULAR_ACCEL = 3.0                      # Max angular acceleration (rad/s^2); limits joystick ramp rate
ROBOT_WATCHDOG_TIMEOUT = 3.0                       # Watchdog timeout (seconds); triggers e-stop
ROBOT_RECORD_INTERVAL  = 1.0                       # Recording sample interval (seconds); min 0.2


# ══════════════════════════════════════════════════════════════════════════════
# 05_AutoNav  — Autonomous navigation (PID + Pure Pursuit)
# ══════════════════════════════════════════════════════════════════════════════

AUTONAV_WS_PORT         = 8805                      # HTTP port; WebSocket = AUTONAV_WS_PORT + 1
AUTONAV_IMU_WS          = "ws://localhost:8766"     # imu_bridge WebSocket URL
AUTONAV_RTK_WS          = "ws://localhost:8776"     # rtk_bridge WebSocket URL
AUTONAV_ROBOT_WS        = "ws://localhost:8889"     # robot_bridge WebSocket URL
AUTONAV_GPS_TIMEOUT_S   = 5.0                       # GPS data timeout (seconds)
AUTONAV_CONTROL_HZ      = 5.0                       # Navigation control loop frequency (Hz)
AUTONAV_ROBOT_SEND_TIMEOUT_S = 2.0                  # Max time allowed for one robot WS send (seconds)
LIFT_UP_DURATION_S      = 5.0                       # seconds to run UP actuator to reach high position
LIFT_DOWN_DURATION_S    = 5.0                       # seconds to run DOWN actuator to reach low position

# Waypoint generation from a recorded manual run (see build_waypoints_from_run.py)
WAYPOINT_MIN_DISTANCE_M  = 1.0                      # min spacing between generated waypoints (meters); controls WP density
WAYPOINT_MIN_FIX_QUALITY = 1                        # minimum RTK fix_quality to accept a sample


# ══════════════════════════════════════════════════════════════════════════════
# 06_Camera  — OAK-D camera MJPEG streaming bridge
# ══════════════════════════════════════════════════════════════════════════════

CAM_WS_PORT        = 8815             # HTTP port; WebSocket = CAM_WS_PORT + 1
CAM1_IP            = "10.95.76.11"   # OAK-D camera 1 IP address
CAM2_IP            = "10.95.76.10"   # OAK-D camera 2 IP address
CAM1_STREAM_PORT   = 8080            # Camera 1 MJPEG stream port
CAM2_STREAM_PORT   = 8081            # Camera 2 MJPEG stream port
CAM_FPS            = 25              # Frame rate (matches cam_demo/Depth_Align.py)
CAM_WIDTH          = 640             # Frame width (pixels)
CAM_HEIGHT         = 400             # Frame height (pixels)
CAM_MJPEG_QUALITY  = 80              # MJPEG compression quality (1–100)
CAM_DEFAULT_PLUGIN = "simple_color"  # Default processing plugin
CAM_ENABLE_STEREO     = True         # Enable stereo depth (requires Left/Right/StereoDepth nodes)
CAM_ENABLE_DISPARITY  = False         # Enable raw disparity stream alongside depth (adds load; keep False unless debugging)
