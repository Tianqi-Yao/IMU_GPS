"""
config.py — Global hyperparameter configuration

All default parameters for modules 00~06 are defined here.
Edit this file and restart the relevant module to apply changes.
No CLI argument parsing is added by default; see individual module READMEs.
"""

# ══════════════════════════════════════════════════════════════════════════════
# 00_QR  — LAN QR code launcher page
# ══════════════════════════════════════════════════════════════════════════════
QR_PORT = 8700   # HTTP port for the QR launcher page

# WiFi QR code (phone scans to auto-join network). Set WIFI_SSID to enable;
# leave empty to hide the WiFi card on the launcher page.
WIFI_SSID      = "TP-Link_A5A1"      # WiFi network name, e.g. "MyHomeWiFi"
WIFI_PASSWORD  = "74208999"      # WiFi password (leave "" for WIFI_AUTH_TYPE = "nopass")
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
RTK_BAUD          = 115200                   # Serial baud rate (NMEA standard)
RTK_WS_PORT       = 8775                     # HTTP port; WebSocket = RTK_WS_PORT + 1
RTK_HZ            = 5.0                      # Broadcast rate (Hz)
RTK_DEFAULT_LAT   = 38.9412928598587         # Fallback latitude when no GPS fix
RTK_DEFAULT_LON   = -92.31884600793728       # Fallback longitude when no GPS fix


# ══════════════════════════════════════════════════════════════════════════════
# 03_Nav  — IMU + RTK passthrough (not a real sensor fusion — see 03_Nav/README.md)
# ══════════════════════════════════════════════════════════════════════════════

NAV_WS_PORT           = 8785                      # HTTP port; WebSocket = NAV_WS_PORT + 1
NAV_IMU_WS            = "ws://localhost:8766"     # imu_bridge WebSocket URL
NAV_RTK_WS            = "ws://localhost:8776"     # rtk_bridge WebSocket URL
NAV_HZ                = 10.0                      # Navigation loop broadcast rate (Hz)


# ══════════════════════════════════════════════════════════════════════════════
# 04_Robot  — Robot serial control bridge
# ══════════════════════════════════════════════════════════════════════════════

ROBOT_WS_PORT          = 8888                      # HTTP port; WebSocket = ROBOT_WS_PORT + 1
ROBOT_SERIAL_PORT      = "/dev/cu.usbmodem1301"    # Feather M4 serial port (Linux: /dev/ttyACM0)
ROBOT_SERIAL_BAUD      = 115200                    # Serial baud rate
ROBOT_SERIAL_TIMEOUT   = 0.05                      # Serial read timeout (seconds)
ROBOT_MAX_LINEAR       = 1.0                       # Max linear velocity (m/s)
ROBOT_MAX_ANGULAR      = 1.0                       # Max angular velocity (rad/s)
ROBOT_MAX_LINEAR_ACCEL  = 1.5                      # Max linear acceleration (m/s^2); limits joystick ramp rate
ROBOT_MAX_ANGULAR_ACCEL = 3.0                      # Max angular acceleration (rad/s^2); limits joystick ramp rate
ROBOT_WATCHDOG_TIMEOUT = 3.0                       # Watchdog timeout (seconds); triggers e-stop
ROBOT_RECORD_INTERVAL  = 1.0                       # Recording sample interval (seconds); min 0.2
ROBOT_IMU_WS           = "ws://localhost:8766"     # imu_bridge WebSocket URL (defined here, not borrowed from 03_Nav)
ROBOT_RTK_WS           = "ws://localhost:8776"     # rtk_bridge WebSocket URL (defined here, not borrowed from 03_Nav)


# ══════════════════════════════════════════════════════════════════════════════
# 05_AutoNav  — Autonomous navigation (proportional heading controller + dead zone)
# ══════════════════════════════════════════════════════════════════════════════
# NOTE: this is a P-only heading controller with a hard dead zone, not a PID
# (no integral/derivative terms) and not Pure Pursuit (no lookahead point
# selection — it always aims directly at the current waypoint). This is a
# deliberate, validated v2 simplification; see 05_AutoNav/README.md.

AUTONAV_WS_PORT         = 8805                      # HTTP port; WebSocket = AUTONAV_WS_PORT + 1
AUTONAV_IMU_WS          = "ws://localhost:8766"     # imu_bridge WebSocket URL
AUTONAV_RTK_WS          = "ws://localhost:8776"     # rtk_bridge WebSocket URL
AUTONAV_ROBOT_WS        = "ws://localhost:8889"     # robot_bridge WebSocket URL
AUTONAV_GPS_TIMEOUT_S   = 5.0                       # GPS/IMU data timeout (seconds)
AUTONAV_CONTROL_HZ      = 5.0                       # Navigation control loop frequency (Hz)
AUTONAV_ROBOT_SEND_TIMEOUT_S = 2.0                  # Max time allowed for one robot WS send (seconds)
LIFT_UP_DURATION_S      = 5.0                       # seconds to run UP actuator to reach high position
LIFT_DOWN_DURATION_S    = 5.0                       # seconds to run DOWN actuator to reach low position

# Heading/velocity control tuning (algorithm constants — see autonav_algo.py::compute)
AUTONAV_PID_KP              = 0.02   # proportional gain: angular = KP * heading_error_deg
AUTONAV_DEAD_ZONE_DEG       = 3.0    # |heading_error| below this -> angular = 0 (hard dead zone, not a filter)
AUTONAV_MIN_ANGULAR_KICK    = 0.25   # rad/s floor applied to any nonzero angular command once outside the dead
                                      # zone. On high-friction ground a weak proportional command never breaks
                                      # static friction, so the error keeps growing until a much bigger command
                                      # "breaks free" and overshoots -- this guarantees enough authority to turn
                                      # right away instead. Field-tune: raise if it still doesn't turn near the
                                      # dead zone edge, lower if it snaps into a turn too hard.
AUTONAV_ANGULAR_SLEW_RATE   = 1.5    # rad/s^2 max rate of change of the angular command (ramps toward the
                                      # target instead of jumping), damping the overshoot the kick above can
                                      # cause. Field-tune: lower = smoother but slower to react; 0 disables.
AUTONAV_LINEAR_TAPER_DEG    = 30.0   # heading error (deg) at which linear velocity reaches AUTONAV_LINEAR_TURN_FLOOR,
                                      # tapering continuously from AUTONAV_DEAD_ZONE_DEG. Replaces the old hard
                                      # "stop and pivot" turn-in-place behavior: turning while still rolling only
                                      # needs to overcome kinetic friction, not static friction from a dead stop,
                                      # which is why turning while moving was already working better in the field.
AUTONAV_LINEAR_TURN_FLOOR   = 0.15   # m/s minimum linear speed kept even during a sharp turn (never a full stop);
                                      # never raises speed above what deceleration near a waypoint already set.
AUTONAV_DECEL_RADIUS_M      = 1.5    # start linear deceleration within this distance of the waypoint
AUTONAV_REACH_TOL_M         = 0.5    # waypoint arrival tolerance (meters)
AUTONAV_MAX_LINEAR_VEL      = 1.0    # m/s
AUTONAV_MIN_LINEAR_VEL      = 0.1    # m/s (floor while decelerating, avoids stalling before REACH_TOL)
AUTONAV_MAX_ANGULAR_VEL     = 1.0    # rad/s
AUTONAV_MANUAL_SPEED        = 0.4    # m/s used for idle-state manual drive commands
AUTONAV_ARRIVE_FRAMES       = 1      # consecutive in-tolerance control frames required to confirm arrival (1 = no debounce)

# Waypoint generation from a recorded manual run (see scripts/build_waypoints_from_run.py)
WAYPOINT_MIN_DISTANCE_M  = 5.0                      # min spacing between generated waypoints (meters); controls WP density
WAYPOINT_MIN_FIX_QUALITY = 4                        # minimum RTK fix_quality to accept a sample


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
CAM_MJPEG_QUALITY  = 80              # MJPEG compression quality (1-100)
CAM_DEFAULT_PLUGIN = "simple_color"  # Default processing plugin
CAM_ENABLE_STEREO     = True         # Enable stereo depth (requires Left/Right/StereoDepth nodes)
CAM_ENABLE_DISPARITY  = False        # Enable raw disparity stream alongside depth (adds load; keep False unless debugging)


# ══════════════════════════════════════════════════════════════════════════════
# 07_SysMon  — CPU thermal/load/memory -> WebSocket bridge
# ══════════════════════════════════════════════════════════════════════════════

SYSMON_WS_PORT      = 8825   # HTTP port; WebSocket = SYSMON_WS_PORT + 1
SYSMON_INTERVAL_S   = 2.0    # Sample period (seconds)
