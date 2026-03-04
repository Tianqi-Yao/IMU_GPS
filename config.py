import os

RTK_PORT:    str   = os.environ.get("RTK_PORT",    "/dev/cu.usbmodem2403")
RTK_BAUD:    int   = int(os.environ.get("RTK_BAUD",    "9600"))
RTK_TIMEOUT: float = float(os.environ.get("RTK_TIMEOUT", "1.0"))
