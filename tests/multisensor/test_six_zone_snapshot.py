# tests/multisensor/test_sensor_zones.py

import math
import os
import sys

PROJECT_ROOT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
    )
)

PC_DIR = os.path.join(PROJECT_ROOT, "pc")

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if PC_DIR not in sys.path:
    sys.path.insert(0, PC_DIR)


# ------------------------------------------------------------
# WEBOTS SETUP
# ------------------------------------------------------------

user_home = os.path.expanduser("~")

webots_root = os.path.join(
    user_home,
    "AppData",
    "Local",
    "Programs",
    "Webots",
)

webots_lib = os.path.join(
    webots_root,
    "lib",
    "controller",
)

webots_python = os.path.join(
    webots_lib,
    "python",
)

os.environ["WEBOTS_HOME"] = webots_root

if webots_lib not in os.environ.get("PATH", ""):
    os.environ["PATH"] = (
        webots_lib
        + os.pathsep
        + os.environ.get("PATH", "")
    )

if hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(webots_lib)
    except OSError:
        pass

if webots_python not in sys.path:
    sys.path.insert(0, webots_python)


from vehicle import Driver

from multisensor.webots_sensors import WebotsLidarReader, WebotsProximityReader
from multisensor.sensor_zones import FrontLidarZones


# ------------------------------------------------------------
# HELPERS
# ------------------------------------------------------------

def show(value):

    if math.isfinite(value):
        return f"{value:.2f} m"

    return "CLEAR"


# ------------------------------------------------------------
# CONNECT
# ------------------------------------------------------------

robot = Driver()

lidar = WebotsLidarReader(
    robot=robot,
    device_name="Sick LMS 291",
)

zones = FrontLidarZones()

proximity = WebotsProximityReader(robot=robot)


def show_side(value):
    if not math.isfinite(value):
        return "INVALID"
    if proximity.is_clear(value):
        return "CLEAR"
    return f"{value:.2f} m"


print()
print("=" * 65)
print("ROVE FRONT SENSOR ZONE TEST")
print("=" * 65)


cycle = 0

try:

    while True:

        if robot.step() == -1:
            break

        scan = lidar.read_scan()

        if scan is None:
            continue

        cycle += 1

        if cycle % 10 != 0:
            continue

        result = zones.analyze(
            scan=scan,
            fov_rad=lidar.fov,
        )

        side = proximity.read()

        print(
            f"LEFT_FRONT={show(result['LEFT_FRONT'])} | "
            f"CENTER_FRONT={show(result['CENTER_FRONT'])} | "
            f"RIGHT_FRONT={show(result['RIGHT_FRONT'])} | "
            f"LEFT_SIDE={show_side(side['LEFT_SIDE'])} | "
            f"RIGHT_SIDE={show_side(side['RIGHT_SIDE'])} | "
            f"REAR={show_side(side['REAR'])}"
        )


except KeyboardInterrupt:

    print()
    print("[TEST] Operator stopped zone test.")


finally:

    lidar.stop()
    proximity.stop()

    print("[TEST] Sensor zone test finished.")