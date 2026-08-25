# tests/multisensor/test_lidar_live.py

"""
ROVE Front LiDAR Live Test

Purpose:
    Connect to the experimental Webots vehicle,
    enable the existing SICK LMS 291 LiDAR,
    and print live range measurements.

Safety:
    - No steering commands.
    - No speed commands.
    - No production files modified.
"""

import math
import os
import sys
import time


# -------------------------------------------------------------------------
# PROJECT PATH
# -------------------------------------------------------------------------

PROJECT_ROOT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
    )
)

PC_DIR = os.path.join(
    PROJECT_ROOT,
    "pc",
)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if PC_DIR not in sys.path:
    sys.path.insert(0, PC_DIR)


# -------------------------------------------------------------------------
# WEBOTS PYTHON ENVIRONMENT
# -------------------------------------------------------------------------

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


# -------------------------------------------------------------------------
# IMPORTS
# -------------------------------------------------------------------------

try:
    from vehicle import Driver

except ImportError as exc:
    print(f"[FAIL] Webots Driver import failed: {exc}")
    sys.exit(1)


from multisensor.webots_sensors import WebotsLidarReader


# -------------------------------------------------------------------------
# START WEBOTS CONNECTION
# -------------------------------------------------------------------------

robot = Driver()

lidar = WebotsLidarReader(
    robot=robot,
    device_name="Sick LMS 291",
)

print()
print("=" * 70)
print("ROVE FRONT LIDAR LIVE TEST")
print("=" * 70)
print()


# -------------------------------------------------------------------------
# LIVE SENSOR TEST
# -------------------------------------------------------------------------

sample_number = 0

try:

    while True:

        # Advance Webots exactly once.
        if robot.step() == -1:
            break

        scan = lidar.read_scan()

        if scan is None:
            print("[LIDAR] No valid scan available.")
            continue

        sample_number += 1

        # Print every 10th simulation cycle.
        if sample_number % 10 != 0:
            continue

        resolution = len(scan)

        center_index = resolution // 2

        left_index = resolution // 4
        right_index = (3 * resolution) // 4

        left = scan[left_index]
        center = scan[center_index]
        right = scan[right_index]

        closest = lidar.closest_distance(scan)

        def show(value):
            if math.isfinite(value):
                return f"{value:.2f} m"
            return "NO RETURN"

        print(
            f"[LIDAR] "
            f"samples={resolution} | "
            f"LEFT={show(left)} | "
            f"CENTER={show(center)} | "
            f"RIGHT={show(right)} | "
            f"CLOSEST={show(closest)}"
        )

        time.sleep(0.02)

except KeyboardInterrupt:

    print()
    print("[TEST] Operator stopped LiDAR test.")

finally:

    lidar.stop()

    print()
    print("[TEST] LiDAR live test finished.")