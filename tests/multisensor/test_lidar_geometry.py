# tests/multisensor/test_lidar_geometry.py

"""
ROVE LiDAR Geometry Test

Purpose:
    Determine where an obstacle appears within the SICK LMS 291
    horizontal scan.

This test:
    - reads all 180 LiDAR beams
    - finds the closest valid obstacle
    - reports its scan index
    - estimates its scan angle
    - divides the scan into three temporary zones

Safety:
    - no steering commands
    - no speed commands
    - production controller is not used
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
    sys.path.insert(
        0,
        webots_python,
    )


# -------------------------------------------------------------------------
# IMPORT WEBOTS
# -------------------------------------------------------------------------

try:
    from vehicle import Driver

except ImportError as exc:
    print(
        f"[FAIL] Webots Driver import failed: {exc}"
    )
    sys.exit(1)


from multisensor.webots_sensors import WebotsLidarReader


# -------------------------------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------------------------------

def zone_min(values):
    """
    Return the nearest finite distance within one scan zone.
    """

    valid = [
        value
        for value in values
        if math.isfinite(value)
    ]

    if not valid:
        return math.inf

    return min(valid)


def show(value):
    """
    Convert a distance into readable terminal text.
    """

    if math.isfinite(value):
        return f"{value:.2f} m"

    return "NO RETURN"


# -------------------------------------------------------------------------
# CONNECT TO WEBOTS VEHICLE
# -------------------------------------------------------------------------

robot = Driver()

lidar = WebotsLidarReader(
    robot=robot,
    device_name="Sick LMS 291",
)


print()
print("=" * 70)
print("ROVE LIDAR GEOMETRY TEST")
print("=" * 70)
print()


# -------------------------------------------------------------------------
# LIVE TEST
# -------------------------------------------------------------------------

sample_number = 0

try:

    while True:

        # Advance Webots exactly once.
        if robot.step() == -1:
            break

        scan = lidar.read_scan()

        if scan is None:
            print(
                "[LIDAR] No scan available."
            )
            continue

        sample_number += 1

        # Print only every 10th simulation cycle.
        if sample_number % 10 != 0:
            continue

        resolution = len(scan)

        # -------------------------------------------------------------
        # FIND ALL VALID RETURNS
        # -------------------------------------------------------------

        valid_points = [
            (index, distance)
            for index, distance in enumerate(scan)
            if math.isfinite(distance)
        ]

        if not valid_points:

            print(
                "[LIDAR] No obstacle returns."
            )

            continue

        # -------------------------------------------------------------
        # FIND CLOSEST OBJECT
        # -------------------------------------------------------------

        closest_index, closest_distance = min(
            valid_points,
            key=lambda item: item[1],
        )

        # -------------------------------------------------------------
        # ESTIMATE ANGLE OF THAT BEAM
        # -------------------------------------------------------------

        fov_degrees = math.degrees(
            lidar.fov
        )

        angle_step = (
            fov_degrees
            / max(1, resolution - 1)
        )

        scan_angle = (
            -fov_degrees / 2.0
            + closest_index * angle_step
        )

        # -------------------------------------------------------------
        # TEMPORARY THREE-ZONE SPLIT
        #
        # We deliberately call them A/B/C for now.
        #
        # We have NOT yet proven whether A is physical LEFT
        # or physical RIGHT.
        # -------------------------------------------------------------

        third = resolution // 3

        zone_a = scan[
            0:third
        ]

        zone_b = scan[
            third:2 * third
        ]

        zone_c = scan[
            2 * third:resolution
        ]

        a_min = zone_min(
            zone_a
        )

        b_min = zone_min(
            zone_b
        )

        c_min = zone_min(
            zone_c
        )

        # -------------------------------------------------------------
        # PRINT RESULT
        # -------------------------------------------------------------

        print(
            f"[LIDAR] "
            f"closest={show(closest_distance)} | "
            f"index={closest_index}/{resolution - 1} | "
            f"angle={scan_angle:+.1f} deg | "
            f"A={show(a_min)} | "
            f"B={show(b_min)} | "
            f"C={show(c_min)}"
        )

        time.sleep(
            0.02
        )


except KeyboardInterrupt:

    print()
    print(
        "[TEST] Operator stopped geometry test."
    )


finally:

    lidar.stop()

    print()
    print(
        "[TEST] LiDAR geometry test finished."
    )