# tests/multisensor/test_sensor_read.py

"""
ROVE Webots Multi-Sensor Device Probe

Purpose:
    Discover every Webots device attached to the vehicle before
    implementing the multi-sensor controller.

Safety:
    - Does NOT command steering.
    - Does NOT command throttle/speed.
    - Does NOT modify the production controller.
    - Does NOT modify the production Webots world.
"""

import os
import sys


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
# LOAD WEBOTS DRIVER API
# -------------------------------------------------------------------------

try:
    from vehicle import Driver

except ImportError as exc:
    print()
    print("[FAIL] Could not load Webots Python API.")
    print(f"[FAIL] {exc}")
    print(f"[INFO] Expected Webots location: {webots_root}")
    sys.exit(1)


# -------------------------------------------------------------------------
# CONNECT TO VEHICLE
# -------------------------------------------------------------------------

try:
    robot = Driver()

except Exception as exc:
    print()
    print("[FAIL] Could not connect to Webots vehicle.")
    print(f"[FAIL] {exc}")
    print()
    print(
        "Open the multisensor test world in Webots "
        "and make sure the vehicle controller is <extern>."
    )
    sys.exit(1)


print()
print("=" * 70)
print("ROVE WEBOTS DEVICE PROBE")
print("=" * 70)


# -------------------------------------------------------------------------
# ENUMERATE DEVICES
# -------------------------------------------------------------------------

device_count = robot.getNumberOfDevices()

print(f"[INFO] Number of vehicle devices: {device_count}")
print()


for index in range(device_count):

    try:
        device = robot.getDeviceByIndex(index)
        name = device.getName()

        try:
            model = device.getModel()
        except Exception:
            model = "unknown"

        print(
            f"[DEVICE {index:02d}] "
            f"name='{name}' | "
            f"model='{model}'"
        )

    except Exception as exc:

        print(
            f"[DEVICE {index:02d}] "
            f"ERROR: {exc}"
        )


print()
print("=" * 70)
print("DEVICE PROBE COMPLETE")
print("=" * 70)

print()
print(
    "No steering or speed commands were sent."
)