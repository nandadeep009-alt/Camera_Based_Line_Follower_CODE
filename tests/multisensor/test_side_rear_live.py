import os
import sys

# ------------------------------------------------------------
# WEBOTS PYTHON SETUP
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

# ------------------------------------------------------------
# WEBOTS DRIVER
# ------------------------------------------------------------

from vehicle import Driver

robot = Driver()
timestep = int(robot.getBasicTimeStep())

# ------------------------------------------------------------
# OBSTACLE SENSORS
# ------------------------------------------------------------

sensor_names = {
    "LEFT_SIDE": "left_side_obstacle",
    "RIGHT_SIDE": "right_side_obstacle",
    "REAR": "rear_obstacle",
}

sensors = {}

for zone, name in sensor_names.items():
    sensor = robot.getDevice(name)

    if sensor is None:
        raise RuntimeError(f"Sensor '{name}' not found.")

    sensor.enable(timestep)
    sensors[zone] = sensor

    print(f"[SENSOR READY] {zone} -> '{name}'")

print()
print("=" * 70)
print("ROVE SIDE + REAR RAW SENSOR TEST")
print("=" * 70)
print()

# ------------------------------------------------------------
# LIVE SENSOR LOOP
# ------------------------------------------------------------

cycle = 0

try:
    while True:
        if robot.step() == -1:
            break

        cycle += 1

        if cycle % 10 != 0:
            continue

        left_value = float(sensors["LEFT_SIDE"].getValue())
        right_value = float(sensors["RIGHT_SIDE"].getValue())
        rear_value = float(sensors["REAR"].getValue())

        print(
            f"[RAW] "
            f"LEFT_SIDE={left_value:.3f} | "
            f"RIGHT_SIDE={right_value:.3f} | "
            f"REAR={rear_value:.3f}"
        )

except KeyboardInterrupt:
    print()
    print("[TEST] Operator stopped sensor test.")

finally:
    for sensor in sensors.values():
        try:
            sensor.disable()
        except Exception:
            pass

    print("[TEST] Side + rear sensors stopped.")
