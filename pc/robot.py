# pc/robot.py

# -----------------------------------------------------------------------------
# MASTER PC APPLICATION ENTRY POINT
# -----------------------------------------------------------------------------
#
# Supports:
#
#   1. WEBOTS HIL SIMULATION
#   2. REAL ESP32 CAMERA + MQTT VEHICLE
#
# IMPORTANT:
#   The production control architecture is preserved.
#
#   RobotCommander
#       ↓
#   VisionController
#       ↓
#   MQTTController / VirtualMQTTController
#
# Webots is implemented only through adapters.
# -----------------------------------------------------------------------------

import os
import sys


# -----------------------------------------------------------------------------
# PATH SETUP
# -----------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


# -----------------------------------------------------------------------------
# CONFIGURATION
# -----------------------------------------------------------------------------

try:
    import config

except ImportError:
    print("[FATAL] config.py missing from project root.")
    sys.exit(1)


if not hasattr(config, "RUN_SIMULATION"):
    config.RUN_SIMULATION = True


# -----------------------------------------------------------------------------
# PRODUCTION MODULES
# -----------------------------------------------------------------------------

from pc_vision import VisionController
from pc_command import RobotCommander


# -----------------------------------------------------------------------------
# WEBOTS SETUP
# -----------------------------------------------------------------------------

def run_webots():
    """
    Start the Webots Hardware-in-the-Loop test.

    Existing RobotCommander and VisionController are used unchanged.
    """

    print()
    print("=" * 70)
    print("[SYSTEM] WEBOTS HARDWARE-IN-THE-LOOP SIMULATION")
    print("=" * 70)

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

    # Tell Webots Python bindings where Webots is installed.
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
    # Webots imports
    # -------------------------------------------------------------------------

    try:
        from vehicle import Driver
        from webots_adapter import (
            WebotsCameraAdapter,
            VirtualMQTTController,
        )

    except ImportError as exc:
        print()
        print("[FATAL] Could not load Webots Python API.")
        print(f"[FATAL] {exc}")
        print()
        print("Expected Webots installation:")
        print(webots_root)
        print()
        sys.exit(1)

    # -------------------------------------------------------------------------
    # Create Webots vehicle
    # -------------------------------------------------------------------------

    robot = Driver()

    print("[WEBOTS] Driver initialized.")

    # -------------------------------------------------------------------------
    # Create adapters
    # -------------------------------------------------------------------------

    camera = WebotsCameraAdapter(
        robot=robot,
        camera_name="camera",
        stale_timeout=1.0,
    )

    virtual_mqtt = VirtualMQTTController(
        robot=robot,
        command_secret="WEBOTS_SIM",
        max_speed_mps=3.0,
        reverse_speed_mps=1.0,
        max_steering_rad=0.45,
    )

    # -------------------------------------------------------------------------
    # Existing production vision controller
    # -------------------------------------------------------------------------

    vision = VisionController(
        scale_factor=0.5
    )

    # -------------------------------------------------------------------------
    # Existing production RobotCommander
    # -------------------------------------------------------------------------

    commander = RobotCommander(
        camera_module=camera,
        vision_module=vision,
        mqtt_module=virtual_mqtt,
    )

    print()
    print("[SYSTEM] Virtual HIL pipeline online.")
    print("[SYSTEM] Existing RobotCommander is now controlling Webots.")
    print()
    print("Press 'q' in the OpenCV vision window to stop.")
    print()

    # -------------------------------------------------------------------------
    # IMPORTANT:
    #
    # RobotCommander.run() remains the master control loop.
    # -------------------------------------------------------------------------

    commander.run()


# -----------------------------------------------------------------------------
# REAL VEHICLE
# -----------------------------------------------------------------------------

def run_real_vehicle():
    """
    Existing physical vehicle execution path.

    ESP32 camera → VideoStream → VisionController →
    RobotCommander → MQTTController → ESP32
    """

    import time

    from pc_stream import VideoStream
    from pc_mqtt import (
        MQTTController,
        MQTT_BROKER,
        COMMAND_TOPIC,
        COMMAND_SECRET,
    )

    print()
    print("=" * 70)
    print("[SYSTEM] LIVE VEHICLE MODE")
    print("=" * 70)

    # -------------------------------------------------------------------------
    # Physical camera
    # -------------------------------------------------------------------------

    # Replace with actual ESP32 camera URL.
    ESP32_CAMERA_URL = "http://192.168.x.x/"

    pc_camera = None
    pc_mqtt = None

    try:

        pc_camera = VideoStream(
            src=ESP32_CAMERA_URL
        )

        time.sleep(1.0)

        # ---------------------------------------------------------------------
        # Production vision
        # ---------------------------------------------------------------------

        pc_vision = VisionController(
            scale_factor=0.5
        )

        # ---------------------------------------------------------------------
        # Production MQTT
        # ---------------------------------------------------------------------

        pc_mqtt = MQTTController(
            broker=MQTT_BROKER,
            topic=COMMAND_TOPIC,
            command_secret=COMMAND_SECRET,
        )

        # ---------------------------------------------------------------------
        # Production commander
        # ---------------------------------------------------------------------

        commander = RobotCommander(
            camera_module=pc_camera,
            vision_module=pc_vision,
            mqtt_module=pc_mqtt,
        )

        commander.run()

    except KeyboardInterrupt:

        print("[SYSTEM] Operator requested shutdown.")

    except Exception as exc:

        print()
        print(f"[CRITICAL] Vehicle startup/runtime failure: {exc}")
        print()

        # Emergency STOP attempt.
        if pc_mqtt is not None:

            try:
                pc_mqtt.send_command(
                    90,
                    "STOP"
                )

                time.sleep(0.2)

                pc_mqtt.stop()

            except Exception:
                pass

        if pc_camera is not None:

            try:
                pc_camera.stop()

            except Exception:
                pass

        raise


# -----------------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------------

def main():

    if config.RUN_SIMULATION:

        run_webots()

    else:

        run_real_vehicle()


if __name__ == "__main__":
    main()
