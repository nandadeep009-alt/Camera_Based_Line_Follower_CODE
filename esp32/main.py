# esp32/main.py

"""
=========================================================================================
SUMMARY OF ROBOT BRAIN (OOP PRODUCTION VERSION)
-----------------------------------------------------------------------------------------
This script acts as the main firmware for an ESP32-S3 line-following robot.
=========================================================================================
"""

import machine               # type: ignore # Import system module for hardware reset capability
import utime as time         # type: ignore # Import timing utilities for pauses

# Modular imports from extracted project files
import pin_config            # type: ignore # Import pin definition assignments
from hw import SteeringController, MotorController, UltrasonicSensor # type: ignore # Import physical actuator classes
from net import NetworkController # type: ignore # Import network communication client
from cam import CameraStreamer # type: ignore # Import background MJPEG camera streamer
from vision import OnboardVisionController # type: ignore # Import autonomous vision processing pipeline
from orch import RobotOrchestrator # type: ignore # Import orchestrator state machine manager

if __name__ == "__main__":
    try:
        import pin_config as _cfg # Load board environment credentials configuration file
        WIFI_SSID      = _cfg.WIFI_SSID         # Store target Wi-Fi SSID
        WIFI_PASS      = _cfg.WIFI_PASS         # Store target Wi-Fi password
        MQTT_BROKER    = _cfg.MQTT_BROKER       # Store target MQTT broker IP
        MQTT_USER      = _cfg.MQTT_USER         # Store MQTT authentication username
        MQTT_PASS      = _cfg.MQTT_PASS         # Store MQTT authentication password
        COMMAND_TOPIC  = _cfg.COMMAND_TOPIC     # Store command topic byte string
        COMMAND_SECRET = _cfg.COMMAND_SECRET    # Store authentication shared secret
        MOTION_ARMED   = getattr(_cfg, "MOTION_ARMED", False) # Load safety motion arming status (default False)
        print(f"main.py: MOTION_ARMED = {MOTION_ARMED}.{' Vehicle will move.' if MOTION_ARMED else ' Motors are LOCKED — set MOTION_ARMED = True in config.py to enable motion.'}") # Output arming status
    except ImportError:
        raise Exception(                       # Crash startup immediately if config file is absent
            "FATAL: config.py not found on the board filesystem. "
            "Upload it via Thonny or mpremote before running main.py."
        )

    AUTONOMOUS_MODE = True                     # Select autonomous line-following mode (True) or remote control mode (False)

    try:
        my_steering = SteeringController(pin_num=pin_config.STEERING_SERVO_PIN) # Instantiate steering controller with mapped GPIO pin

        my_motors = MotorController(           # Instantiate motor controller using mapped pin configuration constants
            en_a=pin_config.MOTOR_EN_A,
            in1=pin_config.MOTOR_IN1,
            in2=pin_config.MOTOR_IN2,
            en_b=pin_config.MOTOR_EN_B,
            in3=pin_config.MOTOR_IN3,
            in4=pin_config.MOTOR_IN4
        )

        # ==============================================================
        # HC-SR04 ULTRASONIC SAFETY SENSOR
        # ==============================================================

        my_ultrasonic = UltrasonicSensor(
            trig_pin=pin_config.ULTRASONIC_TRIG,
            echo_pin=pin_config.ULTRASONIC_ECHO
        )

        print(
            "HC-SR04 ultrasonic safety sensor initialized."
        )

        my_network = NetworkController(        # Instantiate network manager with loaded credentials
            WIFI_SSID,
            WIFI_PASS,
            MQTT_BROKER,
            COMMAND_TOPIC,
            mqtt_user=MQTT_USER,
            mqtt_pass=MQTT_PASS
        )

        my_vision = None                        # Pre-declare vision object handle
        if AUTONOMOUS_MODE:                     # Handle camera allocation for autonomous driving
            my_vision = OnboardVisionController() # Instantiate vision controller in Grayscale mode
            print("Camera claimed by OnboardVisionController (GRAYSCALE). JPEG monitoring stream disabled in autonomous mode.") # Log hardware assignment
        else:                                   # Handle camera allocation for teleoperation mode
            my_camera = CameraStreamer()        # Instantiate camera streamer in JPEG mode
            my_camera.start_server()            # Start background video server thread on Core 1

        my_robot = RobotOrchestrator(           # Instantiate orchestrator and inject all required subsystem dependencies
            my_steering,
            my_motors,
            my_network,
            vision=my_vision,
            ultrasonic=my_ultrasonic,
            autonomous_mode=AUTONOMOUS_MODE,
            command_secret=COMMAND_SECRET,
            motion_armed=MOTION_ARMED
        )

        my_robot.run()                          # Execute orchestrator initialization and primary driving loop

    except Exception as init_err:
        print(f"CRITICAL: Hardware initialization failed: {init_err}") # Log hardware initialization exception
        print("Resetting in 3 seconds to attempt a clean recovery...") # Log recovery plan
        time.sleep(3)                           # Pause 3 seconds to flush UART serial output
        machine.reset()                         # Perform full chip reset to attempt recovery from clean boot state