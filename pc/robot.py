"""
=========================================================================================
PC MAIN ENTRY POINT (robot.py)
-----------------------------------------------------------------------------------------
Main executable entry script for the PC vision system.
Instantiates all required sub-modules (VideoStream, VisionController, MQTTController)
and injects them into RobotCommander to launch the autonomous line-following control loop.
=========================================================================================
"""

import time  # type: ignore # Import time module for delay management during startup initialization
from pc_stream import VideoStream  # type: ignore # Import threaded video capture module
from pc_vision import VisionController  # type: ignore # Import vision analysis and drive control module
from pc_mqtt import MQTTController, MQTT_BROKER, COMMAND_TOPIC, COMMAND_SECRET  # type: ignore # Import MQTT module and configuration parameters
from pc_command import RobotCommander  # type: ignore # Import orchestrator commander class


if __name__ == "__main__":  # type: ignore # Application main entry point execution check
    print("Starting Robot Commander PC application...")  # type: ignore
    
    ESP32_CAMERA_URL = "http://192.168.x.x/"  # type: ignore # Target camera stream IP or device index URL
    pc_camera = None  # type: ignore # Pre-declare camera instance reference variable
    pc_mqtt = None  # type: ignore # Pre-declare MQTT controller instance reference variable
    
    try:  # type: ignore # Guard hardware object construction and setup loop execution
        pc_camera = VideoStream(src=ESP32_CAMERA_URL)  # type: ignore # Instantiate VideoStream object attached to camera URL
        time.sleep(1.0)  # type: ignore # Pause 1 second to allow camera capture frame buffer to initialize
        
        pc_vision = VisionController(scale_factor=0.5)  # type: ignore # Instantiate VisionController with 0.5 frame scale
        pc_mqtt = MQTTController(broker=MQTT_BROKER, topic=COMMAND_TOPIC, command_secret=COMMAND_SECRET)  # type: ignore # Instantiate MQTTController
        
        app = RobotCommander(pc_camera, pc_vision, pc_mqtt)  # type: ignore # Instantiate RobotCommander via Dependency Injection
        
        app.run()  # type: ignore # Execute primary application control loop
    except Exception as startup_err:  # type: ignore # Catch any system startup initialization error
        print(f"CRITICAL: Startup failed: {startup_err}")  # type: ignore
        if pc_mqtt is not None:  # type: ignore # Check if MQTT client instance was initialized prior to crash
            try:  # type: ignore # Guard emergency stop broadcast attempt
                pc_mqtt.send_command(90, "STOP")  # type: ignore # Transmit emergency halt command to robot
                time.sleep(0.2)  # type: ignore # Wait briefly for network transmission packet completion
                pc_mqtt.stop()  # type: ignore # Terminate MQTT client connection thread
            except Exception:  # type: ignore # Suppress errors during emergency teardown
                pass  # type: ignore
        if pc_camera is not None:  # type: ignore # Check if camera instance was initialized prior to crash
            try:  # type: ignore # Guard camera resource release attempt
                pc_camera.stop()  # type: ignore # Close video stream and terminate background capture thread
            except Exception:  # type: ignore # Suppress errors during emergency teardown
                pass  # type: ignore
        raise  # type: ignore # Re-raise original startup exception to console output