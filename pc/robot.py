# pc/robot.py
# -----------------------------------------------------------------------------
# MASTER PC APPLICATION ENTRY POINT - ARCHITECTURAL DEPENDENCY INJECTION
# -----------------------------------------------------------------------------
import sys
import os
import time
import numpy as np

# Pull local configuration switches
try:
    import config
except ImportError:
    print("[FATAL] config.py missing in execution path.")
    sys.exit(1)

# Import your repository's exact production logic modules
from pc_vision import VisionController
from pc_command import RobotCommander

# Dynamic Environment Setup Pipeline Gate
WebotsDriver = None
if config.RUN_SIMULATION:
    print("\n[SYSTEM] 🖥️ Booting in Hardware-in-the-Loop SIMULATION Mode (Webots)...")
    user_home = os.path.expanduser('~')
    webots_root = os.path.join(user_home, 'AppData', 'Local', 'Programs', 'Webots')
    os.environ['WEBOTS_HOME'] = webots_root
    webots_lib = os.path.join(webots_root, 'lib', 'controller')
    os.environ['PATH'] = webots_lib + os.path.pathsep + os.environ['PATH']
    if hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(webots_lib)
        
    # Append the master flat python folder location discovered on your disk
    sys.path.append(os.path.join(webots_lib, 'python'))
    
    try:
        from controller import Camera
        from vehicle import Driver as WebotsDriver
        import numpy as np
    except ImportError as e:
        print(f"[ERROR] Could not link Webots simulation libraries: {e}")
        sys.exit(1)
else:
    print("\n[SYSTEM] 🚗 Booting in LIVE HARDWARE Mode (ESP32 Cam Over Wi-Fi)...")
    
if not config.RUN_SIMULATION:
    from pc_stream import VideoStream
    from pc_mqtt import MQTTController

def main():
    # Instantiate the unified image processing class
    vision = VisionController()

    if config.RUN_SIMULATION:
        # =====================================================================
        # PATH A: SIMULATION TOPOLOGY (Feeds Webots to your Exact Signatures)
        # =====================================================================
        robot = WebotsDriver() # type: ignore
        cam = robot.getDevice('camera')
        cam.enable(int(robot.getBasicTimeStep()))
        
        class VirtualMqttClient:
            """Mocks the exact publish_control_packet structure used by pc_mqtt."""
            def publish_control_packet(self, topic, speed, angle):
                # Scale values to Webots simulator limits
                sim_velocity_kmh = (speed / 1023.0) * 30.0
                sim_steering_rad = ((angle - 90) * 3.14159) / 180.0
                robot.setCruisingSpeed(sim_velocity_kmh)
                robot.setSteeringAngle(max(-0.45, min(0.45, sim_steering_rad)))

        mock_mqtt = VirtualMqttClient()
        
        # Injects parameters exactly matching your RobotCommander architecture
        commander = RobotCommander(camera_module=None, vision_module=vision, mqtt_module=mock_mqtt)
        
        print("🚀 Virtual HIL Pipeline Online. Intercepting simulation execution loops...")
        while robot.step() != -1:
            raw_bytes = cam.getImage()
            if not raw_bytes: 
                continue
            
            # Format raw bytes cleanly into a structured matrix for cv2.resize
            image_array = np.frombuffer(raw_bytes, dtype=np.uint8)
            image_matrix = image_array.reshape((cam.getHeight(), cam.getWidth(), 4))
            
            # Executing identical functions matching your repo method keys
            line_error = vision.process_frame(image_matrix)
            commander.calculate_navigation_vectors(line_error)

    else:
        # =====================================================================
        # PATH B: LIVE PHYSICAL HARDWARE TOPOLOGY (No Variable Names Altered)
        # =====================================================================
        stream = VideoStream(src=f"http://{config.MQTT_BROKER}/stream") # type: ignore
        mqtt_client = MQTTController(broker_ip=config.MQTT_BROKER) # type: ignore
        
        if not mqtt_client.connect():
            print("[ERROR] Physical broker link dropped. Aborting boot sequence.")
            return
            
        commander = RobotCommander(camera_module=stream, vision_module=vision, mqtt_module=mqtt_client)
        stream.start()
        
        print("🚀 Production Telemetry Track Online. Streaming live vehicle nodes...")
        while True:
            try:
                live_frame = stream.read_frame()
                if live_frame is None: 
                    continue
                
                line_error = vision.process_frame(live_frame)
                commander.calculate_navigation_vectors(line_error)
                
                time.sleep(0.01)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    main()










"""
=========================================================================================
PC MAIN ENTRY POINT (robot.py)
-----------------------------------------------------------------------------------------
Main executable entry script for the PC vision system.
Instantiates all required sub-modules (VideoStream, VisionController, MQTTController)
and injects them into RobotCommander to launch the autonomous line-following control loop.
=========================================================================================
"""
""""
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
    """