"""
=========================================================================================
PC ROBOT COMMANDER MODULE (pc_command.py)
-----------------------------------------------------------------------------------------
Acts as the central orchestrator (Dependency Injection target) for the PC vision pipeline.
Coordinates frame acquisition from VideoStream, decision-making from VisionController,
and transmission via MQTTController, while handling error recovery and post-breakdown checks.
=========================================================================================
"""

import cv2  # type: ignore # Import OpenCV library for frame rendering and keyboard interaction
import time  # type: ignore # Import time module for rate limiting command broadcasts
from pc_mqtt import FLEET_ALERT_TOPIC  # type: ignore # Import alert topic string constant from MQTT module


class RobotCommander:  # type: ignore # Orchestrates sub-system components into central control loop
    def __init__(self, camera_module, vision_module, mqtt_module):  # type: ignore # Dependency injection constructor
        self.camera = camera_module  # type: ignore # Store injected VideoStream instance reference
        self.vision = vision_module  # type: ignore # Store injected VisionController instance reference
        self.mqtt = mqtt_module  # type: ignore # Store injected MQTTController instance reference
        self.last_send_time = time.time()  # type: ignore # Initialize timestamp marker for command rate limiter
        self.breakdown_active = False  # type: ignore # Track breakdown status state flag
        print("RobotCommander: initialized. Camera, Vision, and MQTT sub-systems injected and ready.")  # type: ignore

    def run(self):  # type: ignore # Runs main execution loop processing video frames and dispatching controls
        print("RobotCommander: main loop starting. Press 'q' in the video window to quit.")  # type: ignore
        
        try:  # type: ignore # Outer try block ensuring safe program teardown on exit
            while True:  # type: ignore # Infinite video processing frame loop
                ret, frame = self.camera.read()  # type: ignore # Fetch latest available camera frame array
                
                if not ret or frame is None or self.camera.is_stale():  # type: ignore # Check for missing or stale frame feeds
                    if not self.breakdown_active:  # type: ignore # Handle first occurrence of camera breakdown event
                        self.breakdown_active = True  # type: ignore # Set breakdown active flag to True
                        print("MID-RUN STOP: camera failure detected. Broadcasting emergency halt to all subscribers...")  # type: ignore

                        warning_payload = f"{self.mqtt.command_secret},WARNING,CAMERA_FAILURE"  # type: ignore # Build alert payload
                        try:  # type: ignore # Attempt publishing breakdown alert message over MQTT
                            self.mqtt.client.publish(FLEET_ALERT_TOPIC, warning_payload)  # type: ignore # Publish alert
                            print(f"FLEET ALERT published to '{FLEET_ALERT_TOPIC}': {warning_payload}")  # type: ignore
                        except Exception as alert_err:  # type: ignore # Catch any MQTT alert broadcast exception
                            print(f"Warning: fleet alert publish failed ({alert_err}). Proceeding with local STOP command regardless.")  # type: ignore

                        self.mqtt.send_command(90, "STOP")  # type: ignore # Transmit emergency STOP control command to vehicle
                        self.last_send_time = time.time()  # type: ignore # Update rate limit timestamp
                        print("Emergency STOP command sent. Vehicle should be halting now.")  # type: ignore

                    all_clear = True  # type: ignore # Initialize recovery verification state flag as True
                    
                    check_ret, check_frame = self.camera.read()  # type: ignore # Perform test read from camera feed
                    if not check_ret or check_frame is None or self.camera.is_stale():  # type: ignore # Check frame health
                        print("POST-BREAKDOWN CHECK 1 FAIL: camera still not returning fresh frames. Holding STOP.")  # type: ignore
                        all_clear = False  # type: ignore # Flag verification failed due to unhealthy frame
                    else:  # type: ignore # Camera feed healthy
                        print("POST-BREAKDOWN CHECK 1 PASS: camera is producing fresh frames.")  # type: ignore

                    if not self.mqtt.connected:  # type: ignore # Verify MQTT connection status during recovery check
                        print("POST-BREAKDOWN CHECK 2 FAIL: MQTT not connected. Attempting reconnect before resume...")  # type: ignore
                        all_clear = False  # type: ignore # Flag verification failed due to missing MQTT connection
                    else:  # type: ignore # MQTT client connected
                        print("POST-BREAKDOWN CHECK 2 PASS: MQTT connection is live.")  # type: ignore

                    if self.vision.state != "DRIVE":  # type: ignore # Verify vision internal state machine state
                        self.vision.state = "DRIVE"  # type: ignore # Force reset vision state to clean DRIVE mode
                        self.vision.last_valid_angle = 90  # type: ignore # Reset default valid steering angle cache to 90
                        print("POST-BREAKDOWN CHECK 3: vision state machine was not in DRIVE - reset to DRIVE and last_valid_angle reset to 90.")  # type: ignore
                    else:  # type: ignore # Vision state clean
                        print("POST-BREAKDOWN CHECK 3 PASS: vision state machine is already in DRIVE state.")  # type: ignore

                    if all_clear:  # type: ignore # Evaluate if all safety conditions passed successfully
                        self.breakdown_active = False  # type: ignore # Clear breakdown active state flag
                        print("POST-BREAKDOWN SAFETY CHECK: ALL CHECKS PASSED. Resuming normal vision processing.")  # type: ignore
                    else:  # type: ignore # Breakdown recovery incomplete
                        print("POST-BREAKDOWN SAFETY CHECK: one or more checks failed. Holding STOP until all systems confirm ready.")  # type: ignore

                    if cv2.waitKey(1) & 0xFF == ord('q'):  # type: ignore # Check for operator exit command keypress
                        break  # type: ignore # Terminate loop on keypress
                    continue  # type: ignore # Skip processing step and iterate to next recovery cycle check

                try:  # type: ignore # Guard frame vision processing logic execution
                    target_angle, engine_state, processed_frame = self.vision.process_frame(frame)  # type: ignore # Process image frame
                    print(f"RobotCommander: vision result received -> angle={target_angle} | state={engine_state}")  # type: ignore
                except Exception as vision_err:  # type: ignore # Catch any vision pipeline exception
                    print(f"RobotCommander: VISION ERROR: {vision_err}. Commanding emergency STOP for safety.")  # type: ignore
                    self.mqtt.send_command(90, "STOP")  # type: ignore # Send emergency halt command on vision exception
                    self.last_send_time = time.time()  # type: ignore # Refresh send timestamp
                    print("RobotCommander: emergency STOP sent due to vision error. Resuming loop to retry on next frame.")  # type: ignore
                    continue  # type: ignore # Skip remainder of loop iteration

                if time.time() - self.last_send_time > 0.1:  # type: ignore # Throttle transmission rate to 10 Hz maximum
                    self.mqtt.send_command(target_angle, engine_state)  # type: ignore # Publish current control command
                    self.last_send_time = time.time()  # type: ignore # Update send timer timestamp

                cv2.putText(processed_frame, f"ANGLE: {target_angle} | {engine_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)  # type: ignore
                cv2.imshow("Bot Vision", processed_frame)  # type: ignore # Display visual feed frame in OpenCV window

                if cv2.waitKey(1) & 0xFF == ord('q'):  # type: ignore # Listen for exit keyboard keypress
                    print("RobotCommander: operator pressed 'q'. Exiting main loop cleanly.")  # type: ignore
                    break  # type: ignore # Exit execution loop on keypress
        finally:  # type: ignore # Execute final cleanup regardless of loop termination origin
            print("RobotCommander: main loop exited. Running final shutdown sequence...")  # type: ignore
            try:  # type: ignore # Guard execution of final emergency stop publish
                self.mqtt.send_command(90, "STOP")  # type: ignore # Send neutral stop command upon program exit
                time.sleep(0.2)  # type: ignore # Pause briefly to ensure socket output buffer flushes command
                print("RobotCommander: final STOP command sent successfully.")  # type: ignore
            except Exception as final_stop_err:  # type: ignore # Catch publish exception on teardown
                print(f"RobotCommander: final STOP publish failed ({final_stop_err}). ESP32 watchdog will halt vehicle within 500ms.")  # type: ignore
            self.shutdown()  # type: ignore # Invoke explicit hardware and resource cleanup

    def shutdown(self):  # type: ignore # Releases hardware devices, closes network sessions, and closes GUI windows
        print("RobotCommander: shutdown() called. Releasing all hardware and network resources...")  # type: ignore
        self.camera.stop()  # type: ignore # Stop video stream capture thread and release hardware handle
        print("RobotCommander: camera released.")  # type: ignore
        self.mqtt.stop()  # type: ignore # Stop MQTT network thread and close TCP connection
        print("RobotCommander: MQTT connection closed.")  # type: ignore
        cv2.destroyAllWindows()  # type: ignore # Destroy all open OpenCV display windows
        print("RobotCommander: OpenCV windows closed. Shutdown complete. All resources released.")  # type: ignore