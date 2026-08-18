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

        def run(self):  # type: ignore # Main PC control loop
            print(
            "RobotCommander: main loop starting. "
            "Press 'q' in the video window to quit."
        )  # type: ignore

        try:  # type: ignore # Ensure safe shutdown even if an error occurs

            # =================================================================
            # CREATE LARGE RESIZABLE VIDEO WINDOW
            # =================================================================
            #
            # IMPORTANT:
            #
            # This changes ONLY the OpenCV display window size.
            #
            # It does NOT:
            #     - resize the camera frame
            #     - change vision processing resolution
            #     - change yellow-line detection
            #     - change steering calculations
            #     - change vehicle speed
            #
            # The actual frame remains exactly as received from the camera.
            # =================================================================

            cv2.namedWindow(
                "Bot Vision",
                cv2.WINDOW_NORMAL
            )  # type: ignore # Create a resizable OpenCV window

            cv2.resizeWindow(
                "Bot Vision",
                1280,
                720
            )  # type: ignore # Set initial display window size


            # =================================================================
            # MAIN CONTROL LOOP
            # =================================================================

            while True:  # type: ignore # Continuously process camera frames

                # -----------------------------------------------------------------
                # STEP 1: READ CAMERA FRAME
                # -----------------------------------------------------------------

                ret, frame = self.camera.read()  # type: ignore # Get latest camera frame


                # -----------------------------------------------------------------
                # STEP 2: CAMERA SAFETY CHECK
                # -----------------------------------------------------------------

                if (
                    not ret
                    or frame is None
                    or self.camera.is_stale()
                ):  # type: ignore # Detect missing/stale camera feed

                    if not self.breakdown_active:  # type: ignore # Handle first failure only

                        self.breakdown_active = True  # type: ignore

                        print(
                            "MID-RUN STOP: camera failure detected. "
                            "Broadcasting emergency halt to all subscribers..."
                        )  # type: ignore


                        # ---------------------------------------------------------
                        # Broadcast camera failure warning
                        # ---------------------------------------------------------

                        warning_payload = (
                            f"{self.mqtt.command_secret},"
                            f"WARNING,"
                            f"CAMERA_FAILURE"
                        )  # type: ignore


                        try:  # type: ignore

                            self.mqtt.client.publish(
                                FLEET_ALERT_TOPIC,
                                warning_payload
                            )  # type: ignore

                            print(
                                f"FLEET ALERT published to "
                                f"'{FLEET_ALERT_TOPIC}': "
                                f"{warning_payload}"
                            )  # type: ignore

                        except Exception as alert_err:  # type: ignore

                            print(
                                f"Warning: fleet alert publish failed "
                                f"({alert_err}). "
                                f"Proceeding with local STOP command regardless."
                            )  # type: ignore


                        # ---------------------------------------------------------
                        # Emergency STOP
                        # ---------------------------------------------------------

                        self.mqtt.send_command(
                            90,
                            "STOP"
                        )  # type: ignore

                        self.last_send_time = time.time()  # type: ignore

                        print(
                            "Emergency STOP command sent. "
                            "Vehicle should be halting now."
                        )  # type: ignore


                    # -------------------------------------------------------------
                    # POST-BREAKDOWN SAFETY CHECK
                    # -------------------------------------------------------------

                    all_clear = True  # type: ignore


                    check_ret, check_frame = self.camera.read()  # type: ignore


                    if (
                        not check_ret
                        or check_frame is None
                        or self.camera.is_stale()
                    ):  # type: ignore

                        print(
                            "POST-BREAKDOWN CHECK 1 FAIL: "
                            "camera still not returning fresh frames. "
                            "Holding STOP."
                        )  # type: ignore

                        all_clear = False  # type: ignore

                    else:

                        print(
                            "POST-BREAKDOWN CHECK 1 PASS: "
                            "camera is producing fresh frames."
                        )  # type: ignore


                    # -------------------------------------------------------------
                    # MQTT CHECK
                    # -------------------------------------------------------------

                    if not self.mqtt.connected:  # type: ignore

                        print(
                            "POST-BREAKDOWN CHECK 2 FAIL: "
                            "MQTT not connected. "
                            "Holding STOP."
                        )  # type: ignore

                        all_clear = False  # type: ignore

                    else:

                        print(
                            "POST-BREAKDOWN CHECK 2 PASS: "
                            "MQTT connection is live."
                        )  # type: ignore


                    # -------------------------------------------------------------
                    # VISION STATE CHECK
                    # -------------------------------------------------------------

                    if self.vision.state != "DRIVE":  # type: ignore

                        self.vision.state = "DRIVE"  # type: ignore
                        self.vision.last_valid_angle = 90  # type: ignore

                        print(
                            "POST-BREAKDOWN CHECK 3: "
                            "vision state reset to DRIVE."
                        )  # type: ignore

                    else:

                        print(
                            "POST-BREAKDOWN CHECK 3 PASS: "
                            "vision state machine is already in DRIVE."
                        )  # type: ignore


                    # -------------------------------------------------------------
                    # RESUME ONLY IF ALL CHECKS PASS
                    # -------------------------------------------------------------

                    if all_clear:  # type: ignore

                        self.breakdown_active = False  # type: ignore

                        print(
                            "POST-BREAKDOWN SAFETY CHECK: "
                            "ALL CHECKS PASSED. "
                            "Resuming normal vision processing."
                        )  # type: ignore

                    else:

                        print(
                            "POST-BREAKDOWN SAFETY CHECK: "
                            "one or more checks failed. "
                            "Holding STOP."
                        )  # type: ignore


                    # -------------------------------------------------------------
                    # Allow operator to quit
                    # -------------------------------------------------------------

                    if cv2.waitKey(1) & 0xFF == ord('q'):  # type: ignore

                        break  # type: ignore

                    continue  # type: ignore


                # =================================================================
                # VISION PROCESSING
                # =================================================================

                try:  # type: ignore

                    target_angle, engine_state, processed_frame = (
                        self.vision.process_frame(frame)
                    )  # type: ignore

                    print(
                        f"RobotCommander: vision result received -> "
                        f"angle={target_angle} | "
                        f"state={engine_state}"
                    )  # type: ignore

                except Exception as vision_err:  # type: ignore

                    print(
                        f"RobotCommander: VISION ERROR: "
                        f"{vision_err}. "
                        f"Commanding emergency STOP for safety."
                    )  # type: ignore

                    self.mqtt.send_command(
                        90,
                        "STOP"
                    )  # type: ignore

                    self.last_send_time = time.time()  # type: ignore

                    print(
                        "RobotCommander: emergency STOP sent "
                        "due to vision error."
                    )  # type: ignore

                    continue  # type: ignore


                # =================================================================
                # SEND VEHICLE COMMAND
                # =================================================================

                if time.time() - self.last_send_time > 0.1:  # type: ignore
                    self.mqtt.send_command(
                        target_angle,
                        engine_state
                    )  # type: ignore

                    self.last_send_time = time.time()  # type: ignore


                # =================================================================
                # DISPLAY FRAME
                # =================================================================
                #
                # IMPORTANT:
                #
                # processed_frame is NOT resized here.
                #
                # The OpenCV WINDOW is large.
                # The actual image data remains unchanged.
                # =================================================================

                cv2.putText(
                    processed_frame,
                    f"ANGLE: {target_angle} | {engine_state}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2
                )  # type: ignore


                cv2.imshow(
                    "Bot Vision",
                    processed_frame
                )  # type: ignore


                # =================================================================
                # OPERATOR EXIT
                # =================================================================

                if cv2.waitKey(1) & 0xFF == ord('q'):  # type: ignore

                    print(
                        "RobotCommander: operator pressed 'q'. "
                        "Exiting main loop cleanly."
                    )  # type: ignore

                    break  # type: ignore


        # =====================================================================
        # FINAL SHUTDOWN
        # =====================================================================

        finally:  # type: ignore

            print(
                "RobotCommander: main loop exited. "
                "Running final shutdown sequence..."
            )  # type: ignore


            try:  # type: ignore

                self.mqtt.send_command(
                    90,
                    "STOP"
                )  # type: ignore

                time.sleep(0.2)  # type: ignore

                print(
                    "RobotCommander: final STOP command sent successfully."
                )  # type: ignore

            except Exception as final_stop_err:  # type: ignore

                print(
                    f"RobotCommander: final STOP publish failed "
                    f"({final_stop_err}). "
                    f"ESP32 watchdog will halt vehicle within 500ms."
                )  # type: ignore


            self.shutdown()  # type: ignore