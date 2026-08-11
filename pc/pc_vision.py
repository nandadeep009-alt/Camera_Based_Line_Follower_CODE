"""
=========================================================================================
PC VISION CONTROLLER MODULE (pc_vision.py)
-----------------------------------------------------------------------------------------
Encapsulates computer vision analysis and proportional steering control logic.
Processes incoming camera frames, performs adaptive thresholding, contour validation,
and calculates proportional (P) steering angles and recovery state machine transitions.
=========================================================================================
"""

import cv2  # type: ignore # Import OpenCV library for image processing functions
import time  # type: ignore # Import time module for state machine timing calculations


class VisionController:  # type: ignore # Encapsulates image processing pipeline and drive control state logic
    def __init__(self, scale_factor=0.5):  # type: ignore # Constructor method
        self.scale_factor = scale_factor  # type: ignore # Image scaling factor for reducing processing latency
        self.servo_center = 90  # type: ignore # Neutral center position value for steering servo (90 degrees)
        self.last_valid_angle = 90  # type: ignore # Cache store for last computed valid forward steering angle

        self.state = "DRIVE"  # type: ignore # Track internal drive state machine ("DRIVE", "SEARCH_STOP", "REVERSE")
        self.lost_line_timestamp = 0  # type: ignore # Store timestamp when tracking line was first lost
        self.reverse_start_timestamp = 0  # type: ignore # Store timestamp when reverse recovery maneuver began

    def map_error_to_angle(self, error):  # type: ignore # Maps line pixel offset from center to steering angle
        kp = 0.1  # type: ignore # Proportional gain factor determining steering response sensitivity
        angle = self.servo_center + int(error * kp)  # type: ignore # Compute base target steering angle
        print(f"map_error_to_angle: pixel_error={error} -> steering_angle={int(max(45, min(135, angle)))} (kp={kp}, center={self.servo_center})")  # type: ignore
        return int(max(45, min(135, angle)))  # type: ignore # Clamp steering angle to physical mechanical boundaries [45, 135]

    def process_frame(self, frame):  # type: ignore # Main analysis function evaluating frame and outputting drive command
        small_frame = cv2.resize(frame, (0, 0), fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_AREA)  # type: ignore # Resize image down
        height, width = small_frame.shape[:2]  # type: ignore # Obtain scaled dimensions of working frame

        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)  # type: ignore # Convert frame to grayscale
        mean_brightness = cv2.mean(gray)[0]  # type: ignore # Calculate average overall image illumination

        if mean_brightness < 20:  # type: ignore # Guard check for under-illuminated or blocked camera environment
            target_angle = self.servo_center  # type: ignore # Keep steering neutral under dark failure condition
            engine_state = "STOP"  # type: ignore # Stop vehicle propulsion when insufficient light detected
            cv2.putText(small_frame, "TOO DARK", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)  # type: ignore
            return target_angle, engine_state, small_frame  # type: ignore # Exit processing and return emergency state

        if mean_brightness > 220:  # type: ignore # Guard check for overexposed image or severe light glare
            target_angle = self.servo_center  # type: ignore # Keep steering neutral during glare
            engine_state = "STOP"  # type: ignore # Stop vehicle during overexposure conditions
            print(f"Frame too bright (brightness={mean_brightness:.1f}). Stopping to avoid glare-induced wrong detection.")  # type: ignore
            cv2.putText(small_frame, f"TOO BRIGHT ({mean_brightness:.0f})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)  # type: ignore
            return target_angle, engine_state, small_frame  # type: ignore # Exit processing early

        clean_frame = small_frame.copy()  # type: ignore # Create clean unannotated copy for vision processing
        overlay_mask = cv2.inRange(clean_frame, (0, 200, 0), (80, 255, 80))  # type: ignore # Mask out bright green drawing overlays
        overlay_mask |= cv2.inRange(clean_frame, (0, 0, 200), (80, 80, 255))  # type: ignore # Mask out bright red text overlays
        clean_frame[overlay_mask > 0] = (128, 128, 128)  # type: ignore # Replace masked overlay pixels with neutral gray
        print(f"Overlay mask: {cv2.countNonZero(overlay_mask)} annotation pixels neutralised before thresholding.") if cv2.countNonZero(overlay_mask) > 0 else None  # type: ignore

        thresh = cv2.adaptiveThreshold(  # type: ignore # Apply adaptive thresholding to isolate track line
            cv2.cvtColor(clean_frame, cv2.COLOR_BGR2GRAY),  # type: ignore # Grayscale converted image
            255,  # type: ignore # Maximum binary value (white)
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,  # type: ignore # Adaptive thresholding algorithm choice
            cv2.THRESH_BINARY_INV,  # type: ignore # Invert output binary image so dark regions become white
            11,  # type: ignore # Pixel neighborhood block size
            2  # type: ignore # Constant subtracted from mean calculation
        )  # type: ignore
        M = cv2.moments(thresh)  # type: ignore # Calculate image moments over binary threshold image

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # type: ignore # Find contours of detected shapes
        valid_contours = []  # type: ignore # Initialize list of valid contour candidate objects
        for cnt in contours:  # type: ignore # Iterate over all detected binary contours
            area = cv2.contourArea(cnt)  # type: ignore # Compute area of current contour in pixels
            if area < 500:  # type: ignore # Filter out small noise artifacts below area limit
                continue  # type: ignore # Skip invalid candidate contour
            x, y, w, h = cv2.boundingRect(cnt)  # type: ignore # Extract bounding box bounds of contour
            aspect = max(w, h) / (min(w, h) + 1)  # type: ignore # Calculate aspect ratio of shape
            if aspect < 1.5:  # type: ignore # Filter out square or circular blob artifacts
                print(f"Rejected blob (aspect={aspect:.2f}, area={area}): too square to be the line, likely a corner mark or shadow.")  # type: ignore
                continue  # type: ignore # Skip non-line-like shapes
            valid_contours.append(cnt)  # type: ignore # Accept valid elongated candidate line contour

        clean_thresh = thresh.copy()  # type: ignore # Copy binary image to clean secondary buffer
        clean_thresh[:] = 0  # type: ignore # Clear binary frame memory buffer
        cv2.drawContours(clean_thresh, valid_contours, -1, 255, -1)  # type: ignore # Redraw only verified valid contours
        M = cv2.moments(clean_thresh)  # type: ignore # Re-evaluate spatial moments using filtered binary mask

        if M["m00"] > 15000:  # type: ignore # Verify sufficient pixel area mass exists for line detection
            cx = int(M["m10"] / M["m00"])  # type: ignore # Calculate horizontal center x-coordinate of line mass
            error = cx - (width // 2)  # type: ignore # Compute pixel error offset from image center
            target_angle = self.map_error_to_angle(error)  # type: ignore # Translate pixel error into target servo angle
            self.last_valid_angle = target_angle  # type: ignore # Store angle for potential future reverse recovery

            if self.state in ["SEARCH_STOP", "REVERSE"]:  # type: ignore # Check if recovering from line loss
                print("Line detected! Returning to forward drive.")  # type: ignore
                self.state = "DRIVE"  # type: ignore # Reset active state back to standard DRIVE mode

            engine_state = "DRIVE"  # type: ignore # Command forward vehicle drive motion
            cv2.circle(small_frame, (cx, height // 2), 5, (0, 255, 0), -1)  # type: ignore # Draw green centroid visual indicator
            cv2.putText(small_frame, f"LINE DETECTED: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # type: ignore
            print(f"process_frame: line detected | centroid_x={cx} | error={error} | angle={target_angle} | state={self.state} | last_valid_angle saved={self.last_valid_angle}")  # type: ignore

        else:  # type: ignore # Handle lost tracking line condition
            current_time = time.time()  # type: ignore # Fetch current clock timestamp

            if self.state == "DRIVE":  # type: ignore # Transition out of normal drive state when line disappears
                self.state = "SEARCH_STOP"  # type: ignore # Enter temporary stationary pause recovery state
                self.lost_line_timestamp = current_time  # type: ignore # Record timestamp when line was first lost
                print(f"process_frame: LINE LOST. Transitioning DRIVE -> SEARCH_STOP. Vehicle will hold still for 3.5s. Lost at t={current_time:.2f}.")  # type: ignore

            if self.state == "SEARCH_STOP":  # type: ignore # Handle stationary search phase logic
                target_angle = self.servo_center  # type: ignore # Keep steering neutral during initial stationary search
                engine_state = "STOP"  # type: ignore # Halt motors while waiting for line reappearance
                print(f"process_frame: SEARCH_STOP active | waiting for line | elapsed={current_time - self.lost_line_timestamp:.2f}s / 3.5s | angle={target_angle}")  # type: ignore

                if current_time - self.lost_line_timestamp > 3.5:  # type: ignore # Check if stationary wait window has expired
                    self.state = "REVERSE"  # type: ignore # Transition state to active reverse maneuver
                    self.reverse_start_timestamp = current_time  # type: ignore # Record start time of reverse recovery maneuver
                    print(f"process_frame: 3.5s wait complete. Transitioning SEARCH_STOP -> REVERSE. Reverse start at t={current_time:.2f}. Last valid angle was {self.last_valid_angle}°.")  # type: ignore

            elif self.state == "REVERSE":  # type: ignore # Handle active reverse recovery maneuver state logic
                elapsed_reverse = current_time - self.reverse_start_timestamp  # type: ignore # Calculate duration of reverse drive phase
                MAX_REVERSE_TIME = 3.0  # type: ignore # Set maximum time threshold window allowed for reversing
                print(f"REVERSE RECOVERY: elapsed={elapsed_reverse:.1f}s / {MAX_REVERSE_TIME}s | mirroring last angle={self.last_valid_angle} -> reverse_angle={180 - self.last_valid_angle}")  # type: ignore

                reverse_angle = 180 - self.last_valid_angle  # type: ignore # Mirror forward steering angle to track path backward
                target_angle = max(45, min(135, reverse_angle))  # type: ignore # Clamp mirrored reverse steering angle

                if elapsed_reverse < MAX_REVERSE_TIME:  # type: ignore # Check if reverse window time limit remains valid
                    engine_state = "REVERSE"  # type: ignore # Engage motors in reverse direction
                else:  # type: ignore # Exceeded safe reverse time limit without acquiring track line
                    engine_state = "STOP"  # type: ignore # Halt vehicle motion permanently until reset
                    print("REVERSE RECOVERY TIMEOUT: line not found within bounded reverse distance. Halting. Operator intervention may be required.")  # type: ignore
                    cv2.putText(small_frame, "REVERSE TIMEOUT - OPERATOR REQUIRED", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)  # type: ignore

            cv2.putText(small_frame, f"FAILSAFE: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)  # type: ignore

        print(f"process_frame COMPLETE: returning angle={target_angle} | engine_state={engine_state} | vision_state={self.state}")  # type: ignore
        return target_angle, engine_state, small_frame  # type: ignore # Return calculated target outputs and display frame