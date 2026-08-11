# esp32/esp_vision.py
import camera        # type: ignore #Import hardware camera driver library
import utime as time # type: ignore #Import timing library for millisecond calculation

class OnboardVisionController:
    def __init__(self, frame_width=320, frame_height=240, roi_rows=12, dark_threshold=90, min_dark_pixels=200, kp=0.25, servo_center=90):
        self.frame_width = frame_width          # Store frame width resolution in pixels
        self.frame_height = frame_height        # Store frame height resolution in pixels
        self.roi_rows = roi_rows                # Store region of interest row height at frame bottom
        self.dark_threshold = dark_threshold    # Store threshold value for line pixel discrimination (0-255)
        self.min_dark_pixels = min_dark_pixels  # Minimum dark pixels required to validate line detection
        self.kp = kp                            # Proportional feedback gain for steering response
        self.servo_center = servo_center        # Baseline straight steering angle in degrees
        self.last_valid_angle = servo_center    # Cache last valid calculated steering angle for recovery
        self.state = "DRIVE"                    # State variable tracking vision state machine ("DRIVE", "SEARCH_STOP", "REVERSE")
        self.lost_line_time = 0                 # Timestamp tracking when line detection was lost
        self.reverse_start_time = 0             # Timestamp tracking start of reverse recovery maneuver
        self.camera_ok = True                   # Flag tracking camera hardware status

        try:
            camera.init(0, format=camera.GRAYSCALE, framesize=camera.FRAME_QVGA) # Init camera in QVGA Grayscale (320x240)
            print("OnboardVisionController: camera initialized in GRAYSCALE mode for autonomous driving.") # Log success
        except Exception as e:
            self.camera_ok = False              # Set hardware health flag to False
            print(f"CRITICAL: OnboardVisionController camera failed to initialize: {e}") # Log initialization failure

    def compute_command(self):
        if not self.camera_ok:                  # Verify camera initialization status
            return self.servo_center, "STOP"    # Fallback to straight steering and STOP command if camera failed

        try:
            buf = camera.capture()              # Capture raw grayscale frame buffer
        except Exception as cam_err:
            print(f"OnboardVisionController: camera.capture() raised {cam_err}. Returning STOP for this frame.") # Log error
            self.camera_ok = False              # Flag camera error state
            return self.servo_center, "STOP"    # Fail safe on frame capture failure
            
        self.camera_ok = True                   # Mark camera status as functional on successful frame capture
        if not buf or len(buf) < self.frame_width * self.frame_height: # Check for empty or truncated byte buffer
            print("OnboardVisionController: bad or missing frame, stopping for safety.") # Output warning log
            return self.servo_center, "STOP"    # Stop motors if frame buffer corrupt

        start_row = self.frame_height - self.roi_rows # Compute starting row index for bottom Region of Interest (ROI)
        dark_pixel_count = 0                    # Counter for total dark line pixels detected
        weighted_sum = 0                        # Accumulator for horizontal column coordinates of dark pixels
        for row in range(start_row, self.frame_height): # Loop over rows in the ROI strip
            row_offset = row * self.frame_width # Compute base memory byte offset for current row
            for col in range(0, self.frame_width, 2): # Iterate through columns, skipping every 2nd pixel for speed
                pixel = buf[row_offset + col]   # Read 8-bit grayscale pixel intensity value
                if pixel < self.dark_threshold: # Check if pixel intensity falls below dark threshold
                    dark_pixel_count += 1       # Increment total detected dark line pixel counter
                    weighted_sum += col          # Accumulate pixel column coordinate for center calculation

        current_time = time.ticks_ms()          # Record current timestamp in milliseconds

        if dark_pixel_count >= self.min_dark_pixels: # Check if detected dark pixels meet minimum noise threshold
            centroid_col = weighted_sum // dark_pixel_count # Compute horizontal centroid column of the line
            error = centroid_col - (self.frame_width // 2)  # Calculate horizontal pixel displacement from center
            angle = self.servo_center + int(error * self.kp) # Convert error to steering angle via P-gain
            angle = max(45, min(135, angle))    # Clamp calculated steering angle within safe mechanical range (45-135 deg)
            self.last_valid_angle = angle       # Cache valid angle for reverse recovery path calculation
            if self.state != "DRIVE":           # Check if recovering from line search state
                print("OnboardVisionController: line reacquired, resuming normal drive.") # Log line recovery event
                self.state = "DRIVE"            # Transition state machine back to normal DRIVE
            return angle, "DRIVE"               # Return target steering angle and DRIVE motion command

        if self.state == "DRIVE":               # Handle initial line loss event
            self.state = "SEARCH_STOP"          # Set state to wait in place
            self.lost_line_time = current_time  # Record timestamp when line was lost
            print("OnboardVisionController: line lost, stopping to wait for recovery.") # Log line loss event

        if self.state == "SEARCH_STOP":         # Process line search pause state
            if time.ticks_diff(current_time, self.lost_line_time) > 3500: # Check if 3.5s timeout expired
                self.state = "REVERSE"          # Transition state machine to REVERSE recovery mode
                self.reverse_start_time = current_time # Record start time of reverse maneuver
                print("OnboardVisionController: wait complete, reversing to recover the line.") # Log transition
            return self.servo_center, "STOP"    # Hold wheels straight and motors stopped during search pause

        elapsed_reverse = time.ticks_diff(current_time, self.reverse_start_time) # Calculate elapsed reverse duration
        reverse_angle = max(45, min(135, 180 - self.last_valid_angle)) # Calculate mirrored angle for reverse tracking
        if elapsed_reverse < 5000:               # Limit reverse recovery duration to 5000ms max
            return reverse_angle, "REVERSE"     # Return mirrored angle and REVERSE state command
            
        return self.servo_center, "STOP"        # Final fallback stop if reverse recovery fails to find line