"""
=========================================================================================
SUMMARY OF WHAT THIS CODE DOES:
-----------------------------------------------------------------------------------------
This script is the main "Eyes and Brain" of the line-following robot, running on your PC.
It uses Object-Oriented Programming (OOP) to divide the computer vision and networking 
tasks into distinct, manageable classes (modules).

1. Video Capture (The Eyes): 
   - It connects to your phone's camera (via DroidCam or USB) in a background thread 
     so the video feed remains smooth and doesn't lag the main program.
2. Vision Processing (The Brain):
   - It takes each video frame, shrinks it, and converts it to black and white.
   - It calculates the center of the dark line and figures out how far off-center it is.
   - It uses a Proportional (P) controller formula to map that error into a steering angle.
3. MQTT Publisher (The Mouth):
   - It connects to a public internet broker (broker.hivemq.com).
   - It limits sending commands to exactly 10 times a second to prevent flooding the network.
   - It publishes the calculated steering angle and drive state (e.g., "90,DRIVE") 
     up to the internet, where the physical robot (ESP32) is waiting to hear it.

OOP Architecture:
- VideoStream: Encapsulates the multi-threaded camera hardware logic.
- VisionController: Encapsulates the math and OpenCV image processing logic.
- MQTTController: Encapsulates the network connection and publishing logic.
- RobotCommander: The central orchestrator that takes the above objects 
  (Dependency Injection) and runs the main video loop.
=========================================================================================
"""

import cv2                       # Import OpenCV for computer vision and image processing
import threading                 # Import threading to run the camera feed in the background without freezing the app
import time                      # Import time to track delays and limit how fast we send messages
import paho.mqtt.client as mqtt  # Import the Paho MQTT library to talk to the internet broker

"""DROID_CAM_INDEX = 0        # Define the camera index (0 is usually the default webcam or DroidCam)"""

# ---------------------------------------------------------
# CREDENTIAL LOADING — secrets are NOT stored in this file
# ---------------------------------------------------------
# SECURITY: Wi-Fi passwords, MQTT credentials, and the command
# secret must never be committed to source control. They belong
# in a separate file (config.py) that is listed in .gitignore.
#
# Create a file called config.py in the same folder as this script
# with the following contents (fill in your actual values):
#
#   MQTT_BROKER    = "192.168.1.50"          # LAN IP of your private Mosquitto broker
#   COMMAND_TOPIC  = "primerail/robot/control"
#   FLEET_ALERT_TOPIC = "primerail/robot/alerts"
#   COMMAND_SECRET = "your-long-random-secret-here"  # Must match main.py on the ESP32
#
# BROKER NOTE: broker.hivemq.com is a FREE PUBLIC broker. Anyone who
# finds this topic name can publish commands. Replace it with your own
# private Mosquitto instance (free, runs on any PC on your LAN) and
# enable TLS + username/password so only your devices can connect.
# Until that is done the COMMAND_SECRET is the only access control.
try:
    import config as _cfg                         # Attempt to import the local config file that holds the real credentials
    MQTT_BROKER       = _cfg.MQTT_BROKER          # Load the broker address from the secure config file
    COMMAND_TOPIC     = _cfg.COMMAND_TOPIC        # Load the command topic from the secure config file
    FLEET_ALERT_TOPIC = _cfg.FLEET_ALERT_TOPIC    # Load the fleet alert topic from the secure config file
    COMMAND_SECRET    = _cfg.COMMAND_SECRET       # Load the shared secret from the secure config file
    print("robot.py: credentials loaded from config.py.") # Confirm the secure config was found and loaded
except ImportError:                               # config.py is missing — fail loudly rather than silently falling back to a placeholder
    raise SystemExit(                      # Exit immediately with a clear message so the operator knows exactly what to do
        "FATAL: config.py not found. Create it with MQTT_BROKER, COMMAND_TOPIC, "
        "FLEET_ALERT_TOPIC, and COMMAND_SECRET before running robot.py. "
        "See the comment block above for the required format."
    )


# ---------------------------------------------------------
# OOP Concept: Encapsulation of Camera Hardware
# ---------------------------------------------------------
class VideoStream:         # Define a class dedicated to handling the live camera feed
    def __init__(self, src=0, stale_reconnect_frames=30): # Constructor method that takes the camera source index
        self.src = src      # ADDED: remember the source so we can re-open it later if the feed dies
        self.stream = cv2.VideoCapture(src) # Create an OpenCV video capture object attached to the camera
        if not self.stream.isOpened(): # Check if the camera successfully turned on
            raise ValueError(f"Unable to open video source: {src}") # Crash with a helpful error if the camera is blocked
        self.grabbed, self.frame = self.stream.read() # Take the very first picture to initialize the variables
        self.last_frame_time = time.time() # ADDED: timestamp of the last successful grab, used to detect a stale/frozen feed
        self.stale_reconnect_frames = stale_reconnect_frames # ADDED: how many consecutive failed reads before we attempt a reconnect
        self.stopped = False # Create a flag to track whether the camera should be running or stopped
        self.thread = threading.Thread(target=self.update, daemon=True) # Create a background worker thread for the camera
        self.thread.start() # Start the background thread so it constantly pulls new frames

    def update(self):      # The method that runs forever in the background thread
        consecutive_failures = 0 # ADDED: count read failures in a row so we know when the feed has actually died vs. one dropped frame
        while not self.stopped: # Keep looping as long as the stopped flag is False
            grabbed, frame = self.stream.read() # Grab the absolute newest frame from the camera hardware
            if grabbed:    # If a frame was successfully captured
                self.grabbed, self.frame = grabbed, frame # Update the class variables with the new picture
                self.last_frame_time = time.time() # ADDED: mark this as the last time we actually got a real frame
                consecutive_failures = 0 # ADDED: reset the failure streak now that we're getting frames again
            else:          # ADDED: the read failed - the feed may have dropped (DroidCam/WiFi hiccup, USB unplug, etc.)
                consecutive_failures += 1 # ADDED: count it
                self.grabbed = False # ADDED: make the failure visible to read() immediately rather than keeping stale "grabbed=True"
                if consecutive_failures >= self.stale_reconnect_frames: # ADDED: FAILSAFE - the feed has been dead for a while, try to recover it rather than silently sitting on old data forever
                    print("Camera feed unresponsive - attempting to reconnect...")
                    try:
                        self.stream.release()
                        time.sleep(1.0)
                        self.stream = cv2.VideoCapture(self.src)
                    except Exception as reconnect_err:
                        print(f"Camera reconnect attempt failed: {reconnect_err}")
                    consecutive_failures = 0 # ADDED: reset so we don't hammer reconnect attempts back-to-back

    def read(self):        # Method for the main program to ask for the latest picture
        # Return a copy of the frame so the main program doesn't accidentally corrupt the live feed memory
        return self.grabbed, self.frame.copy() if self.frame is not None else None 

    def is_stale(self, max_age_s=0.5): # ADDED: lets the caller know if the "current" frame is actually old/frozen data, not just whether the last read() call succeeded
        return (time.time() - self.last_frame_time) > max_age_s

    def stop(self):        # Method to cleanly shut down the camera hardware
        self.stopped = True # Set the flag to True, which breaks the loop inside the update() method
        self.thread.join() # Wait for the background thread to safely finish its last loop
        self.stream.release() # Tell the computer hardware to let go of the camera


# ---------------------------------------------------------
# OOP Concept: Encapsulation of Network Publishing
# ---------------------------------------------------------
class MQTTController:      # Define a class dedicated to handling outbound internet communication
    def __init__(self, broker, topic, command_secret, max_connect_attempts=5): # CHANGED: added command_secret parameter - the ESP32 now rejects any payload that doesn't begin with the correct shared secret token
        self.topic = topic                 # Store the MQTT topic string that the ESP32 is subscribed to
        self.command_secret = command_secret # ADDED: store the shared secret so send_command() can prepend it to every payload it builds
        self.client = mqtt.Client()        # Create the main Paho MQTT client object that manages the broker connection
        self.connected = False             # ADDED: track live connection state so send_command() can know whether a publish will actually go anywhere
        self.client.on_connect = self._on_connect # ADDED: register callback so we find out when we're actually connected (and reconnected)
        self.client.on_disconnect = self._on_disconnect # ADDED: register callback so a dropped broker connection is visible immediately, not just inferred later

        print(f"Connecting to MQTT Broker at {broker}...") # Print a status message to the PC terminal so the operator can see the connection is being attempted
        self.client.reconnect_delay_set(min_delay=1, max_delay=30) # ADDED: tell Paho to automatically try to reconnect if the connection drops unexpectedly, starting with a 1s delay and backing off up to 30s between attempts - this is what answers "why can't it reconnect?": it CAN and WILL, Paho handles it transparently in the background thread started by loop_start() below
        attempt = 0                                    # Track how many connection attempts have been made so we can give up after max_connect_attempts and raise a clear error
        while True:                                    # Retry the initial connect in a loop until it succeeds or we hit the attempt limit
            try:                                       # Guard each individual connect attempt so a failure moves to the next iteration rather than crashing
                self.client.connect(broker, 1883, 60)  # Open the TCP connection to the broker on port 1883 (standard MQTT port) with a 60-second keep-alive ping interval
                print(f"MQTT: initial TCP connection to {broker}:1883 established. Waiting for broker acknowledgement...") # Log the TCP handshake success (the full MQTT session is confirmed in the on_connect callback)
                break                                  # Connection succeeded - exit the retry loop and proceed to loop_start()
            except Exception as e:                     # Catch any failure: DNS resolution error, TCP refused, network unreachable, etc.
                attempt += 1                           # Count this failed attempt
                print(f"MQTT connection attempt {attempt}/{max_connect_attempts} failed: {e}") # Log the exact error and attempt number so the operator can diagnose the cause
                if attempt >= max_connect_attempts:    # Check if we have exhausted all allowed attempts
                    raise ConnectionError(f"Could not reach MQTT broker at {broker} after {max_connect_attempts} attempts. Check broker address, port, and network.") from e # Raise a descriptive error so the __main__ startup guard can catch it and clean up
                print(f"Retrying in 2 seconds...") # Inform the operator that another attempt is coming
                time.sleep(2)                          # Wait 2 seconds before the next attempt to avoid hammering a temporarily unavailable broker

        self.client.loop_start()                       # Start Paho's background network thread - this thread handles all I/O (pings, publishes, receives, and auto-reconnects) so the main program never blocks on network operations
        print("MQTT: background network loop started. Auto-reconnect is ACTIVE.") # Confirm that the background thread is running and that automatic reconnection is enabled

    def _on_connect(self, client, userdata, flags, rc): # Paho callback: fires automatically every time the client successfully (re)connects to the broker
        self.connected = (rc == 0)                     # Set connected flag to True only if the return code is 0, which means the broker accepted our connection
        if self.connected:                             # Check if the connection was actually accepted
            print(f"MQTT: successfully connected to broker. Return code: {rc} (0 = accepted).") # Confirm the connection is live and commands will now be delivered
            self.client.subscribe(self.topic)          # Re-subscribe to the command topic on every (re)connect - Paho does NOT carry subscriptions across reconnections, so this must be called here, not just in __init__
            print(f"MQTT: re-subscribed to topic '{self.topic}' after (re)connect.") # Confirm the subscription was re-registered so no incoming override commands are missed
        else:                                          # The broker actively rejected our connection (wrong credentials, banned, etc.)
            print(f"MQTT: broker rejected connection. Return code: {rc}. Check broker credentials/address.") # Log the rejection code so the operator can diagnose the cause

    def _on_disconnect(self, client, userdata, rc): # Paho callback: fires automatically the instant the broker connection is lost for any reason
        self.connected = False                         # Immediately mark the connection as down so send_command() stops attempting publishes against a dead socket
        if rc == 0:                                    # rc == 0 means we called disconnect() ourselves (a clean, intentional shutdown)
            print("MQTT: cleanly disconnected from broker. Session closed intentionally - no reconnect will be attempted.") # Confirm the deliberate shutdown so the operator knows this is expected
        else:                                          # rc != 0 means the broker dropped us unexpectedly (network loss, broker crash, router reboot, etc.)
            print(f"MQTT: unexpected disconnect from broker (rc={rc}). Paho's auto-reconnect is active and will attempt to restore the connection in the background.") # Log the unplanned drop and inform the operator that recovery is already in progress
            print("MQTT: while disconnected, all commands will be dropped locally. The ESP32's 500ms watchdog will halt the vehicle automatically until connection is restored.") # Explain the safety consequence so the operator knows the robot is safe while offline

    def send_command(self, angle, engine_state): # Method to build and publish one authenticated command to the robot over MQTT
        # The payload format is "SECRET,angle,state" — three comma-separated fields.
        # Field 1: the shared secret token. The ESP32's process_message() reads this first
        #          and silently discards the whole packet if it does not match its own copy.
        #          This means anyone who finds or guesses this public topic cannot drive the
        #          vehicle without also knowing the secret.
        # Field 2: the steering angle as an integer (45=full left, 90=straight, 135=full right).
        # Field 3: the motor state string (DRIVE / REVERSE / STOP).
        payload = f"{self.command_secret},{angle},{engine_state}" # Assemble the authenticated three-field payload string
        if not self.connected:                         # Guard: if the broker connection is currently down, a publish would silently fail or raise
            print(f"MQTT WARNING: not connected — command dropped: {payload}") # Log the dropped payload so the operator knows exactly what was missed
            return                                     # Return without publishing; the ESP32's 500ms watchdog will halt the vehicle if commands stop arriving
        self.client.publish(self.topic, payload)       # Transmit the authenticated payload to the broker, which relays it to all subscribers on this topic
        print(f"MQTT: command sent -> topic='{self.topic}' | payload='{payload}'") # Log every outbound command so every steering decision is visible in the terminal

    def stop(self):        # Method to cleanly shut down the MQTT connection — called ONLY at program exit, never during normal operation
        print("MQTT: initiating clean shutdown sequence...") # Log that a deliberate, intentional shutdown has been requested (not a crash or unexpected drop)
        self.client.loop_stop()                        # Stop Paho's background network thread first so no more callbacks fire and no more auto-reconnects trigger during teardown
        print("MQTT: background network thread stopped.") # Confirm the thread is fully stopped so the operator knows no further network activity will occur
        self.client.disconnect()                       # Send a clean MQTT DISCONNECT control packet to the broker; rc=0 in on_disconnect confirms this was intentional and the broker released the session cleanly
        print("MQTT: disconnect packet sent. Connection closed. All MQTT activity has ceased.") # Confirm that both sides of the session are closed and no more commands will be sent or received


# ---------------------------------------------------------
# OOP Concept: Encapsulation of Vision Logic & Math
# ---------------------------------------------------------
class VisionController:    # Define a class dedicated to image processing and steering calculations
    def __init__(self, scale_factor=0.5): # Constructor that allows us to shrink the image to save CPU power
        self.scale_factor = scale_factor # Store the shrink ratio inside the object
        self.servo_center = 90                         # Define 90 degrees as perfectly straight ahead - this is the physical center of the servo's travel range
        self.last_valid_angle = 90                     # INITIALISED to 90 (straight ahead) as a safe default only - this value is OVERWRITTEN with the real detected angle every time the line IS visible (see process_frame). It is only READ during reverse recovery, when the line has just been lost, so at that point it always holds the last angle that was computed from an actual detection, not this 90. The 90 here is a fallback for the very first frame if it happens to be a miss.

        self.state = "DRIVE"     # Initialize the state machine state string ("DRIVE", "SEARCH_STOP", "REVERSE")
        self.lost_line_timestamp = 0 # Initialize a timer for the emergency stop wait window
        self.reverse_start_timestamp = 0 # Initialize a timer for the reverse recovery window

    def map_error_to_angle(self, error): # Method to convert the pixel distance of the line from the frame center into a physical servo steering angle
        kp = 0.1                                       # CHANGED from 0.2 to 0.1: Proportional gain controls how sharply the robot reacts to line offset. 0.2 caused aggressive steering corrections that could overshoot and oscillate. 0.1 gives a calmer, smoother response - the vehicle still corrects the same errors but takes slightly longer, which is safer and less likely to cause a skid or loss of traction on a real vehicle. Increase this only if the vehicle is consistently late in following tight curves.
        angle = self.servo_center + int(error * kp)    # Multiply the pixel error by kp and add to the center angle to get the corrected steering direction
        print(f"map_error_to_angle: pixel_error={error} -> steering_angle={int(max(45, min(135, angle)))} (kp={kp}, center={self.servo_center})") # Log the exact input error and output angle so every steering calculation is traceable
        return int(max(45, min(135, angle)))            # Clamp the result to the safe mechanical range (45=full left, 135=full right) so the servo gears are never stripped

    def process_frame(self, frame): # The main method that analyzes a single picture and decides the next motor and steering action
        # Downscale the image to reduce CPU load and make the vision logic run faster on the PC or ESP32 side
        small_frame = cv2.resize(frame, (0, 0), fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_AREA)
        height, width = small_frame.shape[:2] # Store the reduced frame dimensions so steering math can use the new image size

        # Convert the frame to grayscale so the dark line is easier to isolate from the background
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = cv2.mean(gray)[0] # Measure the overall brightness; low values usually mean the camera is blocked or the room is dark

        # If the image is too dark, stop immediately because the robot cannot trust the line detection
        if mean_brightness < 20:
            target_angle = self.servo_center # Keep the steering straight when the camera cannot see clearly
            engine_state = "STOP" # Prevent the robot from moving blindly in a low-light or blocked-camera condition
            cv2.putText(small_frame, "TOO DARK", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return target_angle, engine_state, small_frame

        # -----------------------------------------------------------------------
        # DETECTION VALIDATION: guard against all the common wrong-detection sources
        # before trusting the threshold image for line-following decisions.
        # -----------------------------------------------------------------------

        # --- Guard 1: Brightness ceiling (camera glare / overexposed surface) ---
        # A very bright frame means the camera is pointed at a reflective surface or
        # direct light source. In that condition adaptive thresholding inverts and the
        # "line" is actually a bright glare blob. We stop rather than steer randomly.
        if mean_brightness > 220:                      # 220 out of 255 means the scene is almost entirely white/overexposed
            target_angle = self.servo_center           # Keep steering straight so the vehicle doesn't veer while blind
            engine_state = "STOP"                      # Stop the motors so the vehicle doesn't drive into whatever is causing the glare
            print(f"Frame too bright (brightness={mean_brightness:.1f}). Stopping to avoid glare-induced wrong detection.") # Log the specific brightness value so the operator can tune the threshold
            cv2.putText(small_frame, f"TOO BRIGHT ({mean_brightness:.0f})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2) # Draw orange warning on the display frame
            return target_angle, engine_state, small_frame # Return early - do not proceed to thresholding on an overexposed frame

        # --- Guard 2: Erase any overlay pixels drawn in previous frames ---
        # The processed_frame returned from this function has text and circles drawn
        # on it by cv2.putText / cv2.circle (the green line-center dot, state labels etc.).
        # If the caller accidentally passes THAT annotated frame back into this function
        # on the next iteration (instead of a fresh camera frame), those bright green/red
        # overlay pixels will register as dark blobs in the threshold step and produce
        # completely wrong centroids. We detect this by checking for pure annotation colours.
        # The correct fix is to always pass a raw camera frame - but as a second defence
        # we wipe known overlay colours from the working copy before thresholding.
        clean_frame = small_frame.copy()               # Always work on a clean copy so we never corrupt the display frame that gets returned to the caller
        # Mask out pure green (the line-center dot) and pure red/orange (warning text) pixels
        overlay_mask = cv2.inRange(clean_frame, (0, 200, 0), (80, 255, 80))   # Detect bright-green pixels (the cv2.circle dot drawn in previous frame)
        overlay_mask |= cv2.inRange(clean_frame, (0, 0, 200), (80, 80, 255)) # Also detect bright-red pixels (the cv2.putText warnings drawn in previous frame)
        clean_frame[overlay_mask > 0] = (128, 128, 128) # Replace overlay pixels with mid-gray so they are neutral in thresholding (not dark = not line, not white = not background)
        print(f"Overlay mask: {cv2.countNonZero(overlay_mask)} annotation pixels neutralised before thresholding.") if cv2.countNonZero(overlay_mask) > 0 else None # Only log when there are actually overlay pixels to neutralise, to avoid flooding the output on every normal frame

        # --- Apply adaptive threshold on the clean (overlay-free) copy ---
        # Use the CLEANED frame, not small_frame, so overlay pixels can never affect detection
        thresh = cv2.adaptiveThreshold(                # Adaptive threshold automatically adjusts to varying lighting across the frame
            cv2.cvtColor(clean_frame, cv2.COLOR_BGR2GRAY), # Convert the cleaned colour frame to grayscale for thresholding
            255,                                       # Maximum output value - detected pixels become white (255)
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,                # Use a Gaussian-weighted local neighbourhood to compute the threshold, more noise-resistant than the mean method
            cv2.THRESH_BINARY_INV,                     # Invert the binary output so DARK pixels (the line) become WHITE in the threshold image (easier for moments)
            11,                                        # Block size: the local neighbourhood is 11x11 pixels - tune this if the line is much thicker or thinner than usual
            2                                          # Constant C: subtract 2 from the computed threshold to reduce noise sensitivity
        )
        M = cv2.moments(thresh)                        # Compute image moments on the threshold image to find the total mass and centroid of all detected (white) pixels

        # --- Guard 3: Reject low-solidity blobs (shadows, dust, isolated noise) ---
        # A real line produces a long, narrow connected region. Shadows and dust produce
        # small, scattered blobs. We measure solidity (filled area / bounding box area)
        # and reject detections that look more like noise than a real line.
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Find all connected white regions in the threshold image
        valid_contours = []                            # Collect only the contours that pass the shape validity tests below
        for cnt in contours:                           # Check every detected contour individually
            area = cv2.contourArea(cnt)                # Measure the filled area of this contour in pixels
            if area < 500:                             # Skip tiny blobs (dust, single pixels, jpeg compression artifacts)
                continue                               # This contour is too small to be a real line segment - skip it
            x, y, w, h = cv2.boundingRect(cnt)        # Get the bounding rectangle of this contour to compute its aspect ratio
            aspect = max(w, h) / (min(w, h) + 1)      # Compute aspect ratio: a real line is elongated (aspect >> 1); a shadow blob or corner mark is roughly square (aspect ≈ 1)
            if aspect < 1.5:                           # Reject roughly-square blobs - they are almost certainly not the line
                print(f"Rejected blob (aspect={aspect:.2f}, area={area}): too square to be the line, likely a corner mark or shadow.") # Log rejected blobs so the operator can tune the aspect threshold if needed
                continue                               # Skip this contour and do not count it towards M["m00"]
            valid_contours.append(cnt)                 # This contour passed both tests - it is a plausible line segment

        # Re-compute moments only on the validated contours so noise blobs don't pollute the centroid
        clean_thresh = thresh.copy()                   # Start with the full threshold image
        clean_thresh[:] = 0                            # Zero it out completely
        cv2.drawContours(clean_thresh, valid_contours, -1, 255, -1) # Re-draw ONLY the validated contours so M is computed on clean data
        M = cv2.moments(clean_thresh)                  # Re-compute moments on the validated, noise-free threshold image

        # If enough dark pixels are found, the line is visible and the robot can continue driving normally
        if M["m00"] > 15000:   # This threshold means the line occupies enough pixels to be considered a real detection
            cx = int(M["m10"] / M["m00"]) # Calculate the horizontal center of the detected line
            error = cx - (width // 2) # Measure how far the line is from the middle of the frame
            target_angle = self.map_error_to_angle(error) # Convert the pixel error into a steering angle
            self.last_valid_angle = target_angle # Save the last good steering angle for reverse recovery planning

            # If the robot was in a recovery state, switch back to normal forward motion as soon as the line is found again
            if self.state in ["SEARCH_STOP", "REVERSE"]:
                print("Line detected! Returning to forward drive.")
                self.state = "DRIVE"

            engine_state = "DRIVE"                     # Command the motors to move forward when the line is visible and the detection has passed all validity guards
            cv2.circle(small_frame, (cx, height // 2), 5, (0, 255, 0), -1) # Draw a green dot at the detected line centroid so the operator can visually verify the detection is on the actual line
            cv2.putText(small_frame, f"LINE DETECTED: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) # Draw the current state so the operator knows the vehicle is in normal drive mode
            print(f"process_frame: line detected | centroid_x={cx} | error={error} | angle={target_angle} | state={self.state} | last_valid_angle saved={self.last_valid_angle}") # Log the full detection result for every frame where the line is found

        # If the line is lost, activate the failsafe behavior: stop, wait, then reverse to recover it
        else:                  # This branch runs only when the line is no longer visible in the frame
            current_time = time.time() # Use the current clock time to measure how long the robot has been in each recovery stage

            # Enter the stop phase the first time the line is lost during normal driving
            if self.state == "DRIVE":                  # Check if this is the very first frame where the line has disappeared (state was DRIVE, meaning it was visible on the previous frame)
                self.state = "SEARCH_STOP"             # Transition to the stop-and-wait recovery phase - the vehicle will hold still to let the line settle back into view before committing to a reverse maneuver
                self.lost_line_timestamp = current_time # Record the exact timestamp when the line was lost so we can time the 3.5-second wait window accurately
                print(f"process_frame: LINE LOST. Transitioning DRIVE -> SEARCH_STOP. Vehicle will hold still for 3.5s. Lost at t={current_time:.2f}.") # Log the state transition with timestamp so the recovery timeline is traceable

            if self.state == "SEARCH_STOP":            # Execute this block every frame while the vehicle is in the hold-still wait phase
                target_angle = self.servo_center       # Keep steering centered (90°) while the vehicle is paused so it does not roll left or right under any residual momentum
                engine_state = "STOP"                  # Hold the motors stopped during the entire wait window - do not move the vehicle at all while waiting for line recovery
                print(f"process_frame: SEARCH_STOP active | waiting for line | elapsed={current_time - self.lost_line_timestamp:.2f}s / 3.5s | angle={target_angle}") # Log the wait progress on every frame so the operator can see how long the vehicle has been stopped

                if current_time - self.lost_line_timestamp > 3.5: # Check if the 3.5-second wait window has expired without the line reappearing
                    self.state = "REVERSE"             # Transition from stop to reverse recovery mode now that waiting alone has not found the line
                    self.reverse_start_timestamp = current_time # Record when the reverse maneuver started so we can bound its duration
                    print(f"process_frame: 3.5s wait complete. Transitioning SEARCH_STOP -> REVERSE. Reverse start at t={current_time:.2f}. Last valid angle was {self.last_valid_angle}°.") # Log the transition with the last known good angle so the operator knows which direction the vehicle will retrace

            # In reverse mode: drive backward along the SAME CURVED PATH the vehicle was on when it lost the line.
            # Three design constraints drive this implementation (see comment tag in the review):
            #  1. SAME DIRECTION: mirror the last valid steering angle so the vehicle retraces its exact path backward
            #     rather than reversing straight (which would take it further from the line into unknown territory).
            #  2. BOUNDED DISTANCE: use time as a proxy for distance. At the minimum reverse speed the vehicle moves
            #     slowly enough that a fixed time window corresponds to a safe, predictable retreat distance.
            #     "Distance" is time-bounded here because the ESP32 has no wheel encoders - on a real vehicle with
            #     encoders you would replace the time check below with an actual odometry distance check.
            #  3. MINIMUM SPEED: the REVERSE command sent to the ESP32 drives at whatever speed the MotorController's
            #     drive_reverse() is configured for. On RC cars this is typically full reverse PWM. For a real vehicle
            #     with variable speed control you would send "REVERSE_SLOW" instead and handle it in main.py.
            #     On this hardware, safety is provided by the short time window (max 3.0s) and the steering mirror,
            #     NOT by speed reduction, since there is no variable-speed reverse in the current MotorController.
            elif self.state == "REVERSE":               # Only execute this block when the state machine is in the REVERSE recovery phase
                elapsed_reverse = current_time - self.reverse_start_timestamp # Measure how many seconds have elapsed since reverse recovery started
                MAX_REVERSE_TIME = 3.0                  # CHANGED from 5.0 to 3.0 seconds: shorter window = bounded reverse distance; at typical RC car speeds (~0.3 m/s reverse) this is ~0.9m maximum retreat, enough to get back on the line without reversing into obstacles or following vehicles
                print(f"REVERSE RECOVERY: elapsed={elapsed_reverse:.1f}s / {MAX_REVERSE_TIME}s | mirroring last angle={self.last_valid_angle} -> reverse_angle={180 - self.last_valid_angle}") # Log every reverse frame with exact timing and steering values so the operator can see the recovery in progress

                # Mirror the last valid FORWARD steering angle to compute the correct REVERSE steering angle.
                # Physics: if the vehicle was steering LEFT (angle < 90) when it lost the line, reversing with
                # the SAME left steer would curve it further away. Mirroring to RIGHT (180 - angle > 90) curves
                # it back along the same arc it came from, which is exactly where the line is.
                reverse_angle = 180 - self.last_valid_angle # Mirror: left-forward becomes right-reverse, right-forward becomes left-reverse
                target_angle = max(45, min(135, reverse_angle)) # Clamp to safe servo range as always

                if elapsed_reverse < MAX_REVERSE_TIME:  # Check if we are still within the safe bounded reverse distance window
                    engine_state = "REVERSE"            # Send REVERSE state to the ESP32 to drive motors backward at minimum available speed
                else:                                   # The maximum reverse window has expired with no line found - stop rather than continuing to back up indefinitely
                    engine_state = "STOP"               # Stop the vehicle completely as the safest action after a failed recovery attempt
                    print("REVERSE RECOVERY TIMEOUT: line not found within bounded reverse distance. Halting. Operator intervention may be required.") # Notify the operator that autonomous recovery has been exhausted
                    cv2.putText(small_frame, "REVERSE TIMEOUT - OPERATOR REQUIRED", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2) # Display the timeout on screen so anyone watching the feed knows the vehicle is stopped and waiting

            cv2.putText(small_frame, f"FAILSAFE: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) # Draw the current failsafe state name in red on the display frame so the operator always knows which recovery phase is active

        print(f"process_frame COMPLETE: returning angle={target_angle} | engine_state={engine_state} | vision_state={self.state}") # Log the exact values being returned so every frame's decision is visible in the terminal output
        return target_angle, engine_state, small_frame # Return the computed steering angle, motor state, and the annotated display frame to the RobotCommander

# ---------------------------------------------------------
# OOP Concept: Dependency Injection & Orchestration
# ---------------------------------------------------------
class RobotCommander:      # Define the main orchestrator class that ties the camera, vision, and network together
    def __init__(self, camera_module, vision_module, mqtt_module): # Constructor accepting our three sub-systems via Dependency Injection
        self.camera = camera_module                    # Store the injected VideoStream object that provides raw camera frames
        self.vision = vision_module                    # Store the injected VisionController object that computes steering angles
        self.mqtt = mqtt_module                        # Store the injected MQTTController object that publishes authenticated commands
        self.last_send_time = time.time()              # Initialize a send-rate timer so we publish at most once every 100ms and do not flood the broker
        self.breakdown_active = False                  # ADDED: flag that tracks whether the system is currently in a mid-run breakdown and post-breakdown safety check. False = running normally. True = camera/system failure detected, holding STOP until all post-breakdown checks pass.
        print("RobotCommander: initialized. Camera, Vision, and MQTT sub-systems injected and ready.") # Confirm that all three sub-systems were received and stored

    def run(self):         # The main infinite loop method that runs the application from first frame to shutdown
        print("RobotCommander: main loop starting. Press 'q' in the video window to quit.") # Print startup instructions so the operator knows how to exit cleanly
        
        try:               # Start a try-finally block to ensure we shut down safely even if there is a crash
            while True:    # Start an infinite loop that processes video frames as fast as possible
                ret, frame = self.camera.read() # Ask the camera object for the absolute newest picture
                # CHANGED: FAILSAFE - previously a failed/missing frame just did `continue`, silently skipping
                # this iteration and trusting the ESP32's own 500ms watchdog to eventually notice the silence.
                # That's a fine backstop, but this side should never rely on it alone: as soon as we know the
                # camera is blind (read failed, OR the "current" frame is actually stale leftover data from a
                # frozen feed), we explicitly command a stop right now instead of waiting out the far-side timer.
                if not ret or frame is None or self.camera.is_stale(): # Check if the camera read failed, returned nothing, or the frame is frozen/stale old data
                    # -----------------------------------------------------------------------
                    # MID-RUN CAMERA FAILURE: NOTIFY ALL PARTIES + SELF-CHECK BEFORE RESUME
                    # -----------------------------------------------------------------------
                    # Comment 7: When the vehicle stops mid-run due to a hardware error, every
                    # system that could be affected must be informed IMMEDIATELY:
                    #   - The main MQTT controller (operator/fleet manager): receives the explicit
                    #     STOP command and can see the WARNING alert topic to know WHY it stopped.
                    #   - Vehicles behind / passing pods: in a real GRT deployment (BIEBUS etc.)
                    #     you would publish to a FLEET_ALERT topic so following vehicles brake early.
                    #     That topic is published here. Subscribing vehicles and trackside systems
                    #     should subscribe to FLEET_ALERT_TOPIC to receive these notifications.
                    #   - The operator dashboard: the WARNING payload is human-readable and includes
                    #     the specific failure reason so the operator can triage remotely.
                    # Comment 8: Before allowing the main loop to resume after the camera comes back,
                    # we run a POST-BREAKDOWN SAFETY CHECK to confirm the system is clean:
                    #     1. Camera is actually producing fresh frames (not still returning stale data)
                    #     2. MQTT connection is live (so commands will reach the vehicle)
                    #     3. Vision state machine is reset to DRIVE (no leftover REVERSE or SEARCH_STOP
                    #        state that would cause unexpected behavior on first good frame)
                    # Only after all three checks pass do we clear the breakdown flag and let vision
                    # processing resume. If any check fails, we hold the STOP command and try again.
                    # -----------------------------------------------------------------------

                    if not self.breakdown_active:      # Only trigger the notification sequence once per breakdown event, not on every frame during the outage
                        self.breakdown_active = True   # Set the breakdown flag so subsequent frames skip straight to the safety check below without re-broadcasting
                        print("MID-RUN STOP: camera failure detected. Broadcasting emergency halt to all subscribers...") # Log the breakdown trigger

                        # --- Notify the main controller via a dedicated WARNING alert topic ---
                        warning_payload = f"{self.mqtt.command_secret},WARNING,CAMERA_FAILURE" # Build a human-readable warning payload including the shared secret for authentication
                        try:                           # Guard the alert publish so a simultaneous MQTT outage can't suppress the safety command below
                            self.mqtt.client.publish(FLEET_ALERT_TOPIC, warning_payload) # Publish the warning to the fleet alert topic - operator dashboard + following vehicles subscribe here
                            print(f"FLEET ALERT published to '{FLEET_ALERT_TOPIC}': {warning_payload}") # Confirm the alert went out
                        except Exception as alert_err: # If the alert publish itself fails (e.g. MQTT also just dropped)
                            print(f"Warning: fleet alert publish failed ({alert_err}). Proceeding with local STOP command regardless.") # Log but do not block the safety stop

                        # --- Send the actual STOP command to the robot ---
                        self.mqtt.send_command(90, "STOP") # Stop the vehicle: center steering + cut motors. This is the primary safety action.
                        self.last_send_time = time.time()  # Reset the send timer so the rate limiter does not suppress the next command after recovery
                        print("Emergency STOP command sent. Vehicle should be halting now.") # Confirm the stop was issued

                    # --- POST-BREAKDOWN SAFETY CHECK (runs every frame until all checks pass) ---
                    all_clear = True                   # Assume all checks will pass; any failure flips this to False and holds the stop state
                    # Check 1: camera returning fresh frames
                    check_ret, check_frame = self.camera.read() # Ask the camera for the latest frame
                    if not check_ret or check_frame is None or self.camera.is_stale(): # Test if the camera is actually healthy yet
                        print("POST-BREAKDOWN CHECK 1 FAIL: camera still not returning fresh frames. Holding STOP.") # Log the failure
                        all_clear = False              # Mark the check as failed so we do not resume yet
                    else:                              # Camera is returning frames
                        print("POST-BREAKDOWN CHECK 1 PASS: camera is producing fresh frames.") # Log the pass

                    # Check 2: MQTT connection is live
                    if not self.mqtt.connected:        # Test if the MQTT broker connection is active
                        print("POST-BREAKDOWN CHECK 2 FAIL: MQTT not connected. Attempting reconnect before resume...") # Log the failure and the recovery action
                        #self.mqtt.reconnect()          # Attempt to restore the broker connection using the reconnect() method added per Comment 1
                        all_clear = False              # Do not resume yet - wait for the on_connect callback to set connected=True on the next iteration
                    else:                              # MQTT is connected
                        print("POST-BREAKDOWN CHECK 2 PASS: MQTT connection is live.") # Log the pass

                    # Check 3: vision state machine is reset to a clean DRIVE state
                    if self.vision.state != "DRIVE":   # Test if the vision controller is stuck in a recovery sub-state from before the breakdown
                        self.vision.state = "DRIVE"    # Force-reset to DRIVE so the first good frame after recovery is treated as a fresh start, not the tail of an old REVERSE maneuver
                        self.vision.last_valid_angle = 90 # Also reset the last valid angle to center so reverse recovery, if triggered again, starts from a neutral angle
                        print("POST-BREAKDOWN CHECK 3: vision state machine was not in DRIVE - reset to DRIVE and last_valid_angle reset to 90.") # Log the reset
                    else:                              # Vision state is already clean
                        print("POST-BREAKDOWN CHECK 3 PASS: vision state machine is already in DRIVE state.") # Log the pass

                    if all_clear:                      # All three checks passed - it is safe to resume vision processing
                        self.breakdown_active = False  # Clear the breakdown flag so normal vision processing resumes on the next frame
                        print("POST-BREAKDOWN SAFETY CHECK: ALL CHECKS PASSED. Resuming normal vision processing.") # Announce the all-clear to the operator
                    else:                              # At least one check failed - hold the stop and try again on the next frame
                        print("POST-BREAKDOWN SAFETY CHECK: one or more checks failed. Holding STOP until all systems confirm ready.") # Log that we are still waiting

                    if cv2.waitKey(1) & 0xFF == ord('q'): # Check if the operator pressed q to quit even while the camera is down
                        break                          # Exit the main loop immediately if the operator requests it
                    continue                           # Skip vision processing this iteration - loop back to the top and re-run the safety check

                try: # Guard the vision/steering math itself - a bad frame or an OpenCV hiccup should never crash silently without the robot being told to stop
                    target_angle, engine_state, processed_frame = self.vision.process_frame(frame) # Pass the raw camera frame to the vision brain and receive back: the computed steering angle, the motor state, and the annotated display frame
                    print(f"RobotCommander: vision result received -> angle={target_angle} | state={engine_state}") # Log every successful vision result so the operator can confirm the pipeline is running
                except Exception as vision_err:        # Catch any OpenCV crash, memory error, or math exception inside process_frame
                    print(f"RobotCommander: VISION ERROR: {vision_err}. Commanding emergency STOP for safety.") # Log the exact error so the operator can diagnose the cause
                    self.mqtt.send_command(90, "STOP")  # Send an immediate stop - never drive when the vision system has crashed
                    self.last_send_time = time.time()  # Reset the send timer so the recovery command is not suppressed by the rate limiter
                    print("RobotCommander: emergency STOP sent due to vision error. Resuming loop to retry on next frame.") # Confirm the stop was issued and that the loop will keep running
                    continue                            # Skip the rest of this iteration and try again on the next camera frame
                
                if time.time() - self.last_send_time > 0.1: # Check if at least 100ms have passed since the last publish to maintain a safe, steady 10Hz command rate
                    self.mqtt.send_command(target_angle, engine_state) # Publish the authenticated command payload to the broker
                    self.last_send_time = time.time()  # Reset the rate-limiter timer immediately after sending

                cv2.putText(processed_frame, f"ANGLE: {target_angle} | {engine_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2) # Draw the current angle and motor state in blue at the top of the display frame so the operator can read it at a glance
                cv2.imshow("Bot Vision", processed_frame) # Display the annotated camera frame in the OpenCV window so the operator has a live visual feed

                if cv2.waitKey(1) & 0xFF == ord('q'): # Poll for a keypress; 1ms wait keeps the display responsive without blocking the loop
                    print("RobotCommander: operator pressed 'q'. Exiting main loop cleanly.") # Log the intentional exit
                    break                              # Break out of the main loop; the finally block below will handle safe shutdown
        finally:           # This block runs no matter how the loop exits: normal q-quit, Ctrl+C, unhandled exception, or any other path
            print("RobotCommander: main loop exited. Running final shutdown sequence...") # Announce the shutdown so the operator knows cleanup is in progress
            try:                                       # Guard the final STOP publish so a simultaneous MQTT failure can't prevent shutdown from completing
                self.mqtt.send_command(90, "STOP")     # Send one last authenticated STOP command so the robot halts immediately rather than coasting on the last command during the ~500ms before its own watchdog fires
                time.sleep(0.2)                        # Give the publish 200ms to leave the socket buffer before we stop the Paho network thread below
                print("RobotCommander: final STOP command sent successfully.") # Confirm the stop was issued
            except Exception as final_stop_err:        # If the final stop publish fails (broker already down, etc.)
                print(f"RobotCommander: final STOP publish failed ({final_stop_err}). ESP32 watchdog will halt the vehicle within 500ms.") # Log the failure and explain the fallback safety mechanism
            self.shutdown()                            # Call the full hardware teardown method

    def shutdown(self):    # Method to cleanly release all hardware and network resources at program exit
        print("RobotCommander: shutdown() called. Releasing all hardware and network resources...") # Log the start of the teardown sequence
        self.camera.stop()                             # Signal the background camera thread to stop and release the video capture handle
        print("RobotCommander: camera released.") # Confirm the camera is fully stopped and the handle is free
        self.mqtt.stop()                               # Stop Paho's background thread, send a clean DISCONNECT to the broker, and release the socket
        print("RobotCommander: MQTT connection closed.") # Confirm the broker session is closed (on_disconnect will also fire with rc=0)
        cv2.destroyAllWindows()                        # Close the OpenCV display window so it doesn't linger as a zombie process
        print("RobotCommander: OpenCV windows closed. Shutdown complete. All resources released.") # Confirm full teardown is done


# ---------------------------------------------------------
# MAIN EXECUTION (Object Instantiation and Assembly)
# ---------------------------------------------------------
if __name__ == "__main__": # Check if this script is being run directly (not imported as a module)
    print("Starting Robot Commander PC application...") # Print a startup message so the operator knows the script launched
    
    # --- Instantiate the three hardware/network sub-systems and assemble the commander ---
    ESP32_CAMERA_URL = "http://192.168.x.x/" # Replace x.x with your ESP32's actual LAN IP address (check the ESP32 serial output on boot for its assigned IP)
    pc_camera = None   # Pre-declare so the except block below can safely reference pc_camera even if VideoStream() raises before assigning it
    pc_mqtt = None     # Pre-declare so the except block below can safely reference pc_mqtt even if MQTTController() raises before assigning it
    try:               # FAILSAFE: guard the entire bring-up sequence so a partial init (camera up but broker unreachable, etc.) still cleans up everything that DID open
        pc_camera = VideoStream(src=ESP32_CAMERA_URL) # Open the ESP32's MJPEG camera stream using the board's LAN IP - raises ValueError immediately if the URL is unreachable
        time.sleep(1.0) # Wait 1 second after opening the camera to let the MJPEG stream buffer fill so the first few frames are valid before vision processing starts
        
        pc_vision = VisionController(scale_factor=0.5) # Create the vision + steering math object - scale_factor=0.5 halves the resolution before processing to reduce CPU load on the PC
        pc_mqtt = MQTTController(broker=MQTT_BROKER, topic=COMMAND_TOPIC, command_secret=COMMAND_SECRET) # CHANGED: pass the shared secret so send_command() can build authenticated "SECRET,angle,state" payloads that the ESP32 will accept
        
        app = RobotCommander(pc_camera, pc_vision, pc_mqtt) # Assemble the main commander by injecting the three sub-systems (Dependency Injection pattern - no sub-system creates its own dependencies)
        
        app.run()      # Start the infinite main loop: read camera -> run vision -> send authenticated command -> repeat
    except Exception as startup_err: # Catch anything that goes wrong during object construction, before RobotCommander.run()'s own try/finally would ever be reached
        print(f"CRITICAL: Startup failed: {startup_err}") # Print the exact failure so the operator knows which sub-system failed to initialize
        if pc_mqtt is not None: # If the MQTT client was already connected before the failure, send one final stop and log off cleanly so the robot doesn't keep acting on the last command
            try:               # Guard the emergency stop publish so a broken broker at this point can't prevent the rest of cleanup from running
                pc_mqtt.send_command(90, "STOP") # Send a centered-steering + stop-motors command as the last action before going offline
                time.sleep(0.2) # Give the publish a short moment to leave the socket buffer before we disconnect the broker session
                pc_mqtt.stop() # Cleanly disconnect from the MQTT broker and stop its background network thread
            except Exception:  # If the publish or disconnect itself raises, silently ignore it - the robot's own watchdog will handle the silence
                pass           # Nothing further to do; fall through to the camera cleanup
        if pc_camera is not None: # If the camera capture object was created before the failure, release the stream so the OS doesn't leave the socket open
            try:               # Guard close() so a broken stream at this point can't block the process from exiting
                pc_camera.stop() # Release the camera handle and stop the background capture thread
            except Exception:  # Silently ignore any error from stop() itself
                pass           # Nothing further to do
        raise              # Re-raise the original startup error so the full traceback is visible in the terminal for debugging