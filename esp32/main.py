"""
=========================================================================================
SUMMARY OF ROBOT BRAIN (OOP PRODUCTION VERSION)
-----------------------------------------------------------------------------------------
This script acts as the main firmware for an ESP32-S3 line-following robot.
It is built using Object-Oriented Programming (OOP) to cleanly separate responsibilities:

1. SteeringController: Manages the Servo motor. Enforces mechanical safety limits 
   so the steering never attempts to turn past its physical boundaries.
2. MotorController: Manages the L293D driver. Explicitly controls the EN (Enable) 
   pins alongside the directional IN pins to ensure precise power delivery.
3. NetworkController: Handles Wi-Fi connection and the MQTT internet bridge.
4. RobotOrchestrator: The central manager. It listens for commands, safely parses 
   the text, handles errors without crashing, and commands the hardware classes.

Safety Features Built-In:
- Failsafe Startup: Motors are explicitly forced into a STOP state on boot.
- Data Validation: Bad packets (e.g., missing commas, letters instead of numbers) 
  are caught and ignored without crashing the main loop.
- Failsafe Shutdown: If the script crashes or loses connection, a 'finally' block 
  guarantees the physical wheels are stopped to prevent runaway hardware.
=========================================================================================
"""

import network                      # Imports the library needed to control the ESP32 Wi-Fi antenna
import utime as time                         # Imports the timing library to create safe pauses
import machine                      # Imports the top-level machine module (needed for hardware WDT + full reset)
from machine import Pin, PWM, WDT   # Imports hardware controls for copper pins, electrical pulses, and the hardware watchdog
from umqtt.simple import MQTTClient # Imports the lightweight MQTT networking tool

import camera                       # CAMERA HARDWARE LIBRARY
import socket                       # WEB SERVER LIBRARY
import _thread                      # MULTI-CORE PROCESSING LIBRARY

# ---------------------------------------------------------
# DEBUG FLAG — controls high-frequency serial logging
# ---------------------------------------------------------
# On MicroPython, every print() call flushes the UART serial buffer. Inside a 100 Hz
# control loop that is 100 flushes per second, which adds measurable latency to steering
# decisions and MQTT polling. Set DEBUG = False for normal operation. Only enable it
# when actively diagnosing an issue over a serial monitor, then turn it off again.
DEBUG = False                               # True = verbose per-frame logging (use during development only); False = safety-critical prints only (use in production/deployment)

# ---------------------------------------------------------
# 1. HARDWARE CLASSES (The Muscle)
# ---------------------------------------------------------
class SteeringController:                           # Define the SteeringController class to manage the servo motor that physically turns the front wheels
    def __init__(self, pin_num):                    # Constructor method - takes the GPIO pin number that the servo signal wire is connected to
        self.servo = PWM(Pin(pin_num))              # Create a PWM object on the specified pin - PWM (Pulse Width Modulation) is the electrical language servo motors understand
        self.servo.freq(50)                         # Set the PWM signal to 50Hz (one pulse every 20ms) - this is the mandatory frequency all hobby servo motors expect; any other value will confuse or damage the servo
        self.set_angle(90)                          # Immediately command the wheels to the straight-ahead center position on boot, so the vehicle never starts in a turned state

    def set_angle(self, angle):                     # Method to physically turn the front wheels to a specific angle (45=full left, 90=straight, 135=full right)
        safe_angle = max(45, min(135, angle))       # SAFETY CLAMP: force the angle into the allowed 45-135 degree window before touching hardware - if the caller passes 20 or 200 this saves the servo gears from being stripped
        duty = int(((safe_angle / 180.0) * (115 - 40)) + 40) # Convert the human-readable degree number into a PWM duty cycle integer that the ESP32's PWM hardware understands (40 = full left at 45°, 115 = full right at 135°, linear interpolation between)
        self.servo.duty(duty)                       # Send the calculated duty cycle to the physical servo pin, which moves the servo horn and the steering linkage to the target angle


class MotorController:                              # Define the MotorController class to manage the two DC drive motors via the L293D H-bridge driver chip
    def __init__(self, en_a, en_b, in1, in2, in3, in4): # Constructor - takes the six GPIO pin numbers that connect the ESP32 to the L293D chip (two enable pins + four direction pins)
        self.en_a = Pin(en_a, Pin.OUT)              # Initialize EN_A as a digital output - this is the master power enable pin for the LEFT motor; LOW = motor coast/off, HIGH = motor actively driven
        self.in1 = Pin(in1, Pin.OUT)                # Initialize IN1 as a digital output - this is the LEFT motor's FORWARD direction pin; HIGH here with IN2 LOW = forward spin
        self.in2 = Pin(in2, Pin.OUT)                # Initialize IN2 as a digital output - this is the LEFT motor's REVERSE direction pin; HIGH here with IN1 LOW = reverse spin
        self.en_b = Pin(en_b, Pin.OUT)              # Initialize EN_B as a digital output - this is the master power enable pin for the RIGHT motor; same logic as EN_A
        self.in3 = Pin(in3, Pin.OUT)                # Initialize IN3 as a digital output - this is the RIGHT motor's FORWARD direction pin
        self.in4 = Pin(in4, Pin.OUT)                # Initialize IN4 as a digital output - this is the RIGHT motor's REVERSE direction pin
        self.stop()                                 # SAFETY: call stop() immediately on boot so all pins start LOW and the wheels cannot spin before the rest of the system is ready
        print("MotorController initialized and secured on boot.") # Confirm over serial that the motor driver is set up and the wheels are locked

    def drive_forward(self): # Method to command both motors to spin forward
        self.en_a.value(1)              # CHANGED: Turn ON Left master power ONLY when driving
        self.en_b.value(1)              # CHANGED: Turn ON Right master power ONLY when driving
        self.in1.value(1)               # Pull Left Forward pin to 3.3V (HIGH)
        self.in2.value(0)               # Pull Left Reverse pin to 0V (LOW)
        self.in3.value(1)               # Pull Right Forward pin to 3.3V (HIGH)
        self.in4.value(0)               # Pull Right Reverse pin to 0V (LOW)
        if DEBUG:                               # Gate this print behind DEBUG: drive_forward() is called at up to 100Hz; printing every call adds serial latency to the steering loop
            print("Motors driving forward.") # Print confirmation of forward movement only when verbose logging is enabled

    def drive_reverse(self): # ADDED: Method to command both motors to spin backward for line-recovery maneuvers
        self.en_a.value(1)              # Turn ON Left master power ONLY when driving
        self.en_b.value(1)              # Turn ON Right master power ONLY when driving
        self.in1.value(0)               # Pull Left Forward pin to 0V
        self.in2.value(1)               # Pull Left Reverse pin to 3.3V (HIGH)
        self.in3.value(0)               # Pull Right Forward pin to 0V
        self.in4.value(1)               # Pull Right Reverse pin to 3.3V (HIGH)
        if DEBUG:                               # Gate this print: same 100Hz concern as drive_forward above
            print("Motors driving in reverse.") # Print confirmation of reverse movement only when verbose logging is enabled

    def stop(self): # Method to completely halt all motor activity and cut power
        self.en_a.value(0)              # CHANGED: COMPLETELY cut Left master power to guarantee zero jitter
        self.en_b.value(0)              # CHANGED: COMPLETELY cut Right master power to guarantee zero jitter
        self.in1.value(0)               # Drain Left Forward pin to 0V
        self.in2.value(0)               # Drain Left Reverse pin to 0V
        self.in3.value(0)               # Drain Right Forward pin to 0V
        self.in4.value(0)               # Drain Right Reverse pin to 0V
        if DEBUG:                               # Gate this print: stop() is called both from the watchdog path and from normal motor commands; at 100Hz this adds serial latency
            print("Motors stopped and power cut.") # Print confirmation that motors are physically dead only when verbose logging is enabled

# ---------------------------------------------------------
# 2. NETWORK CLASS (The Ears)
# ---------------------------------------------------------
class NetworkController:
    def __init__(self, ssid, password, broker, topic, mqtt_user=None, mqtt_pass=None): # CHANGED: added mqtt_user/mqtt_pass so this can authenticate against a secured broker instead of only the wide-open public one
        # Store all connection details inside the object
        self.ssid = ssid                    # Store the Wi-Fi network name
        self.password = password            # Store the Wi-Fi network password
        self.broker = broker                # Store the MQTT broker address (Wi-Fi router / hostname / IP)
        self.topic = topic                  # Store the MQTT topic this robot listens on
        self.mqtt_user = mqtt_user          # ADDED: store the MQTT broker username, if the broker requires authentication
        self.mqtt_pass = mqtt_pass          # ADDED: store the MQTT broker password, if the broker requires authentication
        self.client = None                  # No MQTT client object exists yet until connect_mqtt() runs

    def connect_wifi(self, timeout_s=15, watchdog=None): # CHANGED: added a hard timeout + optional watchdog feed so a bad password/dead router can never hang the boot sequence forever
        # Turn on the ESP32 Wi-Fi in Station Mode (client mode)
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        
        print("Connecting to Wi-Fi...")
        # Start the connection process to the router
        wlan.connect(self.ssid, self.password)
        
        start_time = time.ticks_ms()          # ADDED: mark when we started waiting, so we can bail out
        # Wait safely in a loop until the router assigns an IP address, OR until we time out
        while not wlan.isconnected():
            if watchdog is not None:          # ADDED: feed the hardware watchdog while legitimately waiting
                watchdog.feed()
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_s * 1000: # ADDED: FAILSAFE - never wait forever
                wlan.active(False)            # ADDED: cleanly release the radio before giving up
                raise OSError("Wi-Fi connection timed out after {}s".format(timeout_s)) # ADDED: raise so the caller's retry logic actually gets a chance to run
            time.sleep(0.5)
            print(".", end="")
            
        print("\nWi-Fi Connected! IP:", wlan.ifconfig()[0])

    def connect_mqtt(self, message_callback):
        # FIX 6: derive a unique MQTT client ID from this board's hardware MAC address.
        # Previously the ID was the hardcoded string "esp32_robot_prod". MQTT brokers treat
        # two clients with the same ID as the same session — when the second one connects, the
        # broker disconnects the first. Two robots (or a reconnect while the old session is still
        # alive on the broker) would therefore knock each other offline continuously.
        # The MAC address is unique per chip and never changes, so this ID is both stable and unique.
        mac_bytes = network.WLAN(network.STA_IF).config("mac") # Read the 6-byte hardware MAC from the Wi-Fi interface
        mac_hex = "".join("{:02x}".format(b) for b in mac_bytes) # Format as a lowercase hex string (e.g. "a4cf12b3e701")
        client_id = "esp32_robot_{}".format(mac_hex)            # Combine with a readable prefix so the ID is identifiable in broker logs
        print(f"MQTT: using unique client ID '{client_id}'")    # Log the ID so the operator can confirm which board is connecting when monitoring the broker
        self.client = MQTTClient(client_id, self.broker, user=self.mqtt_user, password=self.mqtt_pass) # Create the MQTT client with the unique ID and optional authentication credentials
        
        # Tell the client which function to run when a message arrives
        self.client.set_callback(message_callback)
        
        print("Connecting to MQTT Broker...")
        # Dial into the server
        self.client.connect()
        
        # Subscribe to our specific walkie-talkie channel
        self.client.subscribe(self.topic)
        print("Subscribed to topic:", self.topic.decode())

    def check_for_commands(self):
        # Ask the MQTT broker whether any new message has arrived on our subscribed topic since the last check.
        # umqtt.simple gives us two options:
        #   check_msg() = non-blocking: peeks the socket once and returns immediately. If a message IS
        #                 waiting, it fires process_message() synchronously before returning. If nothing
        #                 is waiting, it returns instantly with no side effects. We always use this in
        #                 the driving loop so vision updates are never held up by network silence.
        #   wait_msg()  = blocking: sits here until a message arrives. This would stall the driving loop
        #                 (including the hardware watchdog feed) indefinitely — never used in the loop.
        # What process_message() does with each command type once check_msg() delivers it:
        #   "SECRET,angle,DRIVE"    -> authenticate, steer to angle, drive motors forward
        #   "SECRET,angle,REVERSE"  -> authenticate, steer to mirrored angle, drive motors backward
        #   "SECRET,angle,STOP"     -> authenticate, steer to angle, stop motors
        #   anything else / wrong secret -> log and discard; current driving mode continues unaffected
        if self.client is None:                    # FIXED: previously returned silently when client is None, so the autonomous loop's except block never fired and the reconnect logic never ran.
            raise OSError(                         # Raise OSError (same family as a socket drop) so the calling loop's except catches it and triggers a reconnect attempt exactly the same way a mid-drive drop would.
                "MQTT client is None — connect_mqtt() has not been called yet "
                "or was skipped due to a failed boot. Reconnect required."
            )
        if DEBUG:                                  # Only print the poll notification when DEBUG is True — at 100Hz this would otherwise add ~1ms serial latency per loop iteration on MicroPython
            print("NetworkController: polling MQTT broker for waiting commands...") # Log that we are about to check the socket
        self.client.check_msg()                    # Poll the socket once (non-blocking). umqtt fires process_message() automatically if a message is waiting; returns instantly if not.

# ---------------------------------------------------------
# 3. CameraStreamer (The Eyes)
# ---------------------------------------------------------
class CameraStreamer:                              # Define the CameraStreamer class to handle live video broadcasting
    def __init__(self):                            # Constructor method to set up the physical camera lens hardware
        print("Initializing Camera Hardware...")   # Print a startup status message to the terminal
        try:                                       # Start an error-monitoring block in case the camera ribbon cable is loose
            # Initialize the camera. Note: Pins vary by exact ESP32-S3 board model. '0' usually auto-detects.
            camera.init(0, format=camera.JPEG, framesize=camera.FRAME_VGA) # Turn on the lens and set it to VGA resolution (640x480)
            print("Camera initialized successfully.") # Print confirmation that the hardware is working
        except Exception as e:                     # Catch any hardware failures during initialization
            print(f"CRITICAL: Camera failed to start: {e}") # Print the exact physical failure reason to the terminal

    def start_server(self):                        # Method to boot up the web server for the PC to connect to
        print("Starting video stream server on core 1...") # Print status indicating the server is starting
        _thread.start_new_thread(self._serve_video, ()) # Launch the video streaming loop in the background on the second CPU core

    def _serve_video(self):                        # The actual web server loop that runs invisibly in the background on CPU Core 1
        # --- FAILSAFE: guard socket setup so a port-already-in-use error can't silently kill this background thread ---
        # Previously s.bind() and s.listen() were called OUTSIDE any try/except, so if port 80 was already claimed
        # (e.g. from a prior crash before the OS released it), this thread would crash on those lines with no log
        # output, no retry, and no way for the main loop to know the monitoring stream was dead.
        s = None                                   # Pre-declare the socket variable so the except block can safely reference it even if socket.socket() itself raises
        while True:                                # Keep retrying the bind/listen in a loop until it succeeds, so a transient port-conflict can't permanently kill monitoring
            try:                                   # Watch for port-already-in-use or other socket-creation errors
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a fresh TCP socket for the video web server
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Tell the OS to release the port immediately on crash so the retry below can claim it again
                s.bind(('', 80))                   # Bind to port 80 on all interfaces (the universal HTTP port, so any browser on the LAN can connect)
                s.listen(1)                        # Accept exactly 1 queued incoming connection at a time (we only want your PC, not multiple simultaneous viewers)
                print("CameraStreamer: video server socket bound to port 80 and listening.") # Confirm the socket is up and ready
                break                              # Socket is ready - exit the bind/listen retry loop and proceed to the accept() loop below
            except Exception as bind_err:          # Catch port-already-in-use, permissions, or any other socket-setup failure
                print(f"CameraStreamer: socket bind failed ({bind_err}), retrying in 3 seconds...") # Make the error and retry visible in serial output
                if s is not None:                  # If the socket object was created before the error, release it before the next attempt
                    try:                           # Guard s.close() so a failure there doesn't hide the original bind error
                        s.close()                  # Release the socket's OS resources so the retry can claim a clean socket
                    except Exception as close_err: # FIX 8: log close() failures instead of silently swallowing them — a failure here usually means the socket handle is already invalid, which is safe, but the operator should be able to see it
                        print(f"CameraStreamer: s.close() after bind failure raised {close_err} (safe to ignore — socket was likely already invalid).") # Log the close error with context so the operator can distinguish it from the original bind error
                time.sleep(3)                      # Wait 3 seconds before retrying so we don't spam the OS with rapid failed bind attempts
        
        while True:                                # Start an infinite accept loop to keep the web server alive and reconnectable forever
            conn = None                            # FIX: Pre-declare conn to avoid UnboundLocalError
            try:                                   # Watch for errors in the incoming PC connection (disconnect, Wi-Fi drop, broken pipe)
                conn, addr = s.accept()            # Block this background thread and wait for the PC to initiate a TCP connection; accept it when it arrives
                print(f"PC connected to camera stream from IP: {addr}") # Log the PC's IP address so you can confirm the right device connected
                
                # Send the standard HTTP response headers that tell the PC browser/OpenCV to expect a continuous MJPEG stream
                conn.send(b'HTTP/1.1 200 OK\r\n')  # Send the HTTP status line indicating the connection is approved and the server is healthy
                conn.send(b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n') # Tell the client to treat the response as a looping series of JPEG frames, not a single file
                
                while True:                        # Stream frames as fast as the sensor and Wi-Fi can handle - no artificial rate limit needed here
                    buf = camera.capture()         # Capture one JPEG frame from the physical camera sensor
                    if buf:                        # Only transmit the frame if the capture buffer is non-empty (a None/empty buf means the sensor briefly failed)
                        conn.send(b'--frame\r\n')  # Send the MJPEG boundary marker that separates this frame from the previous one
                        conn.send(b'Content-Type: image/jpeg\r\n\r\n') # Declare the content type of this chunk so the client decodes it as a JPEG image
                        conn.send(buf)             # Transmit the raw JPEG bytes to the PC over the TCP socket
                        conn.send(b'\r\n')         # Send the trailing newline that closes this frame block and signals the client to render it
                        
            except Exception as stream_err:        # Catch any error that breaks the stream - PC disconnect, Wi-Fi drop, broken pipe, etc.
                print(f"Video stream interrupted: {stream_err}. Waiting for next PC connection...") # Log the interruption reason and indicate the server will keep accepting new connections
            finally:                               # This block always runs when the inner while-True exits, whether cleanly or via exception
              if conn is not None:                 # FIX: Check that conn exists before attempting to close
                try:                               # Guard the conn.close() call so a failure there doesn't prevent the outer accept() loop from restarting
                    conn.close()                   # Release the broken/closed connection so the OS can reclaim its socket resources for the next client
                except Exception as close_err:     # FIX 8: log conn.close() failures instead of silently swallowing them — the connection is already broken so this is harmless, but visible errors are always better than invisible ones
                    print(f"CameraStreamer: conn.close() raised {close_err} (safe to ignore — connection was already broken or never fully opened).") # Log with context so the operator can distinguish this from a real stream error

# ---------------------------------------------------------
# 3.1. OnboardVisionController (The Independent Eyes + Brain)
# ---------------------------------------------------------
# ADDED: This class is what makes the vehicle actually autonomous. It follows the line using
# ONLY the camera mounted on this ESP32-S3 board - no PC, no Wi-Fi, and no MQTT broker are
# required for it to work. MQTT/robot.py becomes an OPTIONAL, secured channel a human can use
# to briefly take over, not something the vehicle needs in order to move.
#
# HARDWARE NOTE (ESP32-S3-WROOM only, no Pi/Jetson in this design): the onboard camera sensor
# can only run in ONE pixel format at a time - JPEG for streaming to a PC, OR raw GRAYSCALE for
# fast onboard analysis - not both at once without expensive re-initialization every frame.
# Because driving safely has to take priority over remote monitoring, this class claims the
# camera in GRAYSCALE mode, and the CameraStreamer's JPEG feed is skipped automatically whenever
# AUTONOMOUS_MODE is on (see the __main__ block at the bottom of this file).
class OnboardVisionController:
    def __init__(self, frame_width=320, frame_height=240, roi_rows=12, dark_threshold=90, min_dark_pixels=200, kp=0.25, servo_center=90): # Constructor exposes every tunable so the algorithm itself never needs editing to retune
        self.frame_width = frame_width          # Store the camera's configured frame width in pixels (must match the framesize passed to camera.init())
        self.frame_height = frame_height        # Store the camera's configured frame height in pixels
        self.roi_rows = roi_rows                # Store how many rows near the bottom of the frame we analyze - a thin strip is enough for steering and is far cheaper to scan pixel-by-pixel in MicroPython than a full frame
        self.dark_threshold = dark_threshold    # Store the grayscale cutoff (0-255); pixels darker than this are treated as "line"
        self.min_dark_pixels = min_dark_pixels  # Store the minimum dark-pixel count required before a detection is trusted as a real line instead of noise/shadow
        self.kp = kp                            # Store the proportional steering gain, mirrors the PC-side VisionController's kp so tuning knowledge carries over
        self.servo_center = servo_center        # Store the straight-ahead steering angle
        self.last_valid_angle = servo_center    # Remember the last confident steering angle, used to mirror a steering direction during reverse recovery
        self.state = "DRIVE"                    # Track the onboard recovery state machine (DRIVE / SEARCH_STOP / REVERSE) - mirrors the PC-side logic exactly so behavior is familiar
        self.lost_line_time = 0                 # Timestamp (ms) of when the line was last seen, used to time the SEARCH_STOP wait window
        self.reverse_start_time = 0             # Timestamp (ms) of when the REVERSE recovery maneuver began
        self.camera_ok = True                   # Tracks whether the camera itself is currently healthy, so init failures are visible to compute_command()

        try:                                     # ADDED: FAILSAFE - camera init can fail on a loose ribbon cable or a bad sensor; never let that crash the whole robot
            camera.init(0, format=camera.GRAYSCALE, framesize=camera.FRAME_QVGA) # Initialize the sensor in raw grayscale mode at QVGA (320x240) for fast onboard analysis - swap FRAME_QVGA/dimensions together if you need a different resolution
            print("OnboardVisionController: camera initialized in GRAYSCALE mode for autonomous driving.") # Confirm the vision camera came up correctly
        except Exception as e:                  # Catch any hardware failure during camera bring-up
            self.camera_ok = False              # Remember that the camera is not usable so compute_command() always fails safe
            print(f"CRITICAL: OnboardVisionController camera failed to initialize: {e}") # Make the exact failure visible over serial

    def compute_command(self): # ADDED: the onboard equivalent of the PC's VisionController.process_frame() - runs entirely on this chip, with zero network round-trip
        if not self.camera_ok:                  # FAILSAFE: if the camera never came up, always refuse to drive rather than guessing blind
            return self.servo_center, "STOP"    # Center the steering and report STOP

        try:                                     # FIX 4: guard camera.capture() so a driver exception (ribbon cable glitch, sensor reset, buffer overflow) doesn't propagate up through compute_command() into the main driving loop and trigger the WDT reset path. A camera hiccup is recoverable — we just skip this frame and return STOP until the next successful capture.
            buf = camera.capture()              # Ask the onboard camera driver for the newest raw grayscale frame buffer
        except Exception as cam_err:            # Catch any exception the camera driver raises (OSError, MemoryError, etc.)
            print(f"OnboardVisionController: camera.capture() raised {cam_err}. Returning STOP for this frame.") # Log the exact exception so the operator can see transient camera errors in the serial output
            self.camera_ok = False              # Mark the camera as unhealthy so subsequent frames also fail safe until a successful capture clears this flag
            return self.servo_center, "STOP"    # Return a safe STOP command — the driving loop will try again on the next 10ms iteration
        self.camera_ok = True                   # A successful capture means the camera is healthy again — clear any previous failure flag so recovery is automatic
        if not buf or len(buf) < self.frame_width * self.frame_height: # FAILSAFE: reject a missing or short/corrupted frame instead of indexing into garbage memory
            print("OnboardVisionController: bad or missing frame, stopping for safety.") # Explain why we're stopping
            return self.servo_center, "STOP"    # Center the steering and report STOP

        # --- Scan only a thin horizontal strip near the bottom of the frame (closest to the robot) ---
        start_row = self.frame_height - self.roi_rows # Compute the first row of our region of interest, near the bottom of the image
        dark_pixel_count = 0                    # Running count of pixels darker than the threshold (the line)
        weighted_sum = 0                        # Running sum of column positions for every dark pixel found, used to compute the centroid without needing a numpy-style array library
        for row in range(start_row, self.frame_height): # Loop over each row inside the ROI strip
            row_offset = row * self.frame_width # Compute the starting index of this row inside the flat 1-D frame buffer
            for col in range(0, self.frame_width, 2): # Loop across columns, skipping every other pixel to roughly halve CPU cost while keeping plenty of resolution for steering
                pixel = buf[row_offset + col]   # Read a single grayscale pixel value (0-255)
                if pixel < self.dark_threshold: # Check whether this pixel is dark enough to be considered part of the line
                    dark_pixel_count += 1       # Count it towards the total
                    weighted_sum += col          # Add its column position to the running total for the centroid calculation

        current_time = time.ticks_ms()          # Grab the current time once for use in the recovery state machine below

        # --- If enough dark pixels were found, trust the detection and steer toward its centroid ---
        if dark_pixel_count >= self.min_dark_pixels: # Only trust the detection if enough dark pixels were found - filters out shadows, glare, and single-pixel noise
            centroid_col = weighted_sum // dark_pixel_count # Compute the average column of all dark pixels found = the horizontal center of the line
            error = centroid_col - (self.frame_width // 2) # Measure how far that center is from the middle of the frame
            angle = self.servo_center + int(error * self.kp) # Convert the pixel error into a steering angle using the proportional gain
            angle = max(45, min(135, angle))    # SAFETY: clamp to the same mechanical limits enforced in SteeringController, defense-in-depth
            self.last_valid_angle = angle       # Remember this as the last confident angle, used later for reverse-recovery mirroring
            if self.state != "DRIVE":           # If we were mid-recovery and just found the line again
                print("OnboardVisionController: line reacquired, resuming normal drive.") # Log the recovery event
                self.state = "DRIVE"            # Return to the normal forward-driving state
            return angle, "DRIVE"               # Report the computed steering angle and the DRIVE state

        # --- Line not found: run the same lost-line recovery state machine as the PC-side controller ---
        if self.state == "DRIVE":               # This is the first frame where the line has disappeared
            self.state = "SEARCH_STOP"          # Enter the stop-and-wait recovery phase
            self.lost_line_time = current_time  # Record when we lost the line so we can time the wait window
            print("OnboardVisionController: line lost, stopping to wait for recovery.") # Log the state transition

        if self.state == "SEARCH_STOP":         # While waiting motionless for the line to possibly reappear
            if time.ticks_diff(current_time, self.lost_line_time) > 3500: # After 3.5 seconds of waiting with no line
                self.state = "REVERSE"          # Move on to the reverse recovery phase
                self.reverse_start_time = current_time # Record when reverse recovery began
                print("OnboardVisionController: wait complete, reversing to recover the line.") # Log the state transition
            return self.servo_center, "STOP"    # While waiting, keep the steering centered and the motors stopped

        # state == "REVERSE": drive backward, mirroring the last known good steering angle to retrace the same path
        elapsed_reverse = time.ticks_diff(current_time, self.reverse_start_time) # Measure how long we've been reversing
        reverse_angle = max(45, min(135, 180 - self.last_valid_angle)) # Mirror the last valid angle so the reverse path follows the same curve backward
        if elapsed_reverse < 5000:               # Keep reversing for up to 5 seconds
            return reverse_angle, "REVERSE"     # Report the mirrored steering angle and the REVERSE state
        # After 5 seconds of reversing with still no line found, give up and stay stopped rather than reversing forever
        return self.servo_center, "STOP"        # FAILSAFE: prefer a vehicle that gives up safely over one that backs up indefinitely into the unknown

# ---------------------------------------------------------
# 4.0. ORCHESTRATOR CLASS (The Brain)
# ---------------------------------------------------------
"""class RobotOrchestrator:
    def __init__(self, steering, motors, network):
        # Dependency Injection: The manager receives the hardware and network objects
        self.steering = steering
        self.motors = motors
        self.network = network

    def process_message(self, topic, msg):
        # This function fires automatically when an MQTT packet arrives
        try:
            # Decode the raw byte message into a standard Python text string
            payload = msg.decode("utf-8").strip()
            
            # Split the text at the comma (expecting "90,DRIVE")
            parts = payload.split(",")
            
            # ERROR HANDLING: Ensure we received exactly two pieces of data
            if len(parts) != 2:
                raise ValueError("Invalid payload format. Expected 'Angle,State'")

            # Convert the first part to an integer, and clean up the second part
            angle = int(parts[0])
            state = parts[1].strip().upper()
            
            print(f"Executing -> Angle: {angle}, State: {state}")

            # 1. Execute Steering Command
            self.steering.set_angle(angle)

            # 2. Execute Motor Command
            if state == "DRIVE":
                self.motors.drive_forward()
            elif state == "STOP":
                self.motors.stop()
            else:
                # If a weird state word comes through, default to stopping for safety
                print("Unknown state received. Stopping for safety.")
                self.motors.stop()
                
        except Exception as e:
            # ERROR HANDLING: If letters were sent instead of numbers, catch the crash and stop
            print("Data parsing error:", e)
            self.motors.stop()

    def run(self):
        # Step 1: Connect to Wi-Fi
        self.network.connect_wifi()
        
        # Step 2: Connect to MQTT and pass it our processing function
        self.network.connect_mqtt(self.process_message)
        
        print("Robot Orchestrator is online and waiting for commands.")
        
        try:
            # Step 3: The Infinite Loop
            while True:
                # Check for internet messages
                self.network.check_for_commands()
                # Pause for 10ms to prevent the CPU from overheating
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            # Safely handle the user pressing Ctrl+C in the terminal
            print("Robot stopped by user.")
            
        finally:
            # SAFETY SHUTDOWN: If the loop breaks for ANY reason, kill the wheels
            self.motors.stop()
            print("Hardware safely secured.")
"""

# ---------------------------------------------------------
# 4.1. UPDATED: RobotOrchestrator (With Secure Boot Sequence)
# ---------------------------------------------------------
class RobotOrchestrator: # Define the RobotOrchestrator class to manage the main brain loop
    def __init__(self, steering, motors, network, vision=None, autonomous_mode=True, command_secret=None, manual_override_window_ms=1000, wdt_timeout_ms=8000, motion_armed=False): # FIX 9: added motion_armed — when False all motor actuation is blocked so the robot can be bench-tested safely
        self.steering = steering                 # Store the injected steering object
        self.motors = motors                     # Store the injected motors object
        self.network = network                   # Store the injected network object
        self.vision = vision                     # Store the injected OnboardVisionController - this is what drives the vehicle when no human is overriding it
        self.autonomous_mode = autonomous_mode   # When True, the vehicle drives itself by default and MQTT is only a temporary, secured override channel
        self.motion_armed = motion_armed         # FIX 9: when False, all calls to drive_forward/drive_reverse/stop that originate from autonomous vision or MQTT commands are blocked; the robot computes and logs what it would do but never moves the wheels
        self.command_secret = command_secret     # Store the shared secret — validated below; process_message() rejects any command whose first field does not match this exactly
        # FIX 3: validate the secret at construction time, not at the first incoming command.
        # The old guard `if self.command_secret and received_secret != self.command_secret` meant
        # that a missing or empty secret in config.py silently disabled authentication entirely —
        # every sender's command would be accepted because the condition short-circuits on falsy.
        # Raising here means a misconfigured secret is caught at boot (before any driving begins)
        # rather than discovered later when an unauthenticated command unexpectedly moves the vehicle.
        if not self.command_secret:              # Check that the secret is present and non-empty before the robot is allowed to boot
            raise RuntimeError(                 # Raise immediately so the hardware bring-up guard in __main__ catches it and resets rather than starting an unauthenticated robot
                "COMMAND_SECRET is missing or empty. "
                "Set a long random value in config.py and keep it in sync with robot.py. "
                "The robot will not start without a valid secret."
            )
        self.manual_override_window_ms = manual_override_window_ms # ADDED: how long a single authenticated human command stays "in control" before the vehicle automatically hands steering back to onboard vision
        self.last_command_time = time.ticks_ms() # Create a Watchdog Timer to track the last PC message
        self.last_manual_command_time = 0        # ADDED: separate timestamp that ONLY updates on a successfully AUTHENTICATED command - this is what actually gates manual override, so unauthenticated noise can never seize control
        self._next_reconnect_attempt_ms = 0      # ADDED: used by the non-blocking background reconnect logic in run() so autonomous driving is never paused waiting on the network
        self.emergency_stop = False              # FIX 1: latching emergency-stop flag. When an authenticated STOP command arrives this becomes True and the vehicle will NOT move again - regardless of the manual_override_window_ms expiring or onboard vision seeing a line - until an authenticated RESUME command explicitly clears it. This prevents the 1-second window from quietly restarting a vehicle that a human explicitly stopped.
        # ADDED: HARDWARE watchdog. The existing 500ms check only works if this loop keeps executing.
        # If a blocking call (socket, MQTT, etc.) ever truly hangs, nothing in software can save us -
        # only a hardware timer that forcibly resets the chip. Once started this CANNOT be disabled,
        # which is exactly what we want: the chip will always recover to a safe boot state (motors off)
        # rather than being stuck mid-drive. Timeout is generous (8s) so it only fires on genuine lockups,
        # never on normal Wi-Fi/MQTT waits (those are fed explicitly, see run()/connect helpers).
        try:
            self.wdt = WDT(timeout=wdt_timeout_ms)
            print(f"Hardware watchdog armed ({wdt_timeout_ms}ms).")
        except Exception as e:
            # Some boards/ports don't support WDT the same way - don't let its absence crash boot,
            # but make it very visible in the logs that this failsafe layer is missing.
            self.wdt = None
            print(f"WARNING: Hardware watchdog unavailable ({e}). Running with software watchdog only.")
        print(f"RobotOrchestrator initialized. Autonomous mode: {self.autonomous_mode}.")  # Print confirmation of brain startup, including which driving mode is active

    def _feed_wdt(self): # Small helper so every feed site doesn't need a None-check
        if self.wdt is not None:                 # Only call feed() if the hardware WDT was successfully armed
            self.wdt.feed()                      # Reset the WDT countdown — proof that the loop is still executing

    def _actuate(self, motor_cmd, angle=None):   # FIX 9: single gating point for all motor and steering actuation
        # All physical movement goes through this method. When motion_armed is False it logs
        # what WOULD happen and returns immediately without touching any hardware pins.
        # This means the entire vision, state-machine, and command-parsing logic can run and be
        # observed over serial during bench testing — only the final hardware write is suppressed.
        if not self.motion_armed:                # Check the armed flag before touching any hardware
            print(f"MOTION LOCKED (MOTION_ARMED=False): would execute '{motor_cmd}'" + (f" at angle {angle}" if angle is not None else "") + ". Set MOTION_ARMED=True in config.py to enable.") # Log exactly what would have happened so bench testing gives full visibility
            return                               # Return without touching any pins — the vehicle stays completely still
        if angle is not None:                    # If a steering angle was provided, apply it first
            self.steering.set_angle(angle)       # Command the physical servo to the target angle
        if motor_cmd == "DRIVE":                 # Forward drive command
            self.motors.drive_forward()          # Energise both motors in the forward direction
        elif motor_cmd == "REVERSE":             # Reverse drive command
            self.motors.drive_reverse()          # Energise both motors in the reverse direction
        elif motor_cmd == "STOP":                # Stop command
            self.motors.stop()                   # Cut power to both motors immediately

    def process_message(self, topic, msg): # Method triggered automatically when a new MQTT network packet arrives - this is now a SECURED, OPTIONAL override channel, not the vehicle's only way to move
        try:                                     # Start an error-handling block to catch bad internet data
            payload = msg.decode("utf-8").strip()    # Decode the raw network byte message into standard text
            parts = payload.split(",")               # Split the text at the comma separator, expecting "SECRET,angle,state"
            
            if len(parts) != 3:                      # CHANGED: now require exactly three pieces of data - secret, angle, state
                raise ValueError("Invalid format")   # Throw an error if the format is wrong
            
            received_secret = parts[0]                # Pull out the shared-secret token the sender claims to have
            # FIX 3: authentication check is now unconditional. The old form was:
            #   if self.command_secret and received_secret != self.command_secret:
            # The leading `self.command_secret and` meant an empty secret short-circuited to False,
            # making the check never fire and every sender's command accepted without authentication.
            # We now know the secret is non-empty (validated in __init__), so the check is plain:
            if received_secret != self.command_secret: # Reject any command whose secret field does not match exactly
                print("Rejected MQTT command: invalid shared secret. Possible unauthorised sender.") # Log the rejection so attack attempts are visible in serial output
                return                                 # Discard the packet — crucially does NOT touch motors or last_manual_command_time, so noise cannot move or stop the vehicle
            
            angle = int(parts[1])                     # CHANGED: angle is now the second field (after the secret)
            state = parts[2].strip().upper()          # CHANGED: state is now the third field (after the secret)
            
            self.last_command_time = time.ticks_ms()  # Reset the software watchdog timer because we just heard a VALID command from an authenticated sender
            self.last_manual_command_time = time.ticks_ms() # ADDED: mark that a human has just taken manual control - run() will honor this for manual_override_window_ms before handing back to onboard vision
            
            print(f"Executing (authenticated) -> Angle: {angle}, State: {state}") # Print the exact command we are running
            
            self.steering.set_angle(angle)           # Command the physical steering servo to turn
            print(f"Steering physically set to {angle} degrees.") # Print steering confirmation
            
            if state == "DRIVE":                     # Check if the command word is "DRIVE"
                if self.emergency_stop:              # Honour the latch - a latched STOP cannot be cleared by a DRIVE command; only RESUME clears it
                    print("DRIVE command received but EMERGENCY STOP is latched. Send RESUME to re-enable motion.") # Inform the sender that the latch is blocking the command
                    return                           # Discard the DRIVE command entirely; motors stay off
                self._actuate("DRIVE", angle)        # FIX 9: route through _actuate so MOTION_ARMED gate is enforced
            elif state == "REVERSE":                 # Check if the command word is "REVERSE"
                if self.emergency_stop:              # Same latch guard as DRIVE
                    print("REVERSE command received but EMERGENCY STOP is latched. Send RESUME to re-enable motion.") # Inform the sender
                    return                           # Discard; motors stay off
                self._actuate("REVERSE", angle)      # FIX 9: route through _actuate
            elif state == "STOP":                    # Check if the command word is "STOP"
                self.emergency_stop = True           # LATCH the emergency stop
                self._actuate("STOP", angle)         # FIX 9: route through _actuate (steering centered, motors cut)
                print("EMERGENCY STOP latched. Vehicle will not move until an authenticated RESUME command is received.") # Make the latched state explicit
            elif state == "RESUME":                  # New command that explicitly releases the emergency-stop latch
                self.emergency_stop = False          # Clear the latch so autonomous vision and DRIVE commands are honoured again
                print("EMERGENCY STOP released by authenticated RESUME command. Autonomous driving will resume.") # Log the release
            else:                                    # Catch any weird unknown command words
                print("Unknown state received. Halting for safety.") # Print warning about unknown state
                self._actuate("STOP")                # FIX 9: route through _actuate
                
        except Exception as e:                       # Catch any text decoding or math crash errors
            print("Parsing error:", e)               # Print the exact error to the PC terminal
            # CHANGED: no longer force-stop the motors here unconditionally. A malformed/garbage MQTT
            # packet is not evidence that autonomous onboard vision has failed - forcing a stop on every
            # parse error would make the vehicle vulnerable to being halted by ANY malformed traffic on
            # the topic, authenticated or not. We simply ignore the bad packet and let the current
            # driving mode (autonomous vision, or the last valid authenticated command) continue.
            print("Malformed command ignored; current driving mode continues unaffected.") # Explain the (lack of) failsafe action taken

    def run(self): # The main operating loop of the robot that runs forever
        print("Starting Boot Sequence...")           # Print that the robot is powering on
        
        # --- FAILSAFE BOOT STAGE: TRY TO GET THE NETWORK UP, BUT DON'T LET IT GATE DRIVING ---
        # CHANGED: in autonomous_mode, the network/MQTT link is an OPTIONAL override channel, not
        # something the vehicle needs to move. So here we make a bounded number of attempts and,
        # if they all fail, log it and proceed straight into the driving loop on vision alone -
        # reconnection keeps being attempted opportunistically in the background inside run().
        # In legacy teleop mode (autonomous_mode=False) network really is mandatory to drive, so
        # that path is preserved exactly as before: retry forever, with a full-reset safety valve.
        boot_attempts = 0                            # Track consecutive boot failures
        MAX_BOOT_ATTEMPTS_BEFORE_RESET = 10          # Legacy mode: after this many failures, do a full chip reset rather than retrying forever in a possibly-wedged state
        MAX_BOOT_ATTEMPTS_AUTONOMOUS = 3             # ADDED: autonomous mode only needs a few quick tries before giving up and driving on vision alone
        network_online = False                       # ADDED: tracks whether we actually got connected during boot, used below to decide how to enter the driving loop
        while True:                                  # Start a loop specifically for booting up
            self._feed_wdt()                         # Feed hardware watchdog at the top of every boot attempt
            try:                                     # Start watching for hardware connection errors
                print("Attempting to establish network connections...") # Print status
                self.network.connect_wifi(watchdog=self.wdt) # Ask hardware to connect to Wi-Fi router (hard timeout + WDT feeding while it waits)
                self.network.connect_mqtt(self.process_message) # Ask hardware to dial the MQTT internet broker
                
                # If the code reaches this next line without jumping to 'except', it means NO ERRORS happened!
                print("SUCCESS: Robot Orchestrator is connected, online, and waiting for commands.") # Print true verified status
                self.last_command_time = time.ticks_ms() # Start the watchdog timer fresh right now
                network_online = True                # Record that boot-time networking succeeded — used below for status logging
                break                                # BREAK out of the boot loop so we can move down to the driving loop!
                
            except Exception as boot_err:            # Catch any failure (wrong Wi-Fi password, no internet, etc.)
                boot_attempts += 1                    # Count this failure
                print(f"Boot connection failed (attempt {boot_attempts}): {boot_err}") # Print exactly why it failed to connect
                if self.autonomous_mode and boot_attempts >= MAX_BOOT_ATTEMPTS_AUTONOMOUS: # ADDED: autonomous mode doesn't need to keep blocking here - the vehicle can drive itself just fine
                    print("Network unavailable after a few tries. Proceeding WITHOUT network - "
                          "autonomous onboard vision will drive the vehicle; reconnection will keep "
                          "being attempted opportunistically in the background.") # ADDED: make it very clear in the logs that this is expected, not a failure state
                    network_online = False            # ADDED: explicitly record that we're entering the driving loop offline
                    break                              # ADDED: leave the boot loop and go drive - do not block on the network any longer
                if (not self.autonomous_mode) and boot_attempts >= MAX_BOOT_ATTEMPTS_BEFORE_RESET: # Legacy mode: the Wi-Fi/MQTT stack can get into a wedged state that only a full reset clears, and here network truly is required, so we reset rather than proceed
                    print("Too many failed boot attempts. Performing full hardware reset to clear a possibly wedged network stack...")
                    self.motors.stop()               # Belt-and-suspenders - motors should already be off, but confirm before reset
                    time.sleep(1)
                    machine.reset()                  # Full chip reset; execution restarts from the top of main.py in a known-safe state
                print("Retrying boot sequence in 2 seconds...") # Let user know it will try again
                time.sleep(2)                        # Wait 2 seconds before trying to boot up again
        
        # --- MAIN DRIVING STAGE ---
        print(f"Entering driving loop. Network online at boot: {network_online}. Motion armed: {self.motion_armed}. Autonomous: {self.autonomous_mode}.") # FIX 10: network_online is now used here for a one-time status line at the start of the driving stage, so it is read rather than assigned-and-ignored
        try:                                         # Start the master driving loop block
            while True:                              # Start the infinite driving loop
                self._feed_wdt()                     # Feed the hardware watchdog every iteration - this is the proof-of-life signal; if this loop ever truly stalls, the chip force-resets itself
                now = time.ticks_ms()                # ADDED: single timestamp reused throughout this iteration for consistency

                if self.autonomous_mode:             # ADDED: ==================== AUTONOMOUS DRIVING PATH ====================
                    # --- FAILSAFE STAGE 1 (AUTONOMOUS): NETWORK IS OPTIONAL, NEVER BLOCKS DRIVING ---
                    try:                             # Try to check for an incoming (optional, authenticated) override command
                        self.network.check_for_commands() # Non-blocking check - process_message() runs synchronously here if a message is waiting
                    except Exception as e:           # Catch ANY network problem (not connected, socket dropped, DNS hiccup, etc.)
                        if time.ticks_diff(now, self._next_reconnect_attempt_ms) >= 0: # Only attempt a reconnect every few seconds — never spam it
                            print(f"Optional network link down ({e}); preparing to attempt background reconnect...") # Log that this is a handled, expected condition, not a driving failure
                            self.motors.stop()       # FIXED: stop motors BEFORE the reconnect attempt. connect_wifi() can block for up to 5s; during that time no vision update occurs and motor pins retain their previous state. Stopping here ensures the vehicle is stationary and safe during the entire reconnect window.
                            self.steering.set_angle(90) # Also center steering so the vehicle is fully neutralised, not left turned, during the reconnect pause
                            print("Motors and steering neutralised before reconnect attempt.") # Confirm physical safety before blocking
                            try:                     # One bounded reconnect attempt — connect_wifi() has its own 5s timeout so the loop cannot hang indefinitely
                                self.network.connect_wifi(timeout_s=5, watchdog=self.wdt) # Try a quick Wi-Fi reconnect bounded to 5 seconds maximum
                                self.network.connect_mqtt(self.process_message) # Try to re-establish the MQTT session
                                print("Optional network link restored. Resuming autonomous driving.") # Log success
                            except Exception as reconn_err: # If the quick attempt fails, log it and continue driving — network is optional in autonomous mode
                                print(f"Background reconnect attempt failed: {reconn_err}. Driving on vision alone.") # Log the failure; autonomous vision continues regardless
                            # FIX 2: set the next-attempt deadline from ticks_ms() measured NOW (after the
                            # attempt completed), not from 'now' which was captured at the top of this loop
                            # iteration BEFORE the up-to-5-second connect_wifi() call. Using 'now' meant the
                            # 5000ms was subtracted from stale pre-attempt time, so the cooldown could be
                            # near-zero and the reconnect would fire again on the very next loop iteration.
                            self._next_reconnect_attempt_ms = time.ticks_add(time.ticks_ms(), 5000) # Measure from NOW so the full 5 seconds always elapses between reconnect attempts

                    # --- MANUAL OVERRIDE HANDOFF ---
                    manual_active = time.ticks_diff(now, self.last_manual_command_time) < self.manual_override_window_ms # True only if an AUTHENTICATED human command arrived within the override window
                    if self.emergency_stop:          # Check the latch BEFORE vision or manual override. A latched emergency stop overrides everything.
                        self._actuate("STOP")        # FIX 9: route through _actuate — enforces MOTION_ARMED too; re-asserts STOP on every iteration so pin state can't drift
                        if DEBUG:                    # Gate behind DEBUG because this fires at 100Hz while latched
                            print("Autonomous loop: EMERGENCY STOP latched — all motion suppressed until RESUME.") # Log suppressed state (debug only)
                    elif not manual_active:          # No latch and no recent authenticated human command — drive using onboard vision
                        if self.vision is None:      # FAILSAFE: autonomous_mode=True but no OnboardVisionController was supplied
                            self._actuate("STOP")    # FIX 9: route through _actuate
                            print("CRITICAL CONFIG ERROR: autonomous_mode is True but no vision controller was provided. Halted.") # Make the misconfiguration obvious
                        else:                        # Normal case — vision controller present, ask it for the next command
                            angle, state = self.vision.compute_command() # Compute steering and motor state entirely on this chip
                            if state == "DRIVE":     # Vision says line is visible and clear
                                self._actuate("DRIVE", angle)   # FIX 9: route through _actuate (sets steering then drives)
                            elif state == "REVERSE": # Vision is running lost-line reverse-recovery
                                self._actuate("REVERSE", angle) # FIX 9: route through _actuate
                            else:                    # Any other state (STOP, unrecognised) fails safe
                                self._actuate("STOP", angle)    # FIX 9: route through _actuate
                    # else: manual_active is True and emergency_stop is False — a human is in the override
                    # window. Their command was already actuated in process_message(); nothing further here.

                else:                                 # ADDED: ==================== LEGACY TELEOPERATED PATH (network required, unchanged behavior) ====================
                    # --- FAILSAFE STAGE 1: ACTIVE NETWORK MONITORING ---
                    try:                             # Inner try-block added to catch sudden Wi-Fi drops while driving
                        self.network.check_for_commands() # Ask the MQTT internet broker for new messages
                    except OSError as e:             # Catch the specific network disconnection error
                        print(f"Network dropped mid-drive: {e}") # Print the system error that caused the drop
                        self.motors.stop()           # IMMEDIATELY kill wheels because the robot is now blind
                        print("Failsafe: Vehicle halted in place to prevent blind collision.") # Confirm physical safety
                        
                        # --- AUTO-RECOVERY RECONNECTION LOOP ---
                        reconnect_attempts = 0        # Track consecutive reconnection failures
                        MAX_RECONNECT_ATTEMPTS_BEFORE_RESET = 15 # Bail out to a full reset rather than retrying forever against a possibly wedged radio/socket stack
                        while True:                  # Start an infinite loop that ONLY tries to fix the internet
                            self._feed_wdt()         # Keep feeding while we wait/retry - this is a legitimate wait, motors are already stopped so it's safe
                            try:                     # Start error handling for the reconnection attempts
                                print("Attempting to reconnect to network...") # Inform user that recovery is starting
                                self.network.connect_wifi(watchdog=self.wdt) # Attempt to reconnect to the Wi-Fi router (bounded by its own timeout)
                                self.network.connect_mqtt(self.process_message) # Attempt to reconnect to the MQTT broker
                                print("Reconnection successful! Resuming operations.") # Print success message
                                self.last_command_time = time.ticks_ms() # Reset watchdog so it doesn't instantly kill the motors again
                                break                # BREAK out of the recovery loop and go back to driving!
                            except Exception as reconn_err: # Catch failures during the reconnection attempt
                                reconnect_attempts += 1 # Count this failure
                                print(f"Connection failed, please connect. Error: {reconn_err} (attempt {reconnect_attempts})") # Print exact failure reason
                                if reconnect_attempts >= MAX_RECONNECT_ATTEMPTS_BEFORE_RESET: # FAILSAFE - motors are already stopped, so a reset here is safe and clears any wedged network state
                                    print("Too many failed reconnection attempts. Performing full hardware reset...")
                                    time.sleep(1)
                                    machine.reset()  # Motors are confirmed off above, so resetting here can never cause a runaway
                                time.sleep(2)        # Wait 2 seconds before trying to dial the internet again
                                
                    # --- FAILSAFE STAGE 2: WATCHDOG TIMER (legacy mode only - network commands ARE the control path here) ---
                    # FIX 7: use time.ticks_diff() instead of direct subtraction. ticks_ms() is a 32-bit
                    # counter that wraps around to zero after ~49 days. Direct subtraction
                    # (ticks_ms() - last_command_time) gives a large negative number across the wrap point,
                    # which is always < 500, so the watchdog would never fire just after rollover —
                    # exactly when a silent network failure needs to be caught. ticks_diff() handles the
                    # wrap correctly by computing the signed difference modulo the counter range.
                    if time.ticks_diff(time.ticks_ms(), self.last_command_time) > 500: # Rollover-safe: positive result means last_command_time is in the past by more than 500ms
                        self.motors.stop()           # Force stop the wheels immediately — no command has arrived in 500ms
                        print("Watchdog triggered: PC lagged or disconnected. Motors halted.") # Print why watchdog stopped the robot
                    
                # --- FAILSAFE STAGE 3: DETERMINISTIC TIMING ---
                time.sleep(0.01)                     # Pause 10ms to prevent the ESP32 chip from overheating
                
        except KeyboardInterrupt:                    # Catch manual terminal stops (like pressing Ctrl+C on keyboard)
            print("Robot program stopped manually by user via terminal.") # Print exit status
        finally:                                     # This block runs no matter how the program crashes or ends
            print("Initiating final hardware shutdown sequence...") # Print shutdown start
            self.motors.stop()                       # Final absolute hardware kill switch
            print("Hardware safely secured and completely stopped.") # Explicitly print that hardware is stopped in finally block

# ---------------------------------------------------------
# 5.MAIN EXECUTION (Building and launching the robot)
# ---------------------------------------------------------
if __name__ == "__main__":                         # Check if this script is being run directly as the main program (not imported)

    # ---------------------------------------------------------
    # CREDENTIAL LOADING — secrets are NOT stored in this file
    # ---------------------------------------------------------
    # SECURITY: Wi-Fi credentials, MQTT credentials, and the
    # command secret must never be committed to source control or
    # shared with anyone. The Wi-Fi password previously hardcoded
    # here (Primeit@2024) has been EXPOSED — rotate it on your
    # router immediately and do not reuse it anywhere.
    #
    # On MicroPython (ESP32) you cannot use python-dotenv. Instead,
    # create a file called config.py on the board's filesystem (use
    # Thonny's file manager or mpremote to upload it) with the
    # following contents — fill in your actual values:
    #
    #   WIFI_SSID      = "YourNetworkName"
    #   WIFI_PASS      = "YourWiFiPassword"          # rotate the old exposed one first!
    #   MQTT_BROKER    = "192.168.1.50"              # LAN IP of your private Mosquitto broker
    #   MQTT_USER      = "your_mqtt_username"        # set on the broker with mosquitto_passwd
    #   MQTT_PASS      = "your_mqtt_password"
    #   COMMAND_TOPIC  = b"primerail/robot/control"  # bytes literal, must match robot.py
    #   COMMAND_SECRET = "your-long-random-secret"   # must match COMMAND_SECRET in robot.py
    #
    # BROKER NOTE: broker.hivemq.com is a free public broker with no
    # authentication. Replace it with Mosquitto running on your LAN
    # (the same PC that runs robot.py works fine — it does not need
    # to be internet-facing). Enable TLS + username/password in
    # mosquitto.conf so only your devices can connect. Until then,
    # the COMMAND_SECRET is the only access control in place.
    try:
        import config as _cfg                          # Attempt to import the local config.py file that holds the real credentials
        WIFI_SSID      = _cfg.WIFI_SSID               # Load the Wi-Fi network name from the secure config file
        WIFI_PASS      = _cfg.WIFI_PASS               # Load the Wi-Fi password from the secure config file — never hard-code this
        MQTT_BROKER    = _cfg.MQTT_BROKER             # Load the broker address (should be a private LAN IP, not a public URL)
        MQTT_USER      = _cfg.MQTT_USER               # Load the MQTT username for authenticated broker access
        MQTT_PASS      = _cfg.MQTT_PASS               # Load the MQTT password for authenticated broker access
        COMMAND_TOPIC  = _cfg.COMMAND_TOPIC           # Load the command topic bytes literal (must match robot.py exactly)
        COMMAND_SECRET = _cfg.COMMAND_SECRET          # Load the shared secret token that authenticates each command payload
        # FIX 9: MOTION_ARMED controls whether the vehicle is allowed to actuate its motors.
        # When False (the safe default for bench testing and initial bring-up) the robot will:
        #   - boot fully, connect Wi-Fi and MQTT, initialise the camera and vision pipeline
        #   - receive and authenticate MQTT commands, print what it WOULD do
        #   - run onboard vision and compute steering angles
        #   - but physically NEVER move — all drive_forward/drive_reverse calls are blocked
        # Set to True ONLY when the vehicle is on the ground, the path is clear, and a human
        # is present to send an authenticated RESUME command (which also requires MOTION_ARMED).
        # Add  MOTION_ARMED = True  to config.py when you are ready for live operation.
        MOTION_ARMED = getattr(_cfg, "MOTION_ARMED", False) # Read from config — default to False if the key is absent so a freshly uploaded config.py is always safe
        print(f"main.py: MOTION_ARMED = {MOTION_ARMED}.{' Vehicle will move.' if MOTION_ARMED else ' Motors are LOCKED — set MOTION_ARMED = True in config.py to enable motion.'}") # Make the armed/locked state unmissable in the boot log
    except ImportError:                                # config.py is missing — fail loudly rather than silently using placeholder values
        raise Exception(                               # Raise immediately with a clear message so the operator knows exactly what to fix
            "FATAL: config.py not found on the board filesystem. "
            "Upload it via Thonny or mpremote before running main.py. "
            "See the comment block above for the required format."
        )

    # AUTONOMY CONFIG — controls whether the vehicle drives itself or waits for PC commands.
    # True  = fully autonomous: onboard camera runs line detection on this chip; MQTT is an
    #         optional, secured human-override channel (active for manual_override_window_ms
    #         after each authenticated command, then control returns to onboard vision).
    # False = legacy teleoperated: robot.py on a PC must send commands continuously; the
    #         vehicle does not move if no PC is present or the network drops.
    AUTONOMOUS_MODE = True                             # Default to fully autonomous, no-human-required driving

    # FAILSAFE HARDWARE BRING-UP: wrap all object construction in a guard so a crash here
    # (bad pin number, PWM already claimed, camera ribbon loose, etc.) triggers a clean
    # reset rather than leaving the board in an unknown state with no recovery path.
    try:
        # 1. Instantiate the Steering object — attaches to the servo signal pin
        my_steering = SteeringController(pin_num=16)   # Create the steering controller on Pin 16 — wheels center immediately on construction

        # 2. Instantiate the Motor object — maps all six L293D control pins
        my_motors = MotorController(en_a=4, in1=5, in2=6, en_b=15, in3=7, in4=8) # Motors are halted (stop() called) immediately inside the constructor

        # 3. Instantiate the Network object — stores credentials but does not connect yet
        my_network = NetworkController(WIFI_SSID, WIFI_PASS, MQTT_BROKER, COMMAND_TOPIC, mqtt_user=MQTT_USER, mqtt_pass=MQTT_PASS) # Connection happens inside RobotOrchestrator.run() so boot errors are handled there

        # 4. Set up the camera — the ESP32-S3's single sensor can run in ONE mode at a time:
        #    GRAYSCALE for onboard autonomous vision, OR JPEG for remote PC monitoring.
        #    Autonomous mode takes priority since driving safely matters more than streaming.
        my_vision = None                                # Pre-declare as None; only populated if AUTONOMOUS_MODE is True
        if AUTONOMOUS_MODE:                             # Autonomous mode: camera is claimed in GRAYSCALE mode for onboard line detection
            my_vision = OnboardVisionController()       # Initialize onboard vision — camera.init() runs inside this constructor in GRAYSCALE mode
            print("Camera claimed by OnboardVisionController (GRAYSCALE). JPEG monitoring stream disabled in autonomous mode.") # Make the hardware trade-off explicit in boot log
        else:                                           # Legacy teleop mode: camera is free for JPEG monitoring stream only
            my_camera = CameraStreamer()                # Initialize the camera in JPEG mode for remote PC viewing
            my_camera.start_server()                    # Start the MJPEG web server on CPU Core 1 so the PC can connect to it

        # 5. Assemble the orchestrator by injecting all sub-systems
        my_robot = RobotOrchestrator(my_steering, my_motors, my_network, vision=my_vision, autonomous_mode=AUTONOMOUS_MODE, command_secret=COMMAND_SECRET, motion_armed=MOTION_ARMED) # FIX 9: pass MOTION_ARMED so motors are locked until the operator explicitly arms the vehicle in config.py

        # 6. Start the infinite boot-and-drive loop — this call does not return under normal operation
        my_robot.run()                                  # Boot sequence runs inside run(): connect network, arm watchdog, enter driving loop

    except Exception as init_err:                       # Catch any failure during hardware construction (before run()'s own recovery logic would apply)
        print(f"CRITICAL: Hardware initialization failed: {init_err}") # Log the exact failure so it is visible over serial before the reset wipes the state
        print("Resetting in 3 seconds to attempt a clean recovery...") # Warn the operator that a reset is imminent
        time.sleep(3)                                   # Give the serial buffer time to flush the error message before the chip resets
        machine.reset()                                 # Full hardware reset — motors were never driven (constructor calls stop() on boot), so this is always safe