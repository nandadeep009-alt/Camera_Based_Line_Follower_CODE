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

import cv2                 # Import OpenCV for computer vision and image processing
import threading           # Import threading to run the camera feed in the background without freezing the app
import time                # Import time to track delays and limit how fast we send messages
import paho.mqtt.client as mqtt  # Import the Paho MQTT library to talk to the internet broker

"""DROID_CAM_INDEX = 0        # Define the camera index (0 is usually the default webcam or DroidCam)"""
MQTT_BROKER = "broker.hivemq.com" # Define the URL of the free public MQTT broker being used
COMMAND_TOPIC = "primerail/robot/control" # Define the topic name where we will publish our commands


# ---------------------------------------------------------
# OOP Concept: Encapsulation of Camera Hardware
# ---------------------------------------------------------
class VideoStream:         # Define a class dedicated to handling the live camera feed
    def __init__(self, src=0): # Constructor method that takes the camera source index
        self.stream = cv2.VideoCapture(src) # Create an OpenCV video capture object attached to the camera
        if not self.stream.isOpened(): # Check if the camera successfully turned on
            raise ValueError(f"Unable to open video source: {src}") # Crash with a helpful error if the camera is blocked
        self.grabbed, self.frame = self.stream.read() # Take the very first picture to initialize the variables
        self.stopped = False # Create a flag to track whether the camera should be running or stopped
        self.thread = threading.Thread(target=self.update, daemon=True) # Create a background worker thread for the camera
        self.thread.start() # Start the background thread so it constantly pulls new frames

    def update(self):      # The method that runs forever in the background thread
        while not self.stopped: # Keep looping as long as the stopped flag is False
            grabbed, frame = self.stream.read() # Grab the absolute newest frame from the camera hardware
            if grabbed:    # If a frame was successfully captured
                self.grabbed, self.frame = grabbed, frame # Update the class variables with the new picture

    def read(self):        # Method for the main program to ask for the latest picture
        # Return a copy of the frame so the main program doesn't accidentally corrupt the live feed memory
        return self.grabbed, self.frame.copy() if self.frame is not None else None 

    def stop(self):        # Method to cleanly shut down the camera hardware
        self.stopped = True # Set the flag to True, which breaks the loop inside the update() method
        self.thread.join() # Wait for the background thread to safely finish its last loop
        self.stream.release() # Tell the computer hardware to let go of the camera


# ---------------------------------------------------------
# OOP Concept: Encapsulation of Network Publishing
# ---------------------------------------------------------
class MQTTController:      # Define a class dedicated to handling outbound internet communication
    def __init__(self, broker, topic): # Constructor taking the broker URL and the target topic
        self.topic = topic # Store the topic string inside the object
        self.client = mqtt.Client() # Create the main Paho MQTT client object
        
        print(f"Connecting to MQTT Broker at {broker}...") # Print a status message to the PC terminal
        self.client.connect(broker, 1883, 60) # Connect to the broker on standard port 1883 with a 60-second timeout
        
        self.client.loop_start() # Automatically start a background network thread to handle pinging and data flow
        print("Connected to Broker.") # Print a success message to the PC terminal

    def send_command(self, angle, engine_state): # Method to broadcast our math results to the robot
        payload = f"{angle},{engine_state}" # Format the data into the exact comma-separated string the ESP32 expects
        self.client.publish(self.topic, payload) # Fire the formatted text message up to the internet broker

    def stop(self):        # Method to cleanly shut down the internet connection
        self.client.loop_stop() # Stop the background networking thread
        self.client.disconnect() # Politely tell the broker we are logging off


# ---------------------------------------------------------
# OOP Concept: Encapsulation of Vision Logic & Math
# ---------------------------------------------------------
class VisionController:    # Define a class dedicated to image processing and steering calculations
    def __init__(self, scale_factor=0.5): # Constructor that allows us to shrink the image to save CPU power
        self.scale_factor = scale_factor # Store the shrink ratio inside the object
        self.servo_center = 90 # Define 90 degrees as perfectly straight ahead for our physical servo
        self.last_valid_angle = 90 # Store the last known valid steering angle to remember trajectory when the line is lost

        self.state = "DRIVE"     # Initialize the state machine state string ("DRIVE", "SEARCH_STOP", "REVERSE")
        self.lost_line_timestamp = 0 # Initialize a timer for the emergency stop wait window
        self.reverse_start_timestamp = 0 # Initialize a timer for the reverse recovery window

    def map_error_to_angle(self, error): # Method to convert pixel distance into a physical steering angle
        kp = 0.2           # Proportional gain: how aggressively the robot should steer to correct the error
        angle = self.servo_center + int(error * kp) # Calculate the new angle by multiplying the error by our aggressiveness
        return int(max(45, min(135, angle))) # Clamp the angle between 45 and 135 so we don't break the physical steering column

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

        # Create a binary image where the dark line stands out clearly from the lighter floor
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        M = cv2.moments(thresh) # Compute the image moments to find the center of the detected line mass

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

            engine_state = "DRIVE" # Command the motors to move forward when the line is visible
            cv2.circle(small_frame, (cx, height // 2), 5, (0, 255, 0), -1) # Draw a green dot on the detected line center
            cv2.putText(small_frame, f"LINE DETECTED: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # If the line is lost, activate the failsafe behavior: stop, wait, then reverse to recover it
        else:                  # This branch runs only when the line is no longer visible in the frame
            current_time = time.time() # Use the current clock time to measure how long the robot has been in each recovery stage

            # Enter the stop phase the first time the line is lost during normal driving
            if self.state == "DRIVE":
                self.state = "SEARCH_STOP" # Change state so the robot knows it is entering a recovery routine
                self.lost_line_timestamp = current_time # Record when the line was lost to time the stop period
                print("Vehicle lost the line. Stopping and waiting 3-4 seconds to recover.")

            # During the stop phase, keep the robot motionless so it can settle before reversing
            if self.state == "SEARCH_STOP":
                target_angle = self.servo_center # Keep steering centered while the vehicle is paused
                engine_state = "STOP" # Hold the motors still during the wait window

                # After 3.5 seconds, start the reverse recovery phase
                if current_time - self.lost_line_timestamp > 3.5:
                    self.state = "REVERSE" # Switch from stop to reverse recovery mode
                    self.reverse_start_timestamp = current_time # Record when reverse motion began
                    print("Wait complete. Reversing along the same trajectory to find the line.")

            # In reverse mode, drive backward while steering in a mirrored direction to try to find the line again
            elif self.state == "REVERSE":
                elapsed_reverse = current_time - self.reverse_start_timestamp # Measure how long reverse recovery has been active
                print(f"Reversing to recover the line... ({elapsed_reverse:.1f} / 5.0 s)")

                reverse_angle = 180 - self.last_valid_angle # Mirror the last known steering angle so the reverse path follows the same curve
                target_angle = max(45, min(135, reverse_angle)) # Clamp the reverse steering angle to safe servo limits

                # Keep reversing for up to 5 seconds, then stop if the line has still not reappeared
                if elapsed_reverse < 5.0:
                    engine_state = "REVERSE" # Tell the motor controller to drive backward
                else:
                    engine_state = "STOP" # Stop after the maximum reverse window to avoid endless backing up
                    cv2.putText(small_frame, "REVERSE TIMEOUT", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(small_frame, f"FAILSAFE: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return target_angle, engine_state, small_frame

# ---------------------------------------------------------
# OOP Concept: Dependency Injection & Orchestration
# ---------------------------------------------------------
class RobotCommander:      # Define the main orchestrator class that ties the camera, vision, and network together
    def __init__(self, camera_module, vision_module, mqtt_module): # Constructor accepting our three sub-systems
        self.camera = camera_module # Store the injected VideoStream object
        self.vision = vision_module # Store the injected VisionController object
        self.mqtt = mqtt_module     # Store the injected MQTTController object
        self.last_send_time = time.time() # Initialize a timer to keep track of when we last sent a message

    def run(self):         # The main infinite loop method that runs the application
        print("Bot is running. Press 'q' to quit.") # Print instructions to the user
        
        try:               # Start a try-finally block to ensure we shut down safely even if there is a crash
            while True:    # Start an infinite loop that processes video frames as fast as possible
                ret, frame = self.camera.read() # Ask the camera object for the absolute newest picture
                if not ret or frame is None: # If the camera glitched and gave us nothing
                    continue # Skip the rest of this loop and try again

                # Pass the picture to the vision brain, and get back the angle, the motor state, and the marked-up image
                target_angle, engine_state, processed_frame = self.vision.process_frame(frame)
                
                # Check if 0.1 seconds (100 milliseconds) have passed since we last sent a message
                if time.time() - self.last_send_time > 0.1: 
                    self.mqtt.send_command(target_angle, engine_state) # Fire the command to the internet
                    self.last_send_time = time.time() # Reset the stopwatch

                # Draw the angle and motor state as text in the top left corner of the video window
                cv2.putText(processed_frame, f"ANGLE: {target_angle} | {engine_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.imshow("Bot Vision", processed_frame) # Show the marked-up video frame on the computer monitor

                if cv2.waitKey(1) & 0xFF == ord('q'): # Check if the user pressed the 'q' key on their keyboard
                    break  # If they did, break out of the infinite loop
        finally:           # This block runs no matter what, even if the user hits Ctrl+C
            self.shutdown() # Call our clean shutdown method

    def shutdown(self):    # Method to politely turn off all hardware and connections
        print("Shutting down...") # Print a status message
        self.camera.stop() # Tell the camera object to release the USB/DroidCam hardware
        self.mqtt.stop()   # Tell the network object to cleanly log off the HiveMQ server
        cv2.destroyAllWindows() # Tell OpenCV to close the popup video window


# ---------------------------------------------------------
# MAIN EXECUTION (Object Instantiation and Assembly)
# ---------------------------------------------------------
if __name__ == "__main__": # Check if this script is being run directly (not imported as a module)
    print("Starting Camera...") # Print a startup message
    
    # Instantiate the three sub-systems
    ESP32_CAMERA_URL = "http://192.168.x.x/" # Replace with your ESP32's actual IP address
    pc_camera = VideoStream(src=ESP32_CAMERA_URL) # Pass the URL instead of the number 0
    time.sleep(1.0) # Wait 1 second to give the camera hardware time to warm up and focus
    
    pc_vision = VisionController(scale_factor=0.5) # Create the math/vision object, setting it to shrink images by 50%
    pc_mqtt = MQTTController(broker=MQTT_BROKER, topic=COMMAND_TOPIC) # Boot up the internet connection.
    
    # Assemble the final application by injecting the three objects into the Commander
    app = RobotCommander(pc_camera, pc_vision, pc_mqtt) 
    
    # Command the fully assembled app to begin its infinite operating loop
    app.run()