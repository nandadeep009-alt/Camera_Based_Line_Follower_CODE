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
- MQTTController: Encapsulates the network connection and publishing logic.
- VisionController: Encapsulates the math and OpenCV image processing logic.
- RobotCommander: The central orchestrator that takes the above objects 
  (Dependency Injection) and runs the main video loop.
=========================================================================================
"""

import cv2                 # Import OpenCV for computer vision and image processing
import threading           # Import threading to run the camera feed in the background without freezing the app
import time                # Import time to track delays and limit how fast we send messages
import paho.mqtt.client as mqtt  # Import the Paho MQTT library to talk to the internet broker

DROID_CAM_INDEX = 0        # Define the camera index (0 is usually the default webcam or DroidCam)
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

    def map_error_to_angle(self, error): # Method to convert pixel distance into a physical steering angle
        kp = 0.2           # Proportional gain: how aggressively the robot should steer to correct the error
        angle = self.servo_center + int(error * kp) # Calculate the new angle by multiplying the error by our aggressiveness
        return int(max(45, min(135, angle))) # Clamp the angle between 45 and 135 so we don't break the physical steering column

    def process_frame(self, frame): # The main method that analyzes a single picture
        # Shrink the image using the scale factor to make the math run much faster
        small_frame = cv2.resize(frame, (0, 0), fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_AREA)
        height, width = small_frame.shape[:2] # Extract the new height and width of the shrunken image
        
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY) # Convert the color image into grayscale (black and white)
        # Apply a smart threshold to isolate the dark line from the background, creating a high-contrast binary mask
        # NEW FIX: Measure the average brightness of all pixels (0 is pitch black, 255 is blinding light).
        mean_brightness = cv2.mean(gray)[0]

        # If the average brightness is under 20, the camera is covered or the room is pitch black.
        if mean_brightness < 20:
            # Snap the steering back to straight ahead.
            target_angle = self.servo_center
            # Tell the motors to stop.
            engine_state = "STOP"
            # Draw red warning text on the screen so you know why it stopped.
            cv2.putText(small_frame, "TOO DARK", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        else:
            # If the room is bright enough, apply the night-vision contrast filter.
            # This turns everything darker than average into pure white, and everything else pure black.
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
            
            # Calculate the "moments" — this is a geometry math trick to find the center of mass of the white pixels.
            M = cv2.moments(thresh)
            
            if M["m00"] > 15000:   # If m00 is greater than 0, it means we actually found a line in the image
               cx = int(M["m10"] / M["m00"]) # Calculate the exact X-coordinate (horizontal position) of the center of the line
               error = cx - (width // 2) # Calculate how many pixels off-center the line is from the middle of the camera view
               target_angle = self.map_error_to_angle(error) # Pass that pixel error to our math function to get a steering angle
               engine_state = "DRIVE" # Because we see the line, command the back motors to drive forward
               # Draw a small green dot on the image right in the middle of the detected line so we can see it working
               cv2.circle(small_frame, (cx, height // 2), 5, (0, 255, 0), -1) 
            else:              # If m00 is 0, it means the camera sees absolutely no line at all
               target_angle = self.servo_center # Center the steering back to 90 degrees
               engine_state = "STOP" # Command the motors to stop so the robot doesn't drive off a cliff

        return target_angle, engine_state, small_frame # Send the answers and the drawn-on image back to the main program


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
    pc_camera = VideoStream(src=DROID_CAM_INDEX) # Create the camera object
    time.sleep(1.0) # Wait 1 second to give the camera hardware time to warm up and focus
    
    pc_vision = VisionController(scale_factor=0.5) # Create the math/vision object, setting it to shrink images by 50%
    pc_mqtt = MQTTController(broker=MQTT_BROKER, topic=COMMAND_TOPIC) # Boot up the internet connection.
    
    # Assemble the final application by injecting the three objects into the Commander
    app = RobotCommander(pc_camera, pc_vision, pc_mqtt) 
    
    # Command the fully assembled app to begin its infinite operating loop
    app.run()