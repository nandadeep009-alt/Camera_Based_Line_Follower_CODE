import cv2                 # Import the OpenCV library to handle image processing and camera capture.
import requests            # Import the requests library to send HTTP GET commands to the ESP32.
import threading           # Import the threading library to run tasks in the background simultaneously.
import queue               # Import the queue library to safely pass data between different threads.
import time                # Import the time library to allow for pauses, like waiting for the camera to warm up.

ESP32_IP = "http://192.168.1.100"  # Define a constant string for the robot's network address.
DROID_CAM_INDEX = 0                # Define a constant integer for the camera hardware index (0 is usually the default camera).

# ---------------------------------------------------------
# OOP Concept: Encapsulation & Abstraction
# We encapsulate the camera's state (thread, current frame) 
# inside this class so the main program just gets the newest image.
# ---------------------------------------------------------
class VideoStream:                           # Define a blueprint class for handling the camera hardware.
    
    def __init__(self, src=0):               # The constructor method initializes the object's attributes when created.
        self.stream = cv2.VideoCapture(src)  # Create an OpenCV video capture object connected to the camera.
        if not self.stream.isOpened():       # Check if the camera hardware failed to initialize.
            raise ValueError(f"Unable to open video source: {src}")  # Throw an error if the camera is inaccessible.
            
        self.grabbed, self.frame = self.stream.read()  # Read the very first frame to ensure the stream works.
        self.stopped = False                           # Initialize a boolean flag to control when the background thread should stop.
        
        self.thread = threading.Thread(target=self.update, daemon=True)  # Create a background thread targeting the update method.
        self.thread.start()                                              # Start the background thread so it runs parallel to the main program.

    def update(self):                                      # Define the method that will run continuously in the background thread.
        while not self.stopped:                            # Loop indefinitely as long as the stopped flag is False.
            grabbed, frame = self.stream.read()            # Grab the newest frame directly from the hardware buffer.
            if grabbed:                                    # If a frame was successfully read from the camera...
                self.grabbed, self.frame = grabbed, frame  # Update the object's attributes with the newest frame.

    def read(self):                                                                 # Define a public method for the main program to request the latest frame.
        return self.grabbed, self.frame.copy() if self.frame is not None else None  # Return a safe copy of the frame to prevent data corruption.

    def stop(self):            # Define a method to safely shut down the camera object.
        self.stopped = True    # Set the flag to True, which breaks the loop in the update thread.
        self.thread.join()     # Wait for the background thread to finish its final loop and close gracefully.
        self.stream.release()  # Command OpenCV to release its lock on the camera hardware.

# ---------------------------------------------------------
# OOP Concept: Separation of Concerns
# This class's ONLY job is networking. It isolates the 
# HTTP logic away from the vision math.
# ---------------------------------------------------------
class HardwareController:                            # Define a blueprint class for sending network commands to the robot.
    
    def __init__(self, base_url):                    # Constructor method to initialize the hardware controller.
        self.base_url = base_url                     # Store the ESP32's IP address as an attribute of the object.
        self.command_queue = queue.Queue(maxsize=1)  # Create a thread-safe queue holding maximum 1 item to avoid command backlog.
        self.running = True                          # Initialize a boolean flag to control the background networking thread.
        
        self.thread = threading.Thread(target=self._send_loop, daemon=True)  # Create a thread targeting the network sending loop.
        self.thread.start()                          # Start the networking thread in the background.

    def _send_loop(self):                            # Define the internal/private method that processes the queue (denoted by underscore).
        while self.running:                          # Keep running the loop as long as the object is active.
            try:                                     # Start an error-handling block for checking the queue.
                angle, engine_state = self.command_queue.get(timeout=0.1)  # Try to pull a command from the queue, wait up to 0.1s.
                url = f"{self.base_url}/control?angle={angle}&state={engine_state}"  # Format the HTTP GET string with the commanded values.
                try:                                 # Start an error-handling block for the network request.
                    requests.get(url, timeout=5)     # Send the actual HTTP GET request to the robot with a strict timeout.
                except requests.RequestException as e:    # Catch any network errors (like timeouts or disconnects).
                    print(f"Network Error: {e}")     # Ignore network errors silently so the program doesn't crash on a dropped packet.
            except queue.Empty:                      # Catch the exception raised if the queue was empty after the 0.1s wait.
                continue                             # Loop back to the top and wait for a new command again.
 
    def send_command(self, angle, engine_state):     # Define the public interface for the main program to send commands.
        if self.command_queue.full():                # Check if there is already an unsent command waiting in the queue.
            try:                                     # Start an error-handling block to clear the old command.
                self.command_queue.get_nowait()      # Remove the stale command from the queue immediately.
            except queue.Empty:                      # Catch the rare case where the thread emptied it exactly at this microsecond.
                pass                                 # Do nothing if it's already empty.
        self.command_queue.put((angle, engine_state))# Put the brand new steering and engine command into the queue.

    def stop(self):                                  # Define a method to safely shut down the networking object.
        self.running = False                         # Flip the running flag to False to terminate the background thread loop.
        self.thread.join()                           # Wait for the network thread to finish its current loop before exiting.

# ---------------------------------------------------------
# OOP Concept: Single Responsibility
# This class ONLY handles the OpenCV math. It has no idea
# where the frame comes from, or where the command goes.
# ---------------------------------------------------------
class VisionController:                              # Define a blueprint class for analyzing images and calculating steering.
    
    def __init__(self, scale_factor=0.5):            # Constructor method to set up vision parameters.
        self.scale_factor = scale_factor             # Store the image downscaling ratio as an attribute.
        self.servo_center = 90                       # Store the default neutral steering angle (straight ahead).

    def map_error_to_angle(self, error):             # Define a helper method to convert pixel distance to steering angle.
        kp = 0.2                                     # Define a proportional tuning constant (how aggressively to turn).
        angle = self.servo_center + int(error * kp)  # Calculate the new angle by applying the proportional math to the error.
        return int(max(45, min(135, angle)))         # Clamp the resulting angle between 45 and 135 degrees to protect the physical servo.

    def process_frame(self, frame):                  # Define the main method to analyze a single image.
        small_frame = cv2.resize(frame, (0, 0), fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_AREA)  # Shrink the image to compute it faster.
        height, width = small_frame.shape[:2]        # Extract the vertical and horizontal pixel dimensions of the shrunken image.
        
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)  # Convert the color image to grayscale to simplify analysis.
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)  # Convert to pure black and white to isolate the line.
        
        M = cv2.moments(thresh)                      # Calculate the image moments (spatial characteristics) of the white pixels.

        if M["m00"] > 0:                             # Check if any white pixels (the line) were actually found.
            cx = int(M["m10"] / M["m00"])            # Calculate the exact X-coordinate (horizontal center) of the line.
            error = cx - (width // 2)                # Calculate how many pixels off-center the line is from the middle of the camera.
            target_angle = self.map_error_to_angle(error)  # Use the helper method to turn the pixel error into an angle.
            engine_state = "DRIVE"                   # Set the motor command to drive forward since we see the line.
            cv2.circle(small_frame, (cx, height // 2), 5, (0, 255, 0), -1)  # Draw a green dot on the image exactly where the robot thinks the line is.
        else:                                        # If no line was found in the image...
            target_angle = self.servo_center         # Reset the steering to straight ahead.
            engine_state = "STOP"                    # Tell the motors to stop to prevent the robot from running away.

        return target_angle, engine_state, small_frame  # Return the computed steering, motor state, and annotated image back to the main program.

# ---------------------------------------------------------
# The Main Execution Block
# ---------------------------------------------------------
def run_line_follower():                             # Define the main coordinating function for the program.
    print("Starting Camera...")                      # Print a status message to the console.
    video_stream = VideoStream(src=DROID_CAM_INDEX)  # Instantiate (create) the video stream object.
    time.sleep(1.0)                                  # Pause the program for 1 second to allow the camera sensor to fully initialize.
    
    vision_bot = VisionController(scale_factor=0.5)  # Instantiate the vision calculator object.
    hardware_bot = HardwareController(base_url=ESP32_IP)  # Instantiate the hardware communicator object.

    print("Bot is running. Press 'q' to quit.")      # Inform the user how to stop the program safely.

    try:                                             # Start a try-finally block to ensure safe shutdown even if the program crashes.
        while True:                                  # Create an infinite loop to run the robot indefinitely.
            ret, frame = video_stream.read()         # Ask the video stream object for the newest frame.
            if not ret or frame is None:             # Check if the frame was corrupted or missing.
                continue                             # Skip the rest of the loop and try grabbing a frame again.

            target_angle, engine_state, processed_frame = vision_bot.process_frame(frame)                                    # Pass the frame to the vision object and get the driving commands back.
            
            hardware_bot.send_command(target_angle, engine_state)                                                            # Hand the driving commands over to the hardware object to send via WiFi.

            cv2.putText(processed_frame, f"ANGLE: {target_angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)  # Draw text on the video frame showing the current steering angle.
            cv2.imshow("Bot Vision", processed_frame)                                                                        # Pop up a GUI window showing the live camera feed with drawings.

            if cv2.waitKey(1) & 0xFF == ord('q'):    # Listen for keyboard input for 1 millisecond and check if 'q' was pressed.
                break                                # Exit the infinite loop if 'q' was pressed.
    finally:                                         # This block runs unconditionally when the loop exits or crashes.
        print("Shutting down...")                    # Print a shutdown message.
        video_stream.stop()                          # Call the stop method on the video object to kill its thread.
        hardware_bot.stop()                          # Call the stop method on the hardware object to kill its thread.
        cv2.destroyAllWindows()                      # Command OpenCV to close the live video GUI window.
   
if __name__ == "__main__":                           # Check if this script is being run directly (not imported into another file).
    run_line_follower()                              # Execute the main coordinating function to start the robot.