"""
=========================================================================================
PC VIDEO STREAM MODULE (pc_stream.py)
-----------------------------------------------------------------------------------------
Handles threaded video capture hardware abstraction.
Runs frame acquisition in a dedicated background thread to ensure real-time performance
without blocking the main vision processing loop. Includes auto-reconnect failsafes.
=========================================================================================
"""

import cv2  # type: ignore # Import OpenCV library for computer vision and camera stream capture
import threading  # type: ignore # Import threading module to run video grabbing in a parallel thread
import time  # type: ignore # Import time module for tracking timestamps and handling recovery delays


class VideoStream:  # type: ignore # Encapsulates background camera capture and auto-reconnect logic
    def __init__(self, src=0, stale_reconnect_frames=30):  # type: ignore # Constructor method initializing video capture settings
        self.src = src  # type: ignore # Store the video source URL or camera index for re-opening if stream drops
        self.stream = cv2.VideoCapture(src)  # type: ignore # Initialize OpenCV VideoCapture object with given source
        if not self.stream.isOpened():  # type: ignore # Check if camera device or network stream failed to open
            raise ValueError(f"Unable to open video source: {src}")  # type: ignore # Raise error if initial stream connection cannot be made
        self.grabbed, self.frame = self.stream.read()  # type: ignore # Read the initial video frame to populate internal buffer
        self.last_frame_time = time.time()  # type: ignore # Record timestamp of the initial frame capture
        self.stale_reconnect_frames = stale_reconnect_frames  # type: ignore # Set threshold count of consecutive failed frame reads before reconnecting
        self.stopped = False  # type: ignore # Initialize thread control flag to False (running state)
        self.thread = threading.Thread(target=self.update, daemon=True)  # type: ignore # Create background daemon thread pointing to update loop
        self.thread.start()  # type: ignore # Launch background worker thread immediately

    def update(self):  # type: ignore # Target method executing endlessly inside background thread
        consecutive_failures = 0  # type: ignore # Counter tracking consecutive frame read failures
        while not self.stopped:  # type: ignore # Continue looping while stream is active and not stopped
            grabbed, frame = self.stream.read()  # type: ignore # Pull the newest frame hardware buffer from OpenCV
            if grabbed:  # type: ignore # Check if frame capture was successful
                self.grabbed, self.frame = grabbed, frame  # type: ignore # Update thread-shared variables with latest frame data
                self.last_frame_time = time.time()  # type: ignore # Refresh timestamp marking freshest frame receipt
                consecutive_failures = 0  # type: ignore # Reset consecutive failure counter back to zero
            else:  # type: ignore # Execute when frame acquisition fails or drops
                consecutive_failures += 1  # type: ignore # Increment consecutive failure counter
                self.grabbed = False  # type: ignore # Flag frame grab status as False to notify main thread
                if consecutive_failures >= self.stale_reconnect_frames:  # type: ignore # Check if failure threshold is reached
                    print("Camera feed unresponsive - attempting to reconnect...")  # type: ignore # Log reconnect effort
                    try:  # type: ignore # Guard hardware release and re-opening
                        self.stream.release()  # type: ignore # Release dead video stream resources
                        time.sleep(1.0)  # type: ignore # Pause before attempting stream re-initialization
                        self.stream = cv2.VideoCapture(self.src)  # type: ignore # Re-instantiate VideoCapture instance
                    except Exception as reconnect_err:  # type: ignore # Catch any stream re-opening errors
                        print(f"Camera reconnect attempt failed: {reconnect_err}")  # type: ignore # Print error message
                    consecutive_failures = 0  # type: ignore # Reset counter to avoid continuous rapid reconnecting

    def read(self):  # type: ignore # Thread-safe method providing latest frame copy to caller
        return self.grabbed, self.frame.copy() if self.frame is not None else None  # type: ignore # Return grab status and cloned image array

    def is_stale(self, max_age_s=0.5):  # type: ignore # Check if current frame age exceeds safety duration
        return (time.time() - self.last_frame_time) > max_age_s  # type: ignore # Compare current time delta against threshold limit

    def stop(self):  # type: ignore # Cleanly terminate background thread and camera handle
        self.stopped = True  # type: ignore # Raise stop flag to break background loop
        self.thread.join()  # type: ignore # Wait for background thread to exit cleanly
        self.stream.release()  # type: ignore # Release camera hardware or socket resource