#MARK-3 - ROBOT_CODE/--
#CHECK WHETHER WORKS BETTER OR NOT//

import cv2
import requests
import threading
import queue
import time

# ==============================================================================
# CONFIGURATION
# ==============================================================================
ESP32_IP = "http://192.168.1.100"
#VIDEO_SOURCE = ""
CAMERA_INDEX = 0  # DroidCam usually registers as 0 or 1
SCALE_FACTOR = 0.5


# ==============================================================================
# 1. HARDWARE CONTROLLER (Runs on a separate background thread to prevent lag)
# ==============================================================================
class MotorController:
    """Handles Wi-Fi communication with the ESP32 without blocking the camera."""
    
    def __init__(self, base_url):
        self.base_url = base_url
        self.command_queue = queue.Queue(maxsize=5)
        self.running = True
        
        # Start a background thread dedicated solely to networking
        self.network_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.network_thread.start()

    def update_motors(self, angle, state):
        """Drops the latest command into the queue for the background thread."""
        try:
            # If the queue is full (Wi-Fi is very slow), remove the oldest command
            if self.command_queue.full():
                self.command_queue.get_nowait()
            self.command_queue.put_nowait((angle, state))
        except queue.Full:
            pass

    def _send_loop(self):
        """The background worker that constantly sends data to the ESP32."""
        while self.running:
            try:
                # Wait for a command to appear in the queue
                angle, state = self.command_queue.get(timeout=0.1)
                
                # Fast timeout: don't wait longer than 50ms for the ESP32 to reply
                url = f"{self.base_url}/control?angle={angle}&state={state}"
                requests.get(url, timeout=0.05)
                
            except queue.Empty:
                continue  # No commands waiting, loop again
            except requests.exceptions.RequestException:
                pass  # Ignore Wi-Fi timeouts/errors to keep the thread alive

    def shutdown(self):
        """Safely kills the network thread."""
        self.running = False
        self.network_thread.join(timeout=1.0)


# ==============================================================================
# 2. VISION SYSTEM (Handles only pixels and math)
# ==============================================================================
class VisionSystem:
    """Reads camera frames and calculates the steering angle."""
    
    def __init__(self, source_index, scale):
        self.scale = scale
        self.servo_center = 90
        
        self.cap = cv2.VideoCapture(source_index)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Force 1-frame buffer to kill lag
        
        if not self.cap.isOpened():
            raise ValueError(f"CRITICAL: Cannot open camera index {source_index}")

    def _map_angle(self, error):
        """Proportional control math to convert pixels to degrees."""
        kp = 0.2
        target = self.servo_center + int(error * kp)
        return int(max(45, min(135, target))) # Clamp between 45 and 135 degrees

        # 1. Downsample for speed (The rest of your code stays exactly the same!)
        small = cv2.resize(frame, (0, 0), fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        height, width = small.shape[:2]
    
    def get_steering_data(self):
        """Grabs a frame, processes it, and returns the required steering."""
        ret, frame = self.cap.read()
        if not ret:
           return None, None, None

        # 1. Downsample for speed
        small = cv2.resize(frame, (0, 0), fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        height, width = small.shape[:2]
        
        # 2. Vision Math
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        M = cv2.moments(thresh)

        # 3. Logic Branching
        if M["m00"] > 0:
            # Line is found
            cx = int(M["m10"] / M["m00"])
            error = cx - (width // 2)
            angle = self._map_angle(error)
            state = "DRIVE"
            cv2.circle(small, (cx, height // 2), 5, (0, 255, 0), -1)
        else:
            # Line is lost (Failsafe)
            angle = self.servo_center
            state = "STOP"

        # Overlay text for debugging
        cv2.putText(small, f"ANG: {angle} | {state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        return angle, state, small

    def shutdown(self):
        """Releases the camera hardware safely."""
        self.cap.release()


# ==============================================================================
# 3. THE ROBOT MANAGER (Orchestrates the components)
# ==============================================================================
class LineFollowerRobot:
    """The main manager that ties vision and hardware together."""
    
    def __init__(self, vision_system, motor_controller):
        self.eyes = vision_system
        self.muscles = motor_controller

    def run(self):
        """The main execution loop."""
        print("Robot Online. Press 'q' in the video window to stop.")
        
        try:
            while True:
                # 1. Look at the floor and calculate math
                angle, state, frame = self.eyes.get_steering_data()
                
                # If camera fails/ends, break the loop
                if frame is None:
                    print("Video stream lost.")
                    break
                    
                # 2. Tell the hardware thread to move the motors
                self.muscles.update_motors(angle, state)
                
                # 3. Show what the robot sees on the PC screen
                cv2.imshow("Bot Vision", frame)
                
                # 4. Wait 1ms and listen for the 'q' quit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Manual shutdown triggered.")
                    break
                    
        except KeyboardInterrupt:
            print("Keyboard interrupt detected.")
            
        finally:
            self._cleanup()

    def _cleanup(self):
        """Ensures all windows close and motors stop when the script ends."""
        print("Shutting down robot...")
        self.muscles.update_motors(90, "STOP") # Force a final stop command
        time.sleep(0.1) # Give the network thread a split second to send the stop command
        
        self.muscles.shutdown()
        self.eyes.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")


# ==============================================================================
# EXECUTION
# ==============================================================================
if __name__ == "__main__":
    # Initialize the separated components
    vision = VisionSystem(source_index=CAMERA_INDEX, scale=SCALE_FACTOR)
    motors = MotorController(base_url=ESP32_IP)
    
    # Pass them into the Robot Manager and run
    bot = LineFollowerRobot(vision, motors)
    bot.run()