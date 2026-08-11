# esp32/cam.py
import camera        # type: ignore # Import MicroPython camera driver (suppress VS Code warning)
import socket        # Import standard TCP/IP socket network driver
import _thread       # type: ignore # Import MicroPython multi-threading library (suppress VS Code warning)
import utime as time # type: ignore # Import time module for delays

class CameraStreamer:
    def __init__(self):
        print("Initializing Camera Hardware...") # Output camera initialization log
        try:
            camera.init(0, format=camera.JPEG, framesize=camera.FRAME_VGA) # Initialize camera sensor in JPEG VGA format (640x480)
            print("Camera initialized successfully.") # Confirm successful camera sensor hardware boot
        except Exception as e:
            print(f"CRITICAL: Camera failed to start: {e}") # Log hardware initialization failure

    def start_server(self):
        print("Starting video stream server on core 1...") # Log multi-threading thread delegation
        _thread.start_new_thread(self._serve_video, ())   # Spawn video streaming loop on secondary CPU core

    def _serve_video(self):
        s = None                                    # Pre-declare socket handle variable
        while True:                                 # Loop to attempt TCP socket binding and listening
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create new IPv4 TCP stream socket
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Enable instant port reuse upon disconnect
                s.bind(('', 80))                   # Bind socket to HTTP port 80 on all local network interfaces
                s.listen(1)                        # Allow maximum 1 queued incoming TCP client connection
                print("CameraStreamer: video server socket bound to port 80 and listening.") # Log bound state
                break                              # Exit bind retry loop on successful initialization
            except Exception as bind_err:
                print(f"CameraStreamer: socket bind failed ({bind_err}), retrying in 3 seconds...") # Log port conflict error
                if s is not None:                  # Check if socket object was created before error
                    try:
                        s.close()                  # Safely close broken socket handle
                    except Exception as close_err:
                        print(f"CameraStreamer: s.close() after bind failure raised {close_err} (safe to ignore).") # Log cleanup error
                time.sleep(3)                      # Delay 3 seconds prior to next bind attempt
        
        while True:                                 # Main client acceptance loop
            conn = None                             # Initialize incoming client connection handle
            try:
                conn, addr = s.accept()             # Block until a client (PC) establishes TCP connection
                print(f"PC connected to camera stream from IP: {addr}") # Log connected client IP address
                conn.send(b'HTTP/1.1 200 OK\r\n')  # Send standard HTTP 200 OK header
                conn.send(b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n') # Send MJPEG streaming headers
                
                while True:                         # Frame transmission loop
                    buf = camera.capture()          # Read raw JPEG byte array from camera frame buffer
                    if buf:                         # Verify buffer contains valid image data
                        conn.send(b'--frame\r\n')   # Transmit boundary delimiter for MJPEG stream
                        conn.send(b'Content-Type: image/jpeg\r\n\r\n') # Transmit payload header for JPEG image
                        conn.send(buf)              # Transmit raw JPEG image buffer payload
                        conn.send(b'\r\n')          # Transmit trailing frame line break
            except Exception as stream_err:
                print(f"Video stream interrupted: {stream_err}. Waiting for next PC connection...") # Log drop event
            finally:
                if conn is not None:               # Check if client connection exists
                    try:
                        conn.close()               # Close client TCP connection handle
                    except Exception as close_err:
                        print(f"CameraStreamer: conn.close() raised {close_err} (safe to ignore).") # Log close error