# Camera_Based_Line_Follower_CODE
             #   OR
# Autonomous Camera Line Follower System

A real-time, vision-guided autonomous line-following vehicle architecture using a **PC Master Control Node** for computer vision processing and an **ESP32 MicroPython Agent** for low-level motor/servo actuation over MQTT.

---

                        /---System Architecture---/
                                     |
                     +-------------------------------+
                     |      ESP32 Camera Module      |
                     +---------------+---------------+
                                     |
                           (HTTP MJPEG Stream)
                                     v
+-----------------------------------------------------------------------------+
|                               PC MASTER NODE                                |
|  +------------------+    +-------------------+    +----------------------+  |
|  |   pc_stream.py   | -> |   pc_vision.py    | -> |    pc_command.py     |  |
|  | Threaded Capture |    | OpenCV Centroid/P |    |  RobotCommander Hub  |  |
|  +------------------+    +-------------------+    +----------+-----------+  |
+--------------------------------------------------------------|--------------+
                                                               |
                                                  (MQTT CSV Command Stream)
                                                               v
                      +-------------------------------+
                       |       MQTT Broker (1883)      |
                       +---------------+---------------+
                                       |
                                       v
+-----------------------------------------------------------------------------+
|                              ESP32 AGENT NODE                               |
|                                                                             |
|  +------------------+    +-------------------+    +----------------------+  |
|  |      net.py      | -> |      orch.py      | -> |        hw.py         |  |
|  | MQTT Subscriber  |    | Watchdog / Router |    | L298N & Servo Output |  |
|  +------------------+    +-------------------+    +----------------------+  |
+-----------------------------------------------------------------------------+

## 📁 Repository Directory Structure

```text
camera_line_follower/
├── data_definitions.py          # Unified data type declarations & wire specs
├── config.py                    # PC-side credentials & broker parameters (git-ignored)
│   ├── robot.py                  # Main PC application entry point
├── pc/
│   ├── pc_stream.py             # Threaded video capture with auto-reconnect
│   ├── pc_mqtt.py               # Outbound MQTT client with auto-reconnect backoff
│   ├── pc_vision.py             # OpenCV adaptive vision & recovery state machine
│   └── pc_command.py            # Main orchestrator (Dependency Injection target)
├── esp32/
│   ├── main.py                  # ESP32 boot script & main execution loop
│   ├── orch.py                  # Orchestrator & 500ms watchdog logic
│   ├── net.py                   # Wi-Fi and MicroPython MQTT connection manager
│   ├── hw.py                    # L298N motor driver & PWM servo controller
│   ├── cam.py                   # ESP32-CAM stream server interface
│   └── pin_config.py            # Hardware GPIO mapping definitions
├── micropython_config/          # Deployment and JSON configuration templates
│   ├── board_pins.json          # Hardware GPIO allocations
│   ├── deploy_config.json       # MicroPython deployment specs
│   ├── hardware_calibration.json# Steering servo PWM pulse bounds
│   ├── network_template.json    # Wi-Fi SSID & broker defaults
│   ├── project_manifest.json    # Module manifest tracker
│   └── vision_params.json       # Color mask and adaptive threshold parameters
└── README.md                    # System documentation







---

# 📚 Project Documentation

If you are new to ROVE, use the documents below in this order:

1. **[Complete Setup Guide](docs/SETUP.md)**
   Start here if you want to install, configure, and run the project on another computer.

2. **[Development Roadmap](docs/ROADMAP.md)**
   See what has been completed, what is currently being developed, and what remains.

3. **Project Structure**
   The project structure documentation will explain the purpose of the PC, ESP32, Webots, configuration, and documentation folders.

---

## 🚀 Quick Start

```text
NEW USER
   |
   v
Read SETUP.md
   |
   v
Install Python + dependencies
   |
   v
Configure local environment
   |
   v
Run Webots / PC simulation
   |
   v
Study project structure
   |
   v
Continue development
## 🔐 Security

Never commit real passwords, Wi-Fi credentials, MQTT credentials, API keys, tokens, or other private information.

Use local configuration files that are excluded from Git.

---

