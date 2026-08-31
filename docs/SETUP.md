# ROVE — Complete Setup Guide

This document explains how to set up and run the ROVE Camera-Based Line Follower project on a new computer.

The goal is simple:

> Clone the project → install the required software → install Python packages → configure the system → run the simulation (or the real vehicle).

No previous knowledge of the project is required.

---

## 1. What is ROVE?

ROVE is an autonomous camera-based line-following vehicle software system. It contains:

- **PC software** for vision, decision making, and simulation
- **ESP32 software** for real vehicle hardware control
- **Webots simulation** for testing without physical hardware
- **Configuration files** for deployment and calibration

During development, Webots allows the vehicle and road environment to be simulated before using physical hardware.

For a full breakdown of every folder, see [Project Structure](PROJECT_STRUCTURE.md).

---

## 2. Prerequisites

Install these before cloning:

| Tool | Purpose | Notes |
|---|---|---|
| Python 3.12+ | Runs the PC master node | Match the version noted in `requirements.txt` |
| Git | Clone the repository | |
| An MQTT broker (e.g. Mosquitto) | Message bus between PC and ESP32 | Can run locally on `127.0.0.1:1883` |
| Webots | Simulation without physical hardware | Optional if you only have real hardware |
| `esptool` / `mpremote` (or `ampy`) | Flash and deploy MicroPython to the ESP32 | Needed only for physical deployment |

---

## 3. Clone the Repository

```bash
git clone https://github.com/nandadeep009-alt/Camera_Based_Line_Follower_CODE.git
cd Camera_Based_Line_Follower_CODE
```

---

## 4. Install Python Dependencies

Create a virtual environment (recommended) and install the pinned packages:

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install -r requirements.txt
```

`requirements.txt` currently pins: `numpy`, `opencv-python`, `paho-mqtt`, `torch`, `ultralytics`.

---

## 5. Configure the Local Environment

The repository does not store real credentials. Create your local PC configuration by copying the provided template:

```bash
cp pc/config.example.py pc/config.py
```

Then edit `pc/config.py` and set your local values:

```
MQTT_BROKER        # e.g. "127.0.0.1" for a local broker
COMMAND_TOPIC       # MQTT topic the PC publishes drive commands to
FLEET_ALERT_TOPIC   # MQTT topic for fault/alert messages
COMMAND_SECRET      # shared secret between PC and ESP32, change from the default
```

`pc/config.py` is excluded by `.gitignore` and must never be committed.

---

## 6. Start the MQTT Broker

If using Mosquitto locally:

```bash
mosquitto -v
```

Confirm it's listening on the port referenced by `MQTT_BROKER` in `pc/config.py` (default `1883`).

---

## 7. Run in Simulation (Webots)

1. Open Webots and load the world file in `webots/worlds/`.
2. Start the simulation.
3. In a separate terminal, run the PC master node pointed at the Webots adapter:

```bash
python pc/robot.py
```

`pc/webots_adapter.py` bridges the simulated camera and vehicle to the same vision/command pipeline used with real hardware, so you can validate steering, speed control, and fail-safe behavior before touching the physical car.

---

## 8. Run on Real Hardware

1. Flash MicroPython onto the ESP32-S3 (the firmware image is provided in `ESP32_Brain/`).
2. Deploy the contents of `esp32/` to the board, along with your local values from `micropython_config/` (Wi-Fi SSID, broker address, GPIO pins, calibration bounds) — for example with `mpremote`:

   ```bash
   mpremote connect <port> cp esp32/*.py :
   mpremote connect <port> cp micropython_config/*.json :
   ```

3. Power on the vehicle. The ESP32 will connect to Wi-Fi, subscribe to MQTT, and wait for commands from the PC node.
4. On the PC, run:

   ```bash
   python pc/robot.py
   ```

   pointed at the physical camera stream instead of the Webots adapter.

---

## 9. Important Safety and Security Rule

This repository must **never** contain real passwords, Wi-Fi credentials, MQTT passwords, API keys, tokens, or other private secrets.

Do not put real credentials into:

- `README.md` or any file in `docs/`
- Python source files
- ESP32 source files
- JSON configuration files
- Git commits, GitHub Issues, or GitHub Discussions

Use local configuration files that are excluded by `.gitignore` (`pc/config.py` is the primary example).

---

## 10. Troubleshooting

| Symptom | Likely Cause |
|---|---|
| PC node can't connect to broker | Broker not running, or wrong `MQTT_BROKER` value in `pc/config.py` |
| ESP32 doesn't respond to commands | Wrong Wi-Fi credentials in `micropython_config/network_template.json`, or watchdog timeout tripped |
| Vehicle loses the line easily | Re-tune `vision_params.json` thresholds for your lighting/track |
| Steering out of range | Check `hardware_calibration.json` PWM pulse bounds against your servo |

---

Next: read the [Development Roadmap](ROADMAP.md) to see what's finished and what's in progress.
