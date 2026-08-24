# ROVE — Project Structure

This document explains the main folders and files in the ROVE repository.

---

## Repository Overview

```text
camera_line_follower/
|
+-- README.md
+-- requirements.txt
+-- .gitignore
+-- data_definitions.py
|
+-- docs/
|   +-- SETUP.md
|   +-- ROADMAP.md
|   +-- PROJECT_STRUCTURE.md
|
+-- pc/
|   +-- robot.py
|   +-- pc_stream.py
|   +-- pc_mqtt.py
|   +-- pc_vision.py
|   +-- pc_command.py
|   +-- webots_adapter.py
|   +-- backups/
|
+-- esp32/
|   +-- main.py
|   +-- orch.py
|   +-- net.py
|   +-- hw.py
|   +-- cam.py
|   +-- vision.py
|   +-- pin_config.py
|
+-- micropython_config/
|   +-- board_pins.json
|   +-- deploy_config.json
|   +-- hardware_calibration.json
|   +-- network_template.json
|   +-- project_manifest.json
|   +-- vision_params.json
|   +-- vscode_settings.json
|
+-- webots/
|   +-- project/
|   +-- worlds/
|   +-- assets/
|
+-- Archive_Old_Versions/
|
+-- ESP32_Brain/
