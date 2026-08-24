# ROVE — Project Structure

This document explains the important folders and files in the ROVE repository.

---

## 1. Repository Overview

```text
camera_line_follower/
|
+-- README.md
+-- requirements.txt
+-- .gitignore
+-- data_definitions
|
+-- docs/
|   +-- SETUP.md
|   +-- ROADMAP.md
|   +-- PROJECT_STRUCTURE.md
|
+-- pc/
|   +-- pc_stream.py
|   +-- pc_mqtt.py
|   +-- pc_vision.py
|   +-- pc_command.py
|   +-- robot.py
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
|   +-- netwrok_template.json
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
+-- ESP32_Brain/
cat > docs/PROJECT_STRUCTURE.md <<'EOF'
# ROVE — Project Structure

This document explains the important folders and files in the ROVE repository.

---

## 1. Repository Overview

```text
camera_line_follower/
|
+-- README.md
+-- requirements.txt
+-- .gitignore
+-- data_definitions
|
+-- docs/
|   +-- SETUP.md
|   +-- ROADMAP.md
|   +-- PROJECT_STRUCTURE.md
|
+-- pc/
|   +-- pc_stream.py
|   +-- pc_mqtt.py
|   +-- pc_vision.py
|   +-- pc_command.py
|   +-- robot.py
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
|   +-- netwrok_template.json
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
+-- ESP32_Brain/
echo
echo "=== BLOCK 7 VERIFY ==="
git diff --check -- docs/PROJECT_STRUCTURE.md
git status --short
cat > docs/PROJECT_STRUCTURE.md <<'EOF'
# ROVE — Project Structure

This document explains the important folders and files in the ROVE repository.

---

## 1. Repository Overview

```text
camera_line_follower/
|
+-- README.md
+-- requirements.txt
+-- .gitignore
+-- data_definitions
|
+-- docs/
|   +-- SETUP.md
|   +-- ROADMAP.md
|   +-- PROJECT_STRUCTURE.md
|
+-- pc/
|   +-- pc_stream.py
|   +-- pc_mqtt.py
|   +-- pc_vision.py
|   +-- pc_command.py
|   +-- robot.py
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
|   +-- netwrok_template.json
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
+-- ESP32_Brain/
cat > docs/PROJECT_STRUCTURE.md <<'EOF'
# ROVE — Project Structure

This document explains the important folders and files in the ROVE repository.

---

## 1. Repository Overview

```text
camera_line_follower/
|
+-- README.md
+-- requirements.txt
+-- .gitignore
+-- data_definitions
|
+-- docs/
|   +-- SETUP.md
|   +-- ROADMAP.md
|   +-- PROJECT_STRUCTURE.md
|
+-- pc/
|   +-- pc_stream.py
|   +-- pc_mqtt.py
|   +-- pc_vision.py
|   +-- pc_command.py
|   +-- robot.py
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
|   +-- netwrok_template.json
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
+-- ESP32_Brain/
echo
echo "=== BLOCK 7 VERIFY ==="
git diff --check -- docs/PROJECT_STRUCTURE.md
git status --short



