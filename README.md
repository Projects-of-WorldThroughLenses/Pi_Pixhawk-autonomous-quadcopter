# Drone Dashboard v100 — Python-Only Live/Post Computer Vision + Low-Latency Split FPV GCS

This README is for **`drone_dashboard_v100_live_post_ai.py` only**.  
It is **not** for the C++ port, vcpkg, Visual Studio, CMake, or installer packages.

`Drone Dashboard v100` is a single Python ground-control dashboard for UAV research and testing. It combines FPV display, MAVLink telemetry, Betaflight/iNav MSP telemetry, RadioMaster TX15 USB monitoring, computer vision, dataset recording, fuel estimation, and a guarded waypoint-planning interface in one PyQt5 application.

---

## 1. What this program does

`drone_dashboard_v100_live_post_ai.py` is a Python/PyQt5 desktop dashboard. Its main purpose is to let you monitor a drone, view FPV video, record synchronized data, and optionally run computer vision.

Main modules:

- **Pixhawk / ArduPilot MAVLink telemetry** over COM, UDP, or TCP.
- **Betaflight / iNav MSP telemetry** over USB serial / COM.
- **RadioMaster TX15 Max USB joystick/HID monitor** through `pygame`.
- **FPV video input** through OpenCV.
- **Live computer vision** using optional Ultralytics YOLO, plus OpenCV fallback methods.
- **Post-processing computer vision** for recorded FPV video.
- **Dataset recorder** that saves video, telemetry CSV, detection CSV, TX/RX channel CSV, frames, and crops.
- **Main-tab map / waypoint planner** with guarded MAVLink Guided waypoint sending.
- **Dual TX/RX channel display** so you can compare TX USB values and receiver/Pixhawk RC channel values.
- **Scientific fuel / propulsion estimator** for battery, prop, motor, payload, and flight-time planning.

Safety design:

- The program does **not** automatically arm the drone.
- The program does **not** automatically disarm the drone.
- The program does **not** automatically take off.
- Guided waypoint sending requires MAVLink connection, GPS validity, and confirmation.
- Test with propellers removed or in a simulator before any real flight use.

---

## 2. Files you need

Minimum files:

```text
drone_dashboard_v100_live_post_ai.py
drone_dashboard_v100_requirements.txt
```

The requirements file contains:

```text
PyQt5
PyQtWebEngine
opencv-python
numpy
pymavlink
pygame
ultralytics
pyserial
```

Recommended folder:

```text
C:\Users\phone\Documents\Drone_Flight_Estimator\
```

Example:

```text
C:\Users\phone\Documents\Drone_Flight_Estimator\drone_dashboard_v100_live_post_ai.py
C:\Users\phone\Documents\Drone_Flight_Estimator\drone_dashboard_v100_requirements.txt
```

---

## 3. Recommended system

Recommended Windows setup:

```text
OS: Windows 10 or Windows 11
Python: Python 3.10 or Python 3.11 recommended
RAM: 16 GB minimum, 32 GB better
GPU: Optional, NVIDIA GPU useful for YOLO/CUDA
Camera input: USB camera or FPV capture card
Flight controller: Pixhawk/ArduPilot or Betaflight/iNav board
Radio: Optional TX15 Max / EdgeTX USB joystick mode
```

For your Dell XPS 16 / RTX 4070 laptop, use:

```text
FPV backend: DirectShow
FPV format: MJPG
Resolution: 1280x720 first
FPS: 60
Buffer: 1
AI device: cuda:0 if CUDA PyTorch works, otherwise auto or cpu
```

---

## 4. Install Python

Install Python 3.10 or 3.11 from python.org.

During installation, check:

```text
Add Python to PATH
```

After installing, open PowerShell and check:

```powershell
python --version
```

or:

```powershell
py --version
```

You should see something like:

```text
Python 3.10.x
```

or:

```text
Python 3.11.x
```

---

## 5. Create a virtual environment

Open PowerShell:

```powershell
cd "C:\Users\phone\Documents\Drone_Flight_Estimator"

python -m venv dashboard_env
```

If `python` does not work, use:

```powershell
py -3.11 -m venv dashboard_env
```

Activate the environment:

```powershell
.\dashboard_env\Scripts\activate
```

You should see something like this at the beginning of the PowerShell line:

```text
(dashboard_env)
```

Upgrade pip:

```powershell
python -m pip install --upgrade pip setuptools wheel
```

---

## 6. Install requirements

Run this inside the same folder:

```powershell
pip install -r drone_dashboard_v100_requirements.txt
```

If the file name has extra parentheses from download, for example:

```text
drone_dashboard_v100_requirements(2).txt
```

then run:

```powershell
pip install -r "drone_dashboard_v100_requirements(2).txt"
```

The installation may take time because `ultralytics` can install additional machine-learning dependencies.

---

## 7. Run the program

From the same folder, with the virtual environment activated:

```powershell
python drone_dashboard_v100_live_post_ai.py
```

If your file has a downloaded name like:

```text
drone_dashboard_v100_live_post_ai(6).py
```

run:

```powershell
python "drone_dashboard_v100_live_post_ai(6).py"
```

---

## 8. First boot screen

When the app starts, it opens the **Drone Dashboard v100 Boot** screen.

You will choose:

```text
Theme:
- Dark theme
- White theme

Launch display:
- Full screen
- Maximized window
- Windowed

Display scaling:
- Auto
- Compact
- Normal
- Large

Computer vision mode:
- Live Computer Vision
- Post-processing Computer Vision
```

Recommended first boot:

```text
Theme: Dark theme or White theme
Launch display: Maximized window
Display scaling: Auto
Computer vision mode: Live Computer Vision
```

If the UI is too large:

```text
Display scaling: Compact
```

If the UI is too small:

```text
Display scaling: Large
```

Keyboard shortcuts:

```text
F11 = toggle fullscreen
Esc = exit fullscreen
```

---

## 9. Main tabs

The v100 app has these main areas:

### Main Flight

This is the main cockpit screen.

It shows:

- FPV video feed
- AI overlay boxes when enabled
- 3D AHRS / gyroball
- Flight telemetry cards
- Map / waypoint planner
- Timer / fuel / quick actions
- TX USB vs RX/Pixhawk RC channel bars
- ESC telemetry
- Fence status

### AI / Dataset

This tab is for computer vision and dataset recording.

In **Live Computer Vision mode**, it can:

- Start computer vision
- Stop computer vision
- Start AI overlay
- Stop AI overlay
- Start recording data
- Stop recording data
- Show detection log

In **Post-processing Computer Vision mode**, live overlay is disabled and the app lets you process recorded video after recording.

### Technical / Setup

This tab is for:

- MAVLink / MSP connection
- TX15 USB connection
- FPV source setup
- AI model and AI settings
- Dataset output folder
- Fuel / propulsion settings

---

## 10. FPV setup

Go to:

```text
Technical / Setup → FPV capture / zero-latency video
```

Recommended starting settings:

```text
Source: 0
Backend: DirectShow
Width: 1280
Height: 720
FPS: 60
Format: MJPG
Buffer: 1
UI scale: 1.00
```

Then click:

```text
Start FPV only
```

If nothing appears, try:

```text
Source: 1
Source: 2
Source: 3
```

If DirectShow does not work, try:

```text
Backend: MSMF
Backend: Auto
```

If MJPG does not work, try:

```text
Format: Auto
Format: YUY2
```

For low latency, the best first test is:

```text
DirectShow + MJPG + 1280x720 + 60 FPS + Buffer 1
```

---

## 11. Raw FPV recording

The app can record raw FPV video separately from the display path.

Start FPV first, then click:

```text
Record Raw FPV 60FPS
```

Stop with:

```text
Stop FPV Recording
```

The file name format is:

```text
fpv_60fps_YYYYMMDD_HHMMSS.avi
```

Example:

```text
fpv_60fps_20260610_184533.avi
```

The video uses MJPG AVI for lower CPU load.

---

## 12. Live Computer Vision mode

Choose this on boot:

```text
Computer vision mode: Live Computer Vision
```

In this mode:

```text
Start Computer Vision
```

starts detection in the background.

```text
Start AI Overlay
```

shows detection boxes on the Main Flight FPV view.

```text
Stop AI Overlay
```

hides boxes from the main FPV view, but computer vision can continue running in the background.

```text
Stop Computer Vision
```

fully stops detection.

The important design is:

```text
FPV feed stays separate from AI detection.
AI returns boxes.
Main FPV overlays boxes only when overlay is enabled.
```

This is meant to reduce lag compared with processing the same displayed frame directly.

---

## 13. Post-processing Computer Vision mode

Choose this on boot:

```text
Computer vision mode: Post-processing Computer Vision
```

In this mode:

- Live FPV stays clean and smooth.
- Live AI overlay is disabled.
- You record raw FPV first.
- After stopping recording, the program can ask whether you want to process that video.
- You can process all objects, humans only, or things only depending on the dialog/options.
- The output can be a detection CSV and optionally an overlay video.

Post-processing output file examples:

```text
<video_name>_post_ai_YYYYMMDD_HHMMSS_detections.csv
<video_name>_post_ai_YYYYMMDD_HHMMSS_ai_overlay.mp4
```

---

## 14. YOLO model setup

The default model path is:

```text
yolov8n.pt
```

You can use the Browse Model button in the app to choose another model file.

Recommended settings for smooth live detection:

```text
Device: auto or cuda:0
imgsz: 640
AI input width: 640 or 960
Confidence: 0.35
Detect every N frames: 3 to 5
Max detections: 50
FP16 on CUDA: enabled if CUDA works
```

If YOLO is too slow:

```text
- Increase Detect every N frames
- Use imgsz 320 or 640
- Use YOLOv8n instead of a larger model
- Use Post-processing mode instead of Live mode
- Turn off AI overlay when you only need clean FPV
```

If Ultralytics or YOLO fails, the program has OpenCV fallback methods for some detection functions, but YOLO is the main intended detector.

---

## 15. Dataset recording

The dataset recorder saves synchronized flight and vision data.

Default base folder:

```text
C:\Users\<you>\drone_v100_records
```

You can change this in the Technical / Setup tab.

When you start recording data, it creates a session folder:

```text
flight_YYYYMMDD_HHMMSS
```

Inside the session folder:

```text
telemetry.csv
detections.csv
tx_rx_channels.csv
ai_overlay_video.mp4
frames/
crops/
```

### telemetry.csv

Stores flight telemetry such as:

```text
mode
armed
lat/lon
relative altitude
GPS altitude
groundspeed
climb rate
heading
roll/pitch/yaw
battery voltage/current/remaining
GPS fix/satellites
lidar altitude
optical flow quality
radio RSSI
ESC RPM/current
```

### detections.csv

Stores AI detection data such as:

```text
time
frame index
frame file
label
raw label
confidence
box coordinates
source
lat/lon
relative altitude
heading
```

### tx_rx_channels.csv

Stores TX and RX channel values:

```text
TX connected
TX name
TX update rate
RX RSSI
radio RSSI
TX CH1-CH16
RX CH1-CH16
```

### frames/

Saved still frames for later training or analysis.

### crops/

Detection crops saved from the frame, grouped by label.

---

## 16. MAVLink / Pixhawk setup

Go to:

```text
Technical / Setup → Flight Controller Telemetry
```

Select:

```text
Protocol: MAVLink (ArduPilot/PX4)
```

Common connection examples:

```text
COM3
COM4
COM5
udp:127.0.0.1:14550
udp:0.0.0.0:14550
tcp:127.0.0.1:5760
```

Common baud rates:

```text
57600
115200
230400
460800
921600
```

For Pixhawk USB:

```text
Connection: COMx
Baud: 115200
```

For telemetry radio:

```text
Connection: COMx
Baud: 57600
```

Then click:

```text
Connect FC
```

If it says permission denied or cannot open port:

```text
- Close Mission Planner
- Close QGroundControl
- Close any serial monitor
- Check Device Manager for the correct COM port
- Unplug and reconnect Pixhawk
```

Only one program can usually own a COM port at the same time.

---

## 17. MAVLink telemetry shown

When connected to Pixhawk / ArduPilot, the dashboard can show:

```text
Mode
Armed/disarmed state
Roll / pitch / yaw / heading
GPS fix type
GPS satellites
Latitude / longitude
GPS altitude
Relative altitude
Groundspeed
Climb rate
Battery voltage
Battery current
Battery percentage
RC channels
Radio status
Distance sensor / lidar altitude
Optical flow quality
ESC telemetry
Fence status
Home position
```

The program requests message intervals for high-rate attitude, GPS, battery, RC, optical flow, distance sensor, ESC telemetry, fence, and home position messages.

---

## 18. Betaflight / iNav MSP setup

Go to:

```text
Technical / Setup → Flight Controller Telemetry
```

Select:

```text
Protocol: MSP (Betaflight/iNav USB)
```

Use:

```text
Connection: COMx
Baud: 115200
```

Then click:

```text
Connect FC
```

MSP monitoring can show:

```text
Attitude
RC channels
Motor outputs
Battery voltage/current
GPS if available
Altitude if available
Status flags
```

MSP mode is monitoring-only. It does not arm, disarm, write settings, or send motor commands.

---

## 19. TX15 Max USB setup

On the RadioMaster TX15 Max / EdgeTX radio:

```text
1. Connect USB cable to laptop.
2. Choose USB Joystick / HID mode on the radio if prompted.
3. Open the dashboard.
4. Go to Technical / Setup.
5. Click Refresh TX.
6. Select the TX device.
7. Click Connect TX USB.
```

The dashboard reads TX channels through `pygame` joystick/HID.

The app can display:

```text
TX USB channel values
RX/Pixhawk RC channel values
TX update rate
TX connected/disconnected state
```

This is useful for checking whether your radio is sending the same channel values that the flight controller receives.

---

## 20. ESC telemetry

For ArduPilot, ESC telemetry can arrive in these MAVLink groups:

```text
ESC_TELEMETRY_1_TO_4
ESC_TELEMETRY_5_TO_8
ESC_TELEMETRY_9_TO_12
ESC_TELEMETRY_13_TO_16
```

The program stores ESC telemetry for up to 16 ESC slots.

Data can include:

```text
RPM
voltage
current
temperature
```

If Mission Planner shows ESC telemetry but the dashboard does not, check:

```text
- Are you connected to the same MAVLink stream?
- Is Mission Planner already using the COM port?
- Are ESC messages on ESC9-12 instead of ESC1-4?
- Is the MAVLink connection actually receiving ESC_TELEMETRY messages?
- Try UDP forwarding from Mission Planner if direct COM is blocked.
```

---

## 21. Fence status

The dashboard can monitor ArduPilot fence data.

It can show:

```text
Fence breach status
Fence breach type
Fence altitude max
Fence radius
Home position
Altitude remaining before fence warning
Radius remaining before fence warning
```

Dashboard-side warning behavior:

```text
Altitude warning when within about 3 m of altitude fence
Radius warning when within about 10 m of circular fence from home
```

Important:

```text
The dashboard does not replace ArduPilot failsafe.
ArduPilot still owns the real fence and failsafe behavior.
```

---

## 22. Map / waypoint planner

The main tab has an integrated map / waypoint planning panel.

The guided waypoint sender is guarded. It requires:

```text
MAVLink connection
valid GPS
selected waypoint
confirmation dialog
```

The program does not arm or take off. Use waypoint functions only after the drone is already safely airborne and under control, or in simulation.

Recommended first test:

```text
1. Use SITL or simulator.
2. Verify GPS and home position.
3. Click map or enter waypoint.
4. Confirm command behavior.
5. Never test this first with props on near people or obstacles.
```

---

## 23. Fuel / propulsion estimator

The v100 program includes a scientific fuel / propulsion estimator.

It uses settings such as:

```text
battery cells
cell voltage
capacity
usable percentage
reserve percentage
rotor count
drone mass
payload mass
motor KV
prop diameter
prop pitch
blade count
loaded RPM factor
figure of merit
max thrust per motor
max current per motor
ESC rating
altitude
temperature
humidity
```

It can estimate:

```text
air density
battery voltage
loaded RPM
hover thrust per motor
static current estimate
hover current
cruise current
flight time
reserve time
thrust margin
warnings
```

This is a planning tool only. Always verify with actual flight logs and battery telemetry.

---

## 24. Low-latency settings

For lowest FPV latency:

```text
Backend: DirectShow
Format: MJPG
Resolution: 1280x720
FPS: 60
Buffer: 1
AI overlay: OFF unless needed
Computer vision: OFF unless needed
Recording: internal SSD
```

If you see stutters:

```text
- Close browsers and heavy apps
- Try 720p instead of 1080p
- Try 30 FPS for weaker capture cards
- Use MJPG instead of YUY2
- Disable AI overlay
- Use Post-processing mode instead of Live CV
- Use DirectShow instead of Auto/MSMF
- Set buffer to 1
```

True zero latency is impossible because camera, capture card, USB, decoding, and display all add delay. The goal is minimum practical latency.

---

## 25. Common troubleshooting

### Problem: `ModuleNotFoundError`

Example:

```text
ModuleNotFoundError: No module named 'PyQt5'
```

Fix:

```powershell
.\dashboard_env\Scripts\activate
pip install -r drone_dashboard_v100_requirements.txt
```

### Problem: PyQtWebEngine install fails

Try upgrading pip first:

```powershell
python -m pip install --upgrade pip setuptools wheel
pip install PyQtWebEngine
```

If the map does not load, the rest of the dashboard may still work with a fallback depending on your environment.

### Problem: FPV black screen

Try:

```text
Source 0, 1, 2, 3
Backend DirectShow
Backend MSMF
Format MJPG
Format Auto
Resolution 640x480 or 1280x720
Close OBS / Camera app / browser camera tabs
```

### Problem: FPV is delayed

Try:

```text
Use DirectShow
Use MJPG
Use Buffer 1
Use 720p
Turn AI overlay off
Do not record to slow external drive
```

### Problem: Pixhawk COM port cannot connect

Try:

```text
Close Mission Planner
Close QGroundControl
Use correct COM port
Reconnect USB cable
Check Device Manager
Try baud 115200 for USB
Try baud 57600 for telemetry radio
```

### Problem: TX15 not detected

Try:

```text
Use USB Joystick/HID mode on radio
Click Refresh TX
Reconnect USB cable
Restart dashboard
Check Windows Game Controllers panel
Try another USB cable
```

### Problem: YOLO is very slow

Try:

```text
Use yolov8n.pt
Use imgsz 640 or 320
Use detect every 3-5 frames
Use cuda:0 if CUDA PyTorch is installed
Turn off AI overlay
Use Post-processing mode
```

---

## 26. Suggested GitHub repository structure for v100 only

```text
DroneDashboard_v100_Python/
├─ README.md
├─ drone_dashboard_v100_live_post_ai.py
├─ drone_dashboard_v100_requirements.txt
├─ screenshots/
│  ├─ boot_screen.png
│  ├─ main_flight.png
│  ├─ technical_setup.png
│  └─ ai_dataset.png
├─ docs/
│  ├─ FPV_SETUP.md
│  ├─ MAVLINK_SETUP.md
│  ├─ MSP_SETUP.md
│  ├─ TX15_SETUP.md
│  └─ TROUBLESHOOTING.md
└─ sample_outputs/
   └─ README.md
```

---

## 27. Suggested `.gitignore`

```gitignore
# Python
__pycache__/
*.pyc
*.pyo
*.pyd

# Virtual environments
dashboard_env/
venv/
.env/

# Recordings and generated datasets
drone_v100_records/
records/
recordings/
flight_*/
*.avi
*.mp4
*.csv
*.jpg
*.png

# YOLO model weights
*.pt
*.onnx
*.engine

# IDE
.vscode/
.idea/

# Windows
Thumbs.db
Desktop.ini
```

---

## 28. Quick start checklist

```text
1. Put v100 Python file and requirements file in one folder.
2. Open PowerShell in that folder.
3. Create virtual environment.
4. Activate virtual environment.
5. Install requirements.
6. Run python drone_dashboard_v100_live_post_ai.py.
7. Choose boot settings.
8. Start FPV only first.
9. Connect TX15 if needed.
10. Connect Pixhawk or Betaflight board if needed.
11. Test computer vision only after FPV is stable.
12. Test dataset recording.
13. Test with props removed before real field use.
```

Commands:

```powershell
cd "C:\Users\phone\Documents\Drone_Flight_Estimator"
python -m venv dashboard_env
.\dashboard_env\Scripts\activate
python -m pip install --upgrade pip setuptools wheel
pip install -r drone_dashboard_v100_requirements.txt
python drone_dashboard_v100_live_post_ai.py
```

---

## 29. Disclaimer

This program is experimental research software. It is not certified flight-control software, not a safety system, and not a replacement for proper RC control, ArduPilot failsafes, or legal flight planning.

You are responsible for:

```text
safe testing
propeller safety
local drone laws
battery safety
flight area safety
hardware verification
sensor calibration
failsafe configuration
```

Always test in a controlled environment first.
