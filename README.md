# Drone Dashboard v112 — Python FPV + Telemetry Ground Control Dashboard

**Drone Dashboard v112** is a Python-only experimental ground-control dashboard for UAV research, FPV monitoring, telemetry visualization, TX/RX channel monitoring, GPS/map tracking, computer vision testing, and dataset recording.

This project is written as a single Python application:

```text
drone_dashboard_v112_smooth_fpv_gps_fix.py
```

It is based on the original v100 Python dashboard idea, then optimized through later versions for smoother FPV and improved GPS map behavior.

> This README is for the **Python v112 program only**.  
> There is no C++ version, no Qt/C++ build system, no vcpkg, and no compiling required.

---

## Table of Contents

- [1. What This Program Does](#1-what-this-program-does)
- [2. Main Features](#2-main-features)
- [3. Safety Design](#3-safety-design)
- [4. Recommended Hardware](#4-recommended-hardware)
- [5. Software Requirements](#5-software-requirements)
- [6. Folder Structure](#6-folder-structure)
- [7. Installation Guide](#7-installation-guide)
- [8. How to Run the Program](#8-how-to-run-the-program)
- [9. First Launch Setup](#9-first-launch-setup)
- [10. FPV Live View Setup](#10-fpv-live-view-setup)
- [11. Reducing FPV Stutter](#11-reducing-fpv-stutter)
- [12. MAVLink / Pixhawk Setup](#12-mavlink--pixhawk-setup)
- [13. Betaflight / iNav MSP Setup](#13-betaflight--inav-msp-setup)
- [14. RadioMaster TX15 USB Setup](#14-radiomaster-tx15-usb-setup)
- [15. GPS Map and Drone Marker](#15-gps-map-and-drone-marker)
- [16. Computer Vision / AI Features](#16-computer-vision--ai-features)
- [17. Dataset Recording](#17-dataset-recording)
- [18. Fuel / Battery / Propulsion Estimator](#18-fuel--battery--propulsion-estimator)
- [19. Suggested Test Procedure](#19-suggested-test-procedure)
- [20. Troubleshooting](#20-troubleshooting)
- [21. Performance Tips](#21-performance-tips)
- [22. GitHub Publishing Guide](#22-github-publishing-guide)
- [23. Suggested `.gitignore`](#23-suggested-gitignore)
- [24. Version Notes](#24-version-notes)
- [25. Disclaimer](#25-disclaimer)

---

## 1. What This Program Does

Drone Dashboard v112 is a Python ground-control and monitoring dashboard for drone development. It brings several important systems into one cockpit-style interface:

```text
FPV live video
MAVLink telemetry
MSP telemetry
TX15 USB joystick monitoring
GPS map tracking
Drone heading display
ESC telemetry
Fence status
Battery/fuel estimation
Computer vision overlay
Dataset recording
Waypoint planning
```

The goal is to make one dashboard where the pilot/developer can observe:

- what the drone is doing,
- what the flight controller is receiving,
- what the transmitter is sending,
- what the FPV camera sees,
- where the drone is on the map,
- how battery/fuel estimates look,
- and how computer vision behaves during flight testing.

This program is mainly for:

- UAV research
- robotics projects
- FPV testing
- Pixhawk / ArduPilot development
- Betaflight / iNav telemetry monitoring
- AI dataset collection
- experimental ground control station development

---

## 2. Main Features

### 2.1 FPV Live View

The app can open a camera, USB webcam, HDMI capture card, analog FPV capture card, or other OpenCV-compatible video source.

Features:

- OpenCV FPV capture
- live FPV display inside the dashboard
- DirectShow / MSMF / FFMPEG-style backend support depending on OpenCV
- adjustable camera source index
- adjustable resolution
- 30 / 50 / 60 FPS settings
- MJPG / YUY2 / H264 / Auto FourCC choices
- low-latency buffer setting
- latest-frame style display logic
- optional raw FPV recording

Recommended first setting:

```text
Backend: DirectShow
FourCC: MJPG
Resolution: 1280 x 720
FPS: 50
Buffer: 1
Display scale: 1.0
```

---

### 2.2 MAVLink Telemetry for Pixhawk / ArduPilot

The program can connect to Pixhawk / ArduPilot using MAVLink over:

```text
USB serial COM port
Telemetry radio COM port
UDP
TCP
```

Example connection strings:

```text
COM3
COM4
COM5
udp:127.0.0.1:14550
udp:0.0.0.0:14550
tcp:127.0.0.1:5760
```

The dashboard can display data such as:

- arm status
- flight mode
- roll
- pitch
- yaw
- heading
- battery voltage
- battery current
- battery remaining
- GPS fix type
- satellite count
- latitude
- longitude
- altitude
- relative altitude
- ground speed
- airspeed
- climb rate
- lidar distance
- optical flow quality
- RC channels
- radio status
- ESC RPM
- ESC voltage
- ESC current
- ESC temperature
- fence status
- home position

---

### 2.3 Betaflight / iNav MSP Telemetry

The program can connect to Betaflight or iNav boards over USB serial using MSP.

It can monitor:

- attitude
- battery voltage
- current
- RC channels
- GPS data if available
- altitude if available
- motor outputs if available
- MSP packet rate

This is monitoring-only. It is not intended to configure Betaflight/iNav, change settings, arm, disarm, or control motors.

---

### 2.4 RadioMaster TX15 Max USB Monitor

The app can read a RadioMaster TX15 Max / EdgeTX radio through USB joystick/HID mode using `pygame`.

It can display:

- TX channel values
- joystick axis values
- button states
- hats/switch states
- update rate
- comparison against Pixhawk receiver channels

This is useful for checking whether:

```text
TX output = what your radio is sending
RX input = what Pixhawk or FC is receiving
```

---

### 2.5 GPS Map with Drone Icon and Heading Arrow

v112 includes an improved GPS map system.

The GPS map can:

- follow the current drone GPS position,
- update latitude and longitude,
- show the drone marker,
- rotate the marker/arrow with heading,
- display status information,
- use online Leaflet/OpenStreetMap if available,
- fall back to an offline radar-style canvas map if internet or Leaflet fails.

The fallback map is important because FPV field testing may not always have stable internet.

---

### 2.6 Computer Vision / AI Overlay

The v112 program still includes optional computer vision features from the Python dashboard line.

It can support:

- optional Ultralytics YOLO model loading,
- live AI overlay on the FPV feed,
- OpenCV fallback detection paths,
- post-processing of recorded video,
- detection CSV logging,
- dataset recording.

For best FPV smoothness, test FPV first with AI disabled.

---

### 2.7 Dataset Recorder

The program can record synchronized test data for later analysis.

Dataset outputs can include:

```text
video file
telemetry CSV
detections CSV
TX/RX channel CSV
```

This is useful for:

- AI training data
- flight test review
- debugging RC channel behavior
- reviewing GPS and telemetry behavior
- comparing live FPV with telemetry logs

---

### 2.8 Fuel / Battery / Propulsion Estimator

The dashboard includes a scientific estimator for battery and propulsion planning.

It can estimate and compare:

- LiPo cell state of charge,
- air density,
- total battery voltage,
- loaded RPM,
- hover thrust per motor,
- thrust margin,
- hover current,
- cruise current,
- total power,
- estimated flight time,
- reserve time.

This estimator is useful for planning but should not replace real current sensors, battery logs, and conservative flight testing.

---

## 3. Safety Design

This dashboard is designed as a research and monitoring tool.

Important safety points:

```text
The program does not arm the drone automatically.
The program does not disarm the drone automatically.
The program does not take off automatically.
Guided waypoint sending requires confirmation.
Guided waypoint sending requires MAVLink connection and valid GPS.
Real flight testing should be done carefully.
```

Before connecting real motors or flying:

```text
1. Test with propellers removed.
2. Test in a simulator if possible.
3. Confirm all telemetry values are correct.
4. Confirm RC failsafe and manual control.
5. Confirm GPS behavior outside.
6. Confirm FPV delay is acceptable.
7. Confirm the dashboard is not the only safety layer.
```

This software is experimental and not certified for aviation use.

---

## 4. Recommended Hardware

### Laptop / PC

Recommended:

```text
Windows 10 or Windows 11
Intel i7 / i9 or AMD Ryzen 7 / Ryzen 9
16 GB RAM minimum
32 GB RAM recommended
SSD storage
Dedicated GPU optional but helpful for YOLO
USB 3.0 port for capture card
```

The user-tested target class is similar to:

```text
Dell XPS 16
Intel Core Ultra 9
RTX 4070
32 GB RAM
Windows
```

The app should also run on lower machines if AI and high-resolution recording are disabled.

---

### Flight Controllers

Supported or intended targets:

```text
Pixhawk / ArduPilot via MAVLink
Holybro Pixhawk 6C
Pixhawk 2.4.8
Betaflight F405 boards
iNav boards with MSP
```

---

### Radio

Tested/intended radio type:

```text
RadioMaster TX15 Max
EdgeTX USB joystick/HID mode
```

Other radios may work if Windows detects them as joystick/HID devices.

---

### FPV / Camera Sources

Possible sources:

```text
USB webcam
HDMI USB capture card
analog FPV USB capture card
camera index 0 / 1 / 2 / 3
OpenCV-compatible stream or file
```

---

## 5. Software Requirements

This is a Python-only project.

The required Python packages are:

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

Recommended Python version:

```text
Python 3.10 or Python 3.11
```

Python 3.12 may work for many packages, but Python 3.10/3.11 is usually safer for robotics and PyQt/OpenCV/Ultralytics compatibility.

---

## 6. Folder Structure

Recommended GitHub repository structure:

```text
Drone-Dashboard-v112/
├─ README.md
├─ requirements.txt
├─ drone_dashboard_v112_smooth_fpv_gps_fix.py
├─ screenshots/
│  ├─ main_dashboard.png
│  ├─ technical_setup.png
│  ├─ gps_map.png
│  └─ fuel_estimator.png
├─ docs/
│  ├─ setup_windows.md
│  ├─ fpv_optimization.md
│  └─ troubleshooting.md
└─ sample_outputs/
   └─ .gitkeep
```

Minimum required files:

```text
README.md
requirements.txt
drone_dashboard_v112_smooth_fpv_gps_fix.py
```

---

## 7. Installation Guide

### 7.1 Install Python

Download and install Python 3.10 or 3.11.

During installation, enable:

```text
Add Python to PATH
```

Check Python:

```powershell
python --version
```

Check pip:

```powershell
pip --version
```

---

### 7.2 Create Project Folder

Example:

```powershell
cd "C:\Users\phone\Documents"

mkdir Drone-Dashboard-v112

cd Drone-Dashboard-v112
```

Place these files inside the folder:

```text
drone_dashboard_v112_smooth_fpv_gps_fix.py
requirements.txt
README.md
```

---

### 7.3 Create a Virtual Environment

Windows PowerShell:

```powershell
python -m venv .venv
```

Activate it:

```powershell
.\.venv\Scripts\activate
```

You should see something like this:

```text
(.venv) PS C:\Users\phone\Documents\Drone-Dashboard-v112>
```

---

### 7.4 Upgrade pip

```powershell
python -m pip install --upgrade pip setuptools wheel
```

---

### 7.5 Install Requirements

```powershell
pip install -r requirements.txt
```

If you do not have a requirements file yet, create one with:

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

Then run:

```powershell
pip install -r requirements.txt
```

---

### 7.6 Optional: Install PyTorch for YOLO

Ultralytics may install or require PyTorch depending on the environment.

If YOLO does not run, install PyTorch separately from the official PyTorch instructions for your machine.

For FPV-only use, YOLO is not required to be active. You can run the dashboard and keep AI disabled.

---

## 8. How to Run the Program

From the project folder:

```powershell
.\.venv\Scripts\activate

python drone_dashboard_v112_smooth_fpv_gps_fix.py
```

If your Python command is `py`:

```powershell
py drone_dashboard_v112_smooth_fpv_gps_fix.py
```

If the program starts correctly, the dashboard window should open.

---

## 9. First Launch Setup

Recommended first launch settings:

```text
Theme: Dark or White
Launch Display: Maximized
Display Scaling: Auto
```

For testing, do not connect everything at once. Start simple:

```text
1. Open program
2. Start FPV only
3. Confirm live view is smooth
4. Stop FPV
5. Connect Pixhawk/MAVLink
6. Connect TX15 USB
7. Test GPS map
8. Test recording
9. Test AI last
```

---

## 10. FPV Live View Setup

Go to the Technical Setup / FPV section.

Recommended first setup:

```text
Video source: 0
Backend: DirectShow
FourCC: MJPG
Width: 1280
Height: 720
FPS: 50
Buffer size: 1
Display scale: 1.0
```

Then click:

```text
Start FPV
```

If no image appears, try source numbers:

```text
0
1
2
3
```

If DirectShow does not work, try:

```text
MSMF
Auto
FFMPEG
```

If MJPG does not work, try:

```text
Auto
YUY2
H264
```

---

## 11. Reducing FPV Stutter

For the smoothest FPV:

```text
Use 50 FPS instead of 60 FPS if 60 FPS causes micro-stutter.
Use 720p before trying 1080p.
Use MJPG instead of raw YUY2 when possible.
Use buffer size 1.
Use DirectShow on Windows.
Keep AI disabled during FPV smoothness testing.
Do not record while testing baseline smoothness.
Close OBS, browsers, Discord streams, and other camera apps.
Use a USB 3.0 port for capture cards.
Use an internal SSD for recording.
```

Best smoothness test:

```text
Backend: DirectShow
FourCC: MJPG
Width: 1280
Height: 720
FPS: 50
Buffer: 1
AI: Off
Recording: Off
Map tab: not actively being spammed
```

Why 50 FPS may help:

```text
60 FPS gives only about 16.67 ms per frame.
50 FPS gives about 20 ms per frame.
That extra time can reduce Python/Qt/OpenCV scheduling pressure.
```

The program still cannot guarantee true zero latency because USB capture devices, drivers, OpenCV backends, Windows scheduling, and display refresh timing all add delay.

---

## 12. MAVLink / Pixhawk Setup

### 12.1 USB Connection

Connect Pixhawk to the laptop by USB.

Open Windows Device Manager and find the COM port, for example:

```text
COM3
COM4
COM5
```

In the dashboard, use:

```text
Connection: COM3
Baud: 115200
```

or:

```text
Connection: COM3
Baud: 57600
```

Click connect.

---

### 12.2 Telemetry Radio

If using a telemetry radio:

```text
Connection: COMx
Baud: 57600
```

Only one program can usually use the COM port at once. Close Mission Planner or QGroundControl if the dashboard cannot open the port.

---

### 12.3 UDP Forwarding

If you want to use Mission Planner and this dashboard together, use UDP forwarding.

Example dashboard connection:

```text
udp:127.0.0.1:14550
```

or:

```text
udp:0.0.0.0:14550
```

Mission Planner must be configured to forward MAVLink to the correct UDP port.

---

### 12.4 MAVLink Data Shown

The dashboard can show:

```text
mode
armed status
roll / pitch / yaw
heading
GPS fix
satellites
latitude / longitude
altitude
battery voltage
battery current
RC channels
radio status
lidar altitude
optical flow quality
ESC telemetry
fence status
home position
```

---

## 13. Betaflight / iNav MSP Setup

For Betaflight or iNav:

```text
1. Connect the flight controller by USB.
2. Close Betaflight Configurator or iNav Configurator.
3. Find the COM port in Device Manager.
4. Use MSP connection in the dashboard.
```

Recommended settings:

```text
Connection: COMx
Baud: 115200
```

The dashboard can monitor:

```text
attitude
battery
RC channels
GPS if available
altitude if available
motor outputs if available
MSP packet rate
```

This program does not write Betaflight/iNav settings.

---

## 14. RadioMaster TX15 USB Setup

On the TX15:

```text
1. Plug USB cable into laptop.
2. Choose USB Joystick / HID mode if EdgeTX asks.
3. Open the dashboard.
4. Refresh TX devices.
5. Connect TX USB.
```

The dashboard can display TX channel values.

Use this to compare:

```text
TX USB values = what the radio sends to the computer
RX/Pixhawk channels = what the flight controller receives
```

This is useful for debugging channel mapping, switches, arming logic, and mode selection.

---

## 15. GPS Map and Drone Marker

v112 includes a more robust GPS map.

It can use:

```text
Online Leaflet / OpenStreetMap map
Offline fallback radar/canvas map
```

The GPS map can show:

```text
current drone position
drone heading arrow
latitude
longitude
satellite count
altitude
selected waypoint
home/start point
```

If online tiles do not load, the fallback map should still update the drone marker in a radar-style display.

### GPS Map Troubleshooting

If the map does not move:

```text
1. Confirm MAVLink is connected.
2. Confirm GPS fix is 3D fix or better.
3. Confirm latitude and longitude are not zero.
4. Check whether Mission Planner shows GPS movement.
5. Try outdoor GPS, not indoors.
6. Wait for HOME_POSITION / GPS_RAW_INT / GLOBAL_POSITION_INT messages.
7. Check the dashboard log panel for MAVLink messages.
```

If map tiles do not appear:

```text
1. Check internet connection.
2. PyQtWebEngine may be blocking online scripts.
3. The fallback map should still work.
4. Try restarting the dashboard.
```

---

## 16. Computer Vision / AI Features

The program can optionally run AI/computer vision.

Possible modes:

```text
Live FPV only
Live FPV with AI overlay
Post-process recorded video
Dataset recording
```

AI settings may include:

```text
model path
confidence
detect every N frames
save every N frames
AI image size
AI device
half precision
max detections
AI input width
```

Recommended smoothness setup:

```text
Start FPV first.
Confirm FPV is smooth.
Then start AI overlay.
If stutter appears, increase detect-every-N-frames.
Use smaller image size.
Use 720p.
Use GPU if available.
```

For best pure FPV smoothness, leave AI off.

---

## 17. Dataset Recording

Dataset recording can save synchronized data for later analysis.

Possible output files:

```text
video recording
telemetry CSV
detections CSV
TX/RX channel CSV
```

Use dataset recording when you want to collect:

```text
flight footage
AI training footage
telemetry data
GPS data
RC channel data
detection labels
```

Recommended approach:

```text
1. Choose recording folder.
2. Start FPV.
3. Connect telemetry.
4. Confirm data values update.
5. Start recording.
6. Stop recording before closing the app.
```

Do not record to a slow USB drive if you are trying to test FPV smoothness.

---

## 18. Fuel / Battery / Propulsion Estimator

The fuel estimator includes scientific planning functions.

It can consider:

```text
battery cell count
cell voltage
capacity
usable percentage
reserve percentage
drone mass
payload mass
rotor count
motor KV
propeller diameter
propeller pitch
blade count
loaded RPM factor
figure of merit
ESC rating
current limits
air density
altitude
temperature
humidity
```

It can estimate:

```text
battery voltage
loaded RPM
hover thrust per motor
thrust margin
static current
hover current
cruise current
estimated hover time
estimated cruise time
reserve time
warnings
```

Use this as a planning helper only. Real-world current draw must be verified with logs and current sensors.

---

## 19. Suggested Test Procedure

### Stage 1: UI Only

```text
Run program.
Do not connect hardware.
Check that UI opens.
Change theme/scaling.
Close program cleanly.
```

### Stage 2: FPV Only

```text
Connect camera or capture card.
Start FPV at 720p 50 FPS MJPG.
Check smoothness.
Try 60 FPS only after 50 FPS is stable.
```

### Stage 3: TX15 Only

```text
Connect TX15 USB.
Use joystick/HID mode.
Connect TX in dashboard.
Move sticks and switches.
Check channel bars.
```

### Stage 4: MAVLink Only

```text
Remove propellers.
Connect Pixhawk USB.
Connect MAVLink.
Check attitude, battery, GPS, RC channels.
Move drone gently by hand and confirm AHRS changes.
```

### Stage 5: GPS Map

```text
Move outdoors.
Wait for GPS fix.
Confirm lat/lon updates.
Confirm drone icon moves.
Confirm heading arrow rotates.
```

### Stage 6: Combined Test

```text
Start FPV.
Connect MAVLink.
Connect TX15.
Watch CPU/GPU usage.
Check for stutter.
```

### Stage 7: Recording / AI

```text
Start recording only after baseline FPV is smooth.
Start AI only after recording and FPV are tested.
```

---

## 20. Troubleshooting

### Problem: `ModuleNotFoundError`

Example:

```text
ModuleNotFoundError: No module named 'PyQt5'
```

Fix:

```powershell
pip install -r requirements.txt
```

---

### Problem: PyQtWebEngine Error

If the map area has issues:

```powershell
pip install PyQtWebEngine
```

If it still fails, the fallback map may still work, but the online map may not load.

---

### Problem: Camera Black Screen

Try:

```text
Change source index: 0, 1, 2, 3
Use DirectShow
Use MSMF
Use MJPG
Use Auto FourCC
Lower resolution to 640x480
Close other apps using the camera
Reconnect capture card
```

---

### Problem: FPV Freezes Every Few Seconds

Try:

```text
FPS: 50 instead of 60
Resolution: 1280x720
FourCC: MJPG
Buffer: 1
Disable AI
Disable recording
Close other apps
Use USB 3.0
Use DirectShow
Restart the camera/capture card
```

---

### Problem: COM Port Busy

Close:

```text
Mission Planner
QGroundControl
Betaflight Configurator
iNav Configurator
Arduino Serial Monitor
other serial monitor tools
```

Then reconnect.

---

### Problem: GPS Map Does Not Update

Check:

```text
MAVLink connected
GPS fix valid
lat/lon not zero
GPS messages are arriving
drone is outdoors
map tab is visible
no JavaScript/WebEngine error
```

---

### Problem: TX15 Not Detected

Try:

```text
Unplug and reconnect TX15
Select USB joystick/HID mode
Refresh TX devices
Close other joystick tools
Restart dashboard
Try another USB cable
```

---

### Problem: AI Is Too Slow

Try:

```text
Use smaller YOLO model
Lower AI image size
Increase detect-every-N-frames
Use 720p
Use GPU
Disable AI for FPV-only flights
```

---

## 21. Performance Tips

Best performance setup:

```text
FPV: 720p
FPS: 50
FourCC: MJPG
Backend: DirectShow
Buffer: 1
AI: Off
Recording: Off during baseline test
Map: Use only when needed
Power mode: Best performance
Storage: Internal SSD
```

Windows settings:

```text
Use High Performance or Best Performance power mode.
Plug in the laptop charger.
Close Chrome tabs.
Close OBS unless needed.
Close Discord streams.
Avoid screen recording while testing FPV smoothness.
```

---

## 22. GitHub Publishing Guide

### 22.1 Create Local Git Repository

```powershell
cd "C:\Users\phone\Documents\Drone-Dashboard-v112"

git init
```

---

### 22.2 Add Files

```powershell
git add README.md
git add requirements.txt
git add drone_dashboard_v112_smooth_fpv_gps_fix.py
git add .gitignore
```

Or add all:

```powershell
git add .
```

---

### 22.3 First Commit

```powershell
git commit -m "Initial release of Python Drone Dashboard v112"
```

---

### 22.4 Create GitHub Repository

On GitHub:

```text
1. Click New repository
2. Repository name: Drone-Dashboard-v112
3. Description: Python FPV telemetry dashboard for UAV research
4. Choose Public or Private
5. Do not add README if you already made one locally
6. Create repository
```

---

### 22.5 Connect Local Repo to GitHub

Replace the URL with your own repository URL:

```powershell
git remote add origin https://github.com/YOUR_USERNAME/Drone-Dashboard-v112.git
```

---

### 22.6 Push to GitHub

```powershell
git branch -M main

git push -u origin main
```

---

### 22.7 Suggested GitHub Description

```text
Python FPV telemetry dashboard for UAV research with MAVLink, MSP, TX15 USB monitoring, GPS map tracking, AI overlay, dataset recording, and fuel estimation.
```

---

### 22.8 Suggested GitHub Topics

```text
drone
uav
pixhawk
ardupilot
mavlink
betaflight
inav
fpv
opencv
pyqt5
python
robotics
ground-control-station
telemetry
computer-vision
```

---

## 23. Suggested `.gitignore`

Create a file named:

```text
.gitignore
```

Use this content:

```gitignore
# Python cache
__pycache__/
*.pyc
*.pyo
*.pyd

# Virtual environment
.venv/
venv/
env/

# IDE
.vscode/
.idea/

# Logs
*.log

# Dataset recordings
recordings/
records/
dataset/
datasets/
*.avi
*.mp4
*.mov
*.mkv
*.csv

# AI model weights
*.pt
*.onnx
*.engine

# OS files
.DS_Store
Thumbs.db
Desktop.ini

# Temporary files
*.tmp
*.bak
```

Important: Do not upload huge flight recordings, AI model weights, or raw datasets directly to GitHub unless you use Git LFS.

---

## 24. Version Notes

### v100

Original full Python dashboard concept:

```text
MAVLink
MSP
TX15 USB
FPV
AI overlay
dataset recorder
map waypoint planning
fuel estimator
```

### v101

Optimized FPV display path:

```text
reduced frame-copy overhead
reduced UI refresh overhead
improved OpenCV capture tuning
smoother FPV path
```

### v102

Added:

```text
50 FPS option
GPS map update improvements
heading arrow marker
reduced map update spam
```

### v112

Current GitHub release target:

```text
more robust GPS map
Leaflet/OSM online map
offline fallback radar map
drone heading arrow marker
reduced small recurring FPV freeze
50 FPS recommended mode
smoother FPV testing setup
```

---

## 25. Disclaimer

This software is experimental.

It is provided for:

```text
research
education
robotics development
FPV testing
telemetry visualization
dataset collection
```

It is not a certified aviation product, not a safety-critical flight system, and not a replacement for proper pilot control.

The user is responsible for:

```text
safe testing
local drone laws
flight permission
hardware safety
battery safety
propeller safety
manual override
field risk assessment
```

Always test with propellers removed before connecting real motors or flying.

---

## License

Choose a license before publishing publicly.

Recommended simple option:

```text
MIT License
```

If you use third-party packages, respect their own licenses:

```text
PyQt5
PyQtWebEngine
OpenCV
NumPy
pymavlink
pygame
Ultralytics
pyserial
```

---

## Author

Developed by **Phone Myat Thu** as part of an experimental UAV / autonomous drone research and ground-control dashboard project.
