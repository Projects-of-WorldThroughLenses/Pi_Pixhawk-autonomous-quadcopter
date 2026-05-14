# Drone Dashboard v8 — FPV, MAVLink Telemetry, GPS Map, AHRS Gyro Ball and Flight-Time Estimator

A professional PyQt5-based drone monitoring dashboard designed for Pixhawk / ArduPilot / PX4-style telemetry systems. The dashboard combines low-latency FPV video monitoring, MAVLink telemetry, an artificial horizon / gyro ball, live GPS map tracking, RC channel monitoring, flight timer automation, and an engineering-style battery / flight-time estimator in one operator interface.

> **Important safety note:**  
> This software is designed as a monitoring, recording, and estimation dashboard. It does **not** arm, disarm, take off, land, or directly command the aircraft. Flight control authority remains with the pilot, transmitter, and flight controller.

---

## 1. Project Overview

This project was developed as a full-screen drone dashboard for monitoring a Pixhawk-based UAV system from a laptop or companion-computer environment. It is suitable for prototype testing, telemetry observation, FPV monitoring, and pre-flight/endurance estimation.

The software is intended for setups such as:

- Pixhawk 6C / Pixhawk-compatible flight controller
- ArduPilot or PX4-based MAVLink telemetry
- USB serial MAVLink connection or UDP MAVLink forwarding
- FPV camera / USB capture card / webcam video input
- Laptop operator dashboard, especially high-resolution displays
- Optional GPS map display using PyQtWebEngine and Leaflet/OpenStreetMap

---

## 2. Main Capabilities

### 2.1 MAVLink Telemetry Monitoring

The dashboard reads telemetry from Pixhawk / MAVLink-compatible flight controllers and displays live flight data.

Supported telemetry display includes:

- Flight mode
- Armed / disarmed state
- Roll, pitch, yaw and heading
- GPS fix type and satellite count
- GPS altitude
- Relative altitude
- Lidar altitude, when available
- Optical-flow altitude / quality, when available
- Ground speed
- Climb / descent rate
- Battery voltage
- Battery current
- Throttle percentage
- RC transmitter channel values, channels 1–14

The program actively requests common MAVLink streams so that attitude, GPS, RC, battery and status messages are available even when the autopilot is not already streaming them.

---

### 2.2 Direct Gyro Ball / AHRS Display

The software includes a custom artificial horizon / gyro ball panel for real-time flight attitude monitoring.

Features include:

- Roll and pitch display
- Heading tape
- Full 180-degree roll scale
- Direct gyro update from MAVLink attitude packets
- Live roll, pitch, heading and gyro refresh readout
- Dark mode and light mode support

The gyro display updates directly when attitude telemetry arrives, reducing artificial smoothing delay and making the display feel more responsive.

---

### 2.3 FPV Live Video Feed

The dashboard includes an FPV video panel using OpenCV.

Video features:

- OpenCV camera / capture-card input
- Source selection from camera index or custom path
- Three-dot overlay control panel
- Start / stop video
- Scan video sources
- Screenshot capture
- Video recording
- Save folder selection
- MJPG mode option for USB capture devices
- Measured FPS display

Typical video sources:

```text
0
1
2
/dev/video0
rtsp://...
```

For Windows USB capture cards, source `0` or `1` is usually the first option to try.

---

### 2.4 Live GPS Map

The dashboard includes a live map panel using PyQtWebEngine and Leaflet.

Map features:

- OpenStreetMap / Leaflet map display
- Live drone position marker
- Drone-shaped heading marker
- Heading-based marker rotation
- Path trail display
- GPS latitude / longitude display
- Heading, satellites, speed and altitude display

> The map uses online map tiles. Internet access is required for map tiles to load unless a local/offline map tile system is added later.

---

### 2.5 Flight Timer

The software includes both automatic and manual flight timing.

Timer features:

- Manual **ARMING STARTED** button
- Automatic timer start when MAVLink reports the drone is armed
- Timer stop when disarmed
- Reset timer button
- Live flight-time display

This is useful for bench testing, hover testing and recording test sessions.

---

### 2.6 Battery and Flight-Time Estimator

The dashboard includes a detailed engineering-style flight-time and fuel estimator.

Estimator inputs include:

#### Environment

- Altitude above sea level
- Temperature
- Relative humidity

#### Battery

- Cell count
- Cell voltage
- Capacity in mAh
- Usable capacity percentage
- Reserve capacity percentage

#### Drone and Payload

- Drone mass
- Payload mass
- Number of rotors
- Avionics power
- Avionics current

#### Motor and Propeller

- Motor KV
- Propeller diameter
- Propeller pitch
- Blade count
- Loaded RPM factor
- Figure of merit
- Measured maximum thrust
- Maximum current per motor
- ESC current rating

#### Mission and Live Estimation

- Maximum throttle
- Cruise throttle
- Telemetry current correction / bias
- Live current from MAVLink when available
- Fallback throttle-cubic current estimate when telemetry current is unavailable

Estimator outputs include:

- Air density estimate
- No-load RPM
- Loaded RPM estimate
- Static current estimate
- Hover current estimate
- Cruise current estimate
- Maximum total current estimate
- Hover burn rate
- Cruise burn rate
- Estimated hover flight time
- Estimated cruise flight time
- Time until reserve
- Thrust margin warnings
- ESC current caution warnings
- Low endurance warnings

---

### 2.7 Boot Screen and Display Modes

The application includes a boot screen before opening the dashboard.

Boot options:

- Dark mode
- Light mode
- Fullscreen borderless mode
- Windowed mode
- Loading progress bar

Windowed mode is recommended if the GPU or QtWebEngine produces flicker during fullscreen / Alt+Tab usage.

---

## 3. Requirements

### 3.1 Python Version

Recommended:

```text
Python 3.10 or newer
```

Tested target environment:

```text
Windows laptop / desktop
Python virtual environment
PyQt5 GUI environment
```

The software may also run on Linux / Raspberry Pi with suitable display support and installed Qt dependencies.

---

### 3.2 Python Packages

Required Python packages:

```text
PyQt5
PyQtWebEngine
opencv-python
numpy
pymavlink
pyserial
```

These are included in the provided requirements file.

---

## 4. Installation on Windows

Open PowerShell in the project folder.

Example project path:

```powershell
cd "C:\Users\phone\Documents\Drone_Flight_Estimator"
```

Create a virtual environment:

```powershell
python -m venv dashboard_env
```

Activate the virtual environment:

```powershell
.\dashboard_env\Scripts\activate
```

Upgrade pip:

```powershell
python -m pip install --upgrade pip
```

Install packages:

```powershell
pip install -r requirements.txt
```

If you do not have a requirements file, install manually:

```powershell
pip install PyQt5 PyQtWebEngine opencv-python numpy pymavlink pyserial
```

Run the dashboard:

```powershell
python drone_dashboard_v8_direct_gyro_comport_rate.py
```

---

## 5. Installation on Linux / Raspberry Pi

Create a virtual environment:

```bash
python3 -m venv dashboard_env --system-site-packages
```

Activate it:

```bash
source dashboard_env/bin/activate
```

Upgrade pip:

```bash
python -m pip install --upgrade pip
```

Install packages:

```bash
pip install -r requirements.txt
```

Or install manually:

```bash
pip install PyQt5 PyQtWebEngine opencv-python numpy pymavlink pyserial
```

Run:

```bash
python drone_dashboard_v8_direct_gyro_comport_rate.py
```

For Raspberry Pi, some PyQt5 / QtWebEngine packages may be easier to install through `apt` depending on the OS image:

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-pyqt5.qtwebengine
```

Then install the remaining packages in the virtual environment.

---

## 6. How to Start the Program

### 6.1 Start from PowerShell

```powershell
cd "C:\Users\phone\Documents\Drone_Flight_Estimator"
.\dashboard_env\Scripts\activate
python drone_dashboard_v8_direct_gyro_comport_rate.py
```

### 6.2 Choose Startup Options

When the boot window opens:

1. Select **Dark mode** or **Light mode**
2. Select **Fullscreen borderless** or **Windowed mode**
3. Press **Boot Dashboard**
4. Wait for the loading bar to finish

Recommended first test:

```text
Dark mode + Windowed mode
```

After confirming everything works, use fullscreen borderless mode.

---

## 7. MAVLink Connection Guide

### 7.1 Direct USB Connection

Connect the Pixhawk to the computer using USB.

Common Windows connection strings:

```text
COM3
COM4
COM9
COM10
```

Common Linux / Raspberry Pi connection strings:

```text
/dev/ttyACM0
/dev/ttyACM1
/dev/ttyUSB0
/dev/ttyUSB1
```

Typical baud rates:

```text
57600
115200
```

For direct USB on Windows:

1. Close Mission Planner, QGroundControl or any other software using the COM port.
2. Open the dashboard.
3. Select the COM port.
4. Set baud rate, usually `57600` or `115200`.
5. Press **Connect MAVLink**.

> A serial COM port can normally be used by only one program at a time. If Mission Planner is already connected to COM9, this dashboard cannot connect to COM9 directly.

---

### 7.2 Using Mission Planner and Dashboard at the Same Time

To use Mission Planner and this dashboard together, do not connect both directly to the same COM port.

Use MAVLink forwarding through MAVProxy.

Example:

```powershell
mavproxy.py --master=COM9 --baudrate 57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

Then connect:

```text
Dashboard: udp:127.0.0.1:14550
Mission Planner: UDP 14551
```

This avoids serial-port permission conflicts.

---

## 8. FPV Video Setup

### 8.1 Basic USB Camera / Capture Card

Start with video source:

```text
0
```

If it does not open, try:

```text
1
2
3
```

Press:

```text
Scan Sources
```

Then press:

```text
Start Video
```

### 8.2 Recording and Screenshots

Use the FPV three-dot menu to access:

- Screenshot
- Start Recording
- Stop Recording
- Save folder

Default save folder is usually:

```text
C:\Users\<username>\Videos\DroneDashboard
```

---

## 9. Recommended Test Procedure

Before using near a real aircraft:

1. Remove propellers.
2. Power the Pixhawk safely.
3. Connect USB MAVLink.
4. Start the dashboard.
5. Confirm heartbeat / flight mode is detected.
6. Move the aircraft gently by hand and confirm:
   - Roll changes
   - Pitch changes
   - Heading changes
   - Gyro ball responds
7. Confirm RC channel values change when moving transmitter sticks.
8. Confirm battery voltage is displayed.
9. Confirm GPS fix and satellite count when outdoors.
10. Start FPV video and verify recording path.

Only proceed to flight testing after bench validation.

---

## 10. Troubleshooting

### 10.1 Permission Denied on COM Port

Example error:

```text
Permission denied: COM9
```

Cause:

- Another program is already using the COM port.

Fix:

- Close Mission Planner / QGroundControl / serial monitor.
- Disconnect and reconnect Pixhawk USB.
- Reopen the dashboard.
- Or use MAVProxy UDP forwarding.

---

### 10.2 Dashboard Connects but Gyro Does Not Move

Possible causes:

- MAVLink attitude messages are not streaming.
- Wrong baud rate.
- Wrong COM port.
- Flight controller is connected to another program.
- Telemetry stream rate is too low.

Fix:

- Confirm the status label says telemetry is connected.
- Try another baud rate: `57600` or `115200`.
- Close other GCS software.
- Use direct COM first before using UDP.
- Restart the dashboard after disconnecting the Pixhawk.

---

### 10.3 GPS Map Does Not Load

Possible causes:

- No internet connection.
- PyQtWebEngine is not installed.
- Map tile provider cannot be reached.
- GPU / QtWebEngine issue in fullscreen mode.

Fix:

```powershell
pip install PyQtWebEngine
```

Then try:

```text
Windowed mode
```

---

### 10.4 FPV Video Does Not Open

Try different video source values:

```text
0
1
2
3
```

Also check:

- USB capture card is connected.
- Camera is not being used by another app.
- MJPG mode is enabled.
- Correct Windows camera permission is allowed.

---

### 10.5 Fullscreen Flicker or Alt+Tab Flashing

This can happen on some systems using PyQtWebEngine, NVIDIA/Intel hybrid graphics and borderless fullscreen.

Fix:

- Use **Windowed mode** from the boot screen.
- Update GPU drivers.
- Close other GPU-heavy apps.
- Avoid switching repeatedly between Mission Planner and the dashboard in fullscreen mode.

---

## 11. Suggested Repository Structure

```text
Drone_Flight_Estimator/
│
├── drone_dashboard_v8_direct_gyro_comport_rate.py
├── requirements.txt
├── README.md
│
├── screenshots/
│   ├── dashboard_dark_mode.png
│   ├── dashboard_light_mode.png
│   └── fpv_overlay.png
│
├── docs/
│   ├── hardware_block_diagram.png
│   └── telemetry_setup_notes.md
│
└── recordings/
    └── README.md
```

Recommended GitHub image references:

```markdown
![Dashboard Dark Mode](screenshots/dashboard_dark_mode.png)
![Hardware Block Diagram](docs/hardware_block_diagram.png)
```

---

## 12. Safety and Limitations

This dashboard is not a replacement for a certified ground control station or flight controller software.

Current limitations:

- Map requires internet access for online map tiles.
- FPV latency depends on camera, capture card and USB bandwidth.
- Flight-time estimates are theoretical and must be validated by real current measurements.
- MAVLink serial ports cannot be shared directly by multiple programs.
- Telemetry quality depends on autopilot stream rates and connection stability.

Always test with propellers removed before live aircraft testing.

---

## 13. Technical Notes

The program uses:

- **PyQt5** for GUI layout and dashboard interface
- **PyQtWebEngine** for the embedded GPS map
- **OpenCV** for video capture, screenshots and recording
- **NumPy** for image handling
- **pymavlink** for MAVLink communication
- **pyserial** for COM-port detection

The dashboard uses separate worker threads for video capture and telemetry reading to prevent the main UI from freezing.

---

## 14. Author

Developed by **Phone Myat Thu** as part of an autonomous drone / Pixhawk companion-computer software workflow.

Project focus:

```text
FPV monitoring + MAVLink telemetry + live flight estimation + professional drone dashboard interface
```
