#!/usr/bin/env python3
"""
Medical Delivery Drone GCS v50 Light  —  PyQt5 Edition
====================================================
Major revision from v10.4:
  • FIXED (v11.4): GPS and Dead Reckoning are both fully local Pi
    go-to controllers. Pixhawk is used only for telemetry + stabilised
    flight in the TX-selected mode.
  • FIXED: Operator Abort now means "stop this go-to here" while keeping
    Pi keyboard control available immediately if the override switch
    remains ON. No forced TX hand-back is done on operator abort.
  • FIXED (v11.3): Camera Module 3 NoIR colour path corrected
    for this setup. The preview now uses the sensor's native bytes by
    default, with an optional R/B swap toggle if a specific Pi build
    still shows reversed colours.
  • FIXED: heading=0 (north) no longer kills GPS autopilot
  • FIXED: GUI no longer freezes during override release
  • FIXED: import inside paintEvent moved to module level
  • IMPROVED: Cleaner, more polished dark UI
  • IMPROVED: Colour-check swatches shown for first 5 seconds
  • IMPROVED: Better code structure and comments

CORE PRINCIPLE — Pi is a joystick, not a flight controller
  The Pi NEVER calls set_mode().  The Pixhawk's flight mode is
  always owned by the TX switch.  Pi only nudges ch1/ch2/ch4
  (roll/pitch/yaw) as if it were the right and left sticks of a
  transmitter.  Motor mixing, stabilisation, altitude hold, RTL,
  Loiter — all of that stays in Pixhawk / ArduPilot as normal.

OVERRIDE SWITCH (TX channel 9 by default)
  • Switch LOW  (< 1300 PWM) → Pi sends NO_OVERRIDE on all channels.
    TX has 100% control.  Pi is invisible to Pixhawk.
  • Switch HIGH (> 1700 PWM) → Pi overrides ch1 / ch2 / ch4 only.
    TX still controls ch3 (throttle) and flight mode.

GPS + DEAD RECKONING — local Pi go-to logic, NO GUIDED mode
  The Pi reads telemetry from Pixhawk, computes bearing / yaw error /
  distance remaining, then sends only ch1/ch2/ch4 stick-style RC
  overrides. Pixhawk stays in the TX-selected mode and handles
  stabilisation, altitude hold, motor mixing, etc.

ABORT + RETURN BEHAVIOUR (v50 Light)
  Operator Abort still stops only the current GPS/DR go-to. If the override
  switch is still ON, the Pi remains available for immediate keyboard
  control and simply returns ch1/ch2/ch4 to neutral.

RETURN TO START (v50 Light)
  Optional auto-return can be enabled independently for GPS and Dead
  Reckoning. The Pi records the mission start position locally and, after
  reaching the outbound target, computes a return leg back to that start
  position using only local telemetry + RC stick simulation.
"""
import os
import sys
import time
import math
import threading
from datetime import datetime

import numpy as np
import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QLineEdit, QSpinBox,
    QProgressBar, QRadioButton, QButtonGroup,
    QGroupBox, QFrame, QSizePolicy, QMessageBox,
)
from PyQt5.QtCore import (
    Qt, QTimer, QUrl, QRectF,
    pyqtSignal, QObject, QThread,
)
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QFont, QBrush,
    QPalette, QLinearGradient, QImage, QPixmap,
    QConicalGradient,
)
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage

# ─────────────────────────────────────────────────────────────
#  CONFIG  — edit these to match your setup
# ─────────────────────────────────────────────────────────────
PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_29004B001051333235363832-if00"

# Override switch channel
OVERRIDE_SWITCH_CH = 9          # TX channel number (1-indexed)
SWITCH_LOW_PWM     = 1300       # below this = switch OFF
SWITCH_HIGH_PWM    = 1700       # above this = switch ON

# PWM constants
NO_OVERRIDE  = 65535
NEUTRAL      = 1500
MIN_PWM      = 1100
MAX_PWM      = 1900
STEP         = 10
SEND_HZ      = 20
UI_HZ        = 10
MAX_PWM_RATE = 400              # max PWM change/sec for smooth stick

# Face safety
AUTO_CLOSE_RATIO  = 0.18
AUTO_SAFE_RATIO   = 0.15
PITCH_BACKOFF_PWM = 1200
PITCH_NEUTRAL_PWM = 1500

# GPS autopilot (stick-nudge)
WAYPOINT_RADIUS_M = 3.0
AP_YAW_KP         = 2.0        # deg error → extra PWM
YAW_DEADBAND_DEG  = 8.0
AP_PITCH_FWD      = 1600       # forward pitch PWM toward target

# Dead reckoning
DR_HZ              = 10
CRUISE_SPEED_MPS   = 2.0
DR_MIN_GS_VALID    = 0.2
YAW_ALIGN_SEC      = 1.0
DR_PROGRESS_RADIUS_M = WAYPOINT_RADIUS_M

# Camera
TARGET_FPS     = 60
DETECT_EVERY   = 2
DETECT_SCALE   = 0.30
MIN_FACE_SMALL = (60, 60)
BOX_TIMEOUT    = 0.6
SMOOTH_BOX     = 0.55

# Camera colour path (v11.3)
# Pi Camera Module 3 NoIR on some Pi5 builds reports "BGR888" but the
# captured bytes are already in the order Qt expects for RGB888 preview.
# That means an extra cv2 BGR→RGB conversion can create the exact symptom:
# blue face / orange shirt.
#
# Default for this file: DO NOT swap channels.
# If your own setup still shows reversed colours, set CAMERA_SWAP_RB = True.
CAMERA_NATIVE_FORMAT = "BGR888"
CAMERA_SWAP_RB      = False
SHOW_COLOR_SWATCHES = True

# Scene identification / recording
ID_EVERY = 4
HOG_SCALE = 0.50
VEGETATION_GREEN_RATIO = 0.18
RECORD_DIR = os.path.expanduser("~/Videos/drone_gcs_recordings")

# Servo / payload release (direct Raspberry Pi GPIO servo)
# Default signal pin: BCM GPIO18  (physical pin 12)
# Recommended power: external regulated 5V supply + common ground with Pi.
SERVO_GPIO_PIN = 18
SERVO_OPEN_PWM = 1900
SERVO_MID_PWM = 1500
SERVO_CLOSE_PWM = 1100
PAYLOAD_OPEN_SEC = 5.0
PAYLOAD_WAIT_SEC = 5.0

# Map defaults
DEFAULT_MAP_LAT  = 3.1390
DEFAULT_MAP_LON  = 101.6869
DEFAULT_MAP_ZOOM = 16

# ─────────────────────────────────────────────────────────────
#  COLOUR PALETTE
# ─────────────────────────────────────────────────────────────
C_BG       = "#f5f7fb"
C_SURFACE  = "#ffffff"
C_CARD     = "#eef3f9"
C_BORDER   = "#cfd8e3"
C_ACCENT   = "#2f7de1"
C_GREEN    = "#2e9b45"
C_YELLOW   = "#b7791f"
C_RED      = "#d64545"
C_CYAN     = "#1f8f83"
C_TEXT     = "#1f2937"
C_DIM      = "#5f6b7a"
C_ORANGE   = "#d97706"

# ─────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────
def clamp(v):
    return max(MIN_PWM, min(MAX_PWM, int(v)))


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1, lon1, lat2, lon2):
    la1, la2 = math.radians(lat1), math.radians(lat2)
    dlo = math.radians(lon2 - lon1)
    x = math.sin(dlo) * math.cos(la2)
    y = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlo)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def gps_offset(lat, lon, bearing_deg, dist_m):
    R = 6_371_000.0
    b = math.radians(bearing_deg)
    la = math.radians(lat)
    lo = math.radians(lon)
    la2 = math.asin(
        math.sin(la) * math.cos(dist_m / R)
        + math.cos(la) * math.sin(dist_m / R) * math.cos(b)
    )
    lo2 = lo + math.atan2(
        math.sin(b) * math.sin(dist_m / R) * math.cos(la),
        math.cos(dist_m / R) - math.sin(la) * math.sin(la2),
    )
    return math.degrees(la2), math.degrees(lo2)


def wrap180(a):
    return (a + 180) % 360 - 180


# ─────────────────────────────────────────────────────────────
#  LIGHTWEIGHT LOCAL IDENTIFICATION
#
#  No internet/model download is required for this script to run.
#  It uses local OpenCV detectors and simple scene heuristics so the
#  Pi can label what it sees and record annotated video locally.
# ─────────────────────────────────────────────────────────────
def _find_haar_dir():
    candidates = [
        "/usr/share/opencv4/haarcascades",
        "/usr/share/opencv/haarcascades",
        "/usr/local/share/opencv4/haarcascades",
    ]
    xml = "haarcascade_frontalface_default.xml"
    for d in candidates:
        if os.path.isfile(os.path.join(d, xml)):
            return d
    for root_dir in ["/usr/share", "/usr/local/share"]:
        for dp, _, files in os.walk(root_dir):
            if xml in files:
                return dp
    return None


def _load_cascade(*names):
    base = _find_haar_dir()
    if base:
        for name in names:
            p = os.path.join(base, name)
            if os.path.isfile(p):
                c = cv2.CascadeClassifier(p)
                if not c.empty():
                    return c
    for root in [os.getcwd(), os.path.dirname(os.path.abspath(__file__))]:
        for name in names:
            p = os.path.join(root, name)
            if os.path.isfile(p):
                c = cv2.CascadeClassifier(p)
                if not c.empty():
                    return c
    return None


face_cascade = _load_cascade("haarcascade_frontalface_default.xml")
upperbody_cascade = _load_cascade("haarcascade_upperbody.xml")
fullbody_cascade = _load_cascade("haarcascade_fullbody.xml")
cat_cascade = _load_cascade("haarcascade_frontalcatface_extended.xml", "haarcascade_frontalcatface.xml")
car_cascade = _load_cascade("haarcascade_car.xml", "cars.xml")

PERSON_HOG = cv2.HOGDescriptor()
PERSON_HOG.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


def _unique_labels(dets):
    out = []
    seen = set()
    for d in dets:
        lab = d["label"]
        if lab not in seen:
            out.append(lab)
            seen.add(lab)
    return out


def _detect_scene(frame_rgb):
    dets = []
    try:
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape

        def add_boxes(cascade, label, min_size=(60, 60), scale_factor=1.1, min_neighbors=5):
            if cascade is None:
                return
            rects = cascade.detectMultiScale(gray, scale_factor, min_neighbors, minSize=min_size)
            for (x, y, bw, bh) in rects[:4]:
                dets.append({"label": label, "box": (int(x), int(y), int(bw), int(bh))})

        # People detector on a resized image for speed
        small = cv2.resize(frame_bgr, (0, 0), fx=HOG_SCALE, fy=HOG_SCALE)
        rects, _weights = PERSON_HOG.detectMultiScale(small, winStride=(8, 8), padding=(8, 8), scale=1.05)
        for (x, y, bw, bh) in rects[:4]:
            dets.append({
                "label": "person",
                "box": (int(x / HOG_SCALE), int(y / HOG_SCALE), int(bw / HOG_SCALE), int(bh / HOG_SCALE))
            })

        add_boxes(face_cascade, "face")
        add_boxes(upperbody_cascade, "upper-body")
        add_boxes(fullbody_cascade, "body")
        add_boxes(cat_cascade, "animal/cat", min_size=(50, 50))
        add_boxes(car_cascade, "car", min_size=(70, 40))

        # Simple green-scene heuristic for trees / vegetation
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        green_mask = cv2.inRange(hsv, (30, 35, 25), (95, 255, 255))
        green_ratio = float(np.count_nonzero(green_mask)) / float(green_mask.size)
        if green_ratio >= VEGETATION_GREEN_RATIO:
            dets.append({"label": "tree/vegetation", "box": None})

        # Very lightweight vehicle-like heuristic if no explicit car cascade exists
        lower = frame_bgr[h // 2 :, :]
        edges = cv2.Canny(lower, 80, 160)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vehicle_like = False
        for c in cnts[:80]:
            x, y, bw, bh = cv2.boundingRect(c)
            area = bw * bh
            aspect = bw / float(max(1, bh))
            if area > (w * h * 0.02) and 1.2 <= aspect <= 4.5 and bh < h * 0.22:
                vehicle_like = True
                break
        if vehicle_like and car_cascade is None:
            dets.append({"label": "vehicle-like", "box": None})

    except Exception:
        pass

    labels = _unique_labels(dets)
    summary = ", ".join(labels[:6]) if labels else "no major object tagged"
    return dets, summary


def _draw_scene_overlay(frame_rgb, dets, summary, recording=False):
    disp = frame_rgb.copy()
    if SHOW_COLOR_SWATCHES:
        cv2.rectangle(disp, (10, 10), (40, 35), (255, 0, 0), -1)
        cv2.putText(disp, "R", (14, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.rectangle(disp, (50, 10), (80, 35), (0, 255, 0), -1)
        cv2.putText(disp, "G", (54, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        cv2.rectangle(disp, (90, 10), (120, 35), (0, 0, 255), -1)
        cv2.putText(disp, "B", (94, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    for det in dets:
        box = det.get("box")
        label = det["label"]
        if box is None:
            continue
        x, y, bw, bh = box
        cv2.rectangle(disp, (x, y), (x + bw, y + bh), (56, 132, 255), 2)
        cv2.putText(disp, label, (x, max(22, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (20, 20, 20), 3)
        cv2.putText(disp, label, (x, max(22, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)

    footer = f"Detected: {summary}"
    cv2.rectangle(disp, (0, disp.shape[0] - 34), (disp.shape[1], disp.shape[0]), (245, 247, 250), -1)
    cv2.putText(disp, footer[:100], (12, disp.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (50, 60, 70), 2)

    if recording:
        cv2.circle(disp, (disp.shape[1] - 24, 22), 8, (255, 40, 40), -1)
        cv2.putText(disp, "REC", (disp.shape[1] - 68, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 40, 40), 2)

    return disp


def request_streams(m):
    streams = [
        (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20),
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
        (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 20),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),
        (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20),
    ]
    for stream, rate in streams:
        m.mav.request_data_stream_send(
            m.target_system, m.target_component, stream, rate, 1)


# ─────────────────────────────────────────────────────────────
#  AUTOPILOT MODES
# ─────────────────────────────────────────────────────────────
class APMode:
    NONE = "none"
    GPS  = "gps_waypoint"
    DR   = "dead_reckoning"


class DRInputMode:
    BEARING_DIST = "bearing_distance"
    COORDINATES  = "coordinates"
    HEADING      = "heading"


class DirectServoController:
    def __init__(self, gpio_pin):
        self.gpio_pin = int(gpio_pin)
        self.backend = "unavailable"
        self.error = ""
        self._pi = None
        self._gpio = None
        self._pwm = None

        try:
            import pigpio
            pi = pigpio.pi()
            if pi.connected:
                self._pi = pi
                self.backend = "pigpio"
                self._pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
                self._pi.set_servo_pulsewidth(self.gpio_pin, 0)
                return
            self.error = "pigpio daemon not running"
        except Exception as ex:
            self.error = str(ex)

        try:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            self._gpio = GPIO
            self._pwm = GPIO.PWM(self.gpio_pin, 50)
            self._pwm.start(0)
            self.backend = "RPi.GPIO"
            self.error = ""
            return
        except Exception as ex:
            if self.error:
                self.error += f" | RPi.GPIO fallback failed: {ex}"
            else:
                self.error = f"RPi.GPIO fallback failed: {ex}"

    def describe(self):
        if self.backend == "unavailable":
            return f"GPIO{self.gpio_pin} unavailable"
        return f"GPIO{self.gpio_pin} via {self.backend}"

    def set_pulsewidth(self, pulse_us):
        pulse_us = int(max(500, min(2500, pulse_us)))
        if self.backend == "pigpio" and self._pi is not None:
            self._pi.set_servo_pulsewidth(self.gpio_pin, pulse_us)
            return
        if self.backend == "RPi.GPIO" and self._pwm is not None:
            duty = (pulse_us / 20000.0) * 100.0
            self._pwm.ChangeDutyCycle(duty)

    def disable(self):
        if self.backend == "pigpio" and self._pi is not None:
            self._pi.set_servo_pulsewidth(self.gpio_pin, 0)
        elif self.backend == "RPi.GPIO" and self._pwm is not None:
            self._pwm.ChangeDutyCycle(0)

    def close(self):
        try:
            self.disable()
        except Exception:
            pass
        try:
            if self.backend == "pigpio" and self._pi is not None:
                self._pi.stop()
        except Exception:
            pass
        try:
            if self.backend == "RPi.GPIO" and self._pwm is not None:
                self._pwm.stop()
            if self.backend == "RPi.GPIO" and self._gpio is not None:
                self._gpio.cleanup(self.gpio_pin)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────
#  SHARED STATE  (thread-safe via self.lock)
# ─────────────────────────────────────────────────────────────
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()

        # Keyboard stick targets
        self.k_roll  = NEUTRAL
        self.k_pitch = NEUTRAL
        self.k_yaw   = NEUTRAL
        self.manual_backoff = False

        # Smoothed PWM outputs
        self.o_roll  = NEUTRAL
        self.o_pitch = NEUTRAL
        self.o_yaw   = NEUTRAL

        # Manual/mission state
        self.locked     = False
        self.ap_mode       = APMode.NONE
        self.ap_active     = False
        self.ap_status     = "Idle"
        self.ap_target_lat = None
        self.ap_target_lon = None
        self.ap_roll_pwm   = NEUTRAL
        self.ap_pitch_pwm  = NEUTRAL
        self.ap_yaw_pwm    = NEUTRAL
        self.force_tx_release = False

        # Dead reckoning
        self.dr_input_mode     = DRInputMode.BEARING_DIST
        self.dr_target_bearing = 0.0
        self.dr_target_dist_m  = 0.0
        self.dr_remaining_m    = 0.0
        self.dr_travelled_m    = 0.0
        self.dr_start_lat      = None
        self.dr_start_lon      = None
        self.dr_est_lat        = None
        self.dr_est_lon        = None
        self.dr_target_lat     = None
        self.dr_target_lon     = None
        self.dr_phase          = "idle"
        self.dr_align_since    = None
        self.dr_last_update    = None

        # Mission memory / auto return
        self.mission_start_lat   = None
        self.mission_start_lon   = None
        self.auto_return_enabled = True
        self.returning_to_start  = False
        self.return_context      = ""
        self.payload_stage       = "idle"
        self.payload_stage_deadline = 0.0

        # Video / identification
        self.latest_detections   = []
        self.detection_summary   = "no major object tagged"
        self.recording           = False
        self.record_path         = ""
        self.record_started_at   = 0.0
        self.record_error        = ""

        # Servo release (direct Raspberry Pi GPIO control)
        self.servo_target_pwm = SERVO_CLOSE_PWM
        self.servo_status     = "CLOSED"

        # Override switch (from RC_CHANNELS telemetry)
        self.override_switch_pwm = 0
        self.override_active     = False

        # Telemetry dict
        self.tel = {
            "mode": "UNKNOWN", "armed": False,
            "bat_v": None, "bat_a": None,
            "roll_deg": None, "pitch_deg": None, "yaw_deg": None,
            "heading_deg": None, "rel_alt_m": None, "alt_msl_m": None,
            "gs_mps": None, "climb_mps": None,
            "gps_fix": None, "gps_sats": None,
            "lat": None, "lon": None,
            "ax": None, "ay": None, "az": None,
            "gx": None, "gy": None, "gz": None,
            "thr_pwm": None, "sw_pwm": None,
        }

        self.latest_frame = None
        self.rc_channels  = {}
        self.stop = False


# ─────────────────────────────────────────────────────────────
#  CAMERA THREAD
#
#  COLOUR FIX (v11.3):
#  v11.1 forced a BGR → RGB conversion on every frame.  On this
#  user's Pi Camera Module 3 NoIR setup, that produced the exact
#  symptom being reported: blue/cyan skin tones and blue fabric
#  turning orange/red.
#
#  v11.2 therefore uses the camera's native bytes directly by
#  default and keeps the R/B swap as a simple config toggle:
#      CAMERA_SWAP_RB = False   → use frame as-is  (default)
#      CAMERA_SWAP_RB = True    → apply cv2 BGR→RGB swap
#
#  The top-left R/G/B swatches remain for quick verification.
# ─────────────────────────────────────────────────────────────
def camera_thread_fn(S: SharedState):
    os.makedirs(RECORD_DIR, exist_ok=True)
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": (1280, 720), "format": CAMERA_NATIVE_FORMAT},
        controls={"FrameRate": TARGET_FPS},
    )
    picam2.configure(cfg)
    picam2.start()

    writer = None
    writer_path = ""
    detections = []
    summary = "starting…"
    frame_count = 0

    try:
        while True:
            with S.lock:
                if S.stop:
                    break
                rec_on = S.recording

            raw_frame = picam2.capture_array()
            if CAMERA_SWAP_RB:
                frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
            else:
                frame = raw_frame.copy()

            if frame_count % ID_EVERY == 0:
                detections, summary = _detect_scene(frame)

            disp = _draw_scene_overlay(frame, detections, summary, recording=rec_on)

            if rec_on and writer is None:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                writer_path = os.path.join(RECORD_DIR, f"gcs_capture_{ts}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(writer_path, fourcc, 20.0, (disp.shape[1], disp.shape[0]))
                with S.lock:
                    S.record_path = writer_path
                    S.record_started_at = time.time()
                    if not writer.isOpened():
                        S.record_error = "Could not open video writer"
                        S.recording = False
                        writer.release()
                        writer = None
                    else:
                        S.record_error = ""
            elif (not rec_on) and writer is not None:
                writer.release()
                writer = None

            if writer is not None:
                try:
                    writer.write(cv2.cvtColor(disp, cv2.COLOR_RGB2BGR))
                except Exception:
                    with S.lock:
                        S.record_error = "Recording write failed"
                        S.recording = False
                    writer.release()
                    writer = None

            with S.lock:
                S.latest_frame = disp
                S.latest_detections = [d["label"] for d in detections]
                S.detection_summary = summary
                S.locked = False

            frame_count += 1
    finally:
        if writer is not None:
            writer.release()
        picam2.stop()


# ─────────────────────────────────────────────────────────────
#  MAVLINK WORKER  (QThread)
# ─────────────────────────────────────────────────────────────
class MavWorker(QObject):
    connected = pyqtSignal(str)
    error     = pyqtSignal(str)

    def __init__(self, S: SharedState):
        super().__init__()
        self.S = S
        self.m = None
        self._running = True

    def run(self):
        try:
            m  = mavutil.mavlink_connection(PORT)
            hb = m.wait_heartbeat(timeout=15)
            request_streams(m)
            self.m = m
            self.connected.emit(
                f"Connected  |  sys={m.target_system}  |  {mavutil.mode_string_v10(hb)}")
        except Exception as ex:
            self.error.emit(str(ex))
            return

        while self._running:
            with self.S.lock:
                if self.S.stop:
                    break

            for _ in range(250):
                msg = self.m.recv_match(blocking=False)
                if not msg:
                    break
                t = msg.get_type()

                with self.S.lock:
                    tel = self.S.tel

                    if t == "HEARTBEAT":
                        tel["mode"]  = mavutil.mode_string_v10(msg)
                        tel["armed"] = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                    elif t == "SYS_STATUS":
                        if msg.voltage_battery not in (None, 0xFFFF):
                            tel["bat_v"] = msg.voltage_battery / 1000
                        if msg.current_battery not in (None, -1):
                            tel["bat_a"] = msg.current_battery / 100

                    elif t == "ATTITUDE":
                        tel["roll_deg"]  = math.degrees(msg.roll)
                        tel["pitch_deg"] = math.degrees(msg.pitch)
                        tel["yaw_deg"]   = math.degrees(msg.yaw)

                    elif t == "VFR_HUD":
                        tel["heading_deg"] = msg.heading
                        tel["gs_mps"]      = msg.groundspeed
                        tel["climb_mps"]   = msg.climb

                    elif t == "GLOBAL_POSITION_INT":
                        tel["rel_alt_m"] = msg.relative_alt / 1000
                        tel["alt_msl_m"] = msg.alt / 1000

                    elif t == "GPS_RAW_INT":
                        tel["gps_fix"]  = msg.fix_type
                        tel["gps_sats"] = msg.satellites_visible
                        tel["lat"]      = msg.lat / 1e7
                        tel["lon"]      = msg.lon / 1e7

                    elif t == "RAW_IMU":
                        g = 9.80665 / 1000
                        tel["ax"] = msg.xacc * g
                        tel["ay"] = msg.yacc * g
                        tel["az"] = msg.zacc * g
                        tel["gx"] = msg.xgyro / 1000
                        tel["gy"] = msg.ygyro / 1000
                        tel["gz"] = msg.zgyro / 1000

                    elif t == "RC_CHANNELS":
                        raw3 = getattr(msg, "chan3_raw", 0)
                        tel["thr_pwm"] = raw3 if raw3 else None

                        rc_vals = {}
                        for ch_n in range(1, 19):
                            v = getattr(msg, f"chan{ch_n}_raw", 0) or 0
                            if v > 0:
                                rc_vals[ch_n] = v
                        self.S.rc_channels = rc_vals

                        sw_ch = OVERRIDE_SWITCH_CH
                        sw = rc_vals.get(sw_ch, 0)
                        tel["sw_pwm"] = sw
                        self.S.override_switch_pwm = sw

                        if sw > SWITCH_HIGH_PWM:
                            self.S.override_active = True
                        elif 0 < sw < SWITCH_LOW_PWM:
                            self.S.override_active = False

            time.sleep(0.01)

    def stop(self):
        self._running = False


# ─────────────────────────────────────────────────────────────
#  ARTIFICIAL HORIZON  (custom painted widget)
# ─────────────────────────────────────────────────────────────
class HorizonWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll_deg  = 0.0
        self.pitch_deg = 0.0
        self._valid    = False
        self.setMinimumSize(340, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_attitude(self, roll, pitch):
        if roll is None or pitch is None:
            self._valid = False
        else:
            self.roll_deg  = roll
            self.pitch_deg = pitch
            self._valid    = True
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2

        # Background
        p.fillRect(0, 0, w, h, QColor(C_CARD))

        if not self._valid:
            p.setPen(QColor(C_DIM))
            p.setFont(QFont("Consolas", 11))
            p.drawText(self.rect(), Qt.AlignCenter, "Waiting for attitude data…")
            return

        pitch_px = self.pitch_deg * (h / 45.0)
        sky_h = int(cy - pitch_px)

        # Rotated sky + ground
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll_deg)
        p.translate(-cx, -cy)

        sky = QLinearGradient(0, 0, 0, sky_h)
        sky.setColorAt(0, QColor("#06101f"))
        sky.setColorAt(1, QColor("#1a4a8a"))
        p.fillRect(-w, -h, w * 3, sky_h + int(cy), QBrush(sky))

        gnd = QLinearGradient(0, sky_h, 0, h)
        gnd.setColorAt(0, QColor("#5c3d1e"))
        gnd.setColorAt(1, QColor("#2a1a08"))
        p.fillRect(-w, sky_h, w * 3, h * 3, QBrush(gnd))

        # Horizon line
        p.setPen(QPen(QColor(C_TEXT), 2))
        p.drawLine(-w, int(cy - pitch_px), w * 2, int(cy - pitch_px))

        # Pitch ladder
        p.setPen(QPen(QColor(C_TEXT), 1))
        p.setFont(QFont("Consolas", 8))
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y_line = int(cy - pitch_px - deg * (h / 45.0))
            half = (w // 6) if deg % 20 == 0 else (w // 10)
            p.drawLine(int(cx - half), y_line, int(cx + half), y_line)
            p.drawText(int(cx + half + 4), y_line + 4, f"{abs(deg)}")
        p.restore()

        # Aircraft reference (fixed)
        p.setPen(QPen(QColor(C_YELLOW), 3))
        p.drawLine(int(cx - 50), int(cy), int(cx - 15), int(cy))
        p.drawLine(int(cx + 15), int(cy), int(cx + 50), int(cy))
        p.drawLine(int(cx), int(cy - 8), int(cx), int(cy + 8))

        # Roll arc
        arc_r = min(w, h) * 0.42
        rect = QRectF(cx - arc_r, cy - arc_r, arc_r * 2, arc_r * 2)
        p.setPen(QPen(QColor(C_ACCENT), 2))
        p.drawArc(rect, int((90 - 60) * 16), int(120 * 16))

        # Roll pointer
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll_deg)
        p.setPen(QPen(QColor(C_YELLOW), 3))
        p.drawLine(0, int(-arc_r + 2), 0, int(-arc_r + 14))
        p.restore()

        # Text overlay
        p.setPen(QColor(C_TEXT))
        p.setFont(QFont("Consolas", 10, QFont.Bold))
        p.drawText(8, 16, f"Roll  {self.roll_deg:+.1f}°")
        p.drawText(8, 32, f"Pitch {self.pitch_deg:+.1f}°")


# ─────────────────────────────────────────────────────────────
#  MAP PAGE  (intercepts gcs:// URLs from Leaflet clicks)
# ─────────────────────────────────────────────────────────────
class MapPage(QWebEnginePage):
    waypoint_set = pyqtSignal(float, float)

    def acceptNavigationRequest(self, url, nav_type, is_main):
        s = url.toString()
        if s.startswith("gcs://wp/"):
            try:
                _, _, coords = s.partition("gcs://wp/")
                lat_s, lon_s = coords.split("/")
                self.waypoint_set.emit(float(lat_s), float(lon_s))
            except Exception:
                pass
            return False
        return super().acceptNavigationRequest(url, nav_type, is_main)


# ─────────────────────────────────────────────────────────────
#  LEAFLET MAP HTML
# ─────────────────────────────────────────────────────────────
LEAFLET_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"/>
<style>
  body,html{{margin:0;padding:0;background:#0b0f14;}}
  #map{{width:100vw;height:100vh;}}
  @keyframes pulsering{{
    0%  {{transform:scale(1);  opacity:0.8;}}
    60% {{transform:scale(1.7);opacity:0.0;}}
    100%{{transform:scale(1);  opacity:0.0;}}
  }}
</style>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
</head><body><div id="map"></div>
<script>
var map=L.map('map',{{zoomControl:true}}).setView([{LAT},{LON}],{ZOOM});
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{
  attribution:'&copy; OSM',maxZoom:19}}).addTo(map);
map.getContainer().style.filter='brightness(1.05) saturate(0.95)';

function makeDroneIcon(h){{
  h=h||0;
  var svg='<svg xmlns="http://www.w3.org/2000/svg" width="56" height="56" viewBox="-28 -28 56 56">'+
    '<circle cx="0" cy="0" r="26" fill="rgba(77,166,255,0.12)" stroke="#4da6ff" stroke-width="1.5"/>'+
    '<g transform="rotate('+h+')">'+
      '<polygon points="0,-22 9,6 0,0 -9,6" fill="#4da6ff" stroke="#fff" stroke-width="1.8" stroke-linejoin="round"/>'+
      '<polygon points="0,0 5,14 0,10 -5,14" fill="#2563eb" stroke="#fff" stroke-width="1"/>'+
    '</g>'+
    '<circle cx="0" cy="0" r="4" fill="#fff" stroke="#4da6ff" stroke-width="2"/>'+
  '</svg>'+
  '<div style="position:absolute;left:50%;top:58px;transform:translateX(-50%);">'+
    '<span style="background:#2563eb;color:#fff;font-family:monospace;font-size:10px;'+
      'font-weight:bold;padding:2px 6px;border-radius:3px;white-space:nowrap;'+
      'box-shadow:0 1px 4px rgba(0,0,0,0.5);">'+Math.round(h)+'\\u00b0</span></div>';
  return L.divIcon({{
    html:'<div style="position:relative;width:56px;height:56px;">'+svg+'</div>',
    className:'',iconSize:[56,56],iconAnchor:[28,28],popupAnchor:[0,-30]}});
}}
function makeWpIcon(){{
  var html='<div style="position:relative;width:60px;height:60px;">'+
    '<div style="position:absolute;width:56px;height:56px;border-radius:50%;'+
      'border:3px solid #e3b341;top:2px;left:2px;animation:pulsering 1.8s ease-out infinite;"></div>'+
    '<div style="position:absolute;width:42px;height:42px;border-radius:50%;'+
      'border:3px solid #e3b341;background:rgba(227,179,65,0.12);top:9px;left:9px;"></div>'+
    '<div style="position:absolute;width:2px;height:42px;background:#e3b341;top:9px;left:29px;"></div>'+
    '<div style="position:absolute;width:42px;height:2px;background:#e3b341;top:29px;left:9px;"></div>'+
    '<div style="position:absolute;width:10px;height:10px;border-radius:50%;'+
      'background:#e3b341;border:2px solid #fff;top:25px;left:25px;"></div>'+
    '<div style="position:absolute;left:50%;top:62px;transform:translateX(-50%);">'+
      '<span style="background:#e3b341;color:#000;font-family:monospace;font-size:10px;'+
        'font-weight:bold;padding:2px 6px;border-radius:3px;white-space:nowrap;'+
        'box-shadow:0 1px 4px rgba(0,0,0,0.5);">TARGET</span></div></div>';
  return L.divIcon({{html:html,className:'',iconSize:[60,60],iconAnchor:[30,30],popupAnchor:[0,-35]}});
}}
var droneMarker=null,wpMarker=null;
function setDrone(lat,lon,heading){{
  var icon=makeDroneIcon(heading||0);
  if(!droneMarker){{droneMarker=L.marker([lat,lon],{{icon:icon,zIndexOffset:1000}}).addTo(map);}}
  else{{droneMarker.setLatLng([lat,lon]);droneMarker.setIcon(icon);}}
}}
function setWaypoint(lat,lon){{
  var icon=makeWpIcon();
  if(!wpMarker){{wpMarker=L.marker([lat,lon],{{icon:icon,zIndexOffset:500}}).addTo(map);}}
  else{{wpMarker.setLatLng([lat,lon]);}}
}}
function clearWaypoint(){{if(wpMarker){{map.removeLayer(wpMarker);wpMarker=null;}}}}
map.on('click',function(e){{
  window.location.href='gcs://wp/'+e.latlng.lat.toFixed(7)+'/'+e.latlng.lng.toFixed(7);
}});
</script></body></html>
""".format(LAT=DEFAULT_MAP_LAT, LON=DEFAULT_MAP_LON, ZOOM=DEFAULT_MAP_ZOOM)


# ─────────────────────────────────────────────────────────────
#  STYLESHEET
# ─────────────────────────────────────────────────────────────
QSS = f"""
/* ── Base ───────────────────────────────────────────── */
QMainWindow, QWidget {{
    background: {C_BG};
    color: {C_TEXT};
    font-family: "Segoe UI", "Ubuntu", "Cantarell", sans-serif;
    font-size: 13px;
}}

/* ── Tabs ───────────────────────────────────────────── */
QTabWidget::pane {{
    border: 1px solid {C_BORDER};
    background: {C_BG};
    border-radius: 8px;
}}
QTabBar::tab {{
    background: {C_SURFACE};
    color: {C_DIM};
    border: 1px solid {C_BORDER};
    padding: 9px 24px;
    margin-right: 2px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    font-weight: 600;
}}
QTabBar::tab:selected {{
    background: {C_BG};
    color: {C_TEXT};
    border-bottom: 2px solid {C_ACCENT};
}}
QTabBar::tab:hover {{
    color: {C_TEXT};
    background: {C_CARD};
}}

/* ── Cards (QGroupBox) ──────────────────────────────── */
QGroupBox {{
    border: 1px solid {C_BORDER};
    border-radius: 10px;
    margin-top: 12px;
    padding: 14px 10px 10px 10px;
    background: {C_CARD};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 10px;
    color: {C_DIM};
    font-weight: bold;
    font-size: 11px;
    letter-spacing: 0.8px;
    text-transform: uppercase;
}}

/* ── Buttons ────────────────────────────────────────── */
QPushButton {{
    background: {C_SURFACE};
    color: {C_TEXT};
    border: 1px solid {C_BORDER};
    border-radius: 8px;
    padding: 8px 16px;
    font-size: 13px;
    font-weight: 500;
}}
QPushButton:hover {{
    background: {C_CARD};
    border-color: {C_ACCENT};
}}
QPushButton:pressed {{
    background: {C_BG};
}}
QPushButton#btnFly {{
    background: {C_ACCENT};
    color: #000;
    border: none;
    font-weight: bold;
    font-size: 14px;
    padding: 10px;
    border-radius: 8px;
}}
QPushButton#btnFly:hover {{
    background: #79c0ff;
}}
QPushButton#btnAbort {{
    background: {C_RED};
    color: white;
    border: none;
    font-weight: bold;
    font-size: 14px;
    padding: 10px;
    border-radius: 8px;
}}
QPushButton#btnAbort:hover {{
    background: #ff8080;
}}
QPushButton#btnUnlock {{
    background: {C_GREEN};
    color: #000;
    border: none;
    font-weight: bold;
    font-size: 13px;
    padding: 8px 16px;
    border-radius: 8px;
}}
QPushButton#btnUnlock:hover {{
    background: #5fe07a;
}}

/* ── Inputs ─────────────────────────────────────────── */
QLineEdit, QSpinBox {{
    background: {C_BG};
    color: {C_TEXT};
    border: 1px solid {C_BORDER};
    border-radius: 6px;
    padding: 6px 10px;
    font-family: "Consolas", "Courier New", monospace;
}}
QLineEdit:focus, QSpinBox:focus {{
    border-color: {C_ACCENT};
}}
QSpinBox::up-button, QSpinBox::down-button {{
    background: {C_SURFACE};
    border: 1px solid {C_BORDER};
}}
"""


# ─────────────────────────────────────────────────────────────
#  UI HELPERS
# ─────────────────────────────────────────────────────────────
def card(title):
    return QGroupBox(title)


def mono(text="—", color=None, size=12):
    lbl = QLabel(text)
    style = f"font-family: Consolas, monospace; font-size: {size}px;"
    if color:
        style += f" color: {color};"
    lbl.setStyleSheet(style)
    return lbl


def _sep():
    s = QFrame()
    s.setFrameShape(QFrame.VLine)
    s.setStyleSheet(f"color: {C_BORDER};")
    return s


# ─────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────
class GCSWindow(QMainWindow):
    def __init__(self, S: SharedState):
        super().__init__()
        self.S = S
        self.m = None
        self._wp_lat = None
        self._wp_lon = None
        self._B_bearing = None
        self._B_dist = None
        self._mav_thread = None
        self._mav_worker = None
        self._release_count = 0
        self._last_servo_pwm_sent = None
        self.servo_hw = DirectServoController(SERVO_GPIO_PIN)

        self.setWindowTitle("Medical Delivery Drone GCS v50 Light  |  Pi local go-to + direct GPIO servo")
        self.resize(1480, 900)
        self.setStyleSheet(QSS)
        self.setFocusPolicy(Qt.StrongFocus)

        self._build_ui()
        self._connect_mavlink()
        self._start_timers()
        self._apply_servo_pwm_direct(SERVO_CLOSE_PWM)

    # ── MAVLink connection ────────────────────────────────────
    def _connect_mavlink(self):
        self._mav_thread = QThread()
        self._mav_worker = MavWorker(self.S)
        self._mav_worker.moveToThread(self._mav_thread)
        self._mav_thread.started.connect(self._mav_worker.run)
        self._mav_worker.connected.connect(self._on_mav_connected)
        self._mav_worker.error.connect(self._on_mav_error)
        self._mav_thread.start()

    def _on_mav_connected(self, msg):
        self.m = self._mav_worker.m
        self.status_lbl.setText(f"  {msg}")
        self.status_lbl.setStyleSheet(
            f"color: {C_GREEN}; font-weight: bold; font-size: 12px;")

    def _on_mav_error(self, msg):
        self.status_lbl.setText(f"  Connection failed: {msg}")
        self.status_lbl.setStyleSheet(
            f"color: {C_RED}; font-weight: bold; font-size: 12px;")

    # ── Timers ────────────────────────────────────────────────
    def _start_timers(self):
        self._t_ui = QTimer(self)
        self._t_ui.timeout.connect(self._tick_ui)
        self._t_ui.start(int(1000 / UI_HZ))

        self._t_send = QTimer(self)
        self._t_send.timeout.connect(self._tick_send)
        self._t_send.start(int(1000 / SEND_HZ))

        self._t_ap = QTimer(self)
        self._t_ap.timeout.connect(self._tick_ap)
        self._t_ap.start(100)

        self._t_map = QTimer(self)
        self._t_map.timeout.connect(self._tick_map)
        self._t_map.start(500)

        self._t_servo = QTimer(self)
        self._t_servo.timeout.connect(self._tick_servo)
        self._t_servo.start(50)

    # ══════════════════════════════════════════════════════════
    #  BUILD UI
    # ══════════════════════════════════════════════════════════
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 8, 12, 8)
        root.setSpacing(6)

        # ── Header ────────────────────────────────────────────
        hdr = QHBoxLayout()
        title = QLabel("MEDICAL DELIVERY DRONE GCS")
        title.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {C_ACCENT};"
            f" letter-spacing: 2px;")
        ver = QLabel("v50 Light")
        ver.setStyleSheet(f"color: {C_DIM}; font-size: 12px;")
        self.status_lbl = QLabel("  Connecting to Pixhawk…")
        self.status_lbl.setStyleSheet(f"color: {C_YELLOW}; font-size: 12px;")
        hdr.addWidget(title)
        hdr.addWidget(ver)
        hdr.addStretch()
        hdr.addWidget(self.status_lbl)
        root.addLayout(hdr)

        # ── Status strip ──────────────────────────────────────
        self._build_status_strip()
        root.addWidget(self._strip_frame)

        # ── Tabs ──────────────────────────────────────────────
        self.tabs = QTabWidget()
        root.addWidget(self.tabs, stretch=1)
        self._build_dashboard_tab()
        self._build_map_tab()
        self._build_nav_tab()

        # ── Keyboard hint ─────────────────────────────────────
        hint = QLabel(
            "W/S pitch  ·  A/D roll  ·  ←/→ yaw  ·  SPACE center sticks  ·  8 servo mid  ·  9 servo open  ·  0 servo close  ·  X release  ·  R record  ·  ESC quit  ·  "
            f"TX ch{OVERRIDE_SWITCH_CH} HIGH → Pi local stick simulation")
        hint.setStyleSheet(f"color: {C_DIM}; font-size: 11px;")
        hint.setAlignment(Qt.AlignCenter)
        root.addWidget(hint)

    # ── Status strip (compact top bar) ────────────────────────
    def _build_status_strip(self):
        self._strip_frame = QFrame()
        self._strip_frame.setFixedHeight(32)
        self._strip_frame.setStyleSheet(
            f"background: {C_SURFACE}; border: 1px solid {C_BORDER};"
            f" border-radius: 6px;")
        row = QHBoxLayout(self._strip_frame)
        row.setContentsMargins(10, 0, 10, 0)
        row.setSpacing(0)

        def chip(key, default, w=120):
            lbl = QLabel(default)
            lbl.setFixedWidth(w)
            lbl.setStyleSheet(
                f"font-family: Consolas, monospace; font-size: 11px; color: {C_TEXT};")
            lbl.setAlignment(Qt.AlignCenter)
            row.addWidget(lbl)
            row.addWidget(_sep())
            return lbl

        self._mb = {}
        self._mb["mode"]  = chip("mode",  "MODE: —",      110)
        self._mb["armed"] = chip("armed", "DISARMED",      100)
        self._mb["sw"]    = chip("sw",    "SW: —",         120)
        self._mb["bat"]   = chip("bat",   "BAT: —V",       90)
        self._mb["hdg"]   = chip("hdg",   "HDG: —°",       80)
        self._mb["alt"]   = chip("alt",   "ALT: —m",       80)
        self._mb["gs"]    = chip("gs",    "GS: — m/s",    100)
        self._mb["gps"]   = chip("gps",   "GPS: —",       100)
        self._mb["ap"]    = chip("ap",    "AP: Idle",      260)
        row.addStretch()

    # ── Dashboard tab ─────────────────────────────────────────
    def _build_dashboard_tab(self):
        tab = QWidget()
        self.tabs.addTab(tab, "  Dashboard")
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(8)

        cam_box = card("Camera · Recording · Identification")
        cam_lay = QVBoxLayout(cam_box)
        cam_lay.setContentsMargins(4, 4, 4, 4)
        self.cam_label = QLabel("Camera initialising…")
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.cam_label.setStyleSheet(
            f"background: #eef2f7; color: {C_DIM}; border-radius: 6px;")
        cam_lay.addWidget(self.cam_label)
        left.addWidget(cam_box, stretch=5)

        lower = QHBoxLayout()
        lower.setSpacing(8)

        att_box = card("Attitude")
        att_lay = QVBoxLayout(att_box)
        self.horizon = HorizonWidget()
        self.horizon.setMinimumHeight(160)
        att_lay.addWidget(self.horizon)
        lower.addWidget(att_box, stretch=1)

        tel_box = card("Telemetry · RC")
        tg = QGridLayout(tel_box)
        tg.setSpacing(5)
        tg.setContentsMargins(10, 10, 10, 10)

        def trow(r, label_text, color=C_TEXT):
            lbl_key = QLabel(label_text)
            lbl_key.setStyleSheet(f"color: {C_DIM}; font-size: 12px;")
            tg.addWidget(lbl_key, r, 0)
            lbl_val = mono("—", color)
            tg.addWidget(lbl_val, r, 1)
            return lbl_val

        self._tl = {}
        self._tl["mode"]  = trow(0, "Mode")
        self._tl["armed"] = trow(1, "Armed")
        self._tl["bat"]   = trow(2, "Battery")
        self._tl["nav"]   = trow(3, "Hdg / Alt / GS")
        self._tl["gps"]   = trow(4, "GPS")
        self._tl["imu"]   = trow(5, "IMU Accel")
        self._tl["thr"]   = trow(6, "TX Throttle")
        self._tl["rc"]    = trow(7, "RC Channels")
        lower.addWidget(tel_box, stretch=1)

        left.addLayout(lower, stretch=3)
        layout.addLayout(left, stretch=4)

        right = QVBoxLayout()
        right.setSpacing(8)

        sw_box = card(f"Override Switch · TX ch{OVERRIDE_SWITCH_CH}")
        swl = QVBoxLayout(sw_box)
        self.lbl_switch = QLabel("TX FULL CONTROL")
        self.lbl_switch.setStyleSheet(
            f"font-size: 18px; font-weight: bold; color: {C_GREEN};")
        self.lbl_switch.setAlignment(Qt.AlignCenter)
        self.lbl_switch_pwm = mono(
            f"ch{OVERRIDE_SWITCH_CH} PWM: —  |  LOW<{SWITCH_LOW_PWM}  HIGH>{SWITCH_HIGH_PWM}", C_DIM)
        self.lbl_switch_pwm.setAlignment(Qt.AlignCenter)
        sw_cfg = QHBoxLayout()
        sw_cfg.addWidget(QLabel("Switch ch:"))
        self.spin_sw_ch = QSpinBox()
        self.spin_sw_ch.setRange(1, 18)
        self.spin_sw_ch.setValue(OVERRIDE_SWITCH_CH)
        self.spin_sw_ch.setFixedWidth(70)
        self.spin_sw_ch.valueChanged.connect(self._on_sw_ch_change)
        sw_cfg.addWidget(self.spin_sw_ch)
        sw_cfg.addStretch()
        swl.addWidget(self.lbl_switch)
        swl.addWidget(self.lbl_switch_pwm)
        swl.addLayout(sw_cfg)
        right.addWidget(sw_box)

        rec_box = card("Recording · Scene Tags")
        rl = QVBoxLayout(rec_box)
        self.lbl_state = QLabel("MANUAL READY")
        self.lbl_state.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {C_ACCENT};")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_scene = mono("Detected: —", C_DIM)
        self.lbl_scene.setAlignment(Qt.AlignCenter)
        self.lbl_record = mono("Recording: OFF", C_DIM)
        self.lbl_record.setAlignment(Qt.AlignCenter)
        self.btn_record = QPushButton("START RECORDING")
        self.btn_record.clicked.connect(self.toggle_recording)
        rl.addWidget(self.lbl_state)
        rl.addWidget(self.lbl_scene)
        rl.addWidget(self.lbl_record)
        rl.addWidget(self.btn_record)
        right.addWidget(rec_box)

        pwm_box = card("PWM Outputs · ch1/ch2/ch4")
        pl = QVBoxLayout(pwm_box)
        self.lbl_targets = mono("Target   R=1500  P=1500  Y=1500", C_DIM)
        self.lbl_outputs = mono("Output   R=1500  P=1500  Y=1500", C_ACCENT, 13)
        self.lbl_outputs.setStyleSheet(
            f"font-size: 13px; font-weight: bold; color: {C_ACCENT}; font-family: Consolas;")
        note = QLabel(f"ch3 throttle = NO_OVERRIDE  ·  TX owns flight mode  ·  servo on Pi GPIO{SERVO_GPIO_PIN}")
        note.setStyleSheet(f"color: {C_DIM}; font-size: 10px;")
        pl.addWidget(self.lbl_targets)
        pl.addWidget(self.lbl_outputs)
        pl.addWidget(note)
        right.addWidget(pwm_box)

        servo_box = card(f"Servo Release · Pi GPIO{SERVO_GPIO_PIN}")
        svl = QVBoxLayout(servo_box)
        self.lbl_servo = mono("Servo: CLOSED", C_ORANGE, 12)
        self.lbl_servo.setAlignment(Qt.AlignCenter)
        servo_btns = QHBoxLayout()
        btn_mid = QPushButton("MID")
        btn_mid.clicked.connect(self.set_servo_mid)
        btn_open = QPushButton("OPEN")
        btn_open.clicked.connect(self.set_servo_open)
        btn_close = QPushButton("CLOSE")
        btn_close.clicked.connect(self.set_servo_close)
        servo_btns.addWidget(btn_mid)
        servo_btns.addWidget(btn_open)
        servo_btns.addWidget(btn_close)
        svl.addWidget(self.lbl_servo)
        self.lbl_servo_hw = QLabel(self.servo_hw.describe())
        self.lbl_servo_hw.setStyleSheet(f"color: {C_DIM}; font-size: 11px;")
        self.lbl_servo_hw.setAlignment(Qt.AlignCenter)
        svl.addWidget(self.lbl_servo_hw)
        svl.addLayout(servo_btns)
        svl.addWidget(QLabel("Keyboard: 8 = mid  ·  9 = open  ·  0 = close"))
        right.addWidget(servo_box)

        ap_box = card("Mission Status")
        al = QVBoxLayout(ap_box)
        al.setSpacing(8)
        self.lbl_ap_status = QLabel("Idle")
        self.lbl_ap_status.setStyleSheet(
            f"color: {C_CYAN}; font-size: 12px; font-family: Consolas;")
        self.lbl_ap_status.setWordWrap(True)
        al.addWidget(self.lbl_ap_status)
        self.btn_abort_dash = QPushButton("ABORT CURRENT GO-TO")
        self.btn_abort_dash.setObjectName("btnAbort")
        self.btn_abort_dash.clicked.connect(self.do_abort)
        al.addWidget(self.btn_abort_dash)
        right.addWidget(ap_box)

        right.addStretch()
        layout.addLayout(right, stretch=2)

    # ── Map tab ───────────────────────────────────────────────
    def _build_map_tab(self):
        tab = QWidget()
        self.tabs.addTab(tab, "  Map & GPS Autopilot")
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        gps_box = card("GPS Target · local stick-sim go-to · auto drop + auto return")
        gr = QHBoxLayout(gps_box)
        gr.setSpacing(10)

        gr.addWidget(QLabel("Lat:"))
        self.inp_gps_lat = QLineEdit("— click map —")
        self.inp_gps_lat.setFixedWidth(140)
        gr.addWidget(self.inp_gps_lat)

        gr.addWidget(QLabel("Lon:"))
        self.inp_gps_lon = QLineEdit("—")
        self.inp_gps_lon.setFixedWidth(140)
        gr.addWidget(self.inp_gps_lon)

        self.lbl_map_dist = QLabel("Dist: —")
        self.lbl_map_dist.setStyleSheet(f"color: {C_YELLOW}; font-weight: bold;")
        gr.addWidget(self.lbl_map_dist)

        gr.addWidget(QLabel("Payload:"))
        btn_mid = QPushButton("MID")
        btn_mid.clicked.connect(self.set_servo_mid)
        btn_open = QPushButton("OPEN")
        btn_open.clicked.connect(self.set_servo_open)
        btn_close = QPushButton("CLOSE")
        btn_close.clicked.connect(self.set_servo_close)
        gr.addWidget(btn_mid)
        gr.addWidget(btn_open)
        gr.addWidget(btn_close)
        gr.addStretch()

        btn_fly = QPushButton("FLY TO TARGET")
        btn_fly.setObjectName("btnFly")
        btn_fly.setFixedWidth(180)
        btn_fly.clicked.connect(self.start_gps_autopilot)
        gr.addWidget(btn_fly)

        btn_abort = QPushButton("ABORT")
        btn_abort.setObjectName("btnAbort")
        btn_abort.setFixedWidth(110)
        btn_abort.clicked.connect(self.do_abort)
        gr.addWidget(btn_abort)

        btn_clr = QPushButton("Clear")
        btn_clr.clicked.connect(self._clear_wp)
        gr.addWidget(btn_clr)

        layout.addWidget(gps_box)

        self.map_page = MapPage()
        self.map_page.waypoint_set.connect(self._on_map_click)
        self.map_view = QWebEngineView()
        self.map_view.setPage(self.map_page)
        self.map_view.setHtml(LEAFLET_HTML, QUrl("http://localhost/"))
        self.map_view.setStyleSheet("border-radius: 8px;")
        layout.addWidget(self.map_view, stretch=1)


    # ── Dead reckoning tab ───────────────────────────────────
    def _build_nav_tab(self):
        tab = QWidget()
        self.tabs.addTab(tab, "  Dead Reckoning")
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        mode_box = card("Dead Reckoning Input Mode")
        ml = QHBoxLayout(mode_box)
        ml.setSpacing(18)
        self._dr_mode_group = QButtonGroup(self)
        for i, (text_label, mode_value) in enumerate([
            ("Mode A — Bearing + Distance", DRInputMode.BEARING_DIST),
            ("Mode B — Start + End Coordinates", DRInputMode.COORDINATES),
        ]):
            rb = QRadioButton(text_label)
            rb.setProperty("drmode", mode_value)
            if i == 0:
                rb.setChecked(True)
            self._dr_mode_group.addButton(rb, i)
            ml.addWidget(rb)
        ml.addStretch()
        self._dr_mode_group.buttonClicked.connect(self._on_dr_mode_change)
        layout.addWidget(mode_box)

        self.dr_panel_area = QWidget()
        self.dr_panel_layout = QVBoxLayout(self.dr_panel_area)
        self.dr_panel_layout.setContentsMargins(0, 0, 0, 0)
        self.dr_panel_layout.setSpacing(8)
        layout.addWidget(self.dr_panel_area)

        self._build_dr_panel_A()
        self._build_dr_panel_B()
        self._show_dr_panel(DRInputMode.BEARING_DIST)

        status_box = card("Dead Reckoning Status")
        stl = QVBoxLayout(status_box)
        self.lbl_dr_status = QLabel("Status: Idle")
        self.lbl_dr_status.setStyleSheet(
            f"color: {C_CYAN}; font-size: 12px; font-family: Consolas;")
        self.lbl_dr_prog = mono("0.0 / 0.0 m", C_TEXT)
        self.lbl_dr_speed = mono("Speed: —", C_YELLOW)
        prog_row = QHBoxLayout()
        prog_row.addWidget(self.lbl_dr_prog)
        prog_row.addStretch()
        prog_row.addWidget(self.lbl_dr_speed)
        self.dr_pbar = QProgressBar()
        self.dr_pbar.setRange(0, 100)
        self.dr_pbar.setValue(0)
        self.dr_pbar.setTextVisible(False)
        self.lbl_dr_pos = mono("Estimated position: —", C_GREEN)
        payload_row = QHBoxLayout()
        payload_row.addWidget(QLabel("Payload:"))
        btn_mid = QPushButton("MID")
        btn_mid.clicked.connect(self.set_servo_mid)
        btn_open = QPushButton("OPEN")
        btn_open.clicked.connect(self.set_servo_open)
        btn_close = QPushButton("CLOSE")
        btn_close.clicked.connect(self.set_servo_close)
        payload_row.addWidget(btn_mid)
        payload_row.addWidget(btn_open)
        payload_row.addWidget(btn_close)
        payload_row.addStretch()
        btns = QHBoxLayout()
        self.btn_dr_fly = QPushButton("START DEAD RECKONING")
        self.btn_dr_fly.setObjectName("btnFly")
        self.btn_dr_fly.clicked.connect(self.start_dr_autopilot)
        self.btn_dr_abort = QPushButton("ABORT")
        self.btn_dr_abort.setObjectName("btnAbort")
        self.btn_dr_abort.clicked.connect(self.do_abort)
        btns.addWidget(self.btn_dr_fly, stretch=3)
        btns.addWidget(self.btn_dr_abort, stretch=1)
        stl.addWidget(self.lbl_dr_status)
        stl.addLayout(prog_row)
        stl.addWidget(self.dr_pbar)
        stl.addWidget(self.lbl_dr_pos)
        stl.addLayout(payload_row)
        stl.addLayout(btns)
        layout.addWidget(status_box)
        layout.addStretch()

    def _build_dr_panel_A(self):
        self._panel_A = card("Mode A — Bearing + Distance")
        ag = QGridLayout(self._panel_A)
        ag.addWidget(QLabel("Bearing (°):"), 0, 0)
        self.dr_A_bearing = QLineEdit("0")
        self.dr_A_bearing.setFixedWidth(90)
        ag.addWidget(self.dr_A_bearing, 0, 1)
        ag.addWidget(QLabel("Distance (m):"), 1, 0)
        self.dr_A_dist = QLineEdit("10")
        self.dr_A_dist.setFixedWidth(90)
        ag.addWidget(self.dr_A_dist, 1, 1)
        self.lbl_A_est = mono("", C_YELLOW, 11)
        ag.addWidget(self.lbl_A_est, 2, 0, 1, 4)
        self.dr_A_dist.textChanged.connect(self._refresh_A)
        self._refresh_A()
        self.dr_panel_layout.addWidget(self._panel_A)

    def _build_dr_panel_B(self):
        self._panel_B = card("Mode B — Start + End Coordinates")
        bg = QGridLayout(self._panel_B)
        bg.addWidget(QLabel("Start Lat:"), 0, 0)
        self.dr_B_slat = QLineEdit()
        self.dr_B_slat.setFixedWidth(140)
        bg.addWidget(self.dr_B_slat, 0, 1)
        bg.addWidget(QLabel("Start Lon:"), 0, 2)
        self.dr_B_slon = QLineEdit()
        self.dr_B_slon.setFixedWidth(140)
        bg.addWidget(self.dr_B_slon, 0, 3)
        btn_gps = QPushButton("Use GPS")
        btn_gps.clicked.connect(self._use_gps_start)
        bg.addWidget(btn_gps, 0, 4)

        bg.addWidget(QLabel("End Lat:"), 1, 0)
        self.dr_B_elat = QLineEdit()
        self.dr_B_elat.setFixedWidth(140)
        bg.addWidget(self.dr_B_elat, 1, 1)
        bg.addWidget(QLabel("End Lon:"), 1, 2)
        self.dr_B_elon = QLineEdit()
        self.dr_B_elon.setFixedWidth(140)
        bg.addWidget(self.dr_B_elon, 1, 3)
        btn_map = QPushButton("Use Map Target")
        btn_map.clicked.connect(self._use_map_end)
        bg.addWidget(btn_map, 1, 4)

        self.lbl_B_result = mono("Bearing: —   Distance: —", C_YELLOW, 11)
        bg.addWidget(self.lbl_B_result, 2, 0, 1, 4)
        btn_calc = QPushButton("Calculate")
        btn_calc.clicked.connect(self._calc_B)
        bg.addWidget(btn_calc, 2, 4)
        self.dr_panel_layout.addWidget(self._panel_B)

    def _show_dr_panel(self, mode):
        for panel, mode_value in [
            (self._panel_A, DRInputMode.BEARING_DIST),
            (self._panel_B, DRInputMode.COORDINATES),
        ]:
            panel.setVisible(mode_value == mode)

    def _on_dr_mode_change(self, btn):
        self._show_dr_panel(btn.property("drmode"))

    def _current_dr_mode(self):
        btn = self._dr_mode_group.checkedButton()
        return btn.property("drmode") if btn else DRInputMode.BEARING_DIST

    def _refresh_A(self):
        try:
            d = float(self.dr_A_dist.text())
            self.lbl_A_est.setText(
                f"Estimated travel time: ~{d / CRUISE_SPEED_MPS:.1f} s")
        except Exception:
            self.lbl_A_est.setText("")

    def _use_gps_start(self):
        with self.S.lock:
            lat = self.S.tel["lat"]
            lon = self.S.tel["lon"]
        if lat is None or lon is None or lat == 0.0:
            QMessageBox.warning(self, "No GPS", "GPS start position is not available yet.")
            return
        self.dr_B_slat.setText(f"{lat:.7f}")
        self.dr_B_slon.setText(f"{lon:.7f}")

    def _use_map_end(self):
        if self._wp_lat is None or self._wp_lon is None:
            QMessageBox.warning(self, "No Map Target", "Click the map first.")
            return
        self.dr_B_elat.setText(f"{self._wp_lat:.7f}")
        self.dr_B_elon.setText(f"{self._wp_lon:.7f}")

    def _calc_B(self):
        try:
            slat = float(self.dr_B_slat.text())
            slon = float(self.dr_B_slon.text())
            elat = float(self.dr_B_elat.text())
            elon = float(self.dr_B_elon.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Enter valid start/end coordinates.")
            return
        d = haversine_m(slat, slon, elat, elon)
        b = bearing_to(slat, slon, elat, elon)
        self._B_bearing = b
        self._B_dist = d
        self.lbl_B_result.setText(
            f"Bearing: {b:.1f}°   Distance: {d:.1f} m   Est: ~{d / CRUISE_SPEED_MPS:.1f} s")

    # ══════════════════════════════════════════════════════════
    #  KEYBOARD INPUT
    # ══════════════════════════════════════════════════════════
    def keyPressEvent(self, e):
        k = e.key()
        if k == Qt.Key_Escape:
            self.close()
            return
        if k == Qt.Key_X:
            self.do_release_overrides()
            return
        if k == Qt.Key_R:
            self.toggle_recording()
            return
        if k == Qt.Key_Space:
            with self.S.lock:
                if not self.S.ap_active:
                    self.S.k_roll = self.S.k_pitch = self.S.k_yaw = NEUTRAL
                    self.S.o_roll = self.S.o_pitch = self.S.o_yaw = NEUTRAL
                    self.S.manual_backoff = False
            return

        if k == Qt.Key_8:
            self.set_servo_mid()
            return
        if k == Qt.Key_9:
            self.set_servo_open()
            return
        if k == Qt.Key_0:
            self.set_servo_close()
            return

        with self.S.lock:
            if self.S.ap_active:
                return
            if   k == Qt.Key_W:     self.S.k_pitch = clamp(self.S.k_pitch + STEP)
            elif k == Qt.Key_S:     self.S.k_pitch = clamp(self.S.k_pitch - STEP)
            elif k == Qt.Key_A:     self.S.k_roll  = clamp(self.S.k_roll  - STEP)
            elif k == Qt.Key_D:     self.S.k_roll  = clamp(self.S.k_roll  + STEP)
            elif k == Qt.Key_Left:  self.S.k_yaw   = clamp(self.S.k_yaw   - STEP)
            elif k == Qt.Key_Right: self.S.k_yaw   = clamp(self.S.k_yaw   + STEP)

    # ══════════════════════════════════════════════════════════
    #  CONTROL ACTIONS
    # ══════════════════════════════════════════════════════════
    def try_unlock(self):
        with self.S.lock:
            self.S.locked = False
            self.S.manual_backoff = False
            self.S.force_tx_release = False
            self.S.k_roll = self.S.k_pitch = self.S.k_yaw = NEUTRAL

    def do_backoff(self):
        with self.S.lock:
            self.S.manual_backoff = False

    def toggle_recording(self):
        with self.S.lock:
            self.S.recording = not self.S.recording
            if not self.S.recording:
                self.S.record_started_at = 0.0

    def _set_servo_target(self, pwm, label):
        with self.S.lock:
            self.S.servo_target_pwm = int(pwm)
            self.S.servo_status = label
        self._last_servo_pwm_sent = None

    def set_servo_open(self):
        self._set_servo_target(SERVO_OPEN_PWM, "OPEN")

    def set_servo_mid(self):
        self._set_servo_target(SERVO_MID_PWM, "MID")

    def set_servo_close(self):
        self._set_servo_target(SERVO_CLOSE_PWM, "CLOSED")

    def _apply_servo_pwm_direct(self, pwm):
        try:
            self.servo_hw.set_pulsewidth(int(pwm))
            self._last_servo_pwm_sent = int(pwm)
        except Exception as ex:
            with self.S.lock:
                self.S.record_error = f"Servo GPIO error: {ex}"

    def _tick_servo(self):
        with self.S.lock:
            pwm = int(self.S.servo_target_pwm)
        if self._last_servo_pwm_sent != pwm:
            self._apply_servo_pwm_direct(pwm)

    def do_release_overrides(self):
        """
        Release overrides via QTimer pulses — no GUI blocking.
        Sends NO_OVERRIDE on all 8 channels, 6 times at ~30ms intervals.
        """
        if not self.m:
            return
        try:
            if hasattr(self, "_release_timer") and self._release_timer.isActive():
                self._release_timer.stop()
        except Exception:
            pass
        self._release_count = 0
        self._release_timer = QTimer(self)
        self._release_timer.timeout.connect(self._release_tick)
        self._release_timer.start(30)

    def _release_tick(self):
        """One pulse of the non-blocking override release."""
        if self._release_count >= 6 or not self.m:
            self._release_timer.stop()
            return
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE,
                NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE)
        except Exception:
            pass
        self._release_count += 1

    def do_abort(self, checked=False, reason=None):
        """
        Stop the current GPS/DR go-to only.

        Important PyQt note:
        QPushButton.clicked emits a boolean "checked" argument. In v11.4 the
        slot accepted a single positional parameter called "reason", so a GUI
        click passed False into ap_status and QLabel.setText(False) crashed.

        This handler now accepts the clicked(bool) argument safely while still
        allowing internal calls like self.do_abort("GPS arrived ...") or
        self.do_abort(reason="...").

        v11.4.1 behaviour:
        - no GUIDED mission is cancelled because none is created
        - no forced TX hand-back is done
        - if the override switch stays ON, Pi keyboard control remains available
          immediately and ch1/ch2/ch4 simply return to neutral
        """
        if isinstance(checked, str) and reason is None:
            reason = checked
            checked = False
        if reason is None:
            reason = "Go-to aborted — keyboard ready"
        reason = str(reason)

        with self.S.lock:
            self.S.ap_active = False
            self.S.ap_mode = APMode.NONE
            self.S.ap_status = reason
            self.S.locked = False
            self.S.manual_backoff = False
            self.S.force_tx_release = False
            self.S.ap_roll_pwm = NEUTRAL
            self.S.ap_pitch_pwm = NEUTRAL
            self.S.ap_yaw_pwm = NEUTRAL
            self.S.o_roll = NEUTRAL
            self.S.o_pitch = NEUTRAL
            self.S.o_yaw = NEUTRAL
            self.S.k_roll = NEUTRAL
            self.S.k_pitch = NEUTRAL
            self.S.k_yaw = NEUTRAL
            self.S.dr_phase = "idle"
            self.S.dr_align_since = None
            self.S.dr_last_update = None
            self.S.auto_return_enabled = False
            self.S.returning_to_start = False
            self.S.return_context = ""
            self.S.mission_start_lat = None
            self.S.mission_start_lon = None

    def _on_sw_ch_change(self, val):
        global OVERRIDE_SWITCH_CH
        OVERRIDE_SWITCH_CH = val
        self.lbl_switch_pwm.setText(
            f"ch{val} PWM: —  |  LOW<{SWITCH_LOW_PWM}  HIGH>{SWITCH_HIGH_PWM}")

    def _clear_mission_memory(self):
        with self.S.lock:
            self.S.auto_return_enabled = True
            self.S.returning_to_start = False
            self.S.return_context = ""
            self.S.mission_start_lat = None
            self.S.mission_start_lon = None
            self.S.payload_stage = "idle"
            self.S.payload_stage_deadline = 0.0

    def _begin_payload_sequence(self, mode_name):
        with self.S.lock:
            self.S.payload_stage = "open_hold"
            self.S.payload_stage_deadline = time.time() + PAYLOAD_OPEN_SEC
            self.S.servo_target_pwm = SERVO_OPEN_PWM
            self.S.servo_status = "OPEN"
            self.S.ap_roll_pwm = NEUTRAL
            self.S.ap_pitch_pwm = NEUTRAL
            self.S.ap_yaw_pwm = NEUTRAL
            self.S.ap_status = f"{mode_name} arrived — payload OPEN for {PAYLOAD_OPEN_SEC:.0f}s"
        self._last_servo_pwm_sent = None

    def _start_return_leg(self, mode_name, cur_lat, cur_lon):
        with self.S.lock:
            already = self.S.returning_to_start
            start_lat = self.S.mission_start_lat
            start_lon = self.S.mission_start_lon

        if already or start_lat is None or start_lon is None or cur_lat is None or cur_lon is None:
            return False

        if mode_name == "GPS":
            back_dist = haversine_m(cur_lat, cur_lon, start_lat, start_lon)
            if back_dist <= WAYPOINT_RADIUS_M:
                return False
            with self.S.lock:
                self.S.ap_target_lat = start_lat
                self.S.ap_target_lon = start_lon
                self.S.returning_to_start = True
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = NEUTRAL
                self.S.ap_yaw_pwm = NEUTRAL
                self.S.ap_status = f"GPS return -> {start_lat:.5f},{start_lon:.5f}  |  {back_dist:.1f} m"
            return True

        if mode_name == "DR":
            back_dist = haversine_m(cur_lat, cur_lon, start_lat, start_lon)
            if back_dist <= DR_PROGRESS_RADIUS_M:
                return False
            back_bearing = bearing_to(cur_lat, cur_lon, start_lat, start_lon)
            with self.S.lock:
                self.S.returning_to_start = True
                self.S.dr_target_bearing = back_bearing
                self.S.dr_target_dist_m = back_dist
                self.S.dr_remaining_m = back_dist
                self.S.dr_travelled_m = 0.0
                self.S.dr_est_lat = cur_lat
                self.S.dr_est_lon = cur_lon
                self.S.dr_target_lat = start_lat
                self.S.dr_target_lon = start_lon
                self.S.dr_phase = "align"
                self.S.dr_align_since = None
                self.S.dr_last_update = time.time()
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = NEUTRAL
                self.S.ap_yaw_pwm = NEUTRAL
                self.S.ap_status = f"DR return align -> {back_bearing:.0f}°  |  {back_dist:.1f} m"
            return True

        return False

    # ══════════════════════════════════════════════════════════
    #  MAP
    # ══════════════════════════════════════════════════════════
    def _on_map_click(self, lat, lon):
        self._wp_lat = lat
        self._wp_lon = lon
        with self.S.lock:
            self.S.ap_target_lat = lat
            self.S.ap_target_lon = lon
        self.inp_gps_lat.setText(f"{lat:.6f}")
        self.inp_gps_lon.setText(f"{lon:.6f}")
        self.map_view.page().runJavaScript(f"setWaypoint({lat},{lon});")

    def _clear_wp(self):
        self._wp_lat = None
        self._wp_lon = None
        self._B_bearing = None
        self._B_dist = None
        with self.S.lock:
            self.S.ap_target_lat = None
            self.S.ap_target_lon = None
        self.inp_gps_lat.setText("— click map —")
        self.inp_gps_lon.setText("—")
        self.lbl_map_dist.setText("Dist: —")
        self.map_view.page().runJavaScript("clearWaypoint();")

    def _tick_map(self):
        with self.S.lock:
            lat = self.S.tel["lat"]
            lon = self.S.tel["lon"]
            hdg = self.S.tel["heading_deg"]
            dr_lat = self.S.dr_est_lat
            dr_lon = self.S.dr_est_lon

        if hdg is None:
            hdg = 0

        show_lat = lat if (lat is not None and lat != 0.0 and lon is not None) else dr_lat
        show_lon = lon if (lat is not None and lat != 0.0 and lon is not None) else dr_lon

        if show_lat is not None and show_lon is not None:
            self.map_view.page().runJavaScript(
                f"setDrone({show_lat},{show_lon},{hdg:.1f});")

        if self._wp_lat is not None and show_lat is not None and show_lon is not None:
            d = haversine_m(show_lat, show_lon, self._wp_lat, self._wp_lon)
            self.lbl_map_dist.setText(f"Dist: {d:.1f} m")
        elif self._wp_lat is None:
            self.lbl_map_dist.setText("Dist: —")

    # ══════════════════════════════════════════════════════════
    #  GPS AUTOPILOT START
    # ══════════════════════════════════════════════════════════
    def start_gps_autopilot(self):
        with self.S.lock:
            fix  = self.S.tel["gps_fix"]
            tlat = self.S.ap_target_lat
            tlon = self.S.ap_target_lon
            sw   = self.S.override_active
            cur_lat = self.S.tel["lat"]
            cur_lon = self.S.tel["lon"]

        if tlat is None or tlon is None:
            QMessageBox.warning(self, "No Target",
                                "Click the map to set a waypoint first.")
            return
        if not fix or fix < 3:
            QMessageBox.warning(self, "GPS Not Ready",
                                f"GPS fix type = {fix}  (need >= 3).")
            return
        if cur_lat is None or cur_lon is None or cur_lat == 0.0:
            QMessageBox.warning(self, "GPS Not Ready",
                                "Current GPS position is not available yet.")
            return
        if not sw:
            QMessageBox.warning(self, "Switch Not Active",
                                f"Flip TX ch{OVERRIDE_SWITCH_CH} HIGH to enable Pi control.")
            return
        if not self.m:
            QMessageBox.critical(self, "No MAVLink", "Not connected to Pixhawk.")
            return

        auto_return = True

        with self.S.lock:
            self.S.force_tx_release = False
            self.S.ap_mode   = APMode.GPS
            self.S.ap_active = True
            self.S.locked    = False
            self.S.ap_roll_pwm  = NEUTRAL
            self.S.ap_pitch_pwm = NEUTRAL
            self.S.ap_yaw_pwm   = NEUTRAL
            self.S.mission_start_lat = cur_lat
            self.S.mission_start_lon = cur_lon
            self.S.auto_return_enabled = auto_return
            self.S.returning_to_start = False
            self.S.return_context = "gps"
            self.S.ap_status = (
                f"GPS go-to -> {tlat:.5f},{tlon:.5f}  |  local RC-sim  |  auto-drop + auto-return")

    def start_dr_autopilot(self):
        with self.S.lock:
            sw_on = self.S.override_active
            hdg = self.S.tel["heading_deg"]
            cur_lat = self.S.tel["lat"]
            cur_lon = self.S.tel["lon"]

        if not self.m:
            QMessageBox.critical(self, "No MAVLink", "Not connected to Pixhawk.")
            return
        if not sw_on:
            QMessageBox.warning(self, "Switch Not Active",
                                f"Flip TX ch{OVERRIDE_SWITCH_CH} HIGH to enable Pi control.")
            return
        if hdg is None:
            QMessageBox.warning(self, "Heading Not Ready",
                                "Pixhawk heading is not available yet.")
            return

        mode = self._current_dr_mode()
        bearing = None
        dist = None
        est_lat = None
        est_lon = None
        target_lat = None
        target_lon = None

        try:
            if mode == DRInputMode.BEARING_DIST:
                bearing = float(self.dr_A_bearing.text()) % 360.0
                dist = float(self.dr_A_dist.text())
                if cur_lat is not None and cur_lon is not None and cur_lat != 0.0:
                    est_lat, est_lon = cur_lat, cur_lon
            elif mode == DRInputMode.COORDINATES:
                if self._B_bearing is None or self._B_dist is None:
                    QMessageBox.warning(self, "Not Calculated", "Press Calculate first.")
                    return
                start_lat = float(self.dr_B_slat.text())
                start_lon = float(self.dr_B_slon.text())
                target_lat = float(self.dr_B_elat.text())
                target_lon = float(self.dr_B_elon.text())
                bearing = float(self._B_bearing) % 360.0
                dist = float(self._B_dist)
                est_lat, est_lon = start_lat, start_lon
            else:
                QMessageBox.warning(self, "Mode Removed", "Dead Reckoning Mode C has been removed in v50.")
                return
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Enter valid dead reckoning values.")
            return

        if dist is None or dist <= 0.0:
            QMessageBox.warning(self, "Input Error", "Distance must be greater than zero.")
            return

        auto_return = True

        if target_lat is None or target_lon is None:
            if est_lat is not None and est_lon is not None:
                target_lat, target_lon = gps_offset(est_lat, est_lon, bearing, dist)

        with self.S.lock:
            self.S.force_tx_release = False
            self.S.ap_mode = APMode.DR
            self.S.ap_active = True
            self.S.locked = False
            self.S.ap_roll_pwm = NEUTRAL
            self.S.ap_pitch_pwm = NEUTRAL
            self.S.ap_yaw_pwm = NEUTRAL

            self.S.dr_input_mode = mode
            self.S.dr_target_bearing = bearing
            self.S.dr_target_dist_m = dist
            self.S.dr_remaining_m = dist
            self.S.dr_travelled_m = 0.0
            self.S.dr_start_lat = est_lat
            self.S.dr_start_lon = est_lon
            self.S.dr_est_lat = est_lat
            self.S.dr_est_lon = est_lon
            self.S.dr_target_lat = target_lat
            self.S.dr_target_lon = target_lon
            self.S.dr_phase = "align"
            self.S.dr_align_since = None
            self.S.dr_last_update = time.time()
            self.S.mission_start_lat = est_lat
            self.S.mission_start_lon = est_lon
            self.S.auto_return_enabled = auto_return
            self.S.returning_to_start = False
            self.S.return_context = "dr"
            self.S.ap_status = f"DR go-to align -> {bearing:.0f}°  |  {dist:.1f} m  |  auto-drop + auto-return"

    # ══════════════════════════════════════════════════════════
    #  LOCAL GO-TO TICK  (10 Hz)
    #
    #  BUG FIX (v11): heading=0 (north) is now handled correctly.
    #  Previous code used `not all((..., hdg, ...))` which treated
    #  heading 0 as falsy, silently killing the AP loop.
    # ══════════════════════════════════════════════════════════
    def _tick_ap(self):
        with self.S.lock:
            active = self.S.ap_active
            ap_mode = self.S.ap_mode
            sw_on = self.S.override_active
            payload_stage = self.S.payload_stage
            payload_deadline = self.S.payload_stage_deadline
            return_ctx = self.S.return_context
            returning = self.S.returning_to_start

        if active and not sw_on:
            self.do_abort("Aborted — switch flipped to TX")
            return

        if not active:
            return

        if payload_stage != "idle":
            now = time.time()
            with self.S.lock:
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = NEUTRAL
                self.S.ap_yaw_pwm = NEUTRAL
            if payload_stage == "open_hold" and now >= payload_deadline:
                with self.S.lock:
                    self.S.servo_target_pwm = SERVO_CLOSE_PWM
                    self.S.servo_status = "CLOSED"
                    self.S.payload_stage = "wait_return"
                    self.S.payload_stage_deadline = now + PAYLOAD_WAIT_SEC
                    self.S.ap_status = f"Payload closed — waiting {PAYLOAD_WAIT_SEC:.0f}s before return"
                self._last_servo_pwm_sent = None
                return
            if payload_stage == "wait_return" and now >= payload_deadline:
                with self.S.lock:
                    cur_lat = self.S.tel["lat"] if self.S.tel["lat"] not in (None, 0.0) else self.S.dr_est_lat
                    cur_lon = self.S.tel["lon"] if self.S.tel["lon"] not in (None, 0.0) else self.S.dr_est_lon
                    self.S.payload_stage = "idle"
                    self.S.payload_stage_deadline = 0.0
                mode_name = "GPS" if return_ctx == "gps" else "DR"
                if not self._start_return_leg(mode_name, cur_lat, cur_lon):
                    self.do_abort("Return start unavailable — ready for next command")
                return
            return

        if ap_mode == APMode.GPS:
            with self.S.lock:
                lat = self.S.tel["lat"]
                lon = self.S.tel["lon"]
                hdg = self.S.tel["heading_deg"]
                tlat = self.S.ap_target_lat
                tlon = self.S.ap_target_lon
                returning = self.S.returning_to_start

            if lat is None or lon is None or hdg is None or tlat is None or tlon is None:
                return

            dist = haversine_m(lat, lon, tlat, tlon)
            target_bearing = bearing_to(lat, lon, tlat, tlon)
            yaw_err = wrap180(target_bearing - hdg)

            if dist <= WAYPOINT_RADIUS_M:
                if returning:
                    self.do_abort(f"GPS return arrived ({dist:.1f} m) — ready for next command")
                else:
                    self._begin_payload_sequence("GPS")
                return

            yaw_pwm = clamp(NEUTRAL + yaw_err * AP_YAW_KP)
            pitch_pwm = AP_PITCH_FWD if abs(yaw_err) < YAW_DEADBAND_DEG * 2 else NEUTRAL

            with self.S.lock:
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = pitch_pwm
                self.S.ap_yaw_pwm = yaw_pwm
                self.S.ap_status = (
                    f"GPS: {dist:.1f}m  brg={target_bearing:.0f}°  err={yaw_err:+.0f}°  "
                    f"{'FWD' if pitch_pwm != NEUTRAL else 'aligning'}")
            return

        if ap_mode != APMode.DR:
            return

        with self.S.lock:
            hdg = self.S.tel["heading_deg"]
            gs = self.S.tel["gs_mps"]
            phase = self.S.dr_phase
            bearing = self.S.dr_target_bearing
            target_dist = self.S.dr_target_dist_m
            travelled = self.S.dr_travelled_m
            remaining = self.S.dr_remaining_m
            est_lat = self.S.dr_est_lat
            est_lon = self.S.dr_est_lon
            target_lat = self.S.dr_target_lat
            target_lon = self.S.dr_target_lon
            align_since = self.S.dr_align_since
            last_update = self.S.dr_last_update
            returning = self.S.returning_to_start

        if hdg is None:
            with self.S.lock:
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = NEUTRAL
                self.S.ap_yaw_pwm = NEUTRAL
                self.S.ap_status = "DR waiting for heading..."
            return

        yaw_err = wrap180(bearing - hdg)

        if phase == "align":
            yaw_pwm = clamp(NEUTRAL + yaw_err * AP_YAW_KP) if abs(yaw_err) > YAW_DEADBAND_DEG else NEUTRAL
            pitch_pwm = NEUTRAL
            now = time.time()
            if abs(yaw_err) <= YAW_DEADBAND_DEG:
                if align_since is None:
                    align_since = now
                elif now - align_since >= YAW_ALIGN_SEC:
                    with self.S.lock:
                        self.S.dr_phase = "fly"
                        self.S.dr_align_since = None
                        self.S.dr_last_update = now
                        self.S.ap_status = f"DR fly -> {bearing:.0f}°  |  {target_dist:.1f} m"
                    return
            else:
                align_since = None

            with self.S.lock:
                self.S.dr_align_since = align_since
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = pitch_pwm
                self.S.ap_yaw_pwm = yaw_pwm
                hold_txt = f"hold {max(0.0, YAW_ALIGN_SEC - (time.time() - align_since)):.1f}s" if align_since else "aligning"
                self.S.ap_status = f"DR align: err={yaw_err:+.0f}°  |  {hold_txt}"
            return

        if phase != "fly":
            with self.S.lock:
                self.S.ap_roll_pwm = NEUTRAL
                self.S.ap_pitch_pwm = NEUTRAL
                self.S.ap_yaw_pwm = NEUTRAL
            return

        now = time.time()
        dt = 0.0 if last_update is None else max(0.0, min(0.5, now - last_update))
        speed = gs if (gs is not None and gs > DR_MIN_GS_VALID) else CRUISE_SPEED_MPS
        step_m = speed * dt
        travelled = min(target_dist, travelled + step_m)
        remaining = max(0.0, target_dist - travelled)

        if est_lat is not None and est_lon is not None and step_m > 0.0:
            est_lat, est_lon = gps_offset(est_lat, est_lon, bearing, step_m)

        arrived = remaining <= 0.0
        if not arrived and target_lat is not None and target_lon is not None and est_lat is not None and est_lon is not None:
            arrived = haversine_m(est_lat, est_lon, target_lat, target_lon) <= DR_PROGRESS_RADIUS_M

        if arrived:
            with self.S.lock:
                self.S.dr_travelled_m = travelled
                self.S.dr_remaining_m = 0.0
                self.S.dr_est_lat = est_lat
                self.S.dr_est_lon = est_lon
            if returning:
                self.do_abort(f"DR return arrived ({travelled:.1f} m) — ready for next command")
            else:
                self._begin_payload_sequence("DR")
            return

        yaw_pwm = clamp(NEUTRAL + yaw_err * AP_YAW_KP) if abs(yaw_err) > YAW_DEADBAND_DEG else NEUTRAL
        pitch_pwm = AP_PITCH_FWD if abs(yaw_err) < YAW_DEADBAND_DEG * 2 else NEUTRAL

        with self.S.lock:
            self.S.dr_last_update = now
            self.S.dr_travelled_m = travelled
            self.S.dr_remaining_m = remaining
            self.S.dr_est_lat = est_lat
            self.S.dr_est_lon = est_lon
            self.S.ap_roll_pwm = NEUTRAL
            self.S.ap_pitch_pwm = pitch_pwm
            self.S.ap_yaw_pwm = yaw_pwm
            self.S.ap_status = (
                f"DR: {travelled:.1f}/{target_dist:.1f} m  left={remaining:.1f} m  err={yaw_err:+.0f}°  {speed:.1f} m/s")

    # ══════════════════════════════════════════════════════════
    #  RC OVERRIDE SEND  (20 Hz, local stick simulation only)
    #
    #  ch3 (throttle) = NO_OVERRIDE always — TX owns throttle.
    #  Servo is NOT sent through Pixhawk in v50; it is driven directly from Pi GPIO.
    #  Switch OFF → all NO_OVERRIDE.
    #  Switch ON + AP → computed AP stick values.
    #  Switch ON + keyboard/idle → smoothed local stick values.
    #  After operator abort, outputs return to NEUTRAL but Pi remains
    #  immediately available for keyboard input while the switch stays ON.
    #  Locked → NEUTRAL on ch1/ch2/ch4.
    #  NEVER calls set_mode() or sends guided-navigation commands.
    # ══════════════════════════════════════════════════════════
    def _tick_send(self):
        if not self.m:
            return

        dt = 1.0 / SEND_HZ
        max_step = int(MAX_PWM_RATE * dt)

        with self.S.lock:
            sw_on = self.S.override_active
            locked = self.S.locked
            mb = self.S.manual_backoff
            ap_active = self.S.ap_active
            ap_mode = self.S.ap_mode

            if not sw_on:
                roll_pwm = pitch_pwm = yaw_pwm = NO_OVERRIDE

            elif ap_active and ap_mode in (APMode.GPS, APMode.DR):
                roll_pwm = self.S.ap_roll_pwm
                pitch_pwm = self.S.ap_pitch_pwm
                yaw_pwm = self.S.ap_yaw_pwm

            elif locked:
                roll_pwm = NEUTRAL
                pitch_pwm = PITCH_BACKOFF_PWM if mb else PITCH_NEUTRAL_PWM
                yaw_pwm = NEUTRAL

            else:
                def approach(o, t):
                    if o < t:
                        return min(o + max_step, t)
                    if o > t:
                        return max(o - max_step, t)
                    return o
                self.S.o_roll = approach(self.S.o_roll, self.S.k_roll)
                self.S.o_pitch = approach(self.S.o_pitch, self.S.k_pitch)
                self.S.o_yaw = approach(self.S.o_yaw, self.S.k_yaw)
                roll_pwm = self.S.o_roll
                pitch_pwm = self.S.o_pitch
                yaw_pwm = self.S.o_yaw

        channels = [NO_OVERRIDE] * 8
        channels[0] = int(roll_pwm)
        channels[1] = int(pitch_pwm)
        channels[2] = NO_OVERRIDE
        channels[3] = int(yaw_pwm)

        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                *channels)
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════
    #  UI UPDATE TICK  (10 Hz)
    # ══════════════════════════════════════════════════════════
    def _tick_ui(self):
        # ── Camera frame ──────────────────────────────────────
        with self.S.lock:
            frame = self.S.latest_frame
        if frame is not None:
            try:
                fh, fw, fch = frame.shape
                img = QImage(frame.tobytes(), fw, fh, fch * fw,
                             QImage.Format_RGB888)
                pix = QPixmap.fromImage(img).scaled(
                    self.cam_label.width(), self.cam_label.height(),
                    Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.cam_label.setPixmap(pix)
            except Exception:
                pass

        # ── Read shared state ─────────────────────────────────
        with self.S.lock:
            tel         = dict(self.S.tel)
            locked      = self.S.locked
            detections  = list(self.S.latest_detections)
            summary     = self.S.detection_summary
            mb          = self.S.manual_backoff
            k           = (self.S.k_roll, self.S.k_pitch, self.S.k_yaw)
            o           = (self.S.o_roll, self.S.o_pitch, self.S.o_yaw)
            ap_active   = self.S.ap_active
            ap_mode     = self.S.ap_mode
            ap_status   = self.S.ap_status
            sw_on       = self.S.override_active
            sw_pwm      = self.S.override_switch_pwm
            rc_channels = dict(self.S.rc_channels)
            force_release = self.S.force_tx_release
            dr_phase    = self.S.dr_phase
            dr_done     = self.S.dr_travelled_m
            dr_total    = self.S.dr_target_dist_m
            dr_left     = self.S.dr_remaining_m
            dr_est_lat  = self.S.dr_est_lat
            dr_est_lon  = self.S.dr_est_lon
            dr_bearing  = self.S.dr_target_bearing
            recording   = self.S.recording
            record_path = self.S.record_path
            record_start= self.S.record_started_at
            record_error= self.S.record_error
            servo_status= self.S.servo_status
            payload_stage = self.S.payload_stage
            returning   = self.S.returning_to_start

        armed  = tel["armed"]
        bv     = tel["bat_v"]
        ba     = tel["bat_a"]
        h      = tel["heading_deg"]
        a      = tel["rel_alt_m"]
        gs     = tel["gs_mps"]
        fx     = tel["gps_fix"]
        gps_ok = fx is not None and fx >= 3

        # ── Horizon ───────────────────────────────────────────
        self.horizon.update_attitude(tel["roll_deg"], tel["pitch_deg"])

        # ── Override switch indicator ─────────────────────────
        if sw_on and not force_release:
            self.lbl_switch.setText("PI OVERRIDE ACTIVE  (ch1/ch2/ch4)")
            self.lbl_switch.setStyleSheet(
                f"font-size: 18px; font-weight: bold; color: {C_YELLOW};")
        else:
            self.lbl_switch.setText("TX FULL CONTROL")
            self.lbl_switch.setStyleSheet(
                f"font-size: 18px; font-weight: bold; color: {C_GREEN};")
        self.lbl_switch_pwm.setText(
            f"ch{OVERRIDE_SWITCH_CH} PWM: {sw_pwm}  |  "
            f"LOW<{SWITCH_LOW_PWM}  HIGH>{SWITCH_HIGH_PWM}")

        # ── Telemetry labels ──────────────────────────────────
        self._tl["mode"].setText(tel["mode"])
        self._tl["armed"].setText("ARMED" if armed else "DISARMED")
        self._tl["armed"].setStyleSheet(
            f"color: {C_RED if armed else C_DIM}; font-family: Consolas;")
        self._tl["bat"].setText(
            f"{bv:.2f} V  ·  {ba:.2f} A" if bv else "—")
        if bv:
            c = C_RED if bv < 10.5 else (C_YELLOW if bv < 11.0 else C_GREEN)
            self._tl["bat"].setStyleSheet(f"color: {c}; font-family: Consolas;")
        self._tl["nav"].setText(
            f"{h:.0f}°  ·  {a:.1f} m  ·  {gs:.1f} m/s"
            if h is not None else "—")
        self._tl["gps"].setText(
            f"fix={fx}  sats={tel['gps_sats']}  "
            f"{tel['lat']:.6f}, {tel['lon']:.6f}"
            if tel["lat"] else "—")
        self._tl["gps"].setStyleSheet(
            f"color: {C_GREEN if gps_ok else C_YELLOW};"
            f" font-family: Consolas; font-size: 11px;")
        if tel["ax"] is not None:
            self._tl["imu"].setText(
                f"x={tel['ax']:+.2f}  y={tel['ay']:+.2f}  z={tel['az']:+.2f}")
        self._tl["thr"].setText(
            f"{tel['thr_pwm']} PWM" if tel["thr_pwm"] else "—")

        # RC channels
        sw_ch = OVERRIDE_SWITCH_CH
        if rc_channels:
            parts = []
            for ch in range(1, 10):
                v = rc_channels.get(ch, 0)
                if v > 0:
                    tag = " ◄" if ch == sw_ch else ""
                    parts.append(f"ch{ch}:{v}{tag}")
            self._tl["rc"].setText("  ".join(parts) if parts else "waiting…")
        else:
            self._tl["rc"].setText("waiting…")

        # ── Control state ─────────────────────────────────────
        if force_release or not sw_on:
            self.lbl_state.setText("TX IN CONTROL")
            self.lbl_state.setStyleSheet(
                f"font-size: 16px; font-weight: bold; color: {C_GREEN};")
        elif ap_active and ap_mode == APMode.GPS:
            self.lbl_state.setText("GPS AUTOPILOT")
            self.lbl_state.setStyleSheet(
                f"font-size: 16px; font-weight: bold; color: {C_CYAN};")
        elif ap_active and ap_mode == APMode.DR:
            self.lbl_state.setText("DEAD RECKONING")
            self.lbl_state.setStyleSheet(
                f"font-size: 16px; font-weight: bold; color: {C_CYAN};")
        elif payload_stage != 'idle':
            self.lbl_state.setText("PAYLOAD ACTION")
            self.lbl_state.setStyleSheet(
                f"font-size: 16px; font-weight: bold; color: {C_ORANGE};")
        else:
            self.lbl_state.setText("KEYBOARD ACTIVE")
            self.lbl_state.setStyleSheet(
                f"font-size: 16px; font-weight: bold; color: {C_ACCENT};")

        self.lbl_scene.setText(f"Detected: {summary}")
        if recording:
            secs = max(0, int(time.time() - record_start)) if record_start else 0
            self.lbl_record.setText(f"Recording: ON  ·  {secs}s")
            self.btn_record.setText("STOP RECORDING")
        else:
            self.lbl_record.setText(f"Recording: OFF{'  ·  ' + record_error if record_error else ''}")
            self.btn_record.setText("START RECORDING")
        backend_text = self.servo_hw.describe()
        if getattr(self.servo_hw, "error", "") and self.servo_hw.backend == "unavailable":
            backend_text += f"  ·  {self.servo_hw.error}"
        self.lbl_servo.setText(f"Servo: {servo_status}")
        self.lbl_servo_hw.setText(backend_text)
        self.lbl_targets.setText(f"Target   R={k[0]}  P={k[1]}  Y={k[2]}")
        self.lbl_outputs.setText(f"Output   R={o[0]}  P={o[1]}  Y={o[2]}")
        self.lbl_ap_status.setText(str(ap_status))

        # Dead reckoning panel
        if hasattr(self, "lbl_dr_status"):
            self.lbl_dr_status.setText(str(ap_status) if ap_mode == APMode.DR or dr_total > 0 else "Status: Idle")
            progress_pct = 0
            if dr_total > 0:
                progress_pct = int(max(0.0, min(100.0, (dr_done / dr_total) * 100.0)))
            self.dr_pbar.setValue(progress_pct)
            self.lbl_dr_prog.setText(f"{dr_done:.1f} / {dr_total:.1f} m  |  left={dr_left:.1f} m" if dr_total > 0 else "0.0 / 0.0 m")
            self.lbl_dr_speed.setText(f"Speed: {gs:.1f} m/s" if gs is not None else f"Speed: {CRUISE_SPEED_MPS:.1f} m/s (est)")
            if dr_est_lat is not None and dr_est_lon is not None:
                self.lbl_dr_pos.setText(f"Estimated position: {dr_est_lat:.6f}, {dr_est_lon:.6f}")
            else:
                self.lbl_dr_pos.setText("Estimated position: —")
            if hasattr(self, "lbl_C_live"):
                if h is not None:
                    err = wrap180(dr_bearing - h) if ap_mode == APMode.DR or dr_total > 0 else 0.0
                    self.lbl_C_live.setText(f"Current heading: {h:.0f}°   |   Align error: {err:+.0f}°   |   phase={dr_phase}")
                else:
                    self.lbl_C_live.setText("Current heading: —   |   Align error: —")

        # ── Status strip ──────────────────────────────────────
        self._mb["mode"].setText(f"MODE: {tel['mode']}")
        self._mb["armed"].setText("ARMED" if armed else "DISARMED")
        self._mb["armed"].setStyleSheet(
            f"color: {C_RED if armed else C_DIM};"
            f" font-family: Consolas; font-size: 11px; font-weight: bold;")

        sw_c = C_YELLOW if sw_on else C_GREEN
        self._mb["sw"].setText(f"SW: {sw_pwm} ({'ON' if sw_on else 'OFF'})")
        self._mb["sw"].setStyleSheet(
            f"color: {sw_c}; font-family: Consolas; font-size: 11px; font-weight: bold;")

        self._mb["bat"].setText(f"BAT: {bv:.1f}V" if bv else "BAT: —")
        if bv:
            bc = C_RED if bv < 10.5 else (C_YELLOW if bv < 11.0 else C_GREEN)
            self._mb["bat"].setStyleSheet(
                f"color: {bc}; font-family: Consolas; font-size: 11px;")

        self._mb["hdg"].setText(f"HDG: {h:.0f}°" if h is not None else "HDG: —")
        self._mb["alt"].setText(f"ALT: {a:.1f}m" if a else "ALT: —")
        self._mb["gs"].setText(f"GS: {gs:.1f}" if gs else "GS: —")
        self._mb["gps"].setText(
            f"GPS: {tel['gps_sats']}sat" if tel["gps_sats"] else "GPS: —")
        self._mb["gps"].setStyleSheet(
            f"color: {C_GREEN if gps_ok else C_YELLOW};"
            f" font-family: Consolas; font-size: 11px;")
        ap_c = C_CYAN if ap_active else (C_GREEN if force_release else C_DIM)
        self._mb["ap"].setText(f"AP: {str(ap_status)[:40]}")
        self._mb["ap"].setStyleSheet(
            f"color: {ap_c}; font-family: Consolas; font-size: 11px;"
            f" font-weight: {'bold' if ap_active else 'normal'};")

    # ══════════════════════════════════════════════════════════
    #  CLOSE
    # ══════════════════════════════════════════════════════════
    def closeEvent(self, event):
        with self.S.lock:
            self.S.stop      = True
            self.S.ap_active = False
            self.S.force_tx_release = True
            self.S.recording = False
            self.S.servo_target_pwm = SERVO_CLOSE_PWM
            self.S.servo_status = "CLOSED"

        if self._mav_worker:
            self._mav_worker.stop()

        if self.m:
            try:
                for _ in range(4):
                    self.m.mav.rc_channels_override_send(
                        self.m.target_system, self.m.target_component,
                        NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE,
                        NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE, NO_OVERRIDE)
                    time.sleep(0.03)
            except Exception:
                pass

        try:
            self._apply_servo_pwm_direct(SERVO_CLOSE_PWM)
            self.servo_hw.close()
        except Exception:
            pass

        if self._mav_thread:
            self._mav_thread.quit()
            self._mav_thread.wait(2000)

        event.accept()


# ─────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────
def main():
    S = SharedState()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Start camera in background
    threading.Thread(target=camera_thread_fn, args=(S,), daemon=True).start()

    # Light palette
    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(C_BG))
    pal.setColor(QPalette.WindowText,      QColor(C_TEXT))
    pal.setColor(QPalette.Base,            QColor(C_SURFACE))
    pal.setColor(QPalette.AlternateBase,   QColor(C_BG))
    pal.setColor(QPalette.Text,            QColor(C_TEXT))
    pal.setColor(QPalette.Button,          QColor(C_SURFACE))
    pal.setColor(QPalette.ButtonText,      QColor(C_TEXT))
    pal.setColor(QPalette.Highlight,       QColor(C_ACCENT))
    pal.setColor(QPalette.HighlightedText, QColor(C_BG))
    app.setPalette(pal)

    win = GCSWindow(S)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
