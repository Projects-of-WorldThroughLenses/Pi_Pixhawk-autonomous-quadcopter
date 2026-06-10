"""
Drone Dashboard v100 - Live/Post Computer Vision + Low-Latency Split FPV GCS

Major modules:
- Pixhawk/ArduPilot MAVLink telemetry over COM/UDP/TCP
- Betaflight/iNav MSP telemetry over USB serial/COM
- TX15 Max USB joystick/HID monitor through pygame
- FPV source through OpenCV
- AI object recognition overlay using optional Ultralytics YOLO, plus OpenCV face/HOG fallback
- Dataset recorder: synchronized video + telemetry CSV + detections CSV + TX/RX channel CSV
- Integrated main-tab map waypoint planning and guarded MAVLink Guided waypoint command
- Main-tab dual TX/RX channel display: TX USB values on top, receiver/Pixhawk RC_CHANNELS on bottom

Safety design:
- This program does not arm, disarm, or take off.
- The Guided waypoint sender requires an existing MAVLink connection, valid GPS, and a confirmation dialog.
- Test with props removed / simulator first before any field use.
"""

import os
import sys
import time
import math
import csv
import json
import struct
import threading
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple

import cv2
import numpy as np

try:
    cv2.setUseOptimized(True)
    cv2.setNumThreads(max(2, (os.cpu_count() or 8) - 1))
except Exception:
    pass

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QSize, QObject, pyqtSlot, QUrl, QRectF, QPointF
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QPen, QBrush, QPainterPath, QPolygonF, QLinearGradient, QRadialGradient, QKeySequence
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox, QLineEdit, QVBoxLayout,
    QHBoxLayout, QGridLayout, QGroupBox, QFileDialog, QMessageBox, QTextEdit,
    QCheckBox, QFrame, QDoubleSpinBox, QSpinBox, QProgressBar, QSizePolicy,
    QSplitter, QScrollArea, QTabWidget, QFormLayout, QShortcut, QListView,
    QGraphicsDropShadowEffect
)

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtWebChannel import QWebChannel
    WEBENGINE_AVAILABLE = True
except Exception:
    QWebEngineView = None
    QWebChannel = None
    WEBENGINE_AVAILABLE = False

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except Exception:
    mavutil = None
    PYMAVLINK_AVAILABLE = False

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
    SERIAL_PORT_SCAN_AVAILABLE = True
except Exception:
    serial = None
    list_ports = None
    SERIAL_AVAILABLE = False
    SERIAL_PORT_SCAN_AVAILABLE = False

try:
    import pygame
    PYGAME_AVAILABLE = True
except Exception:
    pygame = None
    PYGAME_AVAILABLE = False

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    YOLO = None
    ULTRALYTICS_AVAILABLE = False

# ----------------------------- Utilities -----------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except Exception:
        return default


def seconds_to_hms(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def discover_mavlink_connection_options() -> List[str]:
    options: List[str] = []
    if SERIAL_PORT_SCAN_AVAILABLE and list_ports is not None:
        try:
            for port in list_ports.comports():
                device = getattr(port, "device", "")
                desc = getattr(port, "description", "")
                if device:
                    options.append(f"{device}    # {desc}" if desc and desc != device else device)
        except Exception:
            pass
    defaults = ["udp:127.0.0.1:14550", "udp:0.0.0.0:14550", "tcp:127.0.0.1:5760"]
    if os.name == "nt":
        defaults += [f"COM{i}" for i in range(3, 13)]
    else:
        defaults += ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    for d in defaults:
        if d not in options:
            options.append(d)
    return options


def clean_connection_string(text: str) -> str:
    return text.split("#", 1)[0].strip()


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    if None in (lat1, lon1, lat2, lon2):
        return 0.0
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def frame_to_qpixmap(frame_bgr: np.ndarray, target_size: QSize) -> QPixmap:
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    h, w = frame_rgb.shape[:2]
    qimg = QImage(frame_rgb.data, w, h, frame_rgb.strides[0], QImage.Format_RGB888).copy()
    pix = QPixmap.fromImage(qimg)
    return pix.scaled(target_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)


# ----------------------------- Scientific fuel / propulsion estimator -----------------------------

G = 9.80665
R_DRY_AIR = 287.05
R_WATER_VAPOUR = 461.495
SEA_LEVEL_PRESSURE = 101325.0
SEA_LEVEL_TEMP = 288.15
LAPSE_RATE = 0.0065
STANDARD_RHO = 1.225


def lipo_soc_from_cell_voltage(v_cell: float) -> float:
    """Approximate under-light-load LiPo SoC curve. Use current sensor for final decisions."""
    points = [
        (4.20, 100), (4.15, 95), (4.10, 90), (4.05, 85), (4.00, 80),
        (3.95, 70), (3.90, 60), (3.85, 50), (3.80, 40), (3.75, 30),
        (3.70, 20), (3.65, 15), (3.60, 10), (3.50, 5), (3.30, 0),
    ]
    if v_cell >= points[0][0]:
        return 100.0
    if v_cell <= points[-1][0]:
        return 0.0
    for i in range(len(points) - 1):
        v1, s1 = points[i]
        v2, s2 = points[i + 1]
        if v1 >= v_cell >= v2:
            t = (v_cell - v2) / max(1e-9, (v1 - v2))
            return s2 + t * (s1 - s2)
    return 0.0


def air_density_kg_m3(altitude_m: float, temp_c: float, relative_humidity_percent: float) -> float:
    """ISA-like density estimate with humidity correction."""
    temp_k = temp_c + 273.15
    rh = clamp(relative_humidity_percent, 0.0, 100.0) / 100.0
    pressure = SEA_LEVEL_PRESSURE * (1.0 - (LAPSE_RATE * altitude_m) / SEA_LEVEL_TEMP) ** 5.25588
    sat_vapour_pressure = 610.94 * math.exp((17.625 * temp_c) / max(1e-6, (temp_c + 243.04)))
    vapour_pressure = rh * sat_vapour_pressure
    dry_air_pressure = pressure - vapour_pressure
    rho = dry_air_pressure / (R_DRY_AIR * temp_k) + vapour_pressure / (R_WATER_VAPOUR * temp_k)
    return max(rho, 0.1)


def prop_disk_area_total(rotor_count: int, prop_diameter_in: float) -> float:
    diameter_m = prop_diameter_in * 0.0254
    radius_m = diameter_m / 2.0
    return max(1, int(rotor_count)) * math.pi * radius_m ** 2


def ideal_hover_power_w(total_mass_kg: float, rotor_count: int, prop_diameter_in: float, rho: float, figure_of_merit: float) -> float:
    weight_n = total_mass_kg * G
    area = prop_disk_area_total(rotor_count, prop_diameter_in)
    p_ideal = (weight_n ** 1.5) / math.sqrt(max(1e-9, 2.0 * rho * area))
    return p_ideal / clamp(figure_of_merit, 0.25, 0.95)


def calibrated_static_current_per_motor(
    kv: float,
    voltage: float,
    prop_diameter_in: float,
    prop_pitch_in: float,
    blade_count: int,
    rho: float,
    loading_factor: float,
    pitch_exponent: float,
    blade_exponent: float,
    ref_current_a: float,
    ref_kv: float,
    ref_voltage: float,
    ref_diameter_in: float,
    ref_pitch_in: float,
    ref_blades: int,
    ref_loading_factor: float,
    ref_rho: float = STANDARD_RHO,
) -> float:
    """Empirical prop/motor current scaling model for what-if planning."""
    kv_factor = (kv / max(ref_kv, 1e-6)) ** 3
    voltage_factor = (voltage / max(ref_voltage, 1e-6)) ** 2
    diameter_factor = (prop_diameter_in / max(ref_diameter_in, 1e-6)) ** 5
    density_factor = rho / max(ref_rho, 1e-6)
    lambda_factor = (loading_factor / max(ref_loading_factor, 1e-6)) ** 3
    pitch_ratio = (prop_pitch_in / max(prop_diameter_in, 1e-6)) / (ref_pitch_in / max(ref_diameter_in, 1e-6))
    pitch_factor = max(pitch_ratio, 0.05) ** pitch_exponent
    blade_factor = (max(1, int(blade_count)) / max(1, int(ref_blades))) ** blade_exponent
    return max(0.0, ref_current_a * kv_factor * voltage_factor * diameter_factor * density_factor * lambda_factor * pitch_factor * blade_factor)


@dataclass
class ScientificFuelSettings:
    altitude_m: float = 0.0
    temp_c: float = 30.0
    humidity_percent: float = 70.0
    cell_count: int = 6
    cell_voltage: float = 3.7
    capacity_mah: float = 3300.0
    usable_percent: float = 75.0
    reserve_percent: float = 20.0
    rotor_count: int = 4
    drone_mass_kg: float = 2.70
    payload_mass_kg: float = 0.0
    avionics_power_w: float = 15.0
    avionics_current_a: float = 2.0
    motor_kv: float = 900.0
    prop_diameter_in: float = 10.0
    prop_pitch_in: float = 4.5
    blade_count: int = 2
    loading_factor: float = 0.70
    figure_of_merit: float = 0.55
    measured_max_thrust_g: float = 4000.0
    max_current_per_motor_a: float = 62.0
    esc_rating_a: float = 65.0
    max_throttle_percent: float = 100.0
    cruise_throttle_percent: float = 40.0
    telemetry_current_bias_a: float = 0.0
    ref_current_a: float = 50.0
    ref_kv: float = 1100.0
    ref_voltage: float = 24.0
    ref_diameter_in: float = 9.0
    ref_pitch_in: float = 4.0
    ref_blades: int = 3
    ref_loading_factor: float = 0.70
    pitch_exponent: float = 1.0
    blade_exponent: float = 0.7

    @property
    def battery_voltage_v(self) -> float:
        return max(1, int(self.cell_count)) * max(0.1, self.cell_voltage)

    @property
    def usable_mah(self) -> float:
        return self.capacity_mah * clamp(self.usable_percent, 1.0, 100.0) / 100.0

    @property
    def reserve_mah(self) -> float:
        return self.capacity_mah * clamp(self.reserve_percent, 0.0, 90.0) / 100.0


def estimate_current_from_throttle(settings: ScientificFuelSettings, throttle_percent: float) -> float:
    max_thr = clamp(settings.max_throttle_percent, 1.0, 100.0)
    thr = clamp(throttle_percent, 0.0, max_thr)
    ratio = thr / max_thr
    propulsion_current = settings.max_current_per_motor_a * max(1, int(settings.rotor_count)) * (ratio ** 3)
    return max(0.0, propulsion_current + settings.avionics_current_a + settings.telemetry_current_bias_a)


def rich_fuel_estimate(settings: ScientificFuelSettings) -> Dict[str, Any]:
    rho = air_density_kg_m3(settings.altitude_m, settings.temp_c, settings.humidity_percent)
    voltage = settings.battery_voltage_v
    rotor_count = max(1, int(settings.rotor_count))
    total_mass_kg = max(0.01, settings.drone_mass_kg + settings.payload_mass_kg)
    no_load_rpm = settings.motor_kv * voltage
    loaded_rpm = no_load_rpm * settings.loading_factor

    static_current_motor = calibrated_static_current_per_motor(
        kv=settings.motor_kv,
        voltage=voltage,
        prop_diameter_in=settings.prop_diameter_in,
        prop_pitch_in=settings.prop_pitch_in,
        blade_count=int(settings.blade_count),
        rho=rho,
        loading_factor=settings.loading_factor,
        pitch_exponent=settings.pitch_exponent,
        blade_exponent=settings.blade_exponent,
        ref_current_a=settings.ref_current_a,
        ref_kv=settings.ref_kv,
        ref_voltage=settings.ref_voltage,
        ref_diameter_in=settings.ref_diameter_in,
        ref_pitch_in=settings.ref_pitch_in,
        ref_blades=int(settings.ref_blades),
        ref_loading_factor=settings.ref_loading_factor,
    )

    hover_thrust_per_motor_g = (total_mass_kg * 1000.0) / rotor_count
    if settings.measured_max_thrust_g > 0:
        thrust_fraction = clamp(hover_thrust_per_motor_g / settings.measured_max_thrust_g, 0.02, 1.50)
        hover_throttle_equivalent = math.sqrt(thrust_fraction)
        hover_current_motor_cubic = static_current_motor * (thrust_fraction ** 1.5)
        thrust_margin_percent = ((settings.measured_max_thrust_g * rotor_count) / (total_mass_kg * 1000.0) - 1.0) * 100.0
    else:
        hover_throttle_equivalent = 0.0
        hover_current_motor_cubic = 0.0
        thrust_margin_percent = 0.0

    p_hover_induced = ideal_hover_power_w(total_mass_kg, rotor_count, settings.prop_diameter_in, rho, settings.figure_of_merit)
    total_hover_current_cubic = hover_current_motor_cubic * rotor_count
    total_hover_power_cubic = total_hover_current_cubic * voltage
    motor_hover_power_w = max(total_hover_power_cubic, p_hover_induced)
    avionics_power_w = max(settings.avionics_power_w, settings.avionics_current_a * voltage)
    total_hover_power_w = motor_hover_power_w + avionics_power_w
    total_hover_current_a = total_hover_power_w / max(voltage, 0.001)

    cruise_current_a = estimate_current_from_throttle(settings, settings.cruise_throttle_percent)
    usable_ah = max(0.001, settings.usable_mah / 1000.0)
    reserve_ah = max(0.0, settings.reserve_mah / 1000.0)

    flight_min_hover = usable_ah / max(total_hover_current_a, 0.001) * 60.0
    flight_min_cruise = usable_ah / max(cruise_current_a, 0.001) * 60.0
    to_reserve_hover = max(0.0, (usable_ah - reserve_ah) / max(total_hover_current_a, 0.001) * 60.0)
    to_reserve_cruise = max(0.0, (usable_ah - reserve_ah) / max(cruise_current_a, 0.001) * 60.0)

    warnings = []
    if settings.measured_max_thrust_g > 0 and thrust_margin_percent < 50:
        warnings.append("LOW THRUST MARGIN")
    if settings.measured_max_thrust_g > 0 and hover_throttle_equivalent > 0.65:
        warnings.append("HIGH HOVER THROTTLE")
    if static_current_motor > settings.esc_rating_a * 0.85:
        warnings.append("ESC CURRENT CAUTION")
    if min(flight_min_hover, flight_min_cruise) < 5:
        warnings.append("SHORT FLIGHT TIME")

    return {
        "rho": rho,
        "voltage": voltage,
        "no_load_rpm": no_load_rpm,
        "loaded_rpm": loaded_rpm,
        "total_mass_kg": total_mass_kg,
        "hover_thrust_per_motor_g": hover_thrust_per_motor_g,
        "static_current_motor": static_current_motor,
        "static_total_current": static_current_motor * rotor_count,
        "static_total_power": static_current_motor * rotor_count * voltage,
        "hover_throttle_equivalent": hover_throttle_equivalent,
        "thrust_margin_percent": thrust_margin_percent,
        "p_hover_induced": p_hover_induced,
        "total_hover_power_w": total_hover_power_w,
        "total_hover_current_a": total_hover_current_a,
        "cruise_current_a": cruise_current_a,
        "max_total_current_a": settings.max_current_per_motor_a * rotor_count + settings.avionics_current_a,
        "flight_min_hover": flight_min_hover,
        "flight_min_cruise": flight_min_cruise,
        "to_reserve_hover": to_reserve_hover,
        "to_reserve_cruise": to_reserve_cruise,
        "warnings": warnings,
    }


# ----------------------------- MAVLink Telemetry -----------------------------

class TelemetryThread(QThread):
    telemetry = pyqtSignal(dict)
    # v30 fix: MAVLink thread must expose the same attitude signal as the MSP thread.
    # v25 connected this signal unconditionally, so Pixhawk/MAVLink connection could crash.
    attitude = pyqtSignal(float, float, float)
    status = pyqtSignal(str)

    def __init__(self, connection_string: str, baud: int = 57600, parent=None):
        super().__init__(parent)
        self.connection_string = connection_string.strip()
        self.baud = int(baud)
        self.running = False
        self.master = None
        self.data = self.default_data()
        self._last_command_status = "No command sent."
        # v51: real packet/update-rate tracking. These are measured from packets
        # as they arrive from the link, not from the QWidget paint FPS.
        self._rate_counts: Dict[str, int] = {}
        self._rate_values: Dict[str, float] = {}
        self._rate_t = time.time()

    @staticmethod
    def default_data() -> Dict[str, Any]:
        return {
            "connected": False,
            "telemetry_protocol": "MAVLink",
            "armed": False,
            "mode": "-",
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "heading_deg": 0.0,
            "battery_voltage_v": None,
            "battery_current_a": None,
            "battery_remaining_percent": None,
            "throttle_percent": 0.0,
            "gps_fix_type": 0,
            "gps_sats": 0,
            "lat": None,
            "lon": None,
            "gps_alt_m": None,
            "rel_alt_m": None,
            "groundspeed_m_s": 0.0,
            "airspeed_m_s": 0.0,
            "climb_m_s": 0.0,
            "lidar_alt_m": None,
            "optical_flow_alt_m": None,
            "optical_flow_quality": None,
            "rc_channels": [0] * 16,
            "rc_rssi": None,
            "radio_rssi": None,
            "radio_remrssi": None,
            "radio_noise": None,
            "radio_remnoise": None,
            "rxerrors": None,
            "fixed": None,
            # ArduPilot can report ESC telemetry in groups 1-4, 5-8, 9-12 and 13-16.
            # User's Pixhawk setup may map motors to ESC9-ESC12, so keep all 16 slots.
            "esc_rpm": [None] * 16,
            "esc_current": [None] * 16,
            "esc_voltage": [None] * 16,
            "esc_temp": [None] * 16,
            "fence_breach_status": None,
            "fence_breach_count": None,
            "fence_breach_type": None,
            "fence_breach_time": None,
            "fence_alt_max_m": None,
            "fence_radius_m": None,
            "fence_enabled": None,
            "fence_type": None,
            "home_lat": None,
            "home_lon": None,
            "home_alt_m": None,
            "fence_alt_remaining_m": None,
            "fence_radius_remaining_m": None,
            "fence_warning": "Fence: waiting",
            "telemetry_rate_hz": 0.0,
            "attitude_rate_hz": 0.0,
            "gps_rate_hz": 0.0,
            "esc_rate_hz": 0.0,
            "fence_rate_hz": 0.0,
            "battery_rate_hz": 0.0,
            "rc_rate_hz": 0.0,
            "last_msg_time": None,
            "system_id": None,
            "component_id": None,
        }


    def _track_rate(self, name: str):
        now = time.time()
        self._rate_counts[name] = self._rate_counts.get(name, 0) + 1
        dt = now - self._rate_t
        if dt >= 1.0:
            for key, count in list(self._rate_counts.items()):
                self._rate_values[key] = count / max(0.001, dt)
                self.data[f"{key}_rate_hz"] = self._rate_values[key]
            # Preserve old/user-facing names.
            self.data["telemetry_rate_hz"] = self._rate_values.get("telemetry", 0.0)
            self.data["attitude_rate_hz"] = self._rate_values.get("attitude", 0.0)
            self.data["gps_rate_hz"] = self._rate_values.get("gps", 0.0)
            self.data["esc_rate_hz"] = self._rate_values.get("esc", 0.0)
            self.data["fence_rate_hz"] = self._rate_values.get("fence", 0.0)
            self.data["battery_rate_hz"] = self._rate_values.get("battery", 0.0)
            self.data["rc_rate_hz"] = self._rate_values.get("rc", 0.0)
            self._rate_counts.clear()
            self._rate_t = now

    def _track_rate_by_mav_type(self, mtype: str):
        if mtype in ("ATTITUDE", "AHRS2", "ATTITUDE_QUATERNION"):
            self._track_rate("attitude")
        elif mtype in ("GLOBAL_POSITION_INT", "GPS_RAW_INT", "HOME_POSITION", "LOCAL_POSITION_NED"):
            self._track_rate("gps")
        elif mtype in ("ESC_TELEMETRY_1_TO_4", "ESC_TELEMETRY_5_TO_8", "ESC_TELEMETRY_9_TO_12", "ESC_TELEMETRY_13_TO_16"):
            self._track_rate("esc")
        elif mtype in ("FENCE_STATUS", "PARAM_VALUE"):
            self._track_rate("fence")
        elif mtype in ("SYS_STATUS", "BATTERY_STATUS"):
            self._track_rate("battery")
        elif mtype in ("RC_CHANNELS", "RADIO_STATUS"):
            self._track_rate("rc")

    def connect_master(self):
        if not PYMAVLINK_AVAILABLE:
            raise RuntimeError("pymavlink is not installed. Install with: pip install pymavlink")
        if not self.connection_string:
            raise RuntimeError("Empty MAVLink connection string")
        if self.connection_string.upper().startswith("COM") or self.connection_string.startswith("/dev/"):
            return mavutil.mavlink_connection(
                self.connection_string, baud=self.baud, autoreconnect=False,
                source_system=255, source_component=190,
            )
        return mavutil.mavlink_connection(
            self.connection_string, autoreconnect=True, source_system=255, source_component=190
        )

    def _close_master(self):
        try:
            if self.master is not None:
                self.master.close()
        except Exception:
            pass
        self.master = None

    def _set_message_interval(self, msg_id: int, hz: float):
        if self.master is None or hz <= 0:
            return
        try:
            interval_us = int(1_000_000 / hz)
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                int(msg_id), interval_us, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    def request_streams(self):
        if self.master is None:
            return
        mav = mavutil.mavlink
        wanted = [
            (getattr(mav, "MAVLINK_MSG_ID_HEARTBEAT", 0), 1),
            (getattr(mav, "MAVLINK_MSG_ID_ATTITUDE", 30), 80),
            (getattr(mav, "MAVLINK_MSG_ID_VFR_HUD", 74), 25),
            (getattr(mav, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33), 20),
            (getattr(mav, "MAVLINK_MSG_ID_GPS_RAW_INT", 24), 10),
            (getattr(mav, "MAVLINK_MSG_ID_SYS_STATUS", 1), 10),
            (getattr(mav, "MAVLINK_MSG_ID_BATTERY_STATUS", 147), 10),
            (getattr(mav, "MAVLINK_MSG_ID_RC_CHANNELS", 65), 50),
            (getattr(mav, "MAVLINK_MSG_ID_RADIO_STATUS", 109), 5),
            (getattr(mav, "MAVLINK_MSG_ID_DISTANCE_SENSOR", 132), 20),
            (getattr(mav, "MAVLINK_MSG_ID_OPTICAL_FLOW", 100), 20),
            (getattr(mav, "MAVLINK_MSG_ID_OPTICAL_FLOW_RAD", 106), 20),
            (getattr(mav, "MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4", 11030), 25),
            (getattr(mav, "MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8", 11031), 25),
            (getattr(mav, "MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12", 11032), 25),
            (getattr(mav, "MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16", 11033), 25),
            (getattr(mav, "MAVLINK_MSG_ID_FENCE_STATUS", 162), 5),
            (getattr(mav, "MAVLINK_MSG_ID_HOME_POSITION", 242), 1),
        ]
        for msg_id, hz in wanted:
            self._set_message_interval(msg_id, hz)
        # Compatibility stream requests for older ArduPilot behaviour.
        for stream_id, hz in [(0, 20), (2, 10), (3, 50), (6, 20), (10, 80), (11, 25), (12, 10)]:
            try:
                self.master.mav.request_data_stream_send(
                    self.master.target_system, self.master.target_component, stream_id, hz, 1
                )
            except Exception:
                pass

    def request_fence_params(self):
        """Ask ArduPilot for fence/home parameters used by the dashboard warning panel."""
        if self.master is None:
            return
        for name in ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_RADIUS", "FENCE_MARGIN"):
            try:
                self.master.mav.param_request_read_send(
                    self.master.target_system,
                    self.master.target_component,
                    name.encode("ascii"),
                    -1,
                )
            except Exception:
                pass
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                getattr(mavutil.mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242),
                0, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    def run(self):
        self.running = True
        try:
            self.status.emit("MAVLink: connecting...")
            self.master = self.connect_master()
            hb = self.master.wait_heartbeat(timeout=10)
            if hb is None:
                raise TimeoutError("No heartbeat received")
            self.data["connected"] = True
            self.data["system_id"] = getattr(self.master, "target_system", None)
            self.data["component_id"] = getattr(self.master, "target_component", None)
            self.status.emit(f"MAVLink: connected sys={self.data['system_id']} comp={self.data['component_id']}")
            self.request_streams()
            self.request_fence_params()
            last_emit = 0.0
            last_request = time.time()
            last_param_request = time.time()
            while self.running:
                msg = self.master.recv_match(blocking=True, timeout=0.03)
                if msg is not None:
                    self.parse_msg(msg)
                    self.data["last_msg_time"] = time.time()
                    # v51: event-driven UI update. Data widgets update when the packet arrives,
                    # not on a slow fixed dashboard timer.
                    self.telemetry.emit(dict(self.data))
                now = time.time()
                if now - last_request > 3.0:
                    self.request_streams()
                    last_request = now
                if now - last_param_request > 15.0:
                    self.request_fence_params()
                    last_param_request = now
                # v51: no fixed 25 Hz telemetry throttle here; packets emit immediately above.
        except PermissionError:
            self.status.emit(f"MAVLink: permission denied on {self.connection_string}. Close Mission Planner or use UDP forwarding.")
        except Exception as e:
            self.status.emit(f"MAVLink: connection failed - {e}")
        finally:
            self._close_master()
            self.data["connected"] = False
            self.telemetry.emit(dict(self.data))
            self.status.emit("MAVLink: stopped")

    def parse_msg(self, msg):
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            return
        self.data["connected"] = True
        if mtype == "HEARTBEAT":
            base_mode = getattr(msg, "base_mode", 0)
            self.data["armed"] = bool(base_mode & 128)
            try:
                self.data["mode"] = mavutil.mode_string_v10(msg)
            except Exception:
                self.data["mode"] = str(getattr(msg, "custom_mode", "-"))
        elif mtype == "ATTITUDE":
            self.data["roll_deg"] = math.degrees(getattr(msg, "roll", 0.0))
            self.data["pitch_deg"] = math.degrees(getattr(msg, "pitch", 0.0))
            yaw = math.degrees(getattr(msg, "yaw", 0.0)) % 360.0
            self.data["yaw_deg"] = yaw
            self.data["heading_deg"] = yaw
            try:
                self.attitude.emit(
                    float(self.data.get("roll_deg", 0.0)),
                    float(self.data.get("pitch_deg", 0.0)),
                    float(self.data.get("heading_deg", 0.0)),
                )
            except Exception:
                pass
        elif mtype == "VFR_HUD":
            self.data["airspeed_m_s"] = safe_float(getattr(msg, "airspeed", 0.0))
            self.data["groundspeed_m_s"] = safe_float(getattr(msg, "groundspeed", 0.0))
            self.data["gps_alt_m"] = safe_float(getattr(msg, "alt", 0.0))
            self.data["climb_m_s"] = safe_float(getattr(msg, "climb", 0.0))
            heading = getattr(msg, "heading", None)
            if heading is not None and int(heading) >= 0:
                self.data["heading_deg"] = safe_float(heading) % 360.0
            self.data["throttle_percent"] = safe_float(getattr(msg, "throttle", 0.0))
        elif mtype == "GLOBAL_POSITION_INT":
            lat = getattr(msg, "lat", 0)
            lon = getattr(msg, "lon", 0)
            if lat != 0 and lon != 0:
                self.data["lat"] = lat / 1e7
                self.data["lon"] = lon / 1e7
            self.data["gps_alt_m"] = safe_float(getattr(msg, "alt", 0)) / 1000.0
            self.data["rel_alt_m"] = safe_float(getattr(msg, "relative_alt", 0)) / 1000.0
            vx = safe_float(getattr(msg, "vx", 0)) / 100.0
            vy = safe_float(getattr(msg, "vy", 0)) / 100.0
            vz = safe_float(getattr(msg, "vz", 0)) / 100.0
            self.data["groundspeed_m_s"] = math.sqrt(vx * vx + vy * vy)
            self.data["climb_m_s"] = -vz
            hdg = getattr(msg, "hdg", 65535)
            if hdg != 65535:
                self.data["heading_deg"] = safe_float(hdg) / 100.0
        elif mtype == "GPS_RAW_INT":
            self.data["gps_fix_type"] = int(getattr(msg, "fix_type", 0))
            self.data["gps_sats"] = int(getattr(msg, "satellites_visible", 0))
        elif mtype == "PARAM_VALUE":
            pid = getattr(msg, "param_id", b"")
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", errors="ignore").strip("\x00")
            else:
                pid = str(pid).strip("\x00")
            val = safe_float(getattr(msg, "param_value", None), None)
            if pid == "FENCE_ALT_MAX":
                self.data["fence_alt_max_m"] = val
            elif pid == "FENCE_RADIUS":
                self.data["fence_radius_m"] = val
            elif pid == "FENCE_ENABLE":
                self.data["fence_enabled"] = val
            elif pid == "FENCE_TYPE":
                self.data["fence_type"] = val
        elif mtype == "HOME_POSITION":
            lat = getattr(msg, "latitude", 0)
            lon = getattr(msg, "longitude", 0)
            alt = getattr(msg, "altitude", 0)
            if lat and lon:
                self.data["home_lat"] = lat / 1e7
                self.data["home_lon"] = lon / 1e7
            if alt is not None:
                self.data["home_alt_m"] = safe_float(alt) / 1000.0
        elif mtype == "FENCE_STATUS":
            self.data["fence_breach_status"] = getattr(msg, "breach_status", None)
            self.data["fence_breach_count"] = getattr(msg, "breach_count", None)
            self.data["fence_breach_type"] = getattr(msg, "breach_type", None)
            self.data["fence_breach_time"] = getattr(msg, "breach_time", None)
        elif mtype == "SYS_STATUS":
            vbatt_mv = getattr(msg, "voltage_battery", -1)
            ibatt_ca = getattr(msg, "current_battery", -1)
            if vbatt_mv is not None and vbatt_mv > 0:
                self.data["battery_voltage_v"] = vbatt_mv / 1000.0
            if ibatt_ca is not None and ibatt_ca >= 0:
                self.data["battery_current_a"] = ibatt_ca / 100.0
            rem = getattr(msg, "battery_remaining", -1)
            if rem is not None and rem >= 0:
                self.data["battery_remaining_percent"] = rem
        elif mtype == "BATTERY_STATUS":
            current_ca = getattr(msg, "current_battery", -1)
            if current_ca is not None and current_ca >= 0:
                self.data["battery_current_a"] = current_ca / 100.0
            rem = getattr(msg, "battery_remaining", -1)
            if rem is not None and rem >= 0:
                self.data["battery_remaining_percent"] = rem
            voltages = getattr(msg, "voltages", None)
            if voltages:
                valid = [v for v in voltages if v not in (0, 65535)]
                if valid:
                    self.data["battery_voltage_v"] = sum(valid) / 1000.0
        elif mtype == "RC_CHANNELS":
            channels = []
            for i in range(1, 17):
                channels.append(int(getattr(msg, f"chan{i}_raw", 0)))
            self.data["rc_channels"] = channels
            rssi = getattr(msg, "rssi", None)
            self.data["rc_rssi"] = rssi
        elif mtype == "RADIO_STATUS":
            self.data["radio_rssi"] = getattr(msg, "rssi", None)
            self.data["radio_remrssi"] = getattr(msg, "remrssi", None)
            self.data["radio_noise"] = getattr(msg, "noise", None)
            self.data["radio_remnoise"] = getattr(msg, "remnoise", None)
            self.data["rxerrors"] = getattr(msg, "rxerrors", None)
            self.data["fixed"] = getattr(msg, "fixed", None)
        elif mtype == "DISTANCE_SENSOR":
            current_cm = getattr(msg, "current_distance", None)
            if current_cm is not None and current_cm > 0:
                self.data["lidar_alt_m"] = current_cm / 100.0
        elif mtype == "OPTICAL_FLOW":
            self.data["optical_flow_quality"] = getattr(msg, "quality", None)
        elif mtype == "OPTICAL_FLOW_RAD":
            self.data["optical_flow_quality"] = getattr(msg, "quality", None)
            dist = getattr(msg, "distance", None)
            if dist is not None and dist > 0:
                self.data["optical_flow_alt_m"] = float(dist)
        elif mtype in ("ESC_TELEMETRY_1_TO_4", "ESC_TELEMETRY_5_TO_8", "ESC_TELEMETRY_9_TO_12", "ESC_TELEMETRY_13_TO_16"):
            if mtype.endswith("1_TO_4"):
                base = 0
            elif mtype.endswith("5_TO_8"):
                base = 4
            elif mtype.endswith("9_TO_12"):
                base = 8
            else:
                base = 12
            try:
                rpm_list = getattr(msg, "rpm", [None] * 4)
                volt_list = getattr(msg, "voltage", [None] * 4)
                curr_list = getattr(msg, "current", [None] * 4)
                temp_list = getattr(msg, "temperature", [None] * 4)
                for i in range(4):
                    idx = base + i
                    rpm = rpm_list[i] if i < len(rpm_list) else None
                    voltage = volt_list[i] if i < len(volt_list) else None
                    current = curr_list[i] if i < len(curr_list) else None
                    temp = temp_list[i] if i < len(temp_list) else None
                    if rpm is not None:
                        self.data["esc_rpm"][idx] = rpm
                    if voltage is not None:
                        self.data["esc_voltage"][idx] = voltage / 100.0 if voltage > 100 else voltage
                    if current is not None:
                        self.data["esc_current"][idx] = current / 100.0 if current > 100 else current
                    if temp is not None:
                        self.data["esc_temp"][idx] = temp
            except Exception:
                pass
        try:
            self._update_fence_warnings()
        except Exception:
            pass

    def _update_fence_warnings(self):
        """Dashboard-side fence margin monitor.

        ArduPilot still owns the real failsafe. This only gives early warnings:
        - altitude fence warning when within 3 m of FENCE_ALT_MAX
        - circular fence warning when within 10 m of FENCE_RADIUS from home
        """
        warnings = []
        breach = self.data.get("fence_breach_status")
        breach_type = self.data.get("fence_breach_type")
        if breach:
            warnings.append(f"BREACH type {breach_type}")

        alt_max = self.data.get("fence_alt_max_m")
        alt_source = self.data.get("lidar_alt_m")
        if alt_source is None or alt_source <= 0:
            alt_source = self.data.get("rel_alt_m")
        if alt_max is not None and alt_source is not None:
            remaining = float(alt_max) - float(alt_source)
            self.data["fence_alt_remaining_m"] = remaining
            if remaining <= 0:
                warnings.append(f"ALT OVER {abs(remaining):.1f}m")
            elif remaining <= 3.0:
                warnings.append(f"ALT {remaining:.1f}m left")

        radius = self.data.get("fence_radius_m")
        lat, lon = self.data.get("lat"), self.data.get("lon")
        home_lat, home_lon = self.data.get("home_lat"), self.data.get("home_lon")
        if home_lat is None and lat is not None:
            # Fallback only for display if HOME_POSITION has not arrived yet.
            self.data["home_lat"] = lat
            self.data["home_lon"] = lon
            home_lat, home_lon = lat, lon
        if radius is not None and lat is not None and lon is not None and home_lat is not None and home_lon is not None:
            dist_home = haversine_m(home_lat, home_lon, lat, lon)
            remaining = float(radius) - float(dist_home)
            self.data["fence_radius_remaining_m"] = remaining
            if remaining <= 0:
                warnings.append(f"RADIUS OVER {abs(remaining):.1f}m")
            elif remaining <= 10.0:
                warnings.append(f"RADIUS {remaining:.1f}m left")

        if not warnings:
            parts = []
            if self.data.get("fence_alt_remaining_m") is not None:
                parts.append(f"Alt {self.data['fence_alt_remaining_m']:.1f}m")
            if self.data.get("fence_radius_remaining_m") is not None:
                parts.append(f"Rad {self.data['fence_radius_remaining_m']:.1f}m")
            self.data["fence_warning"] = "Fence OK " + " | ".join(parts) if parts else "Fence: waiting"
        else:
            self.data["fence_warning"] = " | ".join(warnings)

    def send_guided_goto(self, lat: float, lon: float, alt_m: float, speed_m_s: float = 3.0) -> Tuple[bool, str]:
        """Guarded Guided waypoint command. Does not arm or take off."""
        if self.master is None or not self.data.get("connected"):
            return False, "No MAVLink connection."
        if lat is None or lon is None:
            return False, "No target waypoint selected."
        if self.data.get("gps_fix_type", 0) < 3:
            return False, "GPS fix is not 3D yet."
        try:
            # Switch to GUIDED first. ArduPilot Copter expects Guided for reposition commands.
            try:
                mode_id = self.master.mode_mapping().get("GUIDED")
                if mode_id is not None:
                    self.master.set_mode(mode_id)
                    time.sleep(0.15)
            except Exception:
                pass

            self.master.mav.command_int_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0,
                float(speed_m_s),  # param1: ground speed, m/s. -1 means default.
                0,                 # param2: bitmask/options.
                0,                 # param3: loiter radius, ignored by Copter Guided.
                float('nan'),      # param4: yaw unchanged.
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt_m),
            )
            self._last_command_status = f"Guided waypoint sent: {lat:.7f}, {lon:.7f}, alt {alt_m:.1f}m"
            return True, self._last_command_status
        except Exception as e:
            return False, f"Failed to send waypoint: {e}"

    def stop(self):
        self.running = False
        self._close_master()
        self.wait(1600)


# ----------------------------- TX15 USB HID monitor -----------------------------

# ----------------------------- Betaflight / iNav MSP telemetry -----------------------------

class MspTelemetryThread(QThread):
    """Read Betaflight/iNav flight-controller data over USB serial using MSP v1.

    This is monitoring-only. It does not arm, disarm, write settings, or send motor/RC commands.
    It maps MSP telemetry into the same dashboard dictionary used by MAVLink so the UI can show
    attitude, battery, GPS, altitude and RX channel values in the existing panels.
    """
    telemetry = pyqtSignal(dict)
    attitude = pyqtSignal(float, float, float)
    status = pyqtSignal(str)

    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_RC = 105
    MSP_RAW_GPS = 106
    MSP_MOTOR = 104
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110
    MSP_BATTERY_STATE = 130

    def __init__(self, port: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self.port = port.strip()
        self.baud = int(baud)
        self.running = False
        self.ser = None
        self.data = self.default_data()
        self._rx_buffer = bytearray()
        self._last_request = 0.0
        self._last_emit = 0.0
        self._last_status = 0.0
        self._packets = 0
        self._rate_counts: Dict[str, int] = {}
        self._rate_values: Dict[str, float] = {}
        self._rate_window_t = time.time()
        self._packet_rate = 0.0
        self._rate_t = time.time()

    @staticmethod
    def default_data() -> Dict[str, Any]:
        d = TelemetryThread.default_data()
        d["telemetry_protocol"] = "MSP"
        d["mode"] = "Betaflight MSP"
        d["msp_packets"] = 0
        d["msp_packet_rate_hz"] = 0.0
        d["msp_status_flags"] = None
        d["msp_sensor_flags"] = None
        d["motor_outputs"] = [None] * 16
        return d

    def _msp_request_packet(self, cmd: int, payload: bytes = b"") -> bytes:
        size = len(payload)
        checksum = size ^ (cmd & 0xFF)
        for b in payload:
            checksum ^= b
        return b"$M<" + bytes([size, cmd & 0xFF]) + payload + bytes([checksum & 0xFF])

    def _send_request(self, cmd: int):
        if self.ser is None:
            return
        try:
            self.ser.write(self._msp_request_packet(cmd))
        except Exception:
            pass

    def _request_cycle(self):
        # Keep the request set small so USB/serial stays responsive and the UI stays smooth.
        for cmd in (
            self.MSP_ATTITUDE,
            self.MSP_RC,
            self.MSP_MOTOR,
            self.MSP_ANALOG,
            self.MSP_BATTERY_STATE,
            self.MSP_STATUS,
            self.MSP_RAW_GPS,
            self.MSP_ALTITUDE,
        ):
            self._send_request(cmd)

    def _parse_packets_from_buffer(self):
        while True:
            start = self._rx_buffer.find(b"$M>")
            if start < 0:
                # Retain a few bytes in case header is split across reads.
                if len(self._rx_buffer) > 4:
                    del self._rx_buffer[:-3]
                return
            if start > 0:
                del self._rx_buffer[:start]
            if len(self._rx_buffer) < 6:
                return
            size = self._rx_buffer[3]
            cmd = self._rx_buffer[4]
            total = 6 + size
            if len(self._rx_buffer) < total:
                return
            payload = bytes(self._rx_buffer[5:5 + size])
            checksum = self._rx_buffer[5 + size]
            calc = size ^ cmd
            for b in payload:
                calc ^= b
            del self._rx_buffer[:total]
            if (calc & 0xFF) != checksum:
                continue
            self._packets += 1
            self.parse_msp(cmd, payload)

    @staticmethod
    def _unpack(fmt: str, payload: bytes, offset: int = 0):
        size = struct.calcsize(fmt)
        if len(payload) < offset + size:
            return None
        return struct.unpack_from(fmt, payload, offset)


    def _track_rate(self, name: str):
        now = time.time()
        self._rate_counts[name] = self._rate_counts.get(name, 0) + 1
        dt = now - self._rate_window_t
        if dt >= 1.0:
            for key, count in list(self._rate_counts.items()):
                self._rate_values[key] = count / max(0.001, dt)
                self.data[f"{key}_rate_hz"] = self._rate_values[key]
            self.data["telemetry_rate_hz"] = self._rate_values.get("telemetry", 0.0)
            self.data["attitude_rate_hz"] = self._rate_values.get("attitude", 0.0)
            self.data["gps_rate_hz"] = self._rate_values.get("gps", 0.0)
            self.data["esc_rate_hz"] = self._rate_values.get("esc", 0.0)
            self.data["battery_rate_hz"] = self._rate_values.get("battery", 0.0)
            self.data["rc_rate_hz"] = self._rate_values.get("rc", 0.0)
            self._rate_counts.clear()
            self._rate_window_t = now

    def _track_rate_by_msp_cmd(self, cmd: int):
        self._track_rate("telemetry")
        if cmd == self.MSP_ATTITUDE:
            self._track_rate("attitude")
        elif cmd in (self.MSP_RAW_GPS, self.MSP_ALTITUDE):
            self._track_rate("gps")
        elif cmd == self.MSP_MOTOR:
            self._track_rate("esc")
        elif cmd in (self.MSP_ANALOG, self.MSP_BATTERY_STATE):
            self._track_rate("battery")
        elif cmd == self.MSP_RC:
            self._track_rate("rc")

    def parse_msp(self, cmd: int, payload: bytes):
        now = time.time()
        self.data["connected"] = True
        self.data["telemetry_protocol"] = "MSP"
        self.data["last_msg_time"] = now
        self._track_rate_by_msp_cmd(cmd)

        try:
            if cmd == self.MSP_ATTITUDE and len(payload) >= 6:
                roll10, pitch10, yaw = struct.unpack_from("<hhh", payload, 0)
                roll = roll10 / 10.0
                pitch = pitch10 / 10.0
                heading = float(yaw) % 360.0
                self.data["roll_deg"] = roll
                self.data["pitch_deg"] = pitch
                self.data["yaw_deg"] = heading
                self.data["heading_deg"] = heading
                self.data["last_attitude_time"] = now
                self.data["last_attitude_source"] = "MSP_ATTITUDE"
                self.data["attitude_msg_count"] = int(self.data.get("attitude_msg_count", 0)) + 1
                self.attitude.emit(roll, pitch, heading)

            elif cmd == self.MSP_RC:
                channels = []
                for i in range(min(16, len(payload) // 2)):
                    channels.append(int(struct.unpack_from("<H", payload, i * 2)[0]))
                while len(channels) < 14:
                    channels.append(0)
                self.data["rc_channels"] = channels[:14]

            elif cmd == self.MSP_MOTOR:
                motors = []
                for i in range(min(16, len(payload) // 2)):
                    motors.append(int(struct.unpack_from("<H", payload, i * 2)[0]))
                while len(motors) < 16:
                    motors.append(None)
                self.data["motor_outputs"] = motors[:16]

            elif cmd == self.MSP_ANALOG and len(payload) >= 1:
                # Legacy ANALOG: vbat is commonly reported in 0.1V units.
                vbat = payload[0] / 10.0
                if vbat > 0:
                    self.data["battery_voltage_v"] = vbat
                if len(payload) >= 7:
                    try:
                        rssi = struct.unpack_from("<H", payload, 3)[0]
                        amp_raw = struct.unpack_from("<h", payload, 5)[0]
                        self.data["rc_rssi"] = rssi
                        # Betaflight legacy amperage is often in 0.01A units. Use cautiously.
                        if abs(amp_raw) < 50000:
                            self.data["battery_current_a"] = max(0.0, amp_raw / 100.0)
                    except Exception:
                        pass

            elif cmd == self.MSP_BATTERY_STATE and len(payload) >= 9:
                cell_count = int(payload[0])
                capacity_mah = struct.unpack_from("<H", payload, 1)[0]
                voltage = payload[3] / 10.0
                mah_drawn = struct.unpack_from("<H", payload, 4)[0]
                amp_raw = struct.unpack_from("<h", payload, 6)[0]
                battery_state = int(payload[8])
                if voltage > 0:
                    self.data["battery_voltage_v"] = voltage
                self.data["battery_current_a"] = max(0.0, amp_raw / 100.0)
                self.data["battery_remaining_percent"] = None
                self.data["msp_cell_count"] = cell_count
                self.data["msp_capacity_mah"] = capacity_mah
                self.data["msp_mah_drawn"] = mah_drawn
                self.data["msp_battery_state"] = battery_state

            elif cmd == self.MSP_RAW_GPS and len(payload) >= 16:
                fix, sats = struct.unpack_from("<BB", payload, 0)
                lat = struct.unpack_from("<i", payload, 2)[0]
                lon = struct.unpack_from("<i", payload, 6)[0]
                alt_m = struct.unpack_from("<h", payload, 10)[0]
                speed_cm_s = struct.unpack_from("<H", payload, 12)[0]
                course10 = struct.unpack_from("<H", payload, 14)[0]
                self.data["gps_fix_type"] = int(fix)
                self.data["gps_sats"] = int(sats)
                if lat != 0 and lon != 0:
                    self.data["lat"] = lat / 1e7
                    self.data["lon"] = lon / 1e7
                self.data["gps_alt_m"] = float(alt_m)
                self.data["groundspeed_m_s"] = speed_cm_s / 100.0
                self.data["heading_deg"] = (course10 / 10.0) % 360.0

            elif cmd == self.MSP_ALTITUDE and len(payload) >= 6:
                est_alt_cm = struct.unpack_from("<i", payload, 0)[0]
                vario_cm_s = struct.unpack_from("<h", payload, 4)[0]
                self.data["rel_alt_m"] = est_alt_cm / 100.0
                self.data["climb_m_s"] = vario_cm_s / 100.0

            elif cmd == self.MSP_STATUS and len(payload) >= 11:
                cycle_time, i2c_errors, sensors = struct.unpack_from("<HHH", payload, 0)
                flags = struct.unpack_from("<I", payload, 6)[0]
                self.data["msp_sensor_flags"] = int(sensors)
                self.data["msp_status_flags"] = int(flags)
                self.data["mode"] = "Betaflight MSP"
                # MSP_STATUS mode flags are box-mode bitmasks, not a universal arming-state API.
                # Keep armed conservative unless a later MSP command provides a clear state.
                self.data["armed"] = False
        except Exception as e:
            self.status.emit(f"MSP parse warning cmd={cmd}: {e}")
        # v51: emit immediately after each valid MSP response packet so Betaflight data
        # updates at the speed USB/MSP is actually providing.
        self.telemetry.emit(dict(self.data))

    def run(self):
        self.running = True
        if not SERIAL_AVAILABLE or serial is None:
            self.status.emit("MSP: pyserial is not installed. Install: pip install pyserial")
            self.telemetry.emit(dict(self.data))
            return
        try:
            if not self.port:
                raise RuntimeError("Empty MSP/serial port")
            self.status.emit(f"MSP: connecting {self.port} @ {self.baud}...")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.001, write_timeout=0.05)
            self.data["connected"] = True
            self.status.emit(f"MSP: connected {self.port} @ {self.baud} | requesting Betaflight USB data")
        except Exception as e:
            self.data["connected"] = False
            self.status.emit(f"MSP: connection failed - {e}")
            self.telemetry.emit(dict(self.data))
            return

        try:
            while self.running:
                now = time.time()
                if now - self._last_request >= 0.02:  # v51 ~50 Hz request cycle when the USB link can handle it
                    self._request_cycle()
                    self._last_request = now
                try:
                    chunk = self.ser.read(4096)
                    if chunk:
                        self._rx_buffer.extend(chunk)
                        self._parse_packets_from_buffer()
                except Exception as e:
                    self.status.emit(f"MSP read error: {e}")
                    self.msleep(50)

                if now - self._rate_t >= 1.0:
                    self._packet_rate = self._packets / max(0.001, now - self._rate_t)
                    self.data["msp_packets"] = int(self._packets)
                    self.data["msp_packet_rate_hz"] = self._packet_rate
                    self._packets = 0
                    self._rate_t = now

                # v51: no fixed 30 Hz MSP UI throttle; parse_msp emits on every response packet.

                if now - self._last_status >= 2.0:
                    self.status.emit(
                        f"MSP: connected | packets {self.data.get('msp_packet_rate_hz',0):.0f}Hz | "
                        f"ATT age {now - self.data.get('last_attitude_time', now):.1f}s"
                    )
                    self._last_status = now
                self.msleep(1)
        finally:
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.data["connected"] = False
            self.telemetry.emit(dict(self.data))
            self.status.emit("MSP: stopped")

    def stop(self):
        self.running = False
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.wait(1500)


class TxJoystickThread(QThread):
    tx_data = pyqtSignal(dict)
    status = pyqtSignal(str)
    tx_detected = pyqtSignal(str)

    def __init__(self, joystick_index: Optional[int] = None, parent=None):
        super().__init__(parent)
        self.joystick_index = joystick_index
        self.running = False
        self.joy = None
        self._did_emit_detect = False

    @staticmethod
    def discover_devices(force_rescan: bool = True) -> List[Dict[str, Any]]:
        """Discover USB joystick/HID radios, including devices plugged in after boot.

        pygame/SDL can cache joystick devices on Windows. Re-initialising the
        joystick subsystem before scanning makes RadioMaster/EdgeTX hot-plug
        much more reliable, so the TX15 can be connected after this program is
        already running.
        """
        devices: List[Dict[str, Any]] = []
        if not PYGAME_AVAILABLE:
            return devices
        try:
            pygame.init()
            if force_rescan:
                try:
                    pygame.joystick.quit()
                except Exception:
                    pass
                try:
                    pygame.event.pump()
                except Exception:
                    pass
                pygame.joystick.init()
            else:
                pygame.joystick.init()

            count = pygame.joystick.get_count()
            if count <= 0 and force_rescan:
                time.sleep(0.08)
                try:
                    pygame.joystick.quit()
                except Exception:
                    pass
                pygame.joystick.init()
                count = pygame.joystick.get_count()

            for i in range(count):
                try:
                    j = pygame.joystick.Joystick(i)
                    j.init()
                    name = j.get_name() or f"Joystick {i}"
                    guid = j.get_guid() if hasattr(j, "get_guid") else ""
                    devices.append({
                        "index": i,
                        "name": name,
                        "guid": guid,
                        "axes": j.get_numaxes(),
                        "buttons": j.get_numbuttons(),
                        "hats": j.get_numhats(),
                    })
                except Exception:
                    continue
        except Exception:
            pass
        return devices

    @staticmethod
    def best_tx_index(devices: List[Dict[str, Any]]) -> Optional[int]:
        if not devices:
            return None
        keys = ["radiomaster", "tx15", "edgetx", "opentx", "joystick", "controller"]
        for d in devices:
            name = (d.get("name") or "").lower()
            if any(k in name for k in keys):
                return int(d.get("index", 0))
        if len(devices) == 1:
            return int(devices[0].get("index", 0))
        return None

    @staticmethod
    def axis_to_pwm(v: float, invert: bool = False) -> int:
        # Most joystick axes are -1..+1. Map to 1000..2000. Some radios may need reversing in EdgeTX.
        x = -v if invert else v
        return int(clamp(1500 + x * 500, 1000, 2000))

    def run(self):
        if not PYGAME_AVAILABLE:
            self.status.emit("TX USB: pygame not installed. Install with: pip install pygame")
            return
        self.running = True
        try:
            pygame.init()
            pygame.joystick.init()
            if self.joystick_index is None:
                devices = self.discover_devices()
                self.joystick_index = self.best_tx_index(devices)
            if self.joystick_index is None:
                self.status.emit("TX USB: no joystick/HID radio detected")
                return
            self.joy = pygame.joystick.Joystick(int(self.joystick_index))
            self.joy.init()
            name = self.joy.get_name()
            guid = self.joy.get_guid() if hasattr(self.joy, "get_guid") else "-"
            self.status.emit(f"TX USB: connected {name}")
            self.tx_detected.emit(name)
            last_emit = 0.0
            frames = 0
            rate_t0 = time.time()
            update_rate = 0.0
            while self.running:
                pygame.event.pump()
                axes_raw = [float(self.joy.get_axis(i)) for i in range(self.joy.get_numaxes())]
                buttons = [int(self.joy.get_button(i)) for i in range(self.joy.get_numbuttons())]
                hats = [self.joy.get_hat(i) for i in range(self.joy.get_numhats())]
                pwm = [1500] * 16
                for i, v in enumerate(axes_raw[:16]):
                    pwm[i] = self.axis_to_pwm(v)
                # Use buttons/hats to fill upper channels for visibility if the radio sends few axes.
                for bi, b in enumerate(buttons[:8]):
                    idx = 8 + bi
                    if idx < 16:
                        pwm[idx] = 2000 if b else 1000
                frames += 1
                now = time.time()
                if now - rate_t0 >= 1.0:
                    update_rate = frames / (now - rate_t0)
                    frames = 0
                    rate_t0 = now
                if now - last_emit >= 0.01:
                    self.tx_data.emit({
                        "connected": True,
                        "name": name,
                        "guid": guid,
                        "axes_count": self.joy.get_numaxes(),
                        "buttons_count": self.joy.get_numbuttons(),
                        "hats_count": self.joy.get_numhats(),
                        "axes_raw": axes_raw,
                        "buttons": buttons,
                        "hats": hats,
                        "rc_channels": pwm,
                        "update_rate_hz": update_rate,
                        "timestamp": now,
                    })
                    last_emit = now
                self.msleep(5)
        except Exception as e:
            self.status.emit(f"TX USB error: {e}")
        finally:
            try:
                if self.joy is not None:
                    self.joy.quit()
            except Exception:
                pass
            self.tx_data.emit({"connected": False, "rc_channels": [0] * 16})
            self.status.emit("TX USB: stopped")

    def stop(self):
        self.running = False
        self.wait(1200)


# ----------------------------- AI + FPV Thread -----------------------------

@dataclass
class AISettings:
    video_source: str = "0"
    requested_fps: int = 60
    ai_enabled: bool = False
    model_path: str = "yolov8n.pt"
    confidence: float = 0.35
    detect_every_n_frames: int = 3
    save_every_n_frames: int = 10

    # FPV/capture tuning. These keep FPV smooth even when AI is heavy.
    capture_backend: str = "DirectShow"   # Auto, DirectShow, MSMF, FFMPEG, GStreamer
    fpv_width: int = 1280
    fpv_height: int = 720
    fourcc: str = "MJPG"                  # MJPG, YUY2, H264, Auto
    buffer_size: int = 1
    display_scale: float = 1.0             # 1.0 = full frame, 0.75/0.5 = lighter UI copies

    # AI tuning. The AI worker always runs asynchronously from FPV capture.
    ai_device: str = "auto"               # auto, cuda:0, cpu
    ai_imgsz: int = 640
    ai_half: bool = True
    ai_max_det: int = 50
    ai_input_width: int = 960              # downscale frame for inference then scale boxes back
    ai_async: bool = True


class AIFrameThread(QThread):
    """v100 split-feed FPV pipeline.

    The live FPV feed and the AI feed are intentionally separated:
    - Capture loop always writes the newest camera frame into a shared buffer.
    - The UI pulls the newest frame at ~60 Hz, so Qt does not build a laggy queue.
    - AI receives a copied/downscaled newest frame in a background worker.
    - AI returns only boxes/labels. The live FPV panel overlays those boxes itself.
    - Optional raw FPV recording writes directly from the capture thread at requested FPS.
    """
    frame_ready = pyqtSignal(np.ndarray, list)  # kept for backwards compatibility, not used by v90 main loop
    status = pyqtSignal(str)
    fps_measured = pyqtSignal(float)
    ai_rate = pyqtSignal(float, float)  # ai_fps, inference_ms
    recording_status = pyqtSignal(str)

    def __init__(self, settings: AISettings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.running = False
        self.cap = None
        self.model = None
        self.face_cascade = None
        self.hog = None
        self.frame_index = 0
        self.ai_runtime_enabled = bool(settings.ai_enabled)
        self._model_load_attempted = False
        self._last_inference_ms = 0.0
        self._last_ai_fps = 0.0
        self._last_ai_status_t = 0.0
        self._ai_frames = 0
        self._ai_rate_t0 = time.time()
        self._latest_ai_frame = None
        self._latest_detections: List[Dict[str, Any]] = []
        self._ai_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_id = 0
        self._latest_frame_time = 0.0
        self._ai_thread = None
        self._new_ai_frame_event = threading.Event()
        self._stop_ai_event = threading.Event()
        self.capture_rate_hz = 0.0
        self.recording = False
        self._record_writer = None
        self._record_path = None
        self._record_lock = threading.Lock()
        self._record_fps = float(max(1, int(getattr(settings, 'requested_fps', 60))))
        self._record_size = None

    def set_ai_enabled(self, enabled: bool):
        self.ai_runtime_enabled = bool(enabled)
        if enabled:
            self._ensure_ai_worker()
        else:
            with self._ai_lock:
                self._latest_detections = []
        self.status.emit("Computer vision: ON - split-feed async worker" if enabled else "Computer vision: OFF - smooth FPV only")

    def update_ai_settings(self, settings: AISettings):
        old_model = self.settings.model_path
        self.settings.confidence = float(settings.confidence)
        self.settings.detect_every_n_frames = int(settings.detect_every_n_frames)
        self.settings.save_every_n_frames = int(settings.save_every_n_frames)
        self.settings.ai_device = settings.ai_device
        self.settings.ai_imgsz = int(settings.ai_imgsz)
        self.settings.ai_half = bool(settings.ai_half)
        self.settings.ai_max_det = int(settings.ai_max_det)
        self.settings.ai_input_width = int(settings.ai_input_width)
        self.settings.ai_async = True
        if settings.model_path != old_model:
            self.settings.model_path = settings.model_path
            self.model = None
            self._model_load_attempted = False
            self.status.emit(f"AI model changed; will load on next CV start: {settings.model_path}")
        if self.ai_runtime_enabled:
            self._ensure_ai_worker()

    def get_latest_frame_and_detections(self):
        """Pull newest data from the capture thread. This avoids Qt event-queue backlog."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None, [], self._latest_frame_id, self._latest_frame_time
            frame = self._latest_frame.copy()
            frame_id = self._latest_frame_id
            frame_time = self._latest_frame_time
        with self._ai_lock:
            dets = list(self._latest_detections) if self.ai_runtime_enabled else []
        return frame, dets, frame_id, frame_time

    def start_fpv_recording(self, base_dir: str, fps: Optional[float] = None) -> str:
        base = Path(base_dir).expanduser()
        base.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('fpv_60fps_%Y%m%d_%H%M%S')
        path = base / f'{stamp}.avi'
        with self._record_lock:
            self._record_path = str(path)
            self._record_fps = float(fps or self.settings.requested_fps or 60)
            self.recording = True
            self._record_writer = None
            self._record_size = None
        self.recording_status.emit(f"Raw FPV recording armed: {path}")
        return str(path)

    def stop_fpv_recording(self) -> str:
        with self._record_lock:
            self.recording = False
            path = self._record_path or ''
            try:
                if self._record_writer is not None:
                    self._record_writer.release()
            except Exception:
                pass
            self._record_writer = None
            self._record_path = None
            self._record_size = None
        self.recording_status.emit(f"Raw FPV recording stopped: {path}")
        return path

    def _write_record_frame(self, frame: np.ndarray):
        with self._record_lock:
            if not self.recording or not self._record_path:
                return
            h, w = frame.shape[:2]
            if self._record_writer is None:
                # MJPG AVI is intentionally used for low CPU load and stable 60 FPS recording.
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                self._record_size = (w, h)
                self._record_writer = cv2.VideoWriter(self._record_path, fourcc, max(1.0, self._record_fps), (w, h))
                if not self._record_writer.isOpened():
                    self.recording_status.emit('Raw FPV recording failed to open VideoWriter')
                    self.recording = False
                    return
            if self._record_size != (w, h):
                # Keep writer stable if capture backend changes resolution unexpectedly.
                frame = cv2.resize(frame, self._record_size, interpolation=cv2.INTER_AREA)
            self._record_writer.write(frame)

    def _backend_flag(self):
        backend = str(getattr(self.settings, "capture_backend", "Auto")).lower()
        if backend.startswith("direct") and os.name == "nt":
            return cv2.CAP_DSHOW
        if backend.startswith("msmf") and os.name == "nt":
            return cv2.CAP_MSMF
        if backend.startswith("ffmpeg"):
            return cv2.CAP_FFMPEG
        if backend.startswith("gstream"):
            return cv2.CAP_GSTREAMER
        return 0

    def _open_capture(self):
        src_text = self.settings.video_source.strip()
        try:
            src = int(src_text)
        except Exception:
            src = src_text
        backend_flag = self._backend_flag()
        cap = cv2.VideoCapture(src, backend_flag) if backend_flag else cv2.VideoCapture(src)
        if not cap.isOpened():
            return None
        # Order matters for many USB capture devices: FOURCC first, then resolution/FPS.
        fourcc = str(getattr(self.settings, "fourcc", "MJPG")).upper().strip()
        if fourcc and fourcc != "AUTO":
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
            except Exception:
                pass
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, max(1, int(self.settings.buffer_size)))
        except Exception:
            pass
        if int(self.settings.fpv_width) > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.settings.fpv_width))
        if int(self.settings.fpv_height) > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.settings.fpv_height))
        cap.set(cv2.CAP_PROP_FPS, float(self.settings.requested_fps))
        return cap

    def _load_fallbacks(self):
        try:
            cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
        except Exception:
            self.face_cascade = None
        try:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        except Exception:
            self.hog = None

    def _resolve_device(self) -> str:
        dev = str(getattr(self.settings, "ai_device", "auto")).strip().lower()
        if dev == "auto":
            try:
                import torch
                return "cuda:0" if torch.cuda.is_available() else "cpu"
            except Exception:
                return "cpu"
        return dev

    def _load_yolo_if_needed(self):
        if self.model is not None or self._model_load_attempted:
            return
        self._model_load_attempted = True
        if not ULTRALYTICS_AVAILABLE:
            self.status.emit("AI: ultralytics not installed; using OpenCV fallback")
            return
        try:
            # Allow Torch/OpenCV to use the machine properly without locking the GUI thread.
            try:
                import torch
                n = max(2, (os.cpu_count() or 8) - 2)
                torch.set_num_threads(n)
                torch.set_num_interop_threads(max(1, min(4, n // 2)))
                if torch.cuda.is_available():
                    torch.backends.cudnn.benchmark = True
                    try:
                        torch.set_float32_matmul_precision('high')
                    except Exception:
                        pass
            except Exception:
                pass
            self.status.emit(f"AI: loading {self.settings.model_path} on {self._resolve_device()}...")
            self.model = YOLO(self.settings.model_path)
            self.status.emit(f"AI: loaded {self.settings.model_path}")
        except Exception as e:
            self.status.emit(f"AI model load failed: {e}. Using OpenCV fallback.")
            self.model = None

    @staticmethod
    def normalize_label(label: str) -> str:
        l = (label or "").lower().strip()
        if l == "person":
            return "human"
        if l in ("car", "truck", "bus", "motorbike", "motorcycle", "bicycle", "train"):
            return "vehicle" if l != "car" else "car"
        if l in ("dog", "cat", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"):
            return "animal"
        if l == "bird":
            return "bird"
        return l

    def _resize_for_ai(self, frame: np.ndarray) -> Tuple[np.ndarray, float, float]:
        h, w = frame.shape[:2]
        target_w = int(clamp(int(getattr(self.settings, "ai_input_width", 960)), 320, max(320, w)))
        if target_w >= w:
            return frame, 1.0, 1.0
        scale = target_w / float(w)
        target_h = max(1, int(h * scale))
        small = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        return small, w / float(target_w), h / float(target_h)

    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        detections: List[Dict[str, Any]] = []
        if not self.ai_runtime_enabled:
            return detections
        self._load_yolo_if_needed()
        original_h, original_w = frame.shape[:2]
        ai_frame, sx, sy = self._resize_for_ai(frame)
        t0 = time.time()
        if self.model is not None:
            try:
                device = self._resolve_device()
                results = self.model.predict(
                    ai_frame,
                    conf=float(self.settings.confidence),
                    imgsz=int(self.settings.ai_imgsz),
                    device=device,
                    half=(device.startswith("cuda") and bool(self.settings.ai_half)),
                    max_det=int(self.settings.ai_max_det),
                    verbose=False,
                )
                for r in results:
                    names = r.names
                    boxes = getattr(r, "boxes", None)
                    if boxes is None:
                        continue
                    for b in boxes:
                        cls_id = int(b.cls[0]) if b.cls is not None else -1
                        conf = float(b.conf[0]) if b.conf is not None else 0.0
                        xyxy = b.xyxy[0].detach().cpu().numpy().astype(float).tolist()
                        raw_label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
                        label = self.normalize_label(raw_label)
                        x1 = int(clamp(xyxy[0] * sx, 0, original_w - 1)); y1 = int(clamp(xyxy[1] * sy, 0, original_h - 1))
                        x2 = int(clamp(xyxy[2] * sx, 0, original_w - 1)); y2 = int(clamp(xyxy[3] * sy, 0, original_h - 1))
                        area_pct = max(0.0, (x2 - x1) * (y2 - y1) / max(1.0, original_w * original_h) * 100.0)
                        detections.append({"label": label, "raw_label": raw_label, "confidence": conf, "x1": x1, "y1": y1, "x2": x2, "y2": y2, "area_pct": area_pct, "source": "YOLO-split"})
            except Exception as e:
                self.status.emit(f"AI predict error: {e}")
                self.model = None
        # Keep OpenCV face detection as a light augment/fallback.
        try:
            ai_frame, sx, sy = self._resize_for_ai(frame)
            if self.face_cascade is not None and not self.face_cascade.empty():
                gray = cv2.cvtColor(ai_frame, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.2, 5, minSize=(24, 24))
                for (x, y, fw, fh) in faces[:16]:
                    x1 = int(x * sx); y1 = int(y * sy); x2 = int((x + fw) * sx); y2 = int((y + fh) * sy)
                    area_pct = max(0.0, (x2 - x1) * (y2 - y1) / max(1.0, original_w * original_h) * 100.0)
                    detections.append({"label": "face", "raw_label": "face", "confidence": 0.70, "x1": x1, "y1": y1, "x2": x2, "y2": y2, "area_pct": area_pct, "source": "OpenCV-Haar"})
        except Exception:
            pass
        if self.model is None:
            try:
                ai_frame, sx, sy = self._resize_for_ai(frame)
                if self.hog is not None:
                    rects, weights = self.hog.detectMultiScale(ai_frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
                    for (x, y, rw, rh), weight in zip(rects[:8], weights[:8]):
                        conf = float(weight) if not isinstance(weight, np.ndarray) else float(weight[0])
                        x1 = int(x * sx); y1 = int(y * sy); x2 = int((x + rw) * sx); y2 = int((y + rh) * sy)
                        area_pct = max(0.0, (x2 - x1) * (y2 - y1) / max(1.0, original_w * original_h) * 100.0)
                        detections.append({"label": "human", "raw_label": "person", "confidence": conf, "x1": x1, "y1": y1, "x2": x2, "y2": y2, "area_pct": area_pct, "source": "OpenCV-HOG"})
            except Exception:
                pass
        self._last_inference_ms = (time.time() - t0) * 1000.0
        self._ai_frames += 1
        now = time.time()
        if now - self._ai_rate_t0 >= 1.0:
            self._last_ai_fps = self._ai_frames / max(0.001, now - self._ai_rate_t0)
            self._ai_frames = 0
            self._ai_rate_t0 = now
            self.ai_rate.emit(self._last_ai_fps, self._last_inference_ms)
            if now - getattr(self, "_last_ai_status_t", 0.0) >= 3.0:
                self._last_ai_status_t = now
                self.status.emit(f"AI split-feed: {self._last_ai_fps:.1f} FPS | {self._last_inference_ms:.0f} ms | boxes {len(detections)}")
        return detections

    @staticmethod
    def draw_detections(frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        if not detections:
            return frame
        out = frame.copy()
        for d in detections:
            x1, y1, x2, y2 = int(d["x1"]), int(d["y1"]), int(d["x2"]), int(d["y2"])
            area = safe_float(d.get("area_pct"), 0.0)
            label = f"{d.get('label','?')} {safe_float(d.get('confidence')):.2f} | {area:.1f}%"
            cv2.rectangle(out, (x1, y1), (x2, y2), (5, 20, 5), 5)
            cv2.rectangle(out, (x1, y1), (x2, y2), (40, 255, 120), 2)
            tw, th = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0]
            y_text = max(22, y1 - 7)
            cv2.rectangle(out, (x1, y_text - th - 7), (x1 + tw + 8, y_text + 4), (0, 0, 0), -1)
            cv2.putText(out, label, (x1 + 4, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 255, 150), 2)
        return out

    def _ensure_ai_worker(self):
        if self._ai_thread is not None and self._ai_thread.is_alive():
            return
        self._stop_ai_event.clear()
        self._ai_thread = threading.Thread(target=self._ai_loop, name="SplitFeedYOLOWorker", daemon=True)
        self._ai_thread.start()

    def _ai_loop(self):
        self._load_fallbacks()
        self.status.emit("AI split-feed worker ready")
        while not self._stop_ai_event.is_set():
            self._new_ai_frame_event.wait(0.25)
            self._new_ai_frame_event.clear()
            if self._stop_ai_event.is_set():
                break
            if not self.ai_runtime_enabled:
                continue
            with self._ai_lock:
                frame = None if self._latest_ai_frame is None else self._latest_ai_frame.copy()
            if frame is None:
                continue
            dets = self.detect(frame)
            with self._ai_lock:
                self._latest_detections = dets
        self.status.emit("AI split-feed worker stopped")

    def run(self):
        self.running = True
        self.cap = self._open_capture()
        if self.cap is None:
            self.status.emit(f"FPV: failed to open source {self.settings.video_source}")
            return
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)); actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC)); fourcc_str = ''.join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        self.status.emit(f"FPV split-feed ON | {actual_w}x{actual_h} @ {actual_fps:.1f} | {fourcc_str} | UI pulls 60Hz | CV {'ON' if self.ai_runtime_enabled else 'OFF'}")
        if self.ai_runtime_enabled:
            self._ensure_ai_worker()
        fps_t0 = time.time(); count = 0
        requested_period = 1.0 / max(1.0, float(self.settings.requested_fps))
        while self.running:
            loop_t0 = time.time()
            try:
                ok = self.cap.grab()
                if not ok:
                    self.msleep(1); continue
                ok, frame = self.cap.retrieve()
                if not ok or frame is None:
                    self.msleep(1); continue
                self.frame_index += 1
                now = time.time()
                with self._frame_lock:
                    self._latest_frame = frame
                    self._latest_frame_id += 1
                    self._latest_frame_time = now
                if self.ai_runtime_enabled and (self.frame_index % max(1, int(self.settings.detect_every_n_frames)) == 0):
                    # Give AI a copy. If AI is behind, this replaces the previous pending frame.
                    with self._ai_lock:
                        self._latest_ai_frame = frame.copy()
                    self._new_ai_frame_event.set()
                self._write_record_frame(frame)
                count += 1
                if now - fps_t0 >= 1.0:
                    self.capture_rate_hz = count / max(0.001, now - fps_t0)
                    self.fps_measured.emit(self.capture_rate_hz)
                    count = 0; fps_t0 = now
                # If a camera lies about FPS and returns instantly, don't spin at 100% CPU.
                elapsed = time.time() - loop_t0
                if elapsed < requested_period * 0.35:
                    time.sleep(min(0.002, requested_period * 0.35 - elapsed))
            except Exception as e:
                self.status.emit(f"FPV split-feed loop error: {e}")
                self.msleep(20)
        self._stop_ai_event.set(); self._new_ai_frame_event.set()
        try:
            if self._ai_thread is not None:
                self._ai_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.stop_fpv_recording()
        except Exception:
            pass
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.status.emit("FPV: stopped")

    def stop(self):
        self.running = False
        self._stop_ai_event.set()
        self._new_ai_frame_event.set()
        self.wait(1800)




class VideoPostProcessThread(QThread):
    """Post-process a recorded video with YOLO/OpenCV without affecting live FPV."""
    progress = pyqtSignal(int, str)
    finished_ok = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, video_path: str, output_dir: str, model_path: str = "yolov8n.pt",
                 device: str = "auto", imgsz: int = 640, conf: float = 0.35,
                 half: bool = True, max_det: int = 50, mode: str = "all", draw_overlay: bool = True,
                 parent=None):
        super().__init__(parent)
        self.video_path = str(video_path)
        self.output_dir = str(output_dir)
        self.model_path = str(model_path or "yolov8n.pt")
        self.device = str(device or "auto")
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.half = bool(half)
        self.max_det = int(max_det)
        self.mode = str(mode or "all").lower()
        self.draw_overlay = bool(draw_overlay)
        self._running = True

    def stop(self):
        self._running = False

    def _resolve_device(self):
        d = self.device.lower().strip()
        if d == "auto":
            try:
                import torch
                return "cuda:0" if torch.cuda.is_available() else "cpu"
            except Exception:
                return "cpu"
        return d

    @staticmethod
    def _keep_label(label: str, mode: str) -> bool:
        label = (label or "").lower()
        raw = label
        norm = AIFrameThread.normalize_label(raw)
        if mode.startswith("human"):
            return norm in ("human", "face", "person") or raw == "person"
        if mode.startswith("things"):
            return norm not in ("human", "face", "person") and raw != "person"
        return True

    def run(self):
        try:
            src = Path(self.video_path)
            if not src.exists():
                self.failed.emit(f"Video not found: {src}")
                return
            out_dir = Path(self.output_dir).expanduser()
            out_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("post_ai_%Y%m%d_%H%M%S")
            csv_path = out_dir / f"{src.stem}_{stamp}_detections.csv"
            overlay_path = out_dir / f"{src.stem}_{stamp}_ai_overlay.mp4"
            cap = cv2.VideoCapture(str(src))
            if not cap.isOpened():
                self.failed.emit(f"Could not open video: {src}")
                return
            total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            writer = None
            if self.draw_overlay:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(str(overlay_path), fourcc, max(1.0, fps), (w, h))
                if not writer.isOpened():
                    writer = None
            model = None
            if ULTRALYTICS_AVAILABLE:
                try:
                    try:
                        import torch
                        if torch.cuda.is_available():
                            torch.backends.cudnn.benchmark = True
                            try:
                                torch.set_float32_matmul_precision('high')
                            except Exception:
                                pass
                    except Exception:
                        pass
                    self.progress.emit(0, f"Loading {self.model_path} on {self._resolve_device()}...")
                    model = YOLO(self.model_path)
                except Exception as e:
                    self.progress.emit(0, f"YOLO load failed, fallback disabled: {e}")
                    model = None
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer_csv = csv.DictWriter(f, fieldnames=["frame", "time_s", "label", "raw_label", "confidence", "x1", "y1", "x2", "y2", "area_pct", "source"])
                writer_csv.writeheader()
                frame_i = 0
                while self._running:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        break
                    detections = []
                    if model is not None:
                        try:
                            dev = self._resolve_device()
                            results = model.predict(frame, conf=self.conf, imgsz=self.imgsz, device=dev,
                                                    half=(dev.startswith("cuda") and self.half),
                                                    max_det=self.max_det, verbose=False)
                            for r in results:
                                names = r.names
                                boxes = getattr(r, "boxes", None)
                                if boxes is None:
                                    continue
                                for b in boxes:
                                    cls_id = int(b.cls[0]) if b.cls is not None else -1
                                    conf = float(b.conf[0]) if b.conf is not None else 0.0
                                    raw_label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
                                    if not self._keep_label(raw_label, self.mode):
                                        continue
                                    label = AIFrameThread.normalize_label(raw_label)
                                    x1, y1, x2, y2 = b.xyxy[0].detach().cpu().numpy().astype(float).tolist()
                                    x1 = int(clamp(x1, 0, max(0, w-1))); y1 = int(clamp(y1, 0, max(0, h-1)))
                                    x2 = int(clamp(x2, 0, max(0, w-1))); y2 = int(clamp(y2, 0, max(0, h-1)))
                                    area_pct = max(0.0, (x2-x1)*(y2-y1)/max(1.0,w*h)*100.0)
                                    d = {"label": label, "raw_label": raw_label, "confidence": conf, "x1": x1, "y1": y1, "x2": x2, "y2": y2, "area_pct": area_pct, "source": "post-YOLO"}
                                    detections.append(d)
                                    writer_csv.writerow({"frame": frame_i, "time_s": frame_i/max(1.0,fps), **d})
                        except Exception as e:
                            self.progress.emit(0, f"Post-process predict error: {e}")
                    if writer is not None:
                        writer.write(AIFrameThread.draw_detections(frame, detections) if detections else frame)
                    frame_i += 1
                    if frame_i % 10 == 0:
                        pct = int((frame_i/max(1,total))*100) if total else 0
                        self.progress.emit(min(99, pct), f"Processing {frame_i}/{total or '?'} frames | boxes {len(detections)}")
            cap.release()
            if writer is not None:
                writer.release()
            result = str(overlay_path if self.draw_overlay and overlay_path.exists() else csv_path)
            self.progress.emit(100, f"Post-processing complete: {result}")
            self.finished_ok.emit(result)
        except Exception as e:
            self.failed.emit(str(e))


# ----------------------------- Dataset Recorder -----------------------------

class DatasetRecorder:
    def __init__(self):
        self.active = False
        self.session_dir: Optional[Path] = None
        self.telemetry_file = None
        self.detections_file = None
        self.rc_file = None
        self.telemetry_writer = None
        self.detections_writer = None
        self.rc_writer = None
        self.video_writer = None
        self.frame_count = 0
        self.started_at = 0.0
        self.frame_size = None
        self.last_frame_path = ""

    def start(self, base_dir: str, fps: float = 30.0):
        base = Path(base_dir).expanduser()
        base.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("flight_%Y%m%d_%H%M%S")
        self.session_dir = base / stamp
        self.session_dir.mkdir(parents=True, exist_ok=True)
        (self.session_dir / "frames").mkdir(exist_ok=True)
        (self.session_dir / "crops").mkdir(exist_ok=True)

        self.telemetry_file = open(self.session_dir / "telemetry.csv", "w", newline="", encoding="utf-8")
        self.detections_file = open(self.session_dir / "detections.csv", "w", newline="", encoding="utf-8")
        self.rc_file = open(self.session_dir / "tx_rx_channels.csv", "w", newline="", encoding="utf-8")

        self.telemetry_writer = csv.DictWriter(self.telemetry_file, fieldnames=[
            "t", "mode", "armed", "lat", "lon", "rel_alt_m", "gps_alt_m", "groundspeed_m_s",
            "climb_m_s", "heading_deg", "roll_deg", "pitch_deg", "yaw_deg", "battery_voltage_v",
            "battery_current_a", "battery_remaining_percent", "gps_fix_type", "gps_sats", "lidar_alt_m",
            "optical_flow_alt_m", "optical_flow_quality", "radio_rssi", "radio_remrssi", "rc_rssi",
            "esc_rpm_1", "esc_rpm_2", "esc_rpm_3", "esc_rpm_4", "esc_current_1", "esc_current_2",
            "esc_current_3", "esc_current_4"
        ])
        self.telemetry_writer.writeheader()

        self.detections_writer = csv.DictWriter(self.detections_file, fieldnames=[
            "t", "frame_index", "frame_file", "label", "raw_label", "confidence", "x1", "y1", "x2", "y2",
            "source", "lat", "lon", "rel_alt_m", "heading_deg"
        ])
        self.detections_writer.writeheader()

        rc_fields = ["t", "tx_connected", "tx_name", "tx_rate_hz", "rx_rssi", "radio_rssi", "radio_remrssi"]
        for i in range(1, 17):
            rc_fields += [f"tx_ch{i}", f"rx_ch{i}"]
        self.rc_writer = csv.DictWriter(self.rc_file, fieldnames=rc_fields)
        self.rc_writer.writeheader()

        self.active = True
        self.started_at = time.time()
        self.frame_count = 0
        self.frame_size = None
        self.video_writer = None
        self.last_frame_path = ""

    def _ensure_video(self, frame: np.ndarray, fps: float = 30.0):
        if self.video_writer is not None:
            return
        if self.session_dir is None:
            return
        h, w = frame.shape[:2]
        self.frame_size = (w, h)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out_path = str(self.session_dir / "ai_overlay_video.mp4")
        self.video_writer = cv2.VideoWriter(out_path, fourcc, max(1.0, fps), (w, h))

    def write_frame(self, frame: np.ndarray, detections: List[Dict[str, Any]], save_frame: bool = False, fps: float = 30.0):
        if not self.active:
            return
        self._ensure_video(frame, fps=fps)
        if self.video_writer is not None:
            self.video_writer.write(frame)
        self.frame_count += 1
        if save_frame and self.session_dir is not None:
            frame_name = f"frame_{self.frame_count:06d}.jpg"
            path = self.session_dir / "frames" / frame_name
            cv2.imwrite(str(path), frame)
            self.last_frame_path = str(path.relative_to(self.session_dir))
            # Save crops for detections to make later training easier.
            for j, d in enumerate(detections):
                try:
                    x1, y1, x2, y2 = [int(d[k]) for k in ("x1", "y1", "x2", "y2")]
                    crop = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                    if crop.size > 0:
                        label = str(d.get("label", "object")).replace("/", "_")
                        cv2.imwrite(str(self.session_dir / "crops" / f"{label}_{self.frame_count:06d}_{j:02d}.jpg"), crop)
                except Exception:
                    pass

    def log_telemetry(self, d: Dict[str, Any]):
        if not self.active or self.telemetry_writer is None:
            return
        esc_rpm = d.get("esc_rpm", [None] * 8)
        esc_current = d.get("esc_current", [None] * 8)
        row = {
            "t": time.time(), "mode": d.get("mode"), "armed": d.get("armed"), "lat": d.get("lat"), "lon": d.get("lon"),
            "rel_alt_m": d.get("rel_alt_m"), "gps_alt_m": d.get("gps_alt_m"), "groundspeed_m_s": d.get("groundspeed_m_s"),
            "climb_m_s": d.get("climb_m_s"), "heading_deg": d.get("heading_deg"), "roll_deg": d.get("roll_deg"),
            "pitch_deg": d.get("pitch_deg"), "yaw_deg": d.get("yaw_deg"), "battery_voltage_v": d.get("battery_voltage_v"),
            "battery_current_a": d.get("battery_current_a"), "battery_remaining_percent": d.get("battery_remaining_percent"),
            "gps_fix_type": d.get("gps_fix_type"), "gps_sats": d.get("gps_sats"), "lidar_alt_m": d.get("lidar_alt_m"),
            "optical_flow_alt_m": d.get("optical_flow_alt_m"), "optical_flow_quality": d.get("optical_flow_quality"),
            "radio_rssi": d.get("radio_rssi"), "radio_remrssi": d.get("radio_remrssi"), "rc_rssi": d.get("rc_rssi"),
            "esc_rpm_1": esc_rpm[0] if len(esc_rpm) > 0 else None,
            "esc_rpm_2": esc_rpm[1] if len(esc_rpm) > 1 else None,
            "esc_rpm_3": esc_rpm[2] if len(esc_rpm) > 2 else None,
            "esc_rpm_4": esc_rpm[3] if len(esc_rpm) > 3 else None,
            "esc_current_1": esc_current[0] if len(esc_current) > 0 else None,
            "esc_current_2": esc_current[1] if len(esc_current) > 1 else None,
            "esc_current_3": esc_current[2] if len(esc_current) > 2 else None,
            "esc_current_4": esc_current[3] if len(esc_current) > 3 else None,
        }
        self.telemetry_writer.writerow(row)

    def log_detections(self, detections: List[Dict[str, Any]], telemetry: Dict[str, Any]):
        if not self.active or self.detections_writer is None:
            return
        t = time.time()
        for d in detections:
            self.detections_writer.writerow({
                "t": t, "frame_index": self.frame_count, "frame_file": self.last_frame_path,
                "label": d.get("label"), "raw_label": d.get("raw_label"), "confidence": d.get("confidence"),
                "x1": d.get("x1"), "y1": d.get("y1"), "x2": d.get("x2"), "y2": d.get("y2"),
                "source": d.get("source"), "lat": telemetry.get("lat"), "lon": telemetry.get("lon"),
                "rel_alt_m": telemetry.get("rel_alt_m"), "heading_deg": telemetry.get("heading_deg"),
            })

    def log_rc(self, tx: Dict[str, Any], rx: Dict[str, Any]):
        if not self.active or self.rc_writer is None:
            return
        tx_ch = tx.get("rc_channels", [0] * 16) if tx else [0] * 16
        rx_ch = rx.get("rc_channels", [0] * 16) if rx else [0] * 16
        row = {
            "t": time.time(),
            "tx_connected": bool(tx.get("connected")) if tx else False,
            "tx_name": tx.get("name") if tx else None,
            "tx_rate_hz": tx.get("update_rate_hz") if tx else None,
            "rx_rssi": rx.get("rc_rssi") if rx else None,
            "radio_rssi": rx.get("radio_rssi") if rx else None,
            "radio_remrssi": rx.get("radio_remrssi") if rx else None,
        }
        for i in range(16):
            row[f"tx_ch{i+1}"] = tx_ch[i] if i < len(tx_ch) else 0
            row[f"rx_ch{i+1}"] = rx_ch[i] if i < len(rx_ch) else 0
        self.rc_writer.writerow(row)

    def stop(self):
        if self.video_writer is not None:
            try:
                self.video_writer.release()
            except Exception:
                pass
        for f in (self.telemetry_file, self.detections_file, self.rc_file):
            try:
                if f:
                    f.flush()
                    f.close()
            except Exception:
                pass
        self.active = False
        path = str(self.session_dir) if self.session_dir else ""
        self.telemetry_file = self.detections_file = self.rc_file = None
        self.telemetry_writer = self.detections_writer = self.rc_writer = None
        self.video_writer = None
        return path


# ----------------------------- UI Widgets -----------------------------

class FpvOverlayView(QWidget):
    """Fast FPV display widget.

    Stores a QImage and draws boxes with QPainter. This is much faster than drawing
    boxes into the video frame and converting a new scaled QPixmap every frame.
    """
    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)
        self._qimage = None
        self._img_w = 0
        self._img_h = 0
        self._detections: List[Dict[str, Any]] = []
        self._fps = 0.0
        self.setStyleSheet("background:#03060b;border-radius:12px;")

    def update_frame(self, frame_bgr: np.ndarray, detections: List[Dict[str, Any]], fps: float = 0.0):
        if frame_bgr is None:
            return
        # Use Qt's native BGR888 path when available to avoid a full BGR->RGB copy every frame.
        h, w = frame_bgr.shape[:2]
        fmt = getattr(QImage, "Format_BGR888", QImage.Format_RGB888)
        if fmt == QImage.Format_RGB888:
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            self._qimage = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
        else:
            self._qimage = QImage(frame_bgr.data, w, h, frame_bgr.strides[0], fmt).copy()
        self._img_w, self._img_h = w, h
        self._detections = list(detections or [])
        self._fps = float(fps or 0.0)
        self.update()

    def _target_rect(self) -> QRectF:
        if self._img_w <= 0 or self._img_h <= 0:
            return QRectF(self.rect())
        r = QRectF(self.rect())
        img_ratio = self._img_w / max(1, self._img_h)
        view_ratio = r.width() / max(1.0, r.height())
        if view_ratio > img_ratio:
            h = r.height(); w = h * img_ratio
            return QRectF(r.center().x() - w/2, r.top(), w, h)
        w = r.width(); h = w / img_ratio
        return QRectF(r.left(), r.center().y() - h/2, w, h)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(3, 6, 11))
        if self._qimage is None:
            p.setPen(QColor(220,235,255))
            p.setFont(QFont("Arial", 18, QFont.Bold))
            p.drawText(self.rect(), Qt.AlignCenter, "No FPV feed")
            return
        target = self._target_rect()
        p.setRenderHint(QPainter.SmoothPixmapTransform, False)
        p.drawImage(target, self._qimage)
        # subtle glass frame
        p.setPen(QPen(QColor(130, 180, 255, 100), 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(target.adjusted(1,1,-1,-1), 12, 12)
        if self._detections and self._img_w > 0 and self._img_h > 0:
            sx = target.width() / self._img_w
            sy = target.height() / self._img_h
            for d in self._detections:
                try:
                    x1 = target.left() + int(d.get('x1',0)) * sx
                    y1 = target.top() + int(d.get('y1',0)) * sy
                    x2 = target.left() + int(d.get('x2',0)) * sx
                    y2 = target.top() + int(d.get('y2',0)) * sy
                    rect = QRectF(x1, y1, max(2, x2-x1), max(2, y2-y1))
                    label = f"{d.get('label','?')} {safe_float(d.get('confidence')):.2f} | {safe_float(d.get('area_pct')):.1f}%"
                    p.setPen(QPen(QColor(0, 0, 0, 200), 5))
                    p.drawRoundedRect(rect, 5, 5)
                    p.setPen(QPen(QColor(80, 255, 150), 2))
                    p.drawRoundedRect(rect, 5, 5)
                    fm = p.fontMetrics()
                    box = QRectF(rect.left(), max(target.top()+2, rect.top() - 23), fm.horizontalAdvance(label)+12, 22)
                    p.setBrush(QColor(0,0,0,185)); p.setPen(QPen(QColor(80,255,150,180),1))
                    p.drawRoundedRect(box, 5, 5)
                    p.setPen(QColor(145,255,190)); p.setFont(QFont("Arial", 9, QFont.Bold))
                    p.drawText(box.adjusted(5,0,-4,0), Qt.AlignVCenter|Qt.AlignLeft, label)
                except Exception:
                    continue
        # FPS badge
        if self._fps > 0:
            badge = QRectF(target.left()+10, target.top()+10, 132, 28)
            p.setBrush(QColor(0,0,0,170)); p.setPen(QPen(QColor(120,180,255,150),1))
            p.drawRoundedRect(badge, 8, 8)
            p.setPen(QColor(235,248,255)); p.setFont(QFont("Arial", 10, QFont.Bold))
            p.drawText(badge, Qt.AlignCenter, f"FPV {self._fps:.1f} FPS")

class FPVAIWidget(QGroupBox):
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()

    def __init__(self):
        super().__init__("FPV + AI Recognition")
        self.last_frame = None
        self.detections = []
        self.display_fps = 0.0
        layout = QVBoxLayout(self)
        self.video = FpvOverlayView()
        layout.addWidget(self.video, stretch=1)
        self.summary = QLabel("AI: waiting | split feed inactive")
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)

    def update_frame(self, frame: np.ndarray, detections: List[Dict[str, Any]], display_fps: float = 0.0):
        self.last_frame = frame
        self.detections = detections or []
        self.display_fps = float(display_fps or self.display_fps or 0.0)
        self.video.update_frame(frame, self.detections, self.display_fps)
        counts = {}
        for d in self.detections:
            counts[d.get("label", "object")] = counts.get(d.get("label", "object"), 0) + 1
        if counts:
            txt = " | ".join([f"{k}:{v}" for k, v in sorted(counts.items())])
            self.summary.setText(f"AI boxes overlaid on smooth FPV: {txt}")
        else:
            self.summary.setText("AI boxes: none | FPV path remains separate from AI path")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.video.update()


class TelemetryCards(QGroupBox):
    """Compact v26-style telemetry cards with tighter label/value spacing.

    ESC and fence are intentionally moved out into their own stacked panel so
    this block can stay readable and aligned even when the AHRS panel is small.
    """
    def __init__(self):
        super().__init__("Flight Telemetry")
        self.labels = {}
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 14, 8, 8)
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(5)
        items = [
            ("mode", "Mode"), ("armed", "Armed"),
            ("gps", "GPS"), ("pos", "Position"),
            ("alt", "Altitude"), ("speed", "Speed"),
            ("att", "Attitude"), ("batt", "Battery"),
            ("flow", "H-Flow/Lidar"), ("rssi", "RC Link"),
            ("current", "Current"), ("thr", "Throttle"),
        ]
        for i, (key, name) in enumerate(items):
            title = QLabel(name + ":")
            title.setObjectName("telemetryTitle")
            title.setMinimumWidth(68)
            title.setMaximumWidth(96)
            title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value = QLabel("-")
            value.setObjectName("dynamicValue")
            value.setMinimumHeight(26)
            value.setMaximumHeight(34)
            value.setAlignment(Qt.AlignCenter)
            row = i // 2
            col = (i % 2) * 2
            layout.addWidget(title, row, col)
            layout.addWidget(value, row, col + 1)
            self.labels[key] = value
        layout.setColumnStretch(0, 0); layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0); layout.setColumnStretch(3, 1)

    def update_data(self, d: Dict[str, Any]):
        self.labels["mode"].setText(str(d.get("mode", "-"))[:16])
        self.labels["armed"].setText("ARMED" if d.get("armed") else "DISARMED")
        self.labels["gps"].setText(f"fix {d.get('gps_fix_type',0)} / {d.get('gps_sats',0)} sats")
        lat, lon = d.get("lat"), d.get("lon")
        self.labels["pos"].setText(f"{lat:.6f}, {lon:.6f}" if lat is not None and lon is not None else "-")
        rel = d.get("rel_alt_m"); gps = d.get("gps_alt_m"); lidar = d.get("lidar_alt_m")
        alt = []
        if rel is not None: alt.append(f"rel {safe_float(rel):.1f}m")
        if gps is not None: alt.append(f"gps {safe_float(gps):.1f}m")
        if lidar is not None: alt.append(f"lidar {safe_float(lidar):.2f}m")
        self.labels["alt"].setText(" | ".join(alt) if alt else "-")
        self.labels["speed"].setText(f"GS {safe_float(d.get('groundspeed_m_s')):.1f}m/s | climb {safe_float(d.get('climb_m_s')):+.1f}")
        self.labels["att"].setText(f"R {safe_float(d.get('roll_deg')):+.1f}  P {safe_float(d.get('pitch_deg')):+.1f}  H {safe_float(d.get('heading_deg')):.0f}")
        self.labels["batt"].setText(f"{safe_float(d.get('battery_voltage_v')):.2f}V | {d.get('battery_remaining_percent','-')}%")
        self.labels["flow"].setText(f"lidar {safe_float(d.get('lidar_alt_m')):.2f}m | flow q={d.get('optical_flow_quality')}")
        self.labels["rssi"].setText(f"RC {d.get('rc_rssi')} | radio {d.get('radio_rssi')}/{d.get('radio_remrssi')}")
        self.labels["current"].setText(f"{safe_float(d.get('battery_current_a')):.1f} A")
        self.labels["thr"].setText(f"{safe_float(d.get('throttle_percent')):.0f} %")


class EscStatusPanel(QGroupBox):
    """Separate ESC/Motor panel.

    MAVLink mode: shows ArduPilot ESC telemetry, preferring ESC9-12.
    MSP mode: Betaflight usually does not expose real ESC RPM/temp over MSP v1,
    so this panel shows MSP motor output values when available and keeps the
    panel useful instead of pretending RPM exists.
    """
    def __init__(self):
        super().__init__("ESC / Motor Telemetry")
        self.cells = []
        layout = QGridLayout(self)
        layout.setContentsMargins(6, 12, 6, 6)
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(3)
        for i in range(4):
            lab = QLabel(f"E{i+1}")
            lab.setObjectName("telemetryTitle")
            val = QLabel("-")
            val.setObjectName("dynamicValue")
            val.setAlignment(Qt.AlignCenter)
            val.setMinimumHeight(20)
            val.setMaximumHeight(28)
            layout.addWidget(lab, i, 0)
            layout.addWidget(val, i, 1)
            self.cells.append((lab, val))
        self.summary = QLabel("Waiting for ESC telemetry")
        self.summary.setWordWrap(True)
        self.summary.setObjectName("softText")
        layout.addWidget(self.summary, 4, 0, 1, 2)
        layout.setColumnStretch(1, 1)

    @staticmethod
    def _active_esc_indices(d: Dict[str, Any]):
        rpm = d.get("esc_rpm", [None] * 16)
        temp = d.get("esc_temp", [None] * 16)
        current = d.get("esc_current", [None] * 16)
        groups = [list(range(8, 12)), list(range(0, 4)), list(range(4, 8)), list(range(12, 16))]
        for g in groups:
            if any((i < len(rpm) and rpm[i] not in (None, 0)) or
                   (i < len(temp) and temp[i] not in (None, 0)) or
                   (i < len(current) and current[i] not in (None, 0)) for i in g):
                return g
        return groups[0]

    def update_data(self, d: Dict[str, Any]):
        protocol = str(d.get("telemetry_protocol", "MAVLink"))
        if protocol == "MSP":
            motors = d.get("motor_outputs", [None] * 16)
            for i, (lab, val) in enumerate(self.cells):
                lab.setText(f"M{i+1}")
                m = motors[i] if i < len(motors) else None
                val.setText("-" if m in (None, 0) else str(int(m)))
            self.summary.setText("MSP: Betaflight motor outputs shown. ESC RPM/temp are not normally available over MSP v1 USB.")
            return

        rpm = d.get("esc_rpm", [None] * 16)
        temp = d.get("esc_temp", [None] * 16)
        current = d.get("esc_current", [None] * 16)
        volt = d.get("esc_voltage", [None] * 16)
        idxs = self._active_esc_indices(d)
        for slot, idx in enumerate(idxs[:4]):
            lab, val = self.cells[slot]
            lab.setText(f"E{idx+1}")
            r = rpm[idx] if idx < len(rpm) else None
            t = temp[idx] if idx < len(temp) else None
            c = current[idx] if idx < len(current) else None
            v = volt[idx] if idx < len(volt) else None
            parts = []
            if r not in (None, 0): parts.append(f"{int(r)} rpm")
            if t not in (None, 0): parts.append(f"{int(t)}°C")
            if c not in (None, 0): parts.append(f"{safe_float(c):.1f}A")
            if v not in (None, 0): parts.append(f"{safe_float(v):.1f}V")
            val.setText(" | ".join(parts) if parts else "-")
        self.summary.setText("MAVLink: displaying active ESC group, ESC9-12 preferred when present.")


class FenceStatusPanel(QGroupBox):
    def __init__(self):
        super().__init__("Fence Status")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 12, 6, 6)
        layout.setSpacing(3)
        self.status = QLabel("Available only for MAVLink")
        self.status.setObjectName("dynamicValue")
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.details = QLabel("Altitude and circular fence warnings will appear here.")
        self.details.setObjectName("softText")
        self.details.setWordWrap(True)
        layout.addWidget(self.details)

    def update_data(self, d: Dict[str, Any]):
        protocol = str(d.get("telemetry_protocol", "MAVLink"))
        if protocol == "MSP":
            self.setEnabled(False)
            self.status.setText("Available only for MAVLink")
            self.details.setText("Betaflight/iNav MSP USB telemetry is read-only here; ArduPilot fence status is not available.")
            return
        self.setEnabled(True)
        warn = str(d.get("fence_warning", "Fence: waiting"))
        self.status.setText(warn)
        alt_left = d.get("fence_alt_remaining_m")
        rad_left = d.get("fence_radius_remaining_m")
        alt_max = d.get("fence_alt_max_m")
        radius = d.get("fence_radius_m")
        breach = d.get("fence_breach_status")
        self.details.setText(
            f"Alt left: {'-' if alt_left is None else f'{alt_left:.1f}m'} / max {'-' if alt_max is None else f'{alt_max:.1f}m'}   "
            f"Radius left: {'-' if rad_left is None else f'{rad_left:.1f}m'} / max {'-' if radius is None else f'{radius:.1f}m'}   "
            f"Breach: {breach}"
        )


class EscFenceStack(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.esc_panel = EscStatusPanel()
        self.fence_panel = FenceStatusPanel()
        self.setMinimumWidth(230)
        self.setMaximumWidth(340)
        layout.addWidget(self.esc_panel, 3)
        layout.addWidget(self.fence_panel, 2)

    def update_data(self, d: Dict[str, Any]):
        self.esc_panel.update_data(d)
        self.fence_panel.update_data(d)


class CompactAHRSWidget(QWidget):
    """Realtime 3D AHRS with real incoming-data rates, not paint-FPS rates.

    v51 restores the deeper 3D cockpit look while keeping the compact lower-panel
    footprint. The right AHRS data card is always drawn when there is enough width,
    and it shows the actual incoming attitude/telemetry rates measured by the
    telemetry thread.
    """
    def __init__(self):
        super().__init__()
        self.setMinimumSize(360, 215)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.heading_deg = 0.0
        self.last_target_time = time.monotonic()
        self.last_paint_time = time.monotonic()
        self.paint_fps = 0.0
        self.attitude_age_s = 999.0
        self.attitude_rate_hz = 0.0
        self.telemetry_rate_hz = 0.0
        self.gps_rate_hz = 0.0
        self.esc_rate_hz = 0.0
        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_heading_deg = 0.0
        self.render_fps = 0
        self.render_interval_ms = 0

    def set_render_fps(self, fps: int):
        self.render_fps = 0
        self.render_interval_ms = 0

    def set_data_rates(self, d: Dict[str, Any]):
        self.attitude_rate_hz = safe_float(d.get("attitude_rate_hz"), self.attitude_rate_hz)
        self.telemetry_rate_hz = safe_float(d.get("telemetry_rate_hz"), self.telemetry_rate_hz)
        self.gps_rate_hz = safe_float(d.get("gps_rate_hz"), self.gps_rate_hz)
        self.esc_rate_hz = safe_float(d.get("esc_rate_hz"), self.esc_rate_hz)

    def update_attitude(self, roll: float, pitch: float, heading: float):
        self.roll_deg = float(roll)
        self.pitch_deg = float(pitch)
        self.heading_deg = float(heading) % 360.0
        self.target_roll_deg = self.roll_deg
        self.target_pitch_deg = self.pitch_deg
        self.target_heading_deg = self.heading_deg
        self.last_target_time = time.monotonic()
        self.update()

    @staticmethod
    def _angle_delta_deg(target: float, current: float) -> float:
        return (target - current + 540.0) % 360.0 - 180.0

    def _draw_heading_tape(self, painter: QPainter, tape: QRectF, is_light: bool, scale: float):
        grad = QLinearGradient(tape.topLeft(), tape.bottomLeft())
        if is_light:
            grad.setColorAt(0.0, QColor(248, 252, 255, 245)); grad.setColorAt(1.0, QColor(220, 232, 248, 235))
            border, text_col, minor_col = QColor(130, 155, 190), QColor(10, 25, 45), QColor(72, 90, 115, 150)
        else:
            grad.setColorAt(0.0, QColor(22, 36, 60, 240)); grad.setColorAt(0.48, QColor(7, 16, 31, 235)); grad.setColorAt(1.0, QColor(20, 34, 56, 238))
            border, text_col, minor_col = QColor(105, 135, 180), QColor(238, 248, 255), QColor(210, 230, 255, 120)
        painter.setBrush(QBrush(grad))
        painter.setPen(QPen(border, max(1, int(1.35 * scale))))
        painter.drawRoundedRect(tape, 8 * scale, 8 * scale)
        # subtle inner shine
        painter.setPen(QPen(QColor(255,255,255,70) if not is_light else QColor(255,255,255,150), 1))
        painter.drawLine(int(tape.left()+6*scale), int(tape.top()+3*scale), int(tape.right()-6*scale), int(tape.top()+3*scale))
        cx = tape.center().x()
        px_per_deg = tape.width() / 120.0
        painter.save(); painter.setClipRect(tape.adjusted(5, 1, -5, -1))
        painter.setFont(QFont("Arial", max(7, int(9 * scale)), QFont.Bold))
        centre = int(math.floor(self.heading_deg / 5.0) * 5)
        def label_for(bearing: int) -> str:
            b = bearing % 360
            return "N" if b == 0 else "E" if b == 90 else "S" if b == 180 else "W" if b == 270 else f"{b:03d}"
        for bearing in range(centre - 180, centre + 181, 5):
            b = bearing % 360
            delta = self._angle_delta_deg(float(b), self.heading_deg)
            if abs(delta) > 66:
                continue
            x = cx + delta * px_per_deg
            major = b % 30 == 0
            mid = b % 10 == 0
            tick_h = (16 if major else 10 if mid else 6) * scale
            painter.setPen(QPen(text_col if major else minor_col, max(1, int(2 if major else 1))))
            painter.drawLine(int(x), int(tape.bottom() - 4 * scale), int(x), int(tape.bottom() - 4 * scale - tick_h))
            if major:
                painter.drawText(QRectF(x - 20 * scale, tape.top() + 2 * scale, 40 * scale, 14 * scale), Qt.AlignCenter, label_for(b))
        painter.restore()
        painter.setPen(QPen(QColor(255, 45, 55), max(2, int(3 * scale))))
        painter.drawLine(int(cx), int(tape.top() + 4 * scale), int(cx), int(tape.bottom() - 4 * scale))

    def _draw_value_box(self, painter: QPainter, box: QRectF, is_light: bool, scale: float):
        grad = QLinearGradient(box.topLeft(), box.bottomRight())
        if is_light:
            grad.setColorAt(0.0, QColor(255,255,255,235)); grad.setColorAt(1.0, QColor(225,235,250,220))
            border, label_col, value_col = QColor(135, 160, 195), QColor(14, 30, 55), QColor(210, 0, 18)
            row_bg = QColor(235, 244, 255, 185)
        else:
            grad.setColorAt(0.0, QColor(25, 42, 70, 225)); grad.setColorAt(0.52, QColor(12, 24, 44, 220)); grad.setColorAt(1.0, QColor(8, 16, 31, 230))
            border, label_col, value_col = QColor(95, 125, 170), QColor(220, 235, 255), QColor(255, 22, 35)
            row_bg = QColor(10, 22, 40, 205)
        painter.setBrush(QBrush(grad))
        painter.setPen(QPen(border, max(1, int(1.4 * scale))))
        painter.drawRoundedRect(box, 12 * scale, 12 * scale)
        painter.setPen(QPen(QColor(255,255,255,85), 1))
        painter.drawLine(int(box.left()+8*scale), int(box.top()+4*scale), int(box.right()-8*scale), int(box.top()+4*scale))
        # v60: the AHRS card must contain every row at every screen scale.
        # Earlier versions forced a 17 px minimum row height, so TEL/STREAM could
        # spill outside the rounded rectangle in compact layouts. Here the row
        # height is calculated from the actual box height first, then fonts scale
        # down gracefully.
        title_h = max(16.0, 21.0 * scale)
        pad_x = max(5.0, 7.0 * scale)
        pad_top = max(5.0, 6.0 * scale)
        pad_bottom = max(5.0, 7.0 * scale)
        painter.setFont(QFont("Arial", max(7, int(10.5 * scale)), QFont.Bold))
        painter.setPen(label_col)
        painter.drawText(QRectF(box.left()+pad_x+2*scale, box.top()+pad_top, box.width()-2*pad_x, title_h), Qt.AlignLeft|Qt.AlignVCenter, "AHRS LIVE")
        status = "LIVE" if self.attitude_age_s < 0.30 else ("STALE" if self.attitude_age_s < 2.0 else "NO ATT")
        rows = [
            ("ROLL", f"{self.roll_deg:+.1f}°"),
            ("PITCH", f"{self.pitch_deg:+.1f}°"),
            ("HDG", f"{self.heading_deg:03.0f}°"),
            ("GYRO", f"{self.attitude_rate_hz:.0f} Hz"),
            ("TEL", f"{self.telemetry_rate_hz:.0f} Hz"),
            ("STREAM", status),
        ]
        top = box.top() + pad_top + title_h + max(4.0, 5.0 * scale)
        gap = max(1.5, 3.0 * scale)
        usable_h = max(42.0, box.bottom() - top - pad_bottom)
        row_h = max(12.0, min(23.0 * scale, (usable_h - gap*(len(rows)-1)) / len(rows)))
        # If the box is extremely short, tighten the gap to keep all rows inside.
        if top + row_h*len(rows) + gap*(len(rows)-1) + pad_bottom > box.bottom():
            gap = max(0.8, (box.bottom() - top - pad_bottom - row_h*len(rows)) / max(1, len(rows)-1))
        y = top
        for name, value in rows:
            row = QRectF(box.left()+pad_x, y, box.width()-2*pad_x, row_h)
            painter.setBrush(row_bg)
            painter.setPen(QPen(border, 1))
            painter.drawRoundedRect(row, max(4.0, 5.5 * scale), max(4.0, 5.5 * scale))
            font_px = max(5, min(9, int(row_h * 0.50)))
            painter.setFont(QFont("Arial", font_px, QFont.Bold))
            painter.setPen(label_col)
            painter.drawText(QRectF(row.left()+max(4.0, 6.0*scale), row.top(), row.width()*0.46, row.height()), Qt.AlignLeft|Qt.AlignVCenter, name)
            painter.setPen(value_col)
            painter.drawText(QRectF(row.left()+row.width()*0.42, row.top(), row.width()*0.55, row.height()), Qt.AlignRight|Qt.AlignVCenter, value)
            y += row_h + gap

    def _draw_ball(self, painter: QPainter, cx: float, cy: float, radius: float, is_light: bool, scale: float):
        shadow = QRadialGradient(QPointF(cx, cy + radius * 0.18), radius * 1.45)
        shadow.setColorAt(0.0, QColor(0, 0, 0, 115 if not is_light else 55))
        shadow.setColorAt(0.62, QColor(0, 0, 0, 70 if not is_light else 35))
        shadow.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setPen(Qt.NoPen); painter.setBrush(QBrush(shadow))
        painter.drawEllipse(QRectF(cx-radius*1.24, cy-radius*1.12, radius*2.48, radius*2.25))
        # Deep metallic bevel, closer to the previous cool 3D style.
        bezel = QRadialGradient(QPointF(cx-radius*0.24, cy-radius*0.28), radius*1.35)
        if is_light:
            bezel.setColorAt(0.0, QColor(248,252,255)); bezel.setColorAt(0.32, QColor(165,185,210)); bezel.setColorAt(0.72, QColor(85,105,135)); bezel.setColorAt(1.0, QColor(235,242,252))
        else:
            bezel.setColorAt(0.0, QColor(165,180,205)); bezel.setColorAt(0.34, QColor(58,74,100)); bezel.setColorAt(0.72, QColor(8,13,24)); bezel.setColorAt(1.0, QColor(220,232,248))
        painter.setBrush(QBrush(bezel))
        painter.setPen(QPen(QColor(230,240,255) if not is_light else QColor(120,145,178), max(2, int(2.2*scale))))
        painter.drawEllipse(QPointF(cx, cy), radius+12*scale, radius+12*scale)
        painter.setBrush(QColor(6, 10, 18) if not is_light else QColor(240, 247, 255))
        painter.setPen(QPen(QColor(35,48,70) if not is_light else QColor(135,155,182), max(2, int(2.5*scale))))
        painter.drawEllipse(QPointF(cx, cy), radius+4*scale, radius+4*scale)
        ball_rect = QRectF(cx-radius, cy-radius, radius*2, radius*2)
        path = QPainterPath(); path.addEllipse(ball_rect)
        painter.save(); painter.setClipPath(path)
        painter.translate(cx, cy); painter.rotate(-self.roll_deg)
        pitch_px = self.pitch_deg * (radius / 28.0)
        sky = QLinearGradient(0, -radius*2.5 + pitch_px, 0, pitch_px)
        sky.setColorAt(0.0, QColor(10, 45, 125)); sky.setColorAt(0.55, QColor(45, 135, 230)); sky.setColorAt(1.0, QColor(145, 210, 255))
        painter.setBrush(QBrush(sky)); painter.setPen(Qt.NoPen)
        painter.drawRect(QRectF(-radius*2.5, -radius*2.5 + pitch_px, radius*5, radius*2.5))
        ground = QLinearGradient(0, pitch_px, 0, radius*2.5 + pitch_px)
        ground.setColorAt(0.0, QColor(170, 104, 48)); ground.setColorAt(0.55, QColor(92, 50, 24)); ground.setColorAt(1.0, QColor(35, 20, 14))
        painter.setBrush(QBrush(ground)); painter.drawRect(QRectF(-radius*2.5, pitch_px, radius*5, radius*2.5))
        painter.setPen(QPen(QColor(0,0,0,150), max(3, int(5*scale))))
        painter.drawLine(int(-radius*1.5), int(pitch_px+1), int(radius*1.5), int(pitch_px+1))
        painter.setPen(QPen(QColor(250,250,250), max(1, int(2*scale))))
        painter.drawLine(int(-radius*1.5), int(pitch_px), int(radius*1.5), int(pitch_px))
        painter.setFont(QFont("Arial", max(6, int(8.5*scale)), QFont.Bold))
        for deg in range(-90, 91, 5):
            if deg == 0:
                continue
            y = pitch_px - deg*(radius/28.0)
            if y < -radius*1.16 or y > radius*1.16:
                continue
            major = (deg % 10 == 0)
            length = radius*(0.42 if major else 0.22)
            gap = radius*0.08
            painter.setPen(QPen(QColor(246,250,255,225 if major else 145), max(1, int(2 if major else 1))))
            painter.drawLine(int(-length), int(y), int(-gap), int(y))
            painter.drawLine(int(gap), int(y), int(length), int(y))
            if major:
                painter.drawText(QRectF(length+4*scale, y-9*scale, 30*scale, 18*scale), Qt.AlignLeft|Qt.AlignVCenter, str(abs(deg)))
                painter.drawText(QRectF(-length-34*scale, y-9*scale, 30*scale, 18*scale), Qt.AlignRight|Qt.AlignVCenter, str(abs(deg)))
        painter.restore()
        painter.save(); painter.setClipPath(path)
        highlight = QRadialGradient(QPointF(cx-radius*0.42, cy-radius*0.55), radius*1.08)
        highlight.setColorAt(0.0, QColor(255,255,255,115)); highlight.setColorAt(0.25, QColor(255,255,255,48)); highlight.setColorAt(0.58, QColor(255,255,255,0)); highlight.setColorAt(1.0, QColor(0,0,0,75))
        painter.setBrush(QBrush(highlight)); painter.setPen(Qt.NoPen); painter.drawEllipse(ball_rect); painter.restore()
        painter.setBrush(Qt.NoBrush); painter.setPen(QPen(QColor(250,252,255) if not is_light else QColor(80,100,128), max(2, int(2*scale))))
        painter.drawEllipse(ball_rect)
        # Roll scale with labels, restored from the more detailed look.
        painter.save(); painter.translate(cx, cy); painter.rotate(-self.roll_deg)
        painter.setFont(QFont("Arial", max(5, int(6.5*scale)), QFont.Bold))
        for deg in range(-180, 181, 10):
            painter.save(); painter.rotate(deg)
            major = deg % 30 == 0
            tick = (12 if major else 7) * scale
            painter.setPen(QPen(QColor(230,240,255,170 if major else 95), max(1, int(1.2 if major else 1))))
            painter.drawLine(0, int(-radius-2*scale), 0, int(-radius-tick))
            painter.restore()
        painter.restore()
        for deg in range(-180, 181, 30):
            if abs(deg) == 180 or abs(deg) <= 150:
                a = math.radians(deg - 90)
                tx = math.cos(a) * (radius + 18*scale) + cx
                ty = math.sin(a) * (radius + 18*scale) + cy
                painter.setPen(QColor(235,245,255,170) if not is_light else QColor(70,90,118,170))
                painter.setFont(QFont("Arial", max(5, int(6.2*scale)), QFont.Bold))
                painter.drawText(QRectF(tx-13*scale, ty-7*scale, 26*scale, 14*scale), Qt.AlignCenter, str(abs(deg)) if deg != 0 else "0")
        marker = QPolygonF([QPointF(cx, cy-radius-2*scale), QPointF(cx-8*scale, cy-radius-14*scale), QPointF(cx+8*scale, cy-radius-14*scale)])
        painter.setBrush(QBrush(QColor(255, 214, 65))); painter.setPen(QPen(QColor(12,12,12), 1)); painter.drawPolygon(marker)
        painter.setPen(QPen(QColor(0,0,0,150), max(5, int(6*scale)), Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(int(cx-radius*0.58), int(cy+2), int(cx-radius*0.13), int(cy+2))
        painter.drawLine(int(cx+radius*0.13), int(cy+2), int(cx+radius*0.58), int(cy+2))
        painter.setPen(QPen(QColor(255,208,55), max(3, int(5*scale)), Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(int(cx-radius*0.58), int(cy), int(cx-radius*0.13), int(cy))
        painter.drawLine(int(cx+radius*0.13), int(cy), int(cx+radius*0.58), int(cy))
        painter.drawLine(int(cx), int(cy-radius*0.10), int(cx), int(cy+radius*0.10))
        painter.setBrush(QBrush(QColor(255,232,90))); painter.setPen(QPen(QColor(255,245,170), 1)); painter.drawEllipse(QPointF(cx, cy), 4*scale, 4*scale)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        now = time.monotonic()
        dt = max(0.001, now - self.last_paint_time)
        self.last_paint_time = now
        self.attitude_age_s = now - self.last_target_time
        inst = 1.0 / dt
        self.paint_fps = inst if self.paint_fps <= 0.1 else (0.85 * self.paint_fps + 0.15 * inst)
        is_light = self.palette().color(self.backgroundRole()).lightness() > 150
        bg = QLinearGradient(QPointF(0,0), QPointF(0,self.height()))
        if is_light:
            bg.setColorAt(0.0, QColor(245,250,255)); bg.setColorAt(1.0, QColor(222,234,248))
        else:
            bg.setColorAt(0.0, QColor(9,15,27)); bg.setColorAt(1.0, QColor(3,8,16))
        painter.fillRect(self.rect(), QBrush(bg))
        W, H = max(1, self.width()), max(1, self.height())
        scale = max(0.54, min(1.15, min(W/640.0, H/360.0)))
        margin = 8 * scale
        outer = QRectF(margin, margin, W-2*margin, H-2*margin)
        tape_h = max(25, 31 * scale)
        tape = QRectF(outer.left()+8*scale, outer.top()+4*scale, outer.width()-16*scale, tape_h)
        self._draw_heading_tape(painter, tape, is_light, scale)
        content = outer.adjusted(0, tape_h + 13*scale, 0, 0)
        gap = 12 * scale
        # Keep the rectangle visible. If width is tight, shrink the card before hiding it.
        value_w = min(max(126, W*0.22), 170*scale)
        if W < 510:
            value_w = max(108, W*0.22)
        available_ball_w = max(150, content.width() - value_w - gap)
        radius = min(available_ball_w*0.42, content.height()*0.46)
        radius = max(48, radius)
        cx = content.left() + available_ball_w*0.49
        cy = content.top() + content.height()*0.53
        if cx - radius < content.left()+6*scale:
            cx = content.left()+radius+6*scale
        if cx + radius > content.left()+available_ball_w-4*scale:
            cx = content.left()+available_ball_w-radius-4*scale
        self._draw_ball(painter, cx, cy, radius, is_light, scale)
        # v60: make the AHRS data rectangle use nearly the full content height so
        # the six live rows never overflow while keeping it inside the compact panel.
        value_top = content.top() + max(1.0, content.height()*0.025)
        value_h = max(126.0, content.bottom() - value_top - max(3.0, 4.0*scale))
        value_box = QRectF(content.left()+available_ball_w+gap*0.65, value_top, value_w, value_h)
        if value_box.width() > 90 and value_box.height() > 96:
            self._draw_value_box(painter, value_box, is_light, scale)

class DualRCChannelsWidget(QGroupBox):
    def __init__(self, channel_count: int = 14):
        super().__init__("TX USB vs RX/Pixhawk RC Channels")
        self.channel_count = channel_count
        self.tx_bars: List[QProgressBar] = []
        self.rx_bars: List[QProgressBar] = []
        root = QVBoxLayout(self)
        root.setContentsMargins(7, 12, 7, 7)
        root.setSpacing(4)
        self.link_label = QLabel("TX USB: disconnected | RX RSSI: -")
        self.link_label.setObjectName("softText")
        root.addWidget(self.link_label)
        grid = QGridLayout()
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(3)
        root.addLayout(grid)

        # Compact two-column layout: CH1-7 on the left and CH8-14 on the right.
        # This gives all 14 channels without making the main cockpit too tall.
        rows_per_col = 7 if channel_count <= 14 else int(math.ceil(channel_count / 2))
        for i in range(channel_count):
            block = i // rows_per_col
            row = i % rows_per_col
            col = block * 4
            lab = QLabel(f"CH{i+1}")
            lab.setMinimumWidth(34)
            tx = QProgressBar(); tx.setRange(900, 2100); tx.setValue(1500); tx.setFormat("TX %v"); tx.setFixedHeight(13)
            rx = QProgressBar(); rx.setRange(900, 2100); rx.setValue(1500); rx.setFormat("RX %v"); rx.setFixedHeight(13)
            grid.addWidget(lab, row, col)
            grid.addWidget(tx, row, col + 1)
            grid.addWidget(rx, row, col + 2)
            if block == 0:
                spacer = QLabel(""); spacer.setFixedWidth(4); grid.addWidget(spacer, row, col + 3)
            self.tx_bars.append(tx); self.rx_bars.append(rx)

    @staticmethod
    def set_bar(bar: QProgressBar, value: int, prefix: str):
        if value and value > 0:
            bar.setValue(int(clamp(value, 900, 2100)))
            bar.setFormat(f"{prefix} {int(value)}")
        else:
            bar.setValue(900)
            bar.setFormat(f"{prefix} -")

    def update_data(self, tx: Dict[str, Any], rx: Dict[str, Any]):
        tx_ch = tx.get("rc_channels", [0] * self.channel_count) if tx else [0] * self.channel_count
        rx_ch = rx.get("rc_channels", [0] * self.channel_count) if rx else [0] * self.channel_count
        for i in range(self.channel_count):
            self.set_bar(self.tx_bars[i], int(tx_ch[i]) if i < len(tx_ch) else 0, "TX")
            self.set_bar(self.rx_bars[i], int(rx_ch[i]) if i < len(rx_ch) else 0, "RX")
        tx_status = "connected" if tx and tx.get("connected") else "disconnected"
        self.link_label.setText(
            f"TX USB: {tx_status} {tx.get('name','') if tx else ''} | TX rate {safe_float(tx.get('update_rate_hz') if tx else 0):.0f}Hz | "
            f"RX RC rssi {rx.get('rc_rssi') if rx else '-'} | radio rssi/rem {rx.get('radio_rssi') if rx else '-'}/{rx.get('radio_remrssi') if rx else '-'}"
        )

class MapBridge(QObject):
    waypointSelected = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def mapClicked(self, lat: float, lon: float):
        self.waypointSelected.emit(float(lat), float(lon))


class MissionPlannerWidget(QGroupBox):
    waypoint_command = pyqtSignal(float, float, float, float)

    def __init__(self):
        super().__init__("Map Waypoint Flight Planning")
        self.bridge = MapBridge()
        self.selected_lat = None
        self.selected_lon = None
        self.start_lat = None
        self.start_lon = None
        self.telemetry = {}
        root = QVBoxLayout(self)
        info = QLabel("Click map or type waypoint. This does not arm/take off. Use only after stable manual takeoff/Loiter and GPS lock.")
        info.setWordWrap(True)
        root.addWidget(info)
        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter, stretch=1)

        left = QWidget(); left_layout = QVBoxLayout(left)
        if WEBENGINE_AVAILABLE:
            self.map = QWebEngineView()
            self.channel = QWebChannel()
            self.channel.registerObject("bridge", self.bridge)
            self.map.page().setWebChannel(self.channel)
            self.map.setHtml(self._map_html(), QUrl("qrc:///"))
            left_layout.addWidget(self.map)
        else:
            self.map = QLabel("PyQtWebEngine not installed. Map disabled, but manual lat/lon entry still works.")
            self.map.setAlignment(Qt.AlignCenter)
            left_layout.addWidget(self.map)
        splitter.addWidget(left)

        right = QWidget(); form = QFormLayout(right)
        self.lat_box = QDoubleSpinBox(); self.lat_box.setDecimals(7); self.lat_box.setRange(-90, 90)
        self.lon_box = QDoubleSpinBox(); self.lon_box.setDecimals(7); self.lon_box.setRange(-180, 180)
        self.alt_box = QDoubleSpinBox(); self.alt_box.setDecimals(1); self.alt_box.setRange(1, 500); self.alt_box.setValue(10.0); self.alt_box.setSuffix(" m rel")
        self.speed_box = QDoubleSpinBox(); self.speed_box.setDecimals(1); self.speed_box.setRange(0.5, 20.0); self.speed_box.setValue(3.0); self.speed_box.setSuffix(" m/s")
        self.return_check = QCheckBox("Ask/return to start after reaching waypoint")
        self.arrival_radius = QDoubleSpinBox(); self.arrival_radius.setDecimals(1); self.arrival_radius.setRange(2, 50); self.arrival_radius.setValue(6); self.arrival_radius.setSuffix(" m")
        form.addRow("Waypoint lat", self.lat_box)
        form.addRow("Waypoint lon", self.lon_box)
        form.addRow("Relative altitude", self.alt_box)
        form.addRow("Guided speed", self.speed_box)
        form.addRow("Arrival radius", self.arrival_radius)
        form.addRow("Return option", self.return_check)
        self.selected_label = QLabel("Selected: none")
        self.status_label = QLabel("Mission status: idle")
        self.status_label.setWordWrap(True)
        form.addRow(self.selected_label)
        form.addRow(self.status_label)
        btns = QHBoxLayout()
        self.use_current_btn = QPushButton("Use Current GPS")
        self.set_start_btn = QPushButton("Set Start/Home")
        self.send_btn = QPushButton("Send Guided Waypoint")
        self.return_btn = QPushButton("Send Return")
        btns.addWidget(self.use_current_btn); btns.addWidget(self.set_start_btn); btns.addWidget(self.send_btn); btns.addWidget(self.return_btn)
        form.addRow(btns)
        splitter.addWidget(right)
        splitter.setSizes([900, 360])
        self.bridge.waypointSelected.connect(self.on_map_clicked)
        self.use_current_btn.clicked.connect(self.use_current_gps)
        self.set_start_btn.clicked.connect(self.set_start_from_current)
        self.send_btn.clicked.connect(self.confirm_send)
        self.return_btn.clicked.connect(self.confirm_return)

    def _map_html(self):
        return """
<!DOCTYPE html><html><head><meta charset='utf-8'/>
<style>html,body,#map{height:100%;margin:0;background:#111827}.status{position:absolute;z-index:999;left:10px;top:10px;background:rgba(0,0,0,.72);color:white;padding:8px;border-radius:8px;font-family:Arial;font-size:12px}</style>
<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>
<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>
<script src='qrc:///qtwebchannel/qwebchannel.js'></script></head><body>
<div id='map'></div><div class='status' id='status'>Click map to set waypoint</div>
<script>
var bridge=null; new QWebChannel(qt.webChannelTransport,function(channel){bridge=channel.objects.bridge;});
var map=L.map('map').setView([1.3521,103.8198],16);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:20,attribution:'&copy; OSM'}).addTo(map);
var drone=L.marker([1.3521,103.8198]).addTo(map).bindPopup('Drone');
var waypoint=null; var line=null;
function updateDrone(lat,lon,hdg,sats,alt){ if(!lat||!lon)return; drone.setLatLng([lat,lon]); document.getElementById('status').innerHTML='Drone '+lat.toFixed(6)+', '+lon.toFixed(6)+'<br>HDG '+hdg.toFixed(0)+'° Sats '+sats+' Alt '+alt.toFixed(1)+'m'; }
function setWaypoint(lat,lon){ if(waypoint){waypoint.setLatLng([lat,lon]);} else {waypoint=L.marker([lat,lon],{draggable:true}).addTo(map).bindPopup('Selected waypoint'); waypoint.on('dragend',function(e){var p=e.target.getLatLng(); if(bridge){bridge.mapClicked(p.lat,p.lng);} });} if(line){map.removeLayer(line);} line=L.polyline([drone.getLatLng(),[lat,lon]],{color:'#22c55e',weight:3}).addTo(map); }
map.on('click',function(e){setWaypoint(e.latlng.lat,e.latlng.lng); if(bridge){bridge.mapClicked(e.latlng.lat,e.latlng.lng);} });
</script></body></html>
"""

    def update_telemetry(self, d: Dict[str, Any]):
        self.telemetry = dict(d)
        if WEBENGINE_AVAILABLE and hasattr(self, "map") and isinstance(self.map, QWebEngineView):
            lat, lon = d.get("lat"), d.get("lon")
            if lat is not None and lon is not None:
                js = f"updateDrone({lat:.8f},{lon:.8f},{safe_float(d.get('heading_deg')):.1f},{int(d.get('gps_sats',0))},{safe_float(d.get('rel_alt_m')):.1f});"
                self.map.page().runJavaScript(js)
        if self.start_lat is None and d.get("armed") and d.get("lat") is not None:
            self.start_lat = d.get("lat"); self.start_lon = d.get("lon")

    def on_map_clicked(self, lat: float, lon: float):
        self.selected_lat = lat; self.selected_lon = lon
        self.lat_box.setValue(lat); self.lon_box.setValue(lon)
        self.selected_label.setText(f"Selected: {lat:.7f}, {lon:.7f}")
        answer = QMessageBox.question(self, "Return to start?", "After this waypoint, do you want a return-to-start option saved?", QMessageBox.Yes | QMessageBox.No)
        self.return_check.setChecked(answer == QMessageBox.Yes)

    def use_current_gps(self):
        lat, lon = self.telemetry.get("lat"), self.telemetry.get("lon")
        if lat is None or lon is None:
            QMessageBox.warning(self, "No GPS", "Current GPS position is not available yet.")
            return
        self.on_map_clicked(float(lat), float(lon))

    def set_start_from_current(self):
        lat, lon = self.telemetry.get("lat"), self.telemetry.get("lon")
        if lat is None or lon is None:
            QMessageBox.warning(self, "No GPS", "Current GPS position is not available yet.")
            return
        self.start_lat = float(lat); self.start_lon = float(lon)
        self.status_label.setText(f"Start/Home saved: {self.start_lat:.7f}, {self.start_lon:.7f}")

    def confirm_send(self):
        lat, lon = float(self.lat_box.value()), float(self.lon_box.value())
        alt, speed = float(self.alt_box.value()), float(self.speed_box.value())
        if abs(lat) < 0.000001 and abs(lon) < 0.000001:
            QMessageBox.warning(self, "No waypoint", "Select a real waypoint first.")
            return
        dist = haversine_m(self.telemetry.get("lat"), self.telemetry.get("lon"), lat, lon)
        text = f"Send Guided waypoint?\n\nTarget: {lat:.7f}, {lon:.7f}\nAlt: {alt:.1f} m REL\nSpeed: {speed:.1f} m/s\nDistance: {dist:.1f} m\n\nThis will command the aircraft through MAVLink. It will not arm or take off."
        if QMessageBox.warning(self, "Confirm Guided Waypoint", text, QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.waypoint_command.emit(lat, lon, alt, speed)

    def confirm_return(self):
        if self.start_lat is None or self.start_lon is None:
            QMessageBox.warning(self, "No start", "Set Start/Home first.")
            return
        alt, speed = float(self.alt_box.value()), float(self.speed_box.value())
        if QMessageBox.question(self, "Confirm Return", f"Send return waypoint to start/home?\n{self.start_lat:.7f}, {self.start_lon:.7f}", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.waypoint_command.emit(self.start_lat, self.start_lon, alt, speed)

    def set_status(self, text: str):
        self.status_label.setText(text)




class CompactMissionPlannerWidget(QGroupBox):
    """Space-efficient main-tab map + waypoint sender.

    Same safety model as the full MissionPlannerWidget: it does not arm, disarm,
    or take off. It only sends a guarded Guided waypoint command after the user
    confirms and after the aircraft is already flying manually/Loiter with GPS.
    """
    waypoint_command = pyqtSignal(float, float, float, float)

    def __init__(self):
        super().__init__("3D Map / Waypoint Planner")
        self.bridge = MapBridge()
        self.telemetry = {}
        self.selected_lat = None
        self.selected_lon = None
        self.start_lat = None
        self.start_lon = None
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 12, 8, 8)
        root.setSpacing(6)

        self.map_status = QLabel("Click map or use current GPS. Guided command only; no arm/takeoff.")
        self.map_status.setWordWrap(True)
        self.map_status.setObjectName("softText")
        root.addWidget(self.map_status)

        if WEBENGINE_AVAILABLE:
            self.map = QWebEngineView()
            self.channel = QWebChannel()
            self.channel.registerObject("bridge", self.bridge)
            self.map.page().setWebChannel(self.channel)
            self.map.setHtml(self._map_html(), QUrl("qrc:///"))
            self.map.setMinimumHeight(230)
            root.addWidget(self.map, 1)
        else:
            self.map = QLabel("PyQtWebEngine missing. Manual waypoint entry still works.")
            self.map.setAlignment(Qt.AlignCenter)
            self.map.setMinimumHeight(230)
            self.map.setStyleSheet("background:#05070a;border-radius:10px;color:#dbeafe;")
            root.addWidget(self.map, 1)

        form_box = QFrame()
        form_box.setObjectName("glassFrame")
        form = QGridLayout(form_box)
        form.setContentsMargins(8, 8, 8, 8)
        form.setHorizontalSpacing(6)
        form.setVerticalSpacing(5)

        self.lat_box = QDoubleSpinBox(); self.lat_box.setDecimals(7); self.lat_box.setRange(-90, 90)
        self.lon_box = QDoubleSpinBox(); self.lon_box.setDecimals(7); self.lon_box.setRange(-180, 180)
        self.alt_box = QDoubleSpinBox(); self.alt_box.setDecimals(1); self.alt_box.setRange(1, 500); self.alt_box.setValue(10.0); self.alt_box.setSuffix(" m")
        self.speed_box = QDoubleSpinBox(); self.speed_box.setDecimals(1); self.speed_box.setRange(0.5, 20.0); self.speed_box.setValue(3.0); self.speed_box.setSuffix(" m/s")
        self.arrival_radius = QDoubleSpinBox(); self.arrival_radius.setDecimals(1); self.arrival_radius.setRange(2, 50); self.arrival_radius.setValue(6); self.arrival_radius.setSuffix(" m")
        self.return_check = QCheckBox("Ask return")
        self.selected_label = QLabel("Selected: none")
        self.status_label = QLabel("Mission: idle")
        self.status_label.setWordWrap(True)

        form.addWidget(QLabel("Lat"), 0, 0); form.addWidget(self.lat_box, 0, 1)
        form.addWidget(QLabel("Lon"), 0, 2); form.addWidget(self.lon_box, 0, 3)
        form.addWidget(QLabel("Alt"), 1, 0); form.addWidget(self.alt_box, 1, 1)
        form.addWidget(QLabel("Speed"), 1, 2); form.addWidget(self.speed_box, 1, 3)
        form.addWidget(QLabel("Arrival"), 2, 0); form.addWidget(self.arrival_radius, 2, 1)
        form.addWidget(self.return_check, 2, 2, 1, 2)
        form.addWidget(self.selected_label, 3, 0, 1, 4)
        form.addWidget(self.status_label, 4, 0, 1, 4)

        btn_row = QHBoxLayout()
        self.use_current_btn = QPushButton("Use GPS")
        self.set_start_btn = QPushButton("Set Home")
        self.send_btn = QPushButton("Send WP")
        self.return_btn = QPushButton("Return")
        for b in (self.use_current_btn, self.set_start_btn, self.send_btn, self.return_btn):
            b.setMinimumHeight(28)
            btn_row.addWidget(b)
        form.addLayout(btn_row, 5, 0, 1, 4)
        root.addWidget(form_box)

        self.bridge.waypointSelected.connect(self.on_map_clicked)
        self.use_current_btn.clicked.connect(self.use_current_gps)
        self.set_start_btn.clicked.connect(self.set_start_from_current)
        self.send_btn.clicked.connect(self.confirm_send)
        self.return_btn.clicked.connect(self.confirm_return)

    def _map_html(self):
        return """
<!DOCTYPE html><html><head><meta charset='utf-8'/>
<style>
html,body,#map{height:100%;margin:0;background:#07111f}.status{position:absolute;z-index:999;left:10px;top:10px;background:rgba(3,7,18,.80);color:white;padding:7px 9px;border:1px solid rgba(148,163,184,.45);border-radius:10px;font-family:Arial;font-size:12px;box-shadow:0 4px 16px rgba(0,0,0,.4)}
</style>
<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>
<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>
<script src='qrc:///qtwebchannel/qwebchannel.js'></script></head><body>
<div id='map'></div><div class='status' id='status'>Click map to set waypoint</div>
<script>
var bridge=null; new QWebChannel(qt.webChannelTransport,function(channel){bridge=channel.objects.bridge;});
var map=L.map('map',{zoomControl:false}).setView([1.3521,103.8198],17);
L.control.zoom({position:'bottomright'}).addTo(map);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:20,attribution:'&copy; OSM'}).addTo(map);
var droneIcon=L.divIcon({className:'',html:'<div style="width:28px;height:28px;transform-origin:center"><svg width="28" height="28" viewBox="0 0 34 34"><path d="M17 2 L27 30 L17 24 L7 30 Z" fill="#2563eb" stroke="white" stroke-width="2"/><circle cx="17" cy="17" r="4" fill="#ef4444" stroke="white"/></svg></div>',iconSize:[28,28],iconAnchor:[14,14]});
var drone=L.marker([1.3521,103.8198],{icon:droneIcon}).addTo(map).bindPopup('Drone');
var waypoint=null; var line=null;
function updateDrone(lat,lon,hdg,sats,alt){ if(!lat||!lon)return; drone.setLatLng([lat,lon]); map.panTo([lat,lon],{animate:false}); document.getElementById('status').innerHTML='Drone '+lat.toFixed(6)+', '+lon.toFixed(6)+'<br>HDG '+hdg.toFixed(0)+'° | Sats '+sats+' | Alt '+alt.toFixed(1)+'m'; if(line&&waypoint){line.setLatLngs([drone.getLatLng(),waypoint.getLatLng()]);}}
function setWaypoint(lat,lon){ if(waypoint){waypoint.setLatLng([lat,lon]);} else {waypoint=L.marker([lat,lon],{draggable:true}).addTo(map).bindPopup('Waypoint'); waypoint.on('dragend',function(e){var p=e.target.getLatLng(); if(bridge){bridge.mapClicked(p.lat,p.lng);} });} if(line){map.removeLayer(line);} line=L.polyline([drone.getLatLng(),[lat,lon]],{color:'#22c55e',weight:4,opacity:.9}).addTo(map); }
map.on('click',function(e){setWaypoint(e.latlng.lat,e.latlng.lng); if(bridge){bridge.mapClicked(e.latlng.lat,e.latlng.lng);}});
</script></body></html>
"""

    def on_map_clicked(self, lat: float, lon: float):
        self.selected_lat = float(lat); self.selected_lon = float(lon)
        self.lat_box.setValue(self.selected_lat); self.lon_box.setValue(self.selected_lon)
        self.selected_label.setText(f"Selected: {self.selected_lat:.7f}, {self.selected_lon:.7f}")

    def use_current_gps(self):
        lat, lon = self.telemetry.get("lat"), self.telemetry.get("lon")
        if lat is None or lon is None:
            self.set_status("No current GPS position available.")
            return
        self.on_map_clicked(float(lat), float(lon))
        if WEBENGINE_AVAILABLE:
            try:
                self.map.page().runJavaScript(f"setWaypoint({float(lat):.8f},{float(lon):.8f});")
            except Exception:
                pass

    def set_start_from_current(self):
        lat, lon = self.telemetry.get("lat"), self.telemetry.get("lon")
        if lat is None or lon is None:
            self.set_status("No GPS position to set as start/home.")
            return
        self.start_lat = float(lat); self.start_lon = float(lon)
        self.set_status(f"Start/Home set: {self.start_lat:.7f}, {self.start_lon:.7f}")

    def confirm_send(self):
        lat = float(self.lat_box.value()); lon = float(self.lon_box.value())
        alt = float(self.alt_box.value()); speed = float(self.speed_box.value())
        if abs(lat) < 1e-8 and abs(lon) < 1e-8:
            QMessageBox.warning(self, "Waypoint", "Select or enter a valid waypoint first.")
            return
        msg = f"Send Guided waypoint?\n\nLat: {lat:.7f}\nLon: {lon:.7f}\nAlt: {alt:.1f} m rel\nSpeed: {speed:.1f} m/s\n\nThis does not arm or take off."
        if QMessageBox.question(self, "Confirm waypoint", msg, QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.waypoint_command.emit(lat, lon, alt, speed)
            if self.return_check.isChecked():
                self.set_status("Waypoint sent. Return option selected; press Return after arrival/confirmation.")

    def confirm_return(self):
        if self.start_lat is None or self.start_lon is None:
            self.set_start_from_current()
            if self.start_lat is None:
                return
        alt = float(self.alt_box.value()); speed = float(self.speed_box.value())
        if QMessageBox.question(self, "Confirm return", f"Send Guided return to start/home?\n\nLat: {self.start_lat:.7f}\nLon: {self.start_lon:.7f}\nAlt: {alt:.1f} m rel", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.waypoint_command.emit(float(self.start_lat), float(self.start_lon), alt, speed)

    def update_telemetry(self, d: Dict[str, Any]):
        self.telemetry = d or {}
        lat, lon = d.get("lat"), d.get("lon")
        if WEBENGINE_AVAILABLE and lat is not None and lon is not None:
            try:
                hdg = safe_float(d.get("heading_deg")); sats = int(d.get("gps_sats", 0)); alt = safe_float(d.get("rel_alt_m"))
                self.map.page().runJavaScript(f"updateDrone({float(lat):.8f},{float(lon):.8f},{hdg:.1f},{sats},{alt:.1f});")
            except Exception:
                pass

    def set_status(self, text: str):
        self.status_label.setText(text)

class TechnicalSetupWidget(QGroupBox):
    mav_connect = pyqtSignal(str, int)
    mav_disconnect = pyqtSignal()
    tx_connect = pyqtSignal(object)
    tx_disconnect = pyqtSignal()
    fpv_start = pyqtSignal(object)
    fpv_stop = pyqtSignal()
    fpv_record_start = pyqtSignal()
    fpv_record_stop = pyqtSignal()
    ai_start = pyqtSignal()
    ai_stop = pyqtSignal()
    ai_overlay_start = pyqtSignal()
    ai_overlay_stop = pyqtSignal()
    post_process_requested = pyqtSignal()
    folder_changed = pyqtSignal(str)
    fuel_checked = pyqtSignal()

    def __init__(self):
        super().__init__("Technical / Setup")
        root = QVBoxLayout(self)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); root.addWidget(scroll)
        body = QWidget(); scroll.setWidget(body)
        layout = QHBoxLayout(body)

        left = QVBoxLayout(); mid = QVBoxLayout(); right = QVBoxLayout()
        layout.addLayout(left, 1); layout.addLayout(mid, 1); layout.addLayout(right, 1)

        # MAVLink
        mav_box = QGroupBox("Flight Controller Telemetry: MAVLink / Betaflight MSP")
        mav = QGridLayout(mav_box)
        self.protocol_combo = QComboBox(); self.protocol_combo.addItems(["MAVLink (ArduPilot/PX4)", "MSP (Betaflight/iNav USB)"]); self.protocol_combo.currentIndexChanged.connect(self.on_protocol_changed)
        self.mav_combo = QComboBox(); self.mav_combo.setEditable(True); self.refresh_mav_ports()
        self.baud_combo = QComboBox(); self.baud_combo.addItems(["57600", "115200", "230400", "460800", "921600"]); self.baud_combo.setCurrentText("57600")
        self.mav_status = QLabel("Disconnected")
        btn_refresh = QPushButton("Refresh COM")
        btn_connect = QPushButton("Connect FC")
        btn_disconnect = QPushButton("Disconnect")
        btn_refresh.clicked.connect(self.refresh_mav_ports)
        btn_connect.clicked.connect(lambda: self.mav_connect.emit(clean_connection_string(self.mav_combo.currentText()), int(self.baud_combo.currentText())))
        btn_disconnect.clicked.connect(self.mav_disconnect.emit)
        mav.addWidget(QLabel("Protocol"), 0, 0); mav.addWidget(self.protocol_combo, 0, 1, 1, 2)
        mav.addWidget(QLabel("Connection"), 1, 0); mav.addWidget(self.mav_combo, 1, 1, 1, 2)
        mav.addWidget(QLabel("Baud"), 2, 0); mav.addWidget(self.baud_combo, 2, 1)
        mav.addWidget(btn_refresh, 3, 0); mav.addWidget(btn_connect, 3, 1); mav.addWidget(btn_disconnect, 3, 2)
        mav.addWidget(self.mav_status, 4, 0, 1, 3)
        left.addWidget(mav_box)

        # TX15
        tx_box = QGroupBox("RadioMaster TX15 Max USB HID")
        tx = QGridLayout(tx_box)
        self.tx_combo = QComboBox(); self.refresh_tx_devices()
        self.tx_status = QLabel("Disconnected")
        btn_tx_refresh = QPushButton("Refresh TX")
        btn_tx_connect = QPushButton("Connect TX USB")
        btn_tx_auto = QPushButton("Auto Detect TX15")
        btn_tx_disconnect = QPushButton("Disconnect")
        btn_tx_refresh.clicked.connect(self.refresh_tx_devices)
        btn_tx_auto.clicked.connect(self.auto_detect_tx)
        btn_tx_connect.clicked.connect(self.connect_selected_tx)
        btn_tx_disconnect.clicked.connect(self.tx_disconnect.emit)
        tx.addWidget(QLabel("Device"), 0, 0); tx.addWidget(self.tx_combo, 0, 1, 1, 3)
        tx.addWidget(btn_tx_refresh, 1, 0); tx.addWidget(btn_tx_auto, 1, 1); tx.addWidget(btn_tx_connect, 1, 2); tx.addWidget(btn_tx_disconnect, 1, 3)
        tx.addWidget(self.tx_status, 2, 0, 1, 4)
        left.addWidget(tx_box)
        left.addStretch(1)

        # FPV + AI
        fpv_box = QGroupBox("FPV Source + AI Recognition - separated low-latency pipeline")
        fpv_outer = QVBoxLayout(fpv_box)

        fpv_capture_box = QGroupBox("FPV capture / zero-latency video")
        fpv = QGridLayout(fpv_capture_box)
        self.video_source = QLineEdit("0")
        self.capture_backend = QComboBox(); self.capture_backend.addItems(["DirectShow", "MSMF", "Auto", "FFMPEG", "GStreamer"])
        self.fps_spin = QSpinBox(); self.fps_spin.setRange(1, 240); self.fps_spin.setValue(60)
        self.fpv_width = QSpinBox(); self.fpv_width.setRange(0, 4096); self.fpv_width.setValue(1280); self.fpv_width.setSuffix(" px")
        self.fpv_height = QSpinBox(); self.fpv_height.setRange(0, 2160); self.fpv_height.setValue(720); self.fpv_height.setSuffix(" px")
        self.fourcc_combo = QComboBox(); self.fourcc_combo.addItems(["MJPG", "YUY2", "H264", "Auto"])
        self.buffer_spin = QSpinBox(); self.buffer_spin.setRange(1, 8); self.buffer_spin.setValue(1)
        self.display_scale_spin = QDoubleSpinBox(); self.display_scale_spin.setRange(0.25, 1.0); self.display_scale_spin.setSingleStep(0.05); self.display_scale_spin.setValue(1.0)
        btn_fpv_start = QPushButton("Start FPV only")
        btn_fpv_stop = QPushButton("Stop FPV")
        btn_fpv_rec = QPushButton("Record Raw FPV 60FPS")
        btn_fpv_rec_stop = QPushButton("Stop FPV Recording")
        btn_fpv_start.clicked.connect(self.start_fpv_only)
        btn_fpv_stop.clicked.connect(self.fpv_stop.emit)
        btn_fpv_rec.clicked.connect(self.fpv_record_start.emit)
        btn_fpv_rec_stop.clicked.connect(self.fpv_record_stop.emit)
        fpv.addWidget(QLabel("Source"), 0, 0); fpv.addWidget(self.video_source, 0, 1, 1, 3)
        fpv.addWidget(QLabel("Backend"), 1, 0); fpv.addWidget(self.capture_backend, 1, 1)
        fpv.addWidget(QLabel("FPS"), 1, 2); fpv.addWidget(self.fps_spin, 1, 3)
        fpv.addWidget(QLabel("Width"), 2, 0); fpv.addWidget(self.fpv_width, 2, 1)
        fpv.addWidget(QLabel("Height"), 2, 2); fpv.addWidget(self.fpv_height, 2, 3)
        fpv.addWidget(QLabel("Format"), 3, 0); fpv.addWidget(self.fourcc_combo, 3, 1)
        fpv.addWidget(QLabel("Buffer"), 3, 2); fpv.addWidget(self.buffer_spin, 3, 3)
        fpv.addWidget(QLabel("UI scale"), 4, 0); fpv.addWidget(self.display_scale_spin, 4, 1)
        fpv.addWidget(btn_fpv_start, 5, 0, 1, 2); fpv.addWidget(btn_fpv_stop, 5, 2, 1, 2)
        fpv.addWidget(btn_fpv_rec, 6, 0, 1, 2); fpv.addWidget(btn_fpv_rec_stop, 6, 2, 1, 2)
        fpv_outer.addWidget(fpv_capture_box)

        ai_box = QGroupBox("AI recognition / async RTX pipeline")
        ai = QGridLayout(ai_box)
        self.ai_enabled = QCheckBox("Auto-start AI with FPV"); self.ai_enabled.setChecked(False)
        self.model_path = QLineEdit("yolov8n.pt")
        browse_model = QPushButton("Browse Model")
        browse_model.clicked.connect(self.browse_model)
        self.ai_device_combo = QComboBox(); self.ai_device_combo.addItems(["auto", "cuda:0", "cpu"])
        self.ai_imgsz = QSpinBox(); self.ai_imgsz.setRange(320, 1536); self.ai_imgsz.setSingleStep(32); self.ai_imgsz.setValue(640)
        self.ai_input_width = QSpinBox(); self.ai_input_width.setRange(320, 3840); self.ai_input_width.setSingleStep(80); self.ai_input_width.setValue(960); self.ai_input_width.setSuffix(" px")
        self.ai_half = QCheckBox("FP16 on CUDA"); self.ai_half.setChecked(True)
        self.conf_spin = QDoubleSpinBox(); self.conf_spin.setRange(0.05, 0.95); self.conf_spin.setSingleStep(0.05); self.conf_spin.setValue(0.35)
        self.detect_every = QSpinBox(); self.detect_every.setRange(1, 120); self.detect_every.setValue(3)
        self.ai_max_det = QSpinBox(); self.ai_max_det.setRange(1, 300); self.ai_max_det.setValue(50)
        self.save_every = QSpinBox(); self.save_every.setRange(1, 300); self.save_every.setValue(10)
        self.btn_cv_start = QPushButton("Start Computer Vision")
        self.btn_cv_stop = QPushButton("Stop Computer Vision")
        self.btn_overlay_start = QPushButton("Start AI Overlay")
        self.btn_overlay_stop = QPushButton("Stop AI Overlay")
        self.btn_post_process = QPushButton("Process Recorded Video")
        self.btn_cv_start.clicked.connect(self.ai_start.emit)
        self.btn_cv_stop.clicked.connect(self.ai_stop.emit)
        self.btn_overlay_start.clicked.connect(self.ai_overlay_start.emit)
        self.btn_overlay_stop.clicked.connect(self.ai_overlay_stop.emit)
        self.btn_post_process.clicked.connect(self.post_process_requested.emit)
        ai.addWidget(QLabel("Model"), 0, 0); ai.addWidget(self.model_path, 0, 1, 1, 3); ai.addWidget(browse_model, 0, 4)
        ai.addWidget(QLabel("Device"), 1, 0); ai.addWidget(self.ai_device_combo, 1, 1)
        ai.addWidget(QLabel("imgsz"), 1, 2); ai.addWidget(self.ai_imgsz, 1, 3)
        ai.addWidget(self.ai_half, 1, 4)
        ai.addWidget(QLabel("AI input width"), 2, 0); ai.addWidget(self.ai_input_width, 2, 1)
        ai.addWidget(QLabel("Confidence"), 2, 2); ai.addWidget(self.conf_spin, 2, 3)
        ai.addWidget(QLabel("Detect every"), 3, 0); ai.addWidget(self.detect_every, 3, 1)
        ai.addWidget(QLabel("Max det"), 3, 2); ai.addWidget(self.ai_max_det, 3, 3)
        ai.addWidget(QLabel("Save frame every"), 4, 0); ai.addWidget(self.save_every, 4, 1)
        ai.addWidget(self.ai_enabled, 4, 2, 1, 2)
        ai.addWidget(self.btn_cv_start, 5, 0, 1, 2); ai.addWidget(self.btn_cv_stop, 5, 2, 1, 2)
        ai.addWidget(self.btn_overlay_start, 6, 0, 1, 2); ai.addWidget(self.btn_overlay_stop, 6, 2, 1, 2)
        ai.addWidget(self.btn_post_process, 7, 0, 1, 4)
        note = QLabel("v100 split-feed: raw FPV goes directly to Main at 60Hz; live AI is a separate copied/downscaled feed. Start Computer Vision processes boxes; Start AI Overlay controls whether boxes are painted on the Main FPV. Post-processing mode removes live overlay and processes recorded video later.")
        note.setWordWrap(True)
        ai.addWidget(note, 8, 0, 1, 5)
        fpv_outer.addWidget(ai_box)
        mid.addWidget(fpv_box)

        # Recording
        rec_box = QGroupBox("Dataset Recording Folder")
        rec = QGridLayout(rec_box)
        default_folder = str(Path.home() / "drone_v100_records")
        self.folder_line = QLineEdit(default_folder)
        browse = QPushButton("Browse Folder")
        browse.clicked.connect(self.browse_folder)
        rec.addWidget(QLabel("Base folder"), 0, 0)
        rec.addWidget(self.folder_line, 0, 1)
        rec.addWidget(browse, 0, 2)
        rec_note = QLabel("Recording writes ai_overlay_video.mp4, telemetry.csv, detections.csv, tx_rx_channels.csv, frames, and detection crops.")
        rec_note.setWordWrap(True)
        rec.addWidget(rec_note, 1, 0, 1, 3)
        mid.addWidget(rec_box)
        mid.addStretch(1)

        # Full battery / propulsion calculator
        calc_box = QGroupBox("Scientific Fuel / Propulsion / Live Voltage Estimator")
        calc = QFormLayout(calc_box)
        self.fuel_form = calc
        self.fuel_mode = QComboBox(); self.fuel_mode.addItems(["Manual scientific calculation", "Live voltage estimator from flight controller"]); self.fuel_mode.currentIndexChanged.connect(self.update_fuel_mode_visibility)
        self.cells = QSpinBox(); self.cells.setRange(2, 12); self.cells.setValue(6)
        self.cell_voltage = QDoubleSpinBox(); self.cell_voltage.setRange(3.0, 4.25); self.cell_voltage.setDecimals(3); self.cell_voltage.setValue(3.70); self.cell_voltage.setSuffix(" V/cell nominal")
        self.capacity = QDoubleSpinBox(); self.capacity.setRange(500, 100000); self.capacity.setValue(3300); self.capacity.setSuffix(" mAh")
        self.usable = QDoubleSpinBox(); self.usable.setRange(20, 98); self.usable.setValue(75); self.usable.setSuffix(" % usable")
        self.reserve = QDoubleSpinBox(); self.reserve.setRange(0, 60); self.reserve.setValue(20); self.reserve.setSuffix(" % reserve")
        self.warn_cell_v = QDoubleSpinBox(); self.warn_cell_v.setRange(3.2, 4.2); self.warn_cell_v.setDecimals(2); self.warn_cell_v.setValue(3.60); self.warn_cell_v.setSuffix(" V/cell warn")
        self.reserve_cell_v = QDoubleSpinBox(); self.reserve_cell_v.setRange(3.0, 4.2); self.reserve_cell_v.setDecimals(2); self.reserve_cell_v.setValue(3.50); self.reserve_cell_v.setSuffix(" V/cell reserve")
        self.critical_cell_v = QDoubleSpinBox(); self.critical_cell_v.setRange(2.8, 4.2); self.critical_cell_v.setDecimals(2); self.critical_cell_v.setValue(3.35); self.critical_cell_v.setSuffix(" V/cell critical")
        self.auw = QDoubleSpinBox(); self.auw.setRange(0.2, 50.0); self.auw.setDecimals(2); self.auw.setValue(2.70); self.auw.setSuffix(" kg AUW")
        self.payload_mass = QDoubleSpinBox(); self.payload_mass.setRange(0, 30.0); self.payload_mass.setDecimals(2); self.payload_mass.setValue(0.0); self.payload_mass.setSuffix(" kg payload")
        self.rotors = QSpinBox(); self.rotors.setRange(1, 16); self.rotors.setValue(4)
        self.cruise_throttle = QDoubleSpinBox(); self.cruise_throttle.setRange(1, 100); self.cruise_throttle.setValue(40); self.cruise_throttle.setSuffix(" % cruise throttle")
        self.avg_current = QDoubleSpinBox(); self.avg_current.setRange(1, 800); self.avg_current.setValue(45); self.avg_current.setSuffix(" A measured cruise")
        self.hover_current = QDoubleSpinBox(); self.hover_current.setRange(1, 800); self.hover_current.setValue(55); self.hover_current.setSuffix(" A measured hover")
        self.max_current_motor = QDoubleSpinBox(); self.max_current_motor.setRange(1, 300); self.max_current_motor.setValue(62); self.max_current_motor.setSuffix(" A/motor")
        self.esc_rating = QDoubleSpinBox(); self.esc_rating.setRange(1, 300); self.esc_rating.setValue(65); self.esc_rating.setSuffix(" A ESC")
        self.max_thrust_motor = QDoubleSpinBox(); self.max_thrust_motor.setRange(0, 20000); self.max_thrust_motor.setValue(4000); self.max_thrust_motor.setSuffix(" g/motor measured")
        self.prop_diam = QDoubleSpinBox(); self.prop_diam.setRange(2, 40); self.prop_diam.setValue(10.0); self.prop_diam.setSuffix(" in")
        self.prop_pitch = QDoubleSpinBox(); self.prop_pitch.setRange(1, 25); self.prop_pitch.setValue(4.5); self.prop_pitch.setSuffix(" in")
        self.blades = QSpinBox(); self.blades.setRange(2, 8); self.blades.setValue(2)
        self.motor_kv = QDoubleSpinBox(); self.motor_kv.setRange(50, 5000); self.motor_kv.setValue(900); self.motor_kv.setSuffix(" KV")
        self.loading_factor = QDoubleSpinBox(); self.loading_factor.setRange(0.30, 1.00); self.loading_factor.setDecimals(2); self.loading_factor.setValue(0.70)
        self.figure_merit = QDoubleSpinBox(); self.figure_merit.setRange(0.25, 0.95); self.figure_merit.setDecimals(2); self.figure_merit.setValue(0.55)
        self.altitude_calc = QDoubleSpinBox(); self.altitude_calc.setRange(0, 9000); self.altitude_calc.setValue(0); self.altitude_calc.setSuffix(" m")
        self.temp_c = QDoubleSpinBox(); self.temp_c.setRange(-30, 60); self.temp_c.setValue(30); self.temp_c.setSuffix(" °C")
        self.humidity = QDoubleSpinBox(); self.humidity.setRange(0, 100); self.humidity.setValue(70); self.humidity.setSuffix(" % RH")
        self.ref_current = QDoubleSpinBox(); self.ref_current.setRange(1, 300); self.ref_current.setValue(50); self.ref_current.setSuffix(" A ref")
        self.ref_kv = QDoubleSpinBox(); self.ref_kv.setRange(50, 5000); self.ref_kv.setValue(1100); self.ref_kv.setSuffix(" KV ref")
        self.ref_voltage = QDoubleSpinBox(); self.ref_voltage.setRange(3, 80); self.ref_voltage.setValue(24); self.ref_voltage.setSuffix(" V ref")
        self.ref_diam = QDoubleSpinBox(); self.ref_diam.setRange(2, 40); self.ref_diam.setValue(9); self.ref_diam.setSuffix(" in ref")
        self.ref_pitch = QDoubleSpinBox(); self.ref_pitch.setRange(1, 25); self.ref_pitch.setValue(4); self.ref_pitch.setSuffix(" in ref")
        self.ref_blades = QSpinBox(); self.ref_blades.setRange(2, 8); self.ref_blades.setValue(3)
        self.calc_result = QLabel("Fuel check not completed."); self.calc_result.setWordWrap(True); self.calc_result.setObjectName("dynamicValue")
        self.live_result = QLabel("Live voltage estimator waiting for MAVLink battery voltage."); self.live_result.setWordWrap(True); self.live_result.setObjectName("dynamicValue")
        btn_calc = QPushButton("Run Fuel Check / Arm Ready")
        btn_calc.clicked.connect(self.calculate_battery)
        calc.addRow("Mode", self.fuel_mode)
        calc.addRow("Cells", self.cells)
        calc.addRow("Nominal cell voltage", self.cell_voltage)
        calc.addRow("Capacity", self.capacity)
        calc.addRow("Usable", self.usable)
        calc.addRow("Reserve", self.reserve)
        calc.addRow("Warning voltage", self.warn_cell_v)
        calc.addRow("Reserve voltage", self.reserve_cell_v)
        calc.addRow("Critical voltage", self.critical_cell_v)
        calc.addRow("AUW", self.auw)
        calc.addRow("Payload mass", self.payload_mass)
        calc.addRow("Rotors", self.rotors)
        calc.addRow("Cruise throttle", self.cruise_throttle)
        calc.addRow("Measured cruise current", self.avg_current)
        calc.addRow("Measured hover current", self.hover_current)
        calc.addRow("Max current limit", self.max_current_motor)
        calc.addRow("ESC rating", self.esc_rating)
        calc.addRow("Max thrust/motor", self.max_thrust_motor)
        calc.addRow("Prop diameter", self.prop_diam)
        calc.addRow("Prop pitch", self.prop_pitch)
        calc.addRow("Blade count", self.blades)
        calc.addRow("Motor KV", self.motor_kv)
        calc.addRow("Loaded RPM factor", self.loading_factor)
        calc.addRow("Figure of merit", self.figure_merit)
        calc.addRow("Altitude", self.altitude_calc)
        calc.addRow("Temperature", self.temp_c)
        calc.addRow("Humidity", self.humidity)
        calc.addRow("Reference current", self.ref_current)
        calc.addRow("Reference KV", self.ref_kv)
        calc.addRow("Reference voltage", self.ref_voltage)
        calc.addRow("Reference diameter", self.ref_diam)
        calc.addRow("Reference pitch", self.ref_pitch)
        calc.addRow("Reference blades", self.ref_blades)
        calc.addRow(btn_calc)
        calc.addRow("Manual scientific result", self.calc_result)
        calc.addRow("Live voltage result", self.live_result)
        self.fuel_live_basic_widgets = [
            self.cells, self.cell_voltage, self.capacity, self.usable, self.reserve,
            self.warn_cell_v, self.reserve_cell_v, self.critical_cell_v,
        ]
        self.fuel_manual_widgets = [
            self.auw, self.payload_mass, self.rotors, self.cruise_throttle,
            self.avg_current, self.hover_current, self.max_current_motor, self.esc_rating,
            self.max_thrust_motor, self.prop_diam, self.prop_pitch, self.blades,
            self.motor_kv, self.loading_factor, self.figure_merit, self.altitude_calc,
            self.temp_c, self.humidity, self.ref_current, self.ref_kv, self.ref_voltage,
            self.ref_diam, self.ref_pitch, self.ref_blades,
        ]
        self._last_live_telemetry = {}
        self.update_fuel_mode_visibility()
        right.addWidget(calc_box)

        # Status log        # Status log
        status_box = QGroupBox("System Status Log")
        status_layout = QVBoxLayout(status_box)
        self.status_log = QTextEdit(); self.status_log.setReadOnly(True); self.status_log.setMinimumHeight(220)
        status_layout.addWidget(self.status_log)
        right.addWidget(status_box, 1)

    def on_protocol_changed(self):
        if self.protocol_combo.currentText().startswith("MSP"):
            if self.baud_combo.currentText() == "57600":
                self.baud_combo.setCurrentText("115200")
            self.mav_status.setText("MSP mode selected. Use the Betaflight USB COM port. Close Betaflight Configurator first.")
        else:
            self.mav_status.setText("MAVLink mode selected. Use Pixhawk telemetry/USB/UDP.")

    def refresh_mav_ports(self):
        current = self.mav_combo.currentText() if hasattr(self, "mav_combo") else ""
        self.mav_combo.clear()
        self.mav_combo.addItems(discover_mavlink_connection_options())
        if current:
            self.mav_combo.setCurrentText(current)

    def refresh_tx_devices(self):
        self.tx_combo.clear()
        devices = TxJoystickThread.discover_devices(force_rescan=True)
        if not devices:
            self.tx_combo.addItem("No TX joystick/HID detected", None)
        else:
            for d in devices:
                self.tx_combo.addItem(f"{d['index']}: {d['name']} [{d.get('guid','')}]", int(d["index"]))

    def auto_detect_tx(self):
        devices = TxJoystickThread.discover_devices(force_rescan=True)
        idx = TxJoystickThread.best_tx_index(devices)
        if idx is None:
            QMessageBox.information(self, "TX15 Max", "No likely TX15/EdgeTX joystick device detected.")
            return
        name = next((d.get("name", "TX") for d in devices if int(d.get("index", -1)) == idx), "TX")
        if QMessageBox.question(self, "TX15 Max detected", f"Detected: {name}\nConnect this TX USB device?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.tx_connect.emit(idx)

    def connect_selected_tx(self):
        idx = self.tx_combo.currentData()
        if idx is None:
            QMessageBox.warning(self, "No TX", "No joystick/HID device selected.")
            return
        self.tx_connect.emit(idx)

    def browse_model(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select YOLO/custom model", "", "Model files (*.pt *.onnx *.engine);;All files (*)")
        if path:
            self.model_path.setText(path)

    def browse_folder(self):
        path = QFileDialog.getExistingDirectory(self, "Select recording folder", self.folder_line.text())
        if path:
            self.folder_line.setText(path)
            self.folder_changed.emit(path)

    def build_ai_settings(self, ai_enabled: bool = False) -> AISettings:
        return AISettings(
            video_source=self.video_source.text().strip(),
            requested_fps=int(self.fps_spin.value()),
            ai_enabled=bool(ai_enabled),
            model_path=self.model_path.text().strip() or "yolov8n.pt",
            confidence=float(self.conf_spin.value()),
            detect_every_n_frames=int(self.detect_every.value()),
            save_every_n_frames=int(self.save_every.value()),
            capture_backend=self.capture_backend.currentText(),
            fpv_width=int(self.fpv_width.value()),
            fpv_height=int(self.fpv_height.value()),
            fourcc=self.fourcc_combo.currentText(),
            buffer_size=int(self.buffer_spin.value()),
            display_scale=float(self.display_scale_spin.value()),
            ai_device=self.ai_device_combo.currentText(),
            ai_imgsz=int(self.ai_imgsz.value()),
            ai_half=bool(self.ai_half.isChecked()),
            ai_max_det=int(self.ai_max_det.value()),
            ai_input_width=int(self.ai_input_width.value()),
        )

    def start_fpv_only(self):
        self.fpv_start.emit(self.build_ai_settings(ai_enabled=bool(self.ai_enabled.isChecked())))

    def start_fpv(self):
        self.start_fpv_only()

    def set_cv_workflow_mode(self, mode: str):
        """Switch Technical tab between live CV controls and post-processing workflow."""
        self.cv_workflow_mode = str(mode or "Live Computer Vision")
        post = self.cv_workflow_mode.lower().startswith("post")
        try:
            self.btn_overlay_start.setVisible(not post)
            self.btn_overlay_stop.setVisible(not post)
            self.btn_cv_start.setVisible(not post)
            self.btn_cv_stop.setVisible(not post)
            self.ai_enabled.setVisible(not post)
            self.btn_post_process.setVisible(post)
        except Exception:
            pass
        try:
            if post:
                self.findChild(QGroupBox, "")
        except Exception:
            pass

    def _set_fuel_row_visible(self, widget: QWidget, visible: bool):
        try:
            label = self.fuel_form.labelForField(widget)
            if label is not None:
                label.setVisible(visible)
        except Exception:
            pass
        try:
            widget.setVisible(visible)
        except Exception:
            pass

    def update_fuel_mode_visibility(self, *_):
        live_mode = self.fuel_mode.currentText().startswith("Live voltage")
        for w in getattr(self, "fuel_manual_widgets", []):
            self._set_fuel_row_visible(w, not live_mode)
        # In live-voltage mode, keep only the battery configuration, voltage thresholds,
        # live result and Arm Ready button. Manual scientific result returns in manual mode.
        self._set_fuel_row_visible(self.calc_result, not live_mode)
        self._set_fuel_row_visible(self.live_result, True)
        if live_mode:
            self.live_result.setText("Live voltage mode: using MAVLink battery voltage/current plus your manually entered cells, capacity, usable %, reserve %, and voltage thresholds.")
        else:
            self.live_result.setText("Live voltage estimator still available when MAVLink battery telemetry is connected.")

    def scientific_settings(self) -> ScientificFuelSettings:
        return ScientificFuelSettings(
            altitude_m=float(self.altitude_calc.value()),
            temp_c=float(self.temp_c.value()),
            humidity_percent=float(self.humidity.value()),
            cell_count=int(self.cells.value()),
            cell_voltage=float(self.cell_voltage.value()),
            capacity_mah=float(self.capacity.value()),
            usable_percent=float(self.usable.value()),
            reserve_percent=float(self.reserve.value()),
            rotor_count=int(self.rotors.value()),
            drone_mass_kg=float(self.auw.value()),
            payload_mass_kg=float(self.payload_mass.value()),
            motor_kv=float(self.motor_kv.value()),
            prop_diameter_in=float(self.prop_diam.value()),
            prop_pitch_in=float(self.prop_pitch.value()),
            blade_count=int(self.blades.value()),
            loading_factor=float(self.loading_factor.value()),
            figure_of_merit=float(self.figure_merit.value()),
            measured_max_thrust_g=float(self.max_thrust_motor.value()),
            max_current_per_motor_a=float(self.max_current_motor.value()),
            esc_rating_a=float(self.esc_rating.value()),
            cruise_throttle_percent=float(self.cruise_throttle.value()),
            ref_current_a=float(self.ref_current.value()),
            ref_kv=float(self.ref_kv.value()),
            ref_voltage=float(self.ref_voltage.value()),
            ref_diameter_in=float(self.ref_diam.value()),
            ref_pitch_in=float(self.ref_pitch.value()),
            ref_blades=int(self.ref_blades.value()),
        )

    def live_fuel_state(self, telemetry: Dict[str, Any]) -> Dict[str, Any]:
        cells = max(1, int(self.cells.value()))
        voltage = telemetry.get("battery_voltage_v")
        current = telemetry.get("battery_current_a")
        capacity_ah = self.capacity.value() / 1000.0
        nominal_wh = cells * self.cell_voltage.value() * capacity_ah
        usable_wh = nominal_wh * self.usable.value() / 100.0
        reserve_wh = nominal_wh * self.reserve.value() / 100.0
        if voltage is None:
            return {"ok": False, "text": "Live voltage estimator waiting for MAVLink battery voltage."}
        v = safe_float(voltage)
        i = safe_float(current, 0.0)
        v_cell = v / cells
        soc = lipo_soc_from_cell_voltage(v_cell)
        remaining_wh = nominal_wh * soc / 100.0
        usable_remaining_wh = max(0.0, remaining_wh - reserve_wh)
        power_w = v * i if i > 0.05 else 0.0
        min_to_reserve = usable_remaining_wh / power_w * 60.0 if power_w > 1.0 else None
        warn_total = self.warn_cell_v.value() * cells
        reserve_total = self.reserve_cell_v.value() * cells
        crit_total = self.critical_cell_v.value() * cells
        if v_cell <= self.critical_cell_v.value():
            level = "CRITICAL LAND NOW"
        elif v_cell <= self.reserve_cell_v.value():
            level = "RESERVE LAND SOON"
        elif v_cell <= self.warn_cell_v.value():
            level = "LOW BATTERY WARNING"
        else:
            level = "OK"
        text = (
            f"{level}\n"
            f"Live pack: {v:.2f}V total / {v_cell:.2f}V per cell ({cells}S) | Current {i:.1f}A | Power {power_w:.0f}W\n"
            f"Voltage thresholds: warn {warn_total:.1f}V / reserve {reserve_total:.1f}V / critical {crit_total:.1f}V\n"
            f"Estimated SoC from voltage: {soc:.0f}% | Remaining energy ~{remaining_wh:.1f}Wh | Usable-to-reserve ~{usable_remaining_wh:.1f}Wh"
        )
        if min_to_reserve is not None:
            text += f"\nEstimated time to reserve at current draw: ~{min_to_reserve:.1f} min"
        else:
            text += "\nEstimated time to reserve: waiting for current sensor / motors running."
        return {"ok": True, "level": level, "voltage": v, "cell_voltage": v_cell, "soc": soc, "text": text}

    def update_live_fuel_display(self, telemetry: Dict[str, Any]) -> Dict[str, Any]:
        self._last_live_telemetry = dict(telemetry or {})
        state = self.live_fuel_state(telemetry)
        self.live_result.setText(state.get("text", "-"))
        return state

    def calculate_battery(self):
        if self.fuel_mode.currentText().startswith("Live voltage"):
            state = self.update_live_fuel_display(getattr(self, "_last_live_telemetry", {}))
            self.fuel_check_done = True
            self.last_fuel_check_time = time.time()
            self.calc_result.setText(
                f"LIVE VOLTAGE FUEL CHECK / ARM READY updated at {time.strftime('%H:%M:%S')}\n"
                + state.get("text", "Live estimator waiting for MAVLink battery voltage.")
            )
            try:
                self.fuel_checked.emit()
            except Exception:
                pass
            return
        settings = self.scientific_settings()
        r = rich_fuel_estimate(settings)
        cells = settings.cell_count
        nominal_v = settings.battery_voltage_v
        full_v = cells * 4.2
        capacity_ah = settings.capacity_mah / 1000.0
        wh = nominal_v * capacity_ah
        usable_wh = wh * settings.usable_percent / 100.0
        reserve_wh = wh * settings.reserve_percent / 100.0
        measured_cruise_power = nominal_v * self.avg_current.value()
        measured_hover_power = nominal_v * self.hover_current.value()
        measured_cruise_min = usable_wh / max(1.0, measured_cruise_power) * 60.0
        measured_hover_min = usable_wh / max(1.0, measured_hover_power) * 60.0
        measured_cruise_to_res = max(0.0, (usable_wh - reserve_wh) / max(1.0, measured_cruise_power) * 60.0)
        measured_hover_to_res = max(0.0, (usable_wh - reserve_wh) / max(1.0, measured_hover_power) * 60.0)
        warn_total = self.warn_cell_v.value() * cells
        reserve_total = self.reserve_cell_v.value() * cells
        crit_total = self.critical_cell_v.value() * cells
        warnings = ", ".join(r["warnings"]) if r["warnings"] else "none"
        self.calc_result.setText(
            f"FUEL CHECK PASSED/UPDATED at {time.strftime('%H:%M:%S')}\n"
            f"Battery: {cells}S | {wh:.1f}Wh nominal | {usable_wh:.1f}Wh usable | reserve {reserve_wh:.1f}Wh\n"
            f"Thresholds: warn {warn_total:.1f}V / reserve {reserve_total:.1f}V / critical {crit_total:.1f}V\n"
            f"Scientific hover estimate: {r['total_hover_current_a']:.1f}A, {r['total_hover_power_w']:.0f}W, {r['flight_min_hover']:.1f}min usable, {r['to_reserve_hover']:.1f}min to reserve\n"
            f"Scientific cruise estimate: {r['cruise_current_a']:.1f}A at {settings.cruise_throttle_percent:.0f}% throttle, {r['flight_min_cruise']:.1f}min usable, {r['to_reserve_cruise']:.1f}min to reserve\n"
            f"Measured-current backup: cruise {measured_cruise_min:.1f}min usable / {measured_cruise_to_res:.1f}min to reserve at {self.avg_current.value():.1f}A; hover {measured_hover_min:.1f}min usable / {measured_hover_to_res:.1f}min to reserve at {self.hover_current.value():.1f}A\n"
            f"Thrust: hover need {r['hover_thrust_per_motor_g']:.0f}g/motor | margin {r['thrust_margin_percent']:.0f}% | hover throttle equivalent {r['hover_throttle_equivalent']*100:.0f}%\n"
            f"Static current model: {r['static_current_motor']:.1f}A/motor, {r['static_total_current']:.0f}A total, {r['static_total_power']:.0f}W total\n"
            f"Air density {r['rho']:.3f}kg/m³ at {settings.altitude_m:.0f}m, {settings.temp_c:.0f}°C, {settings.humidity_percent:.0f}%RH\n"
            f"RPM: no-load {r['no_load_rpm']:.0f}, loaded-est {r['loaded_rpm']:.0f} | Prop {settings.prop_diameter_in:.1f}x{settings.prop_pitch_in:.1f}x{settings.blade_count}\n"
            f"Warnings: {warnings}"
        )
        self.fuel_check_done = True
        self.last_fuel_check_time = time.time()
        self.update_live_fuel_display({})
        try:
            self.fuel_checked.emit()
        except Exception:
            pass

    def append_status(self, text: str):
        self.status_log.append(f"[{time.strftime('%H:%M:%S')}] {text}")


# ----------------------------- Main App -----------------------------

class DroneDashboardV23(QWidget):
    def __init__(self, theme: str = "dark", launch_fullscreen: bool = False, scale_mode: str = "Auto", cv_mode: str = "Live Computer Vision"):
        super().__init__()
        self.theme = theme.lower().strip() or "dark"
        self.cv_mode = str(cv_mode or "Live Computer Vision")
        self.post_process_mode = self.cv_mode.lower().startswith("post")
        self.launch_fullscreen = bool(launch_fullscreen)
        self.scale_mode = scale_mode
        self.ui_scale = self.compute_ui_scale(scale_mode)
        self.setWindowTitle("Drone Dashboard v100 - Live/Post Computer Vision + Low-Latency Split FPV GCS")
        self.resize(int(1600 * self.ui_scale), int(950 * self.ui_scale))
        self.telemetry_thread: Optional[TelemetryThread] = None
        self.tx_thread: Optional[TxJoystickThread] = None
        self.fpv_thread: Optional[AIFrameThread] = None
        self._last_fpv_frame_id = -1
        self._fpv_display_frames = 0
        self._fpv_display_t0 = time.time()
        self._fpv_display_fps = 0.0
        self._last_ai_preview_time = 0.0
        self.ai_overlay_enabled = False
        self.last_cv_detections: List[Dict[str, Any]] = []
        self.last_raw_fpv_video = ""
        self.post_process_thread: Optional[VideoPostProcessThread] = None
        self.last_telemetry = TelemetryThread.default_data()
        self.last_tx = {"connected": False, "rc_channels": [0]*16}
        self.last_detections: List[Dict[str, Any]] = []
        self.recorder = DatasetRecorder()
        self.record_base = str(Path.home() / "drone_v100_records")
        self.record_fps = 30.0
        self.last_record_log = 0.0
        self.flight_timer_running = False
        self.flight_timer_start = 0.0
        self.fuel_check_done = False
        self.last_fuel_check_time = None
        self._last_armed_state = False
        self._fuel_warning_shown_this_arm = False
        self.build_ui()
        self.configure_cv_workflow_ui()
        self.apply_auto_display_scaling()
        self.apply_style()
        self.apply_liquid_glass_effects()
        QShortcut(QKeySequence("F11"), self, activated=self.toggle_fullscreen)
        QShortcut(QKeySequence("Esc"), self, activated=self.exit_fullscreen)
        self.ui_timer = QTimer(self); self.ui_timer.timeout.connect(self.periodic_update); self.ui_timer.start(50)
        # v90: UI pulls newest FPV frame at 60Hz instead of accepting every capture signal.
        # This prevents event-queue backlog and keeps tab switching responsive.
        self.fpv_display_timer = QTimer(self)
        self.fpv_display_timer.timeout.connect(self.update_fpv_display_from_thread)
        self.fpv_display_timer.setInterval(16)

        # v90: TX15/EdgeTX hot-plug watcher. The radio can now be plugged in after
        # the dashboard is already running; the app rescans HID devices and asks
        # before connecting.
        self._known_tx_device_signature = set()
        self._tx_hotplug_prompt_active = False
        self.tx_hotplug_timer = QTimer(self)
        self.tx_hotplug_timer.timeout.connect(self.watch_tx_hotplug)
        self.tx_hotplug_timer.start(1500)
        self.auto_detect_tx_at_start()

    def compute_ui_scale(self, scale_mode: str = "Auto") -> float:
        if str(scale_mode).lower().startswith("compact"):
            return 0.86
        if str(scale_mode).lower().startswith("large"):
            return 1.08
        screen = QApplication.primaryScreen()
        if screen is None:
            return 1.0
        geo = screen.availableGeometry()
        w, h = geo.width(), geo.height()
        # Preserve density on small laptop screens while keeping 4K readable.
        if w <= 1366 or h <= 768:
            return 0.78
        if w <= 1600 or h <= 900:
            return 0.86
        if w <= 1920 or h <= 1080:
            return 0.94
        if w >= 3000:
            return 1.06
        return 1.0

    def apply_auto_display_scaling(self):
        s = self.ui_scale
        base_font = max(9, int(12 * s))
        self.setFont(QFont("Arial", base_font))
        try:
            self.fpv_widget.video.setMinimumSize(QSize(int(560 * s), int(315 * s)))
            self.ahrs_widget.setMinimumSize(QSize(int(330 * s), int(225 * s)))
            self.ai_video_big.setMinimumHeight(int(480 * s))
        except Exception:
            pass

    def build_ui(self):
        """v23 polished compact GCS layout.

        Main tab is a real GCS cockpit: FPV/AI overlay, 3D AHRS, telemetry,
        integrated map waypoint planning, timer/fuel, and TX/RX channels.
        Setup tab keeps the heavy battery calculator + MAVLink/TX/FPV source controls.
        AI tab only controls background computer vision and dataset recording.
        """
        root = QVBoxLayout(self); root.setContentsMargins(8, 8, 8, 8); root.setSpacing(6)

        top = QHBoxLayout()
        title = QLabel("DRONE DASHBOARD v90")
        title.setFont(QFont("Arial", 20, QFont.Bold))
        title.setObjectName("appTitle")
        self.connection_status = QLabel("MAVLink disconnected | TX disconnected | FPV stopped | CV OFF")
        self.connection_status.setWordWrap(True)
        self.connection_status.setObjectName("topStatus")
        self.rec_status = QLabel("REC: OFF")
        self.rec_status.setObjectName("recordOff")
        self.btn_full = QPushButton("Fullscreen")
        self.btn_full.setToolTip("Toggle fullscreen. Press Esc or F11 to exit fullscreen.")
        self.btn_full.clicked.connect(self.toggle_fullscreen)
        btn_close = QPushButton("✕"); btn_close.clicked.connect(self.close); btn_close.setFixedWidth(42)
        top.addWidget(title); top.addWidget(self.connection_status, 1); top.addWidget(self.rec_status); top.addWidget(self.btn_full); top.addWidget(btn_close)
        root.addLayout(top)

        self.tabs = QTabWidget(); self.tabs.setDocumentMode(True); root.addWidget(self.tabs, 1)
        self.main_tab = QWidget(); self.ai_tab = QWidget(); self.tech_tab = QWidget()
        self.tabs.addTab(self.main_tab, "Main Flight")
        self.tabs.addTab(self.ai_tab, "AI / Dataset")
        self.tabs.addTab(self.tech_tab, "Technical / Setup")

        # ---------------- Main Flight tab ----------------
        main_layout = QVBoxLayout(self.main_tab); main_layout.setContentsMargins(2, 2, 2, 2); main_layout.setSpacing(6)
        main_split = QSplitter(Qt.Horizontal); main_layout.addWidget(main_split, 1)

        left = QWidget(); left_l = QVBoxLayout(left); left_l.setContentsMargins(0, 0, 4, 0); left_l.setSpacing(6)
        self.fpv_widget = FPVAIWidget(); left_l.addWidget(self.fpv_widget, 8)

        lower = QSplitter(Qt.Horizontal)
        gyro_box = QGroupBox("3D AHRS / Gyro Ball")
        gyro_l = QVBoxLayout(gyro_box); gyro_l.setContentsMargins(6, 12, 6, 6)
        self.ahrs_widget = CompactAHRSWidget(); gyro_l.addWidget(self.ahrs_widget)
        lower.addWidget(gyro_box)
        self.telemetry_cards = TelemetryCards(); lower.addWidget(self.telemetry_cards)
        self.esc_fence_stack = EscFenceStack(); lower.addWidget(self.esc_fence_stack)
        # v60: give more useful width to the gyroball, keep telemetry readable,
        # and slightly shrink the stacked ESC/Fence column.
        lower.setSizes([470, 555, 255])
        lower.setStretchFactor(0, 5)
        lower.setStretchFactor(1, 6)
        lower.setStretchFactor(2, 3)
        left_l.addWidget(lower, 3)
        main_split.addWidget(left)

        right = QWidget(); right_l = QVBoxLayout(right); right_l.setContentsMargins(4, 0, 0, 0); right_l.setSpacing(6)
        self.main_map = CompactMissionPlannerWidget(); right_l.addWidget(self.main_map, 6)
        self.mission_widget = self.main_map
        self.main_map.waypoint_command.connect(self.send_guided_waypoint)

        timer_box = QGroupBox("Flight Timer / Fuel / Quick Actions")
        timer_l = QVBoxLayout(timer_box); timer_l.setContentsMargins(8, 12, 8, 8); timer_l.setSpacing(5)
        self.timer_label = QLabel("00:00:00")
        self.timer_label.setFont(QFont("Arial", 28, QFont.Bold)); self.timer_label.setAlignment(Qt.AlignCenter)
        self.timer_label.setObjectName("timerValue")
        timer_l.addWidget(self.timer_label)
        btns = QHBoxLayout()
        btn_start_timer = QPushButton("Start") ; btn_start_timer.clicked.connect(self.start_timer)
        btn_stop_timer = QPushButton("Stop") ; btn_stop_timer.clicked.connect(self.stop_timer)
        btn_reset_timer = QPushButton("Reset") ; btn_reset_timer.clicked.connect(self.reset_timer)
        btn_ai_tab = QPushButton("AI") ; btn_ai_tab.clicked.connect(lambda: self.tabs.setCurrentWidget(self.ai_tab))
        btn_tech_tab = QPushButton("Setup") ; btn_tech_tab.clicked.connect(lambda: self.tabs.setCurrentWidget(self.tech_tab))
        for b in (btn_start_timer, btn_stop_timer, btn_reset_timer, btn_ai_tab, btn_tech_tab): btns.addWidget(b)
        timer_l.addLayout(btns)
        self.fuel_summary = QLabel("Fuel: waiting for battery/current telemetry or calculator input")
        self.fuel_summary.setWordWrap(True); self.fuel_summary.setObjectName("dynamicValue")
        timer_l.addWidget(self.fuel_summary)
        right_l.addWidget(timer_box, 2)

        self.main_channels_preview = DualRCChannelsWidget(channel_count=14)
        self.dual_channels = self.main_channels_preview
        right_l.addWidget(self.main_channels_preview, 4)
        main_split.addWidget(right)
        main_split.setSizes([1120, 500])

        # ---------------- AI / Dataset tab ----------------
        ai_layout = QVBoxLayout(self.ai_tab); ai_layout.setContentsMargins(6,6,6,6); ai_layout.setSpacing(6)
        ai_controls = QGroupBox("Background Computer Vision + Dataset Recording")
        ai_ctrl_l = QGridLayout(ai_controls); ai_ctrl_l.setContentsMargins(8, 12, 8, 8)
        self.btn_start_cv = QPushButton("Start Computer Vision")
        self.btn_stop_cv = QPushButton("Stop Computer Vision")
        self.btn_start_overlay = QPushButton("Start AI Overlay")
        self.btn_stop_overlay = QPushButton("Stop AI Overlay")
        self.btn_post_process = QPushButton("Process Recorded Video")
        self.btn_start_dataset = QPushButton("Start Recording Data")
        self.btn_stop_dataset = QPushButton("Stop Recording")
        self.btn_start_cv.clicked.connect(self.start_computer_vision)
        self.btn_stop_cv.clicked.connect(self.stop_computer_vision)
        self.btn_start_overlay.clicked.connect(self.start_ai_overlay)
        self.btn_stop_overlay.clicked.connect(self.stop_ai_overlay)
        self.btn_post_process.clicked.connect(self.process_recorded_video)
        self.btn_start_dataset.clicked.connect(self.start_recording)
        self.btn_stop_dataset.clicked.connect(self.stop_recording)
        self.ai_status = QLabel("FPV/CV stopped. Start FPV in Technical tab. Live CV and AI overlay are separate; post mode processes recorded video later.")
        self.ai_status.setWordWrap(True); self.ai_status.setObjectName("dynamicValue")
        ai_ctrl_l.addWidget(self.btn_start_cv, 0, 0)
        ai_ctrl_l.addWidget(self.btn_stop_cv, 0, 1)
        ai_ctrl_l.addWidget(self.btn_start_overlay, 0, 2)
        ai_ctrl_l.addWidget(self.btn_stop_overlay, 0, 3)
        ai_ctrl_l.addWidget(self.btn_post_process, 1, 0, 1, 2)
        ai_ctrl_l.addWidget(self.btn_start_dataset, 1, 2)
        ai_ctrl_l.addWidget(self.btn_stop_dataset, 1, 3)
        ai_ctrl_l.addWidget(self.ai_status, 2, 0, 1, 4)
        ai_note = QLabel("v100: Computer Vision can run live or post-process recorded video. Live CV only paints boxes on the Main FPV when AI Overlay is enabled. Recording remains separate from smooth FPV.")
        ai_note.setWordWrap(True); ai_note.setObjectName("softText")
        ai_ctrl_l.addWidget(ai_note, 3, 0, 1, 4)
        ai_layout.addWidget(ai_controls)
        ai_split = QSplitter(Qt.Horizontal)
        self.ai_video_big = QLabel("AI/Post-processing preview")
        self.ai_video_big.setAlignment(Qt.AlignCenter); self.ai_video_big.setMinimumHeight(520)
        self.ai_video_big.setStyleSheet("background:#03060b;color:#dbeafe;border-radius:14px;border:1px solid #334155;")
        ai_split.addWidget(self.ai_video_big)
        log_box = QGroupBox("Detection Log")
        log_l = QVBoxLayout(log_box)
        self.detection_log = QTextEdit(); self.detection_log.setReadOnly(True)
        log_l.addWidget(self.detection_log)
        ai_split.addWidget(log_box)
        ai_split.setSizes([1050, 420])
        ai_layout.addWidget(ai_split, 1)

        # ---------------- Technical tab ----------------
        tech_layout = QVBoxLayout(self.tech_tab); tech_layout.setContentsMargins(2,2,2,2)
        self.tech = TechnicalSetupWidget()
        tech_layout.addWidget(self.tech)
        self.tech.mav_connect.connect(self.connect_mavlink)
        self.tech.mav_disconnect.connect(self.disconnect_mavlink)
        self.tech.tx_connect.connect(self.connect_tx)
        self.tech.tx_disconnect.connect(self.disconnect_tx)
        self.tech.fpv_start.connect(self.start_fpv_ai)
        self.tech.fpv_stop.connect(self.stop_fpv_ai)
        self.tech.fpv_record_start.connect(self.start_raw_fpv_recording)
        self.tech.fpv_record_stop.connect(self.stop_raw_fpv_recording)
        self.tech.ai_start.connect(self.start_computer_vision)
        self.tech.ai_stop.connect(self.stop_computer_vision)
        self.tech.ai_overlay_start.connect(self.start_ai_overlay)
        self.tech.ai_overlay_stop.connect(self.stop_ai_overlay)
        self.tech.post_process_requested.connect(self.process_recorded_video)
        self.tech.set_cv_workflow_mode(self.cv_mode)
        self.tech.folder_changed.connect(self.set_record_folder)
        self.tech.fuel_checked.connect(self.mark_fuel_checked)

    def configure_cv_workflow_ui(self):
        """Apply Live-CV vs Post-processing workflow selected at boot."""
        post = bool(getattr(self, "post_process_mode", False))
        try:
            self.tech.set_cv_workflow_mode(self.cv_mode)
        except Exception:
            pass
        for w in (getattr(self, "btn_start_overlay", None), getattr(self, "btn_stop_overlay", None)):
            if w is not None:
                w.setVisible(not post)
        for w in (getattr(self, "btn_start_cv", None), getattr(self, "btn_stop_cv", None)):
            if w is not None:
                w.setVisible(not post)
        if hasattr(self, "btn_post_process"):
            self.btn_post_process.setVisible(post)
        try:
            if post:
                self.ai_status.setText("Post-processing CV mode: live FPV stays smooth. Record raw FPV, stop recording, then process the video here.")
            else:
                self.ai_status.setText("Live CV mode: Start Computer Vision begins detection; Start AI Overlay controls boxes on the Main FPV.")
        except Exception:
            pass

    def apply_style(self):
        fs = max(9, int(12 * self.ui_scale))
        if self.theme == "light":
            self.setStyleSheet(f"""
            QWidget {{ background:#eef3fb; color:#0f172a; font-family:Arial; font-size:{fs}px; }}
            QLabel#appTitle {{ color:#0f172a; letter-spacing:1px; }}
            QLabel#topStatus {{ color:#1e3a8a; padding:4px 8px; border:1px solid #b7c4d8; border-radius:10px; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #ffffff, stop:1 #dbeafe); }}
            QLabel#recordOff {{ font-weight:bold; color:#991b1b; padding:4px 10px; border-radius:10px; background:#fff1f2; border:1px solid #fecaca; }}
            QGroupBox {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,235), stop:0.40 rgba(239,246,255,220), stop:1 rgba(214,226,242,210)); border:1px solid rgba(120,145,178,190); border-top:1px solid rgba(255,255,255,250); border-left:1px solid rgba(255,255,255,210); border-radius:16px; margin-top:11px; padding:8px; font-weight:bold; }}
            QGroupBox::title {{ subcontrol-origin: margin; left:12px; padding:0 6px; color:#0f172a; background:#eef3fb; border-radius:6px; }}
            QLabel#softText {{ color:#475569; }}
            QPushButton {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #60a5fa, stop:1 #2563eb); color:white; border:1px solid #1d4ed8; border-radius:9px; padding:7px 10px; font-weight:bold; }}
            QPushButton:hover {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #93c5fd, stop:1 #3b82f6); }}
            QPushButton:pressed {{ background:#1d4ed8; padding-top:8px; padding-bottom:6px; }}
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit {{ background:#ffffff; color:#0f172a; border:1px solid #94a3b8; border-radius:8px; padding:5px; selection-background-color:#60a5fa; }}
            QComboBox QAbstractItemView {{ background:#ffffff; color:#0f172a; selection-background-color:#bfdbfe; selection-color:#0f172a; border:1px solid #94a3b8; outline:0px; }}
            QComboBox::drop-down {{ background:#dbeafe; border-left:1px solid #94a3b8; width:24px; border-top-right-radius:8px; border-bottom-right-radius:8px; }}
            QComboBox::down-arrow {{ image:none; border-left:5px solid transparent; border-right:5px solid transparent; border-top:7px solid #475569; width:0px; height:0px; margin-right:6px; }}
            QSpinBox::up-button, QDoubleSpinBox::up-button {{ background:#dbeafe; border-left:1px solid #94a3b8; border-bottom:1px solid #94a3b8; width:24px; border-top-right-radius:7px; }}
            QSpinBox::down-button, QDoubleSpinBox::down-button {{ background:#dbeafe; border-left:1px solid #94a3b8; width:24px; border-bottom-right-radius:7px; }}
            QTabWidget::pane {{ border:1px solid #94a3b8; border-radius:12px; top:-1px; background:#eef3fb; }}
            QTabBar::tab {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #ffffff, stop:1 #dbeafe); color:#334155; padding:9px 16px; border:1px solid #b7c4d8; border-top-left-radius:10px; border-top-right-radius:10px; margin-right:2px; }}
            QTabBar::tab:selected {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #60a5fa, stop:1 #2563eb); color:white; border-color:#1d4ed8; }}
            QSplitter::handle {{ background:#cbd5e1; border-radius:3px; }}
            QProgressBar {{ background:#e2e8f0; border:1px solid #94a3b8; border-radius:5px; color:#0f172a; text-align:center; height:12px; }}
            QProgressBar::chunk {{ background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #22c55e, stop:1 #86efac); border-radius:4px; }}
            QLabel#telemetryTitle {{ font-weight:normal; padding:2px; }}
            QLabel#dynamicValue {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,240), stop:1 rgba(219,234,254,220)); border:1px solid rgba(148,163,184,190); border-top:1px solid rgba(255,255,255,245); border-radius:9px; padding:3px; color:#0f172a; }}
            QGroupBox:disabled {{ color:#64748b; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(230,235,243,150), stop:1 rgba(200,210,225,120)); border:1px solid rgba(148,163,184,120); }}
            QLabel#timerValue {{ color:#0f172a; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #ffffff, stop:1 #dbeafe); border:1px solid #94a3b8; border-radius:12px; padding:4px; }}
            """)
        else:
            self.setStyleSheet(f"""
            QWidget {{ background:#070b14; color:#e7eefb; font-family:Arial; font-size:{fs}px; }}
            QLabel#appTitle {{ color:#f8fafc; letter-spacing:1px; }}
            QLabel#topStatus {{ color:#c7d2fe; padding:4px 8px; border:1px solid #24324a; border-radius:10px; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111c31, stop:1 #07111f); }}
            QLabel#recordOff {{ font-weight:bold; color:#fecaca; padding:4px 10px; border-radius:10px; background:#1f1116; border:1px solid #7f1d1d; }}
            QGroupBox {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(32,48,78,220), stop:0.45 rgba(13,24,43,215), stop:1 rgba(5,10,18,230)); border:1px solid rgba(74,100,140,185); border-top:1px solid rgba(148,170,205,170); border-left:1px solid rgba(120,145,180,120); border-radius:16px; margin-top:11px; padding:8px; font-weight:bold; }}
            QGroupBox::title {{ subcontrol-origin: margin; left:12px; padding:0 6px; color:#eff6ff; background:#070b14; border-radius:6px; }}
            QFrame#glassFrame {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(30,41,59,190), stop:1 rgba(15,23,42,210)); border:1px solid #334155; border-radius:12px; }}
            QLabel#softText {{ color:#bdc9dd; }}
            QPushButton {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #3b82f6, stop:1 #1d4ed8); color:white; border:1px solid #60a5fa; border-radius:9px; padding:7px 10px; font-weight:bold; }}
            QPushButton:hover {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #60a5fa, stop:1 #2563eb); }}
            QPushButton:pressed {{ background:#1e40af; padding-top:8px; padding-bottom:6px; }}
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit {{ background:#0f172a; color:#e5edf8; border:1px solid #475569; border-radius:8px; padding:5px; selection-background-color:#2563eb; }}
            QComboBox QAbstractItemView {{ background:#0f172a; color:#e5edf8; selection-background-color:#1d4ed8; selection-color:white; border:1px solid #475569; outline:0px; }}
            QComboBox::drop-down {{ background:#111827; border-left:1px solid #475569; width:24px; border-top-right-radius:8px; border-bottom-right-radius:8px; }}
            QSpinBox::up-button, QDoubleSpinBox::up-button {{ background:#111827; border-left:1px solid #475569; border-bottom:1px solid #475569; width:24px; border-top-right-radius:7px; }}
            QSpinBox::down-button, QDoubleSpinBox::down-button {{ background:#111827; border-left:1px solid #475569; width:24px; border-bottom-right-radius:7px; }}
            QTabWidget::pane {{ border:1px solid #334155; border-radius:12px; top:-1px; background:#070b14; }}
            QTabBar::tab {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #172033, stop:1 #0e1627); color:#cbd5e1; padding:9px 16px; border:1px solid #24324a; border-top-left-radius:10px; border-top-right-radius:10px; margin-right:2px; }}
            QTabBar::tab:selected {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #3b82f6, stop:1 #1d4ed8); color:white; border-color:#60a5fa; }}
            QSplitter::handle {{ background:#1e293b; border-radius:3px; }}
            QProgressBar {{ background:#0b1220; border:1px solid #334155; border-radius:5px; color:#f8fafc; text-align:center; height:12px; }}
            QProgressBar::chunk {{ background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #22c55e, stop:1 #86efac); border-radius:4px; }}
            QLabel#telemetryTitle {{ font-weight:normal; padding:2px; }}
            QLabel#dynamicValue {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(30,45,70,220), stop:1 rgba(7,14,26,230)); border:1px solid rgba(86,112,150,190); border-top:1px solid rgba(130,155,190,150); border-radius:9px; padding:3px; color:#f8fafc; }}
            QGroupBox:disabled {{ color:#94a3b8; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(35,43,55,95), stop:1 rgba(10,15,24,120)); border:1px solid rgba(80,90,110,90); }}
            QLabel#timerValue {{ color:#e0f2fe; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111827, stop:1 #020617); border:1px solid #334155; border-radius:12px; padding:4px; }}
            """)

    def _tx_signature_set(self, devices: List[Dict[str, Any]]) -> set:
        return {
            (str(d.get("guid", "")), str(d.get("name", "")), int(d.get("axes", 0)), int(d.get("buttons", 0)), int(d.get("hats", 0)))
            for d in devices
        }

    def watch_tx_hotplug(self):
        """Background HID watcher for TX15/EdgeTX hot-plug after program launch."""
        if self.tx_thread is not None:
            return
        devices = TxJoystickThread.discover_devices(force_rescan=True)
        sig = self._tx_signature_set(devices)
        prev = getattr(self, "_known_tx_device_signature", set())
        if sig != prev:
            self._known_tx_device_signature = sig
            try:
                self.tech.refresh_tx_devices()
            except Exception:
                pass
        new_devices = sig - prev
        if not devices or not new_devices or getattr(self, "_tx_hotplug_prompt_active", False):
            return
        idx = TxJoystickThread.best_tx_index(devices)
        if idx is None:
            return
        name = next((d.get("name", "TX") for d in devices if int(d.get("index", -1)) == idx), "TX")
        self._tx_hotplug_prompt_active = True
        try:
            reply = QMessageBox.question(
                self,
                "TX15 Max detected",
                "Detected USB joystick/radio after startup:\n" + str(name) + "\n\nConnect it for TX-side live channel display?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply == QMessageBox.Yes:
                self.connect_tx(idx)
        finally:
            self._tx_hotplug_prompt_active = False

    def apply_liquid_glass_effects(self):
        """Add subtle acrylic/liquid-glass depth without changing the working layout."""
        try:
            shadow_color = QColor(0, 0, 0, 90) if self.theme != "light" else QColor(80, 105, 140, 55)
            for box in self.findChildren(QGroupBox):
                eff = QGraphicsDropShadowEffect(box)
                eff.setBlurRadius(max(16, int(26 * self.ui_scale)))
                eff.setOffset(0, max(2, int(4 * self.ui_scale)))
                eff.setColor(shadow_color)
                box.setGraphicsEffect(eff)
        except Exception:
            pass

    def auto_detect_tx_at_start(self):
        QTimer.singleShot(900, self._auto_detect_tx_prompt)

    def _auto_detect_tx_prompt(self):
        devices = TxJoystickThread.discover_devices(force_rescan=True)
        self._known_tx_device_signature = self._tx_signature_set(devices)
        try:
            self.tech.refresh_tx_devices()
        except Exception:
            pass
        idx = TxJoystickThread.best_tx_index(devices)
        if idx is None:
            return
        name = next((d.get("name", "TX") for d in devices if int(d.get("index", -1)) == idx), "TX")
        if QMessageBox.question(self, "TX15 Max detected", "Detected USB joystick/radio: " + str(name) + "\nConnect it for TX-side channel display?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.connect_tx(idx)

    def exit_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            try:
                self.btn_full.setText("Fullscreen")
            except Exception:
                pass

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.btn_full.setText("Fullscreen")
        else:
            self.showFullScreen()
            self.btn_full.setText("Exit Fullscreen")

    def mark_fuel_checked(self):
        self.fuel_check_done = True
        self.last_fuel_check_time = time.time()
        self.append_status("Fuel Check / Arm Ready completed.")

    def append_status(self, text: str):
        self.tech.append_status(text)
        self.connection_status.setText(text)

    def connect_mavlink(self, conn: str, baud: int):
        self.disconnect_mavlink()
        protocol = "MAVLink"
        try:
            protocol = self.tech.protocol_combo.currentText()
        except Exception:
            pass
        if str(protocol).startswith("MSP"):
            self.telemetry_thread = MspTelemetryThread(conn, baud)
            self.append_status("Connecting Betaflight/iNav MSP USB data stream...")
        else:
            self.telemetry_thread = TelemetryThread(conn, baud)
            self.append_status("Connecting MAVLink telemetry stream...")
        self.telemetry_thread.telemetry.connect(self.on_telemetry)
        # v30 crash fix: MSP and MAVLink both now expose attitude, but keep this guarded
        # so a future telemetry backend cannot crash the UI during connection.
        if hasattr(self.telemetry_thread, "attitude"):
            self.telemetry_thread.attitude.connect(self.ahrs_widget.update_attitude)
        self.telemetry_thread.status.connect(self.on_mav_status)
        self.telemetry_thread.start()

    def disconnect_mavlink(self):
        if self.telemetry_thread is not None:
            self.telemetry_thread.stop()
            self.telemetry_thread = None

    def on_mav_status(self, text: str):
        self.tech.mav_status.setText(text)
        self.append_status(text)

    def connect_tx(self, idx):
        self.disconnect_tx()
        self.tx_thread = TxJoystickThread(joystick_index=int(idx) if idx is not None else None)
        self.tx_thread.tx_data.connect(self.on_tx_data)
        self.tx_thread.status.connect(self.on_tx_status)
        self.tx_thread.start()

    def disconnect_tx(self):
        if self.tx_thread is not None:
            self.tx_thread.stop()
            self.tx_thread = None

    def on_tx_status(self, text: str):
        self.tech.tx_status.setText(text)
        self.append_status(text)

    def start_fpv_ai(self, settings: AISettings):
        # Technical tab starts/stops the actual FPV capture source.
        self.stop_fpv_ai()
        self.record_fps = float(settings.requested_fps)
        self.fpv_thread = AIFrameThread(settings)
        # v90: do not connect frame_ready; the UI pulls the latest frame at 60Hz.
        self.fpv_thread.status.connect(self.on_fpv_status)
        self.fpv_thread.recording_status.connect(self.on_fpv_status)
        self.fpv_thread.fps_measured.connect(lambda fps: self.ai_status.setText(f"FPV capture {fps:.1f} FPS | display {self._fpv_display_fps:.1f} FPS | CV {'ON' if self.fpv_thread and self.fpv_thread.ai_runtime_enabled else 'OFF'}"))
        self.fpv_thread.ai_rate.connect(lambda ai_fps, ms: self.connection_status.setText(self.connection_status.text().split(' | AI ')[0] + f" | AI {ai_fps:.1f}FPS {ms:.0f}ms"))
        self._last_fpv_frame_id = -1
        self.fpv_thread.start()
        self.fpv_display_timer.start()

    def stop_fpv_ai(self):
        if hasattr(self, "fpv_display_timer"):
            self.fpv_display_timer.stop()
        if self.fpv_thread is not None:
            self.fpv_thread.stop()
            self.fpv_thread = None
        if hasattr(self, "ai_status"):
            self.ai_status.setText("FPV/CV stopped")

    def _settings_from_tech(self, ai_enabled: bool) -> AISettings:
        if hasattr(self.tech, "build_ai_settings"):
            return self.tech.build_ai_settings(ai_enabled=bool(ai_enabled))
        return AISettings(
            video_source=self.tech.video_source.text().strip(),
            requested_fps=int(self.tech.fps_spin.value()),
            ai_enabled=bool(ai_enabled),
            model_path=self.tech.model_path.text().strip() or "yolov8n.pt",
            confidence=float(self.tech.conf_spin.value()),
            detect_every_n_frames=int(self.tech.detect_every.value()),
            save_every_n_frames=int(self.tech.save_every.value()),
        )

    def start_computer_vision(self):
        if getattr(self, "post_process_mode", False):
            QMessageBox.information(self, "Post-processing mode", "Live computer vision is disabled in Post-processing mode. Record FPV, stop recording, then press Process Recorded Video.")
            return
        # Start CV processing only. Main overlay is controlled separately by Start AI Overlay.
        settings = self._settings_from_tech(ai_enabled=True)
        if self.fpv_thread is None:
            self.start_fpv_ai(settings)
        else:
            self.fpv_thread.update_ai_settings(settings)
            self.fpv_thread.set_ai_enabled(True)
        self.append_status("Computer vision started in background. Main FPV remains clean until AI Overlay is enabled.")

    def stop_computer_vision(self):
        if self.fpv_thread is not None:
            self.fpv_thread.set_ai_enabled(False)
        self.ai_overlay_enabled = False
        self.last_detections = []
        self.last_cv_detections = []
        self.append_status("Computer vision stopped. FPV capture continues if running.")

    def start_ai_overlay(self):
        if getattr(self, "post_process_mode", False):
            QMessageBox.information(self, "Post-processing mode", "Live AI overlay is disabled in Post-processing mode.")
            return
        if self.fpv_thread is None or not getattr(self.fpv_thread, "ai_runtime_enabled", False):
            self.start_computer_vision()
        self.ai_overlay_enabled = True
        self.append_status("AI overlay enabled on Main FPV. FPV path is still separate from the AI path.")

    def stop_ai_overlay(self):
        # Important bugfix: stopping overlay should NOT stop computer vision.
        self.ai_overlay_enabled = False
        self.last_detections = []
        self.append_status("AI overlay hidden. Computer vision continues in background if it is running.")


    def update_fpv_display_from_thread(self):
        if self.fpv_thread is None:
            return
        # Do not burn CPU rendering video while the Technical tab is active.
        try:
            visible = self.tabs.currentWidget() in (self.main_tab, self.ai_tab)
        except Exception:
            visible = True
        if not visible:
            return
        frame, detections, frame_id, frame_time = self.fpv_thread.get_latest_frame_and_detections()
        if frame is None or frame_id == self._last_fpv_frame_id:
            return
        self._last_fpv_frame_id = frame_id
        self._fpv_display_frames += 1
        now = time.time()
        if now - self._fpv_display_t0 >= 1.0:
            self._fpv_display_fps = self._fpv_display_frames / max(0.001, now - self._fpv_display_t0)
            self._fpv_display_frames = 0
            self._fpv_display_t0 = now
        self.last_cv_detections = detections or []
        display_detections = self.last_cv_detections if bool(getattr(self, "ai_overlay_enabled", False)) else []
        self.on_frame(frame, display_detections)

    def start_raw_fpv_recording(self):
        if self.fpv_thread is None:
            # Start FPV first, then arm recording.
            self.start_fpv_ai(self._settings_from_tech(ai_enabled=False))
        if self.fpv_thread is None:
            QMessageBox.warning(self, "FPV recording", "FPV capture is not running.")
            return
        try:
            path = self.fpv_thread.start_fpv_recording(self.tech.folder_line.text().strip() or self.record_base, fps=float(self.tech.fps_spin.value()))
            self.append_status(f"Raw FPV 60FPS recording started: {path}")
        except Exception as e:
            QMessageBox.warning(self, "FPV recording failed", str(e))

    def stop_raw_fpv_recording(self):
        if self.fpv_thread is None:
            return
        path = self.fpv_thread.stop_fpv_recording()
        self.last_raw_fpv_video = path or self.last_raw_fpv_video
        self.append_status(f"Raw FPV recording stopped: {path}")
        if getattr(self, "post_process_mode", False) and path:
            if QMessageBox.question(self, "Process recorded video?", "Raw FPV recording stopped. Process this video with AI now?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.process_recorded_video(path)

    def on_fpv_status(self, text: str):
        self.ai_status.setText(text)
        self.append_status(text)

    def set_record_folder(self, path: str):
        self.record_base = path

    def start_recording(self):
        if self.recorder.active:
            QMessageBox.information(self, "Recording", "Recording is already active.")
            return
        self.recorder.start(self.tech.folder_line.text().strip() or self.record_base, fps=self.record_fps)
        self.rec_status.setText("REC: ON")
        self.rec_status.setStyleSheet("font-weight:bold;color:#22c55e;")
        self.append_status(f"Recording started: {self.recorder.session_dir}")

    def stop_recording(self):
        if not self.recorder.active:
            return
        path = self.recorder.stop()
        self.rec_status.setText("REC: OFF")
        self.rec_status.setStyleSheet("font-weight:bold;color:#fca5a5;")
        self.append_status(f"Recording stopped: {path}")
        QMessageBox.information(self, "Recording saved", f"Dataset saved to:\n{path}")

    def on_telemetry(self, d: Dict[str, Any]):
        was_armed = bool(getattr(self, "_last_armed_state", False))
        is_armed = bool(d.get("armed"))
        if is_armed and not was_armed:
            if not self.fuel_check_done and not self._fuel_warning_shown_this_arm:
                self._fuel_warning_shown_this_arm = True
                self.append_status("WARNING: Vehicle armed without running Fuel Check / Arm Ready.")
                QMessageBox.warning(self, "Fuel check not completed", "Vehicle armed, but Fuel Check / Arm Ready was not run in the Technical / Setup tab. Land and run fuel check before mission testing.")
            self.flight_timer_running = True
            if self.flight_timer_start <= 0:
                self.flight_timer_start = time.time()
        if not is_armed and was_armed:
            self._fuel_warning_shown_this_arm = False
        self._last_armed_state = is_armed
        self.last_telemetry = d
        if hasattr(self, "ahrs_widget") and hasattr(self.ahrs_widget, "set_data_rates"):
            self.ahrs_widget.set_data_rates(d)
        self.telemetry_cards.update_data(d)
        if hasattr(self, "esc_fence_stack"):
            self.esc_fence_stack.update_data(d)
        if hasattr(self, "ahrs_widget"):
            self.ahrs_widget.update_attitude(safe_float(d.get("roll_deg")), safe_float(d.get("pitch_deg")), safe_float(d.get("heading_deg")))
        self.mission_widget.update_telemetry(d)
        self.main_map.update_telemetry(d)
        self.dual_channels.update_data(self.last_tx, self.last_telemetry)
        if hasattr(self, "main_channels_preview"):
            self.main_channels_preview.update_data(self.last_tx, self.last_telemetry)
        if self.recorder.active and time.time() - self.last_record_log > 0.08:
            self.recorder.log_telemetry(d)
            self.recorder.log_rc(self.last_tx, self.last_telemetry)
            self.last_record_log = time.time()

    def on_tx_data(self, d: Dict[str, Any]):
        self.last_tx = d
        self.dual_channels.update_data(self.last_tx, self.last_telemetry)
        if hasattr(self, "main_channels_preview"):
            self.main_channels_preview.update_data(self.last_tx, self.last_telemetry)

    def on_frame(self, frame: np.ndarray, detections: List[Dict[str, Any]]):
        # detections = what is drawn on Main FPV. last_cv_detections = what CV is actually seeing.
        self.last_detections = detections or []
        cv_dets = getattr(self, "last_cv_detections", self.last_detections) or []
        self.fpv_widget.update_frame(frame, self.last_detections, display_fps=self._fpv_display_fps)
        now = time.time()
        # AI tab preview is throttled so it cannot make the whole UI sluggish.
        try:
            if self.tabs.currentWidget() == self.ai_tab and now - self._last_ai_preview_time >= 0.15:
                preview = AIFrameThread.draw_detections(frame, cv_dets) if cv_dets else frame
                self.ai_video_big.setPixmap(frame_to_qpixmap(preview, self.ai_video_big.size()))
                self._last_ai_preview_time = now
        except Exception:
            pass
        try:
            log_visible = self.tabs.currentWidget() == self.ai_tab
        except Exception:
            log_visible = False
        if log_visible and cv_dets and now - getattr(self, '_last_detection_log_time', 0.0) >= 0.50:
            line = " | ".join([f"{x.get('label')} {safe_float(x.get('confidence')):.2f}" for x in cv_dets[:10]])
            self.detection_log.append(f"[{time.strftime('%H:%M:%S')}] {line}")
            self._last_detection_log_time = now
        if self.recorder.active:
            save_frame = (self.recorder.frame_count % max(1, int(self.tech.save_every.value())) == 0)
            overlay_for_dataset = AIFrameThread.draw_detections(frame, cv_dets) if cv_dets else frame
            self.recorder.write_frame(overlay_for_dataset, cv_dets, save_frame=save_frame, fps=self.record_fps)
            self.recorder.log_detections(cv_dets, self.last_telemetry)

    def process_recorded_video(self, path: str = ""):
        # Choose recorded video.
        video_path = str(path or getattr(self, "last_raw_fpv_video", "") or "")
        if video_path and Path(video_path).exists():
            if QMessageBox.question(self, "Use last FPV recording?", f"Process this recorded FPV video?\n\n{video_path}", QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
                video_path = ""
        if not video_path:
            video_path, _ = QFileDialog.getOpenFileName(self, "Select recorded FPV video to process", self.record_base, "Video files (*.avi *.mp4 *.mov *.mkv);;All files (*)")
        if not video_path:
            return
        # Ask class mode and overlay choice.
        human_reply = QMessageBox.question(self, "Post-processing target", "Process HUMAN/FACE/person detections only?\n\nYes = human/face only\nNo = all objects / things", QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
        if human_reply == QMessageBox.Cancel:
            return
        mode = "human" if human_reply == QMessageBox.Yes else "all"
        overlay_reply = QMessageBox.question(self, "Post-processing overlay", "Create an output video with AI bounding-box overlays?\n\nYes = overlay video + CSV\nNo = CSV detections only", QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
        if overlay_reply == QMessageBox.Cancel:
            return
        draw_overlay = overlay_reply == QMessageBox.Yes
        if self.post_process_thread is not None and self.post_process_thread.isRunning():
            QMessageBox.information(self, "Post-processing", "A post-processing job is already running.")
            return
        out_dir = str(Path(self.tech.folder_line.text().strip() or self.record_base).expanduser() / "post_processed")
        self.post_process_thread = VideoPostProcessThread(
            video_path=video_path,
            output_dir=out_dir,
            model_path=self.tech.model_path.text().strip() or "yolov8n.pt",
            device=self.tech.ai_device_combo.currentText(),
            imgsz=int(self.tech.ai_imgsz.value()),
            conf=float(self.tech.conf_spin.value()),
            half=bool(self.tech.ai_half.isChecked()),
            max_det=int(self.tech.ai_max_det.value()),
            mode=mode,
            draw_overlay=draw_overlay,
        )
        self.post_process_thread.progress.connect(self.on_post_process_progress)
        self.post_process_thread.finished_ok.connect(self.on_post_process_finished)
        self.post_process_thread.failed.connect(self.on_post_process_failed)
        self.post_process_thread.start()
        self.append_status("Post-processing started. Live FPV stays independent.")

    def on_post_process_progress(self, pct: int, text: str):
        try:
            self.ai_status.setText(f"Post-process {pct}%: {text}")
            self.detection_log.append(f"[{time.strftime('%H:%M:%S')}] POST {pct}% {text}")
        except Exception:
            pass
        self.append_status(f"Post-process {pct}%: {text}")

    def on_post_process_finished(self, result_path: str):
        self.append_status(f"Post-processing complete: {result_path}")
        QMessageBox.information(self, "Post-processing complete", f"Saved result:\n{result_path}")

    def on_post_process_failed(self, error: str):
        self.append_status(f"Post-processing failed: {error}")
        QMessageBox.warning(self, "Post-processing failed", error)

    def send_guided_waypoint(self, lat: float, lon: float, alt: float, speed: float):
        if self.telemetry_thread is None:
            QMessageBox.warning(self, "No flight-controller telemetry", "Connect MAVLink telemetry first for waypoint sending.")
            return
        if not hasattr(self.telemetry_thread, "send_guided_goto"):
            msg = "Waypoint sending is MAVLink-only. Betaflight MSP mode is telemetry/read-only in this dashboard."
            self.mission_widget.set_status(msg)
            self.main_map.set_status(msg)
            QMessageBox.warning(self, "Waypoint unsupported", msg)
            return
        ok, msg = self.telemetry_thread.send_guided_goto(lat, lon, alt, speed)
        self.mission_widget.set_status(msg)
        self.main_map.set_status(msg)
        self.append_status(msg)
        if not ok:
            QMessageBox.warning(self, "Waypoint failed", msg)

    def start_timer(self):
        self.flight_timer_running = True
        self.flight_timer_start = time.time()

    def stop_timer(self):
        self.flight_timer_running = False

    def reset_timer(self):
        self.flight_timer_start = time.time()
        self.timer_label.setText("00:00:00")

    def periodic_update(self):
        if self.flight_timer_running:
            self.timer_label.setText(seconds_to_hms(time.time() - self.flight_timer_start))
        proto = self.last_telemetry.get("telemetry_protocol", "MAVLink")
        mav = f"{proto} connected" if self.last_telemetry.get("connected") else f"{proto} disconnected"
        tx = "TX connected" if self.last_tx.get("connected") else "TX disconnected"
        fpv = "FPV running" if self.fpv_thread is not None else "FPV stopped"
        cv = "CV ON" if (self.fpv_thread is not None and getattr(self.fpv_thread, "ai_runtime_enabled", False)) else "CV OFF"
        ov = "OVR ON" if bool(getattr(self, "ai_overlay_enabled", False)) else "OVR OFF"
        tel_hz = safe_float(self.last_telemetry.get("telemetry_rate_hz"))
        att_hz = safe_float(self.last_telemetry.get("attitude_rate_hz"))
        self.connection_status.setText(f"{mav} | {tx} | {fpv} | {cv} | {ov} | TEL {tel_hz:.0f}Hz ATT {att_hz:.0f}Hz")
        # Main-tab smart fuel snapshot.
        try:
            mode = self.tech.fuel_mode.currentText()
            if mode.startswith("Live"):
                state = self.tech.update_live_fuel_display(self.last_telemetry)
                self.fuel_summary.setText(state.get("text", "Fuel: live estimator waiting."))
            else:
                v = self.last_telemetry.get("battery_voltage_v")
                i = self.last_telemetry.get("battery_current_a")
                rem = self.last_telemetry.get("battery_remaining_percent")
                fuel_state = "Fuel check: OK" if self.fuel_check_done else "Fuel check: NOT DONE"
                if v is not None or i is not None:
                    self.fuel_summary.setText(f"{fuel_state} | Live power: {safe_float(v):.2f}V × {safe_float(i):.1f}A = {safe_float(v)*safe_float(i):.0f}W | Battery remaining: {rem if rem is not None else '-'}%")
                else:
                    self.fuel_summary.setText(f"{fuel_state} | Waiting for flight-controller battery voltage/current.")
        except Exception:
            pass

    def closeEvent(self, event):
        try:
            self.stop_recording()
        except Exception:
            pass
        self.disconnect_mavlink()
        self.disconnect_tx()
        self.stop_fpv_ai()
        event.accept()


class BootLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Dashboard v100 Boot")
        self.resize(760, 490)
        self.dashboard = None
        root = QVBoxLayout(self); root.setContentsMargins(32, 26, 32, 26); root.setSpacing(12)
        title = QLabel("Drone Dashboard v100")
        title.setFont(QFont("Arial", 25, QFont.Bold)); title.setAlignment(Qt.AlignCenter)
        subtitle = QLabel("Choose cockpit theme, launch mode, and computer-vision workflow before booting the compact GCS.")
        subtitle.setAlignment(Qt.AlignCenter); subtitle.setWordWrap(True)
        form_box = QGroupBox("Boot options")
        form = QFormLayout(form_box)
        self.theme_combo = QComboBox(); self.theme_combo.addItems(["Dark theme", "White theme"])
        self.window_combo = QComboBox(); self.window_combo.addItems(["Full screen", "Maximized window", "Windowed"])
        self.scale_combo = QComboBox(); self.scale_combo.addItems(["Auto", "Compact", "Normal", "Large"])
        self.cv_mode_combo = QComboBox(); self.cv_mode_combo.addItems(["Live Computer Vision", "Post-processing Computer Vision"]); self.cv_mode_combo.setCurrentIndex(0)
        for combo in (self.theme_combo, self.window_combo, self.scale_combo, self.cv_mode_combo):
            self._prepare_boot_combo(combo)
        form.addRow("Theme", self.theme_combo)
        form.addRow("Launch display", self.window_combo)
        form.addRow("Display scaling", self.scale_combo)
        form.addRow("Computer vision mode", self.cv_mode_combo)
        self.progress = QProgressBar(); self.progress.setRange(0, 100)
        self.status = QLabel("Ready") ; self.status.setAlignment(Qt.AlignCenter)
        btn = QPushButton("Boot Dashboard") ; btn.clicked.connect(self.boot)
        for w in (title, subtitle, form_box, self.progress, self.status, btn): root.addWidget(w)
        self.setStyleSheet("""
        QWidget { background:#0b1120; color:#e5edf8; font-family:Arial; font-size:13px; }
        QGroupBox { border:1px solid rgba(100,125,170,170); border-top:1px solid rgba(190,210,255,150); border-radius:18px; margin-top:12px; padding:10px; background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(30,41,59,235), stop:0.48 rgba(15,23,42,225), stop:1 rgba(8,13,25,240)); font-weight:bold; }
        QGroupBox::title { subcontrol-origin: margin; left:12px; padding:0 6px; background:#0b1120; }
        QComboBox { background:#0f172a; color:#e5edf8; border:1px solid #475569; border-radius:8px; padding:7px; combobox-popup:1; }
        QComboBox QListView, QComboBox QAbstractItemView { background:#0f172a; color:#e5edf8; selection-background-color:#2563eb; selection-color:white; border:1px solid #475569; outline:0px; padding:0px; margin:0px; }
        QComboBox::drop-down { background:#111827; border-left:1px solid #475569; width:26px; border-top-right-radius:8px; border-bottom-right-radius:8px; }
        QComboBox::down-arrow { image:none; border-left:5px solid transparent; border-right:5px solid transparent; border-top:7px solid #cbd5e1; width:0px; height:0px; margin-right:7px; }
        QPushButton { background:#2563eb; color:white; border:1px solid #60a5fa; border-radius:10px; padding:11px 16px; font-weight:bold; }
        QPushButton:hover { background:#3b82f6; }
        QProgressBar { background:#111827; border:1px solid #334155; border-radius:5px; color:white; text-align:center; }
        QProgressBar::chunk { background:#22c55e; }
        """)

    def _prepare_boot_combo(self, combo: QComboBox):
        """Use a fully styled non-native popup to avoid Windows white bars in dark boot menus."""
        view = QListView(combo)
        view.setObjectName("bootComboPopup")
        view.setUniformItemSizes(True)
        view.setFrameShape(QFrame.NoFrame)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setMouseTracking(True)
        view.setAttribute(Qt.WA_StyledBackground, True)
        view.viewport().setAttribute(Qt.WA_StyledBackground, True)
        combo.setView(view)
        combo.setMaxVisibleItems(6)
        combo.setStyleSheet("""
            QComboBox {
                background:#0f172a; color:#e5edf8;
                border:1px solid #475569; border-radius:8px;
                padding:7px 34px 7px 8px; combobox-popup:1;
            }
            QComboBox::drop-down {
                background:#111827; border-left:1px solid #475569; width:28px;
                border-top-right-radius:8px; border-bottom-right-radius:8px;
            }
            QComboBox::down-arrow {
                image:none; border-left:5px solid transparent; border-right:5px solid transparent;
                border-top:7px solid #cbd5e1; width:0px; height:0px; margin-right:8px;
            }
            QListView#bootComboPopup, QComboBox QAbstractItemView {
                background:#0f172a; color:#e5edf8; border:1px solid #475569;
                outline:0px; padding:0px; margin:0px; selection-background-color:#2563eb; selection-color:white;
            }
            QListView#bootComboPopup::item {
                min-height:30px; padding:6px 10px; background:#0f172a; color:#e5edf8;
                border:0px; margin:0px;
            }
            QListView#bootComboPopup::item:selected, QListView#bootComboPopup::item:hover {
                background:#2563eb; color:white;
            }
            QScrollBar { background:#0f172a; width:0px; height:0px; border:0px; }
        """)
        view.setStyleSheet("""
            QListView#bootComboPopup {
                background:#0f172a; color:#e5edf8; border:1px solid #475569;
                outline:0px; padding:0px; margin:0px;
            }
            QListView#bootComboPopup::item {
                min-height:30px; padding:6px 10px; background:#0f172a; color:#e5edf8;
                border:0px; margin:0px;
            }
            QListView#bootComboPopup::item:selected, QListView#bootComboPopup::item:hover {
                background:#2563eb; color:white;
            }
            QScrollBar { background:#0f172a; width:0px; height:0px; border:0px; }
        """)
        try:
            view.viewport().setStyleSheet("background:#0f172a; color:#e5edf8;")
        except Exception:
            pass

    def boot(self):
        self.progress.setValue(25); self.status.setText("Loading v100 live/post CV cockpit modules..."); QApplication.processEvents()
        theme = "light" if self.theme_combo.currentText().startswith("White") else "dark"
        launch_text = self.window_combo.currentText()
        full = launch_text.startswith("Full")
        self.dashboard = DroneDashboardV23(theme=theme, launch_fullscreen=full, scale_mode=self.scale_combo.currentText(), cv_mode=self.cv_mode_combo.currentText())
        self.progress.setValue(75); QApplication.processEvents()
        if launch_text.startswith("Full"):
            self.dashboard.showFullScreen()
        elif launch_text.startswith("Maximized"):
            self.dashboard.showMaximized()
        else:
            self.dashboard.show()
        self.progress.setValue(100); self.status.setText("Ready")
        self.close()


def main():
    try:
        if hasattr(Qt, "HighDpiScaleFactorRoundingPolicy"):
            QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    except Exception:
        pass
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    if hasattr(Qt, "AA_ShareOpenGLContexts"):
        QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    launcher = BootLauncher()
    launcher.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
