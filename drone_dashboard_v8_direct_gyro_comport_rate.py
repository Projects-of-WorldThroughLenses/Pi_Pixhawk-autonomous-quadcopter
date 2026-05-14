r"""
Drone Dashboard v1 - PyQt5 Borderless FPV + MAVLink Telemetry + Live Fuel Estimator

Built from the earlier drone_flight_estimator_pyqt5_v3 concept and upgraded into a full
monitoring dashboard for a 4K laptop display.

Main features:
- Borderless / frameless dashboard window for high-resolution screens
- Low-latency OpenCV video capture with frame dropping behaviour
- FPV video source controls hidden behind a three-dot button overlaying the feed
- Screenshot and video recording
- MAVLink telemetry reader thread for Pixhawk / ArduPilot / PX4-style telemetry
- Manual expandable COM-port / MAVLink connection selector with refresh button
- Automatic arm detection and automatic flight timer start
- Manual "ARMING STARTED" timer button
- Fuel / flight-time estimator dialog with max throttle and cruise throttle inputs
- Live fuel estimate using telemetry voltage, current and throttle where available
- Artificial horizon / gyro ball with full 180-degree roll scale and heading tape
- GPS / lidar / optical-flow altitude, speed, climb-rate and GPS-lock display
- 14 RC channel display
- Live GPS map panel using PyQtWebEngine + Leaflet when available

Install on Windows example:
    C:\Python312\python.exe -m venv dashboard_env
    .\dashboard_env\Scripts\activate
    python -m pip install --upgrade pip
    pip install PyQt5 PyQtWebEngine opencv-python numpy pymavlink
    python drone_dashboard.py

Install on Raspberry Pi / Linux example:
    python3 -m venv dashboard_env --system-site-packages
    source dashboard_env/bin/activate
    python -m pip install --upgrade pip
    pip install PyQt5 PyQtWebEngine opencv-python numpy pymavlink
    python drone_dashboard.py

Safety note:
This dashboard is designed as a monitoring and estimation tool. It does not arm,
disarm, take off, land, or send flight-control commands to the aircraft.
"""

import os
import sys
import time
import math
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Dict, Any, List

import cv2
import numpy as np

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QSize, QPointF, QRectF
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QPen, QBrush, QPolygonF, QPainterPath, QLinearGradient, QRadialGradient
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox, QLineEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QFileDialog,
    QMessageBox, QTextEdit, QCheckBox, QFrame, QDialog, QFormLayout,
    QSpinBox, QDoubleSpinBox, QProgressBar, QSizePolicy, QSplitter,
    QScrollArea, QRadioButton, QButtonGroup
)

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except Exception:
    mavutil = None
    PYMAVLINK_AVAILABLE = False

try:
    from serial.tools import list_ports
    SERIAL_PORT_SCAN_AVAILABLE = True
except Exception:
    list_ports = None
    SERIAL_PORT_SCAN_AVAILABLE = False

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_AVAILABLE = True
except Exception:
    QWebEngineView = None
    WEBENGINE_AVAILABLE = False


# ----------------------------- Utility physics functions -----------------------------

G = 9.80665
R_DRY_AIR = 287.05
R_WATER_VAPOUR = 461.495
SEA_LEVEL_PRESSURE = 101325.0
SEA_LEVEL_TEMP = 288.15
LAPSE_RATE = 0.0065
STANDARD_RHO = 1.225


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except Exception:
        return default


def air_density_kg_m3(altitude_m: float, temp_c: float, relative_humidity_percent: float) -> float:
    """ISA-like density estimate with humidity correction."""
    temp_k = temp_c + 273.15
    rh = clamp(relative_humidity_percent, 0.0, 100.0) / 100.0
    pressure = SEA_LEVEL_PRESSURE * (1.0 - (LAPSE_RATE * altitude_m) / SEA_LEVEL_TEMP) ** 5.25588
    sat_vapour_pressure = 610.94 * math.exp((17.625 * temp_c) / (temp_c + 243.04))
    vapour_pressure = rh * sat_vapour_pressure
    dry_air_pressure = pressure - vapour_pressure
    rho = dry_air_pressure / (R_DRY_AIR * temp_k) + vapour_pressure / (R_WATER_VAPOUR * temp_k)
    return max(rho, 0.1)


def seconds_to_hms(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def discover_mavlink_connection_options() -> List[str]:
    """Return selectable MAVLink connection strings for the top-bar dropdown.

    Windows: shows real COM ports when pyserial is available, plus useful UDP defaults.
    Linux/Raspberry Pi: shows common /dev Pixhawk paths and UDP defaults.
    The combo box remains editable, so the user can still type COM7, COM12, /dev/ttyACM0,
    udp:127.0.0.1:14550, tcp:127.0.0.1:5760, etc.
    """
    options: List[str] = []

    if SERIAL_PORT_SCAN_AVAILABLE and list_ports is not None:
        try:
            for port in list_ports.comports():
                device = getattr(port, "device", "")
                desc = getattr(port, "description", "")
                if device:
                    if desc and desc != device:
                        options.append(f"{device}    # {desc}")
                    else:
                        options.append(device)
        except Exception:
            pass

    # Useful defaults even if the scanner cannot see anything yet.
    defaults = [
        "udp:127.0.0.1:14550",
        "udp:0.0.0.0:14550",
        "tcp:127.0.0.1:5760",
    ]
    if os.name == "nt":
        defaults += ["COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "COM10"]
    else:
        defaults += [
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/serial/by-id/",
        ]

    for item in defaults:
        if item not in options:
            options.append(item)
    return options


def clean_connection_string(combo_text: str) -> str:
    """Remove human-readable comments from combo entries before connecting."""
    return combo_text.split("#", 1)[0].strip()


def lipo_soc_from_cell_voltage(v_cell: float) -> float:
    """Very approximate open-circuit LiPo SoC curve. Useful only as a fallback."""
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
            t = (v_cell - v2) / (v1 - v2)
            return s2 + t * (s1 - s2)
    return 0.0



@dataclass
class FuelSettings:
    # Environment
    altitude_m: float = 0.0
    temp_c: float = 30.0
    humidity_percent: float = 70.0

    # Battery
    cell_count: int = 4
    cell_voltage: float = 3.8
    capacity_mah: float = 1550.0
    usable_percent: float = 75.0
    reserve_percent: float = 20.0

    # Drone and payload
    rotor_count: int = 4
    drone_mass_kg: float = 1.65
    payload_mass_kg: float = 0.20
    avionics_power_w: float = 15.0
    avionics_current_a: float = 2.0

    # Motor and propeller
    motor_kv: float = 1300.0
    prop_diameter_in: float = 9.0
    prop_pitch_in: float = 4.5
    blade_count: int = 2
    loading_factor: float = 0.70
    figure_of_merit: float = 0.55
    measured_max_thrust_g: float = 0.0
    max_current_per_motor_a: float = 35.0
    esc_rating_a: float = 40.0

    # Mission / live fuel behaviour
    max_throttle_percent: float = 100.0
    cruise_throttle_percent: float = 40.0
    telemetry_current_bias_a: float = 0.0
    use_telemetry_current_first: bool = True

    # Calibration benchmark for amperage scaling
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
    def usable_mah(self) -> float:
        return self.capacity_mah * clamp(self.usable_percent, 1.0, 100.0) / 100.0

    @property
    def reserve_mah(self) -> float:
        return self.capacity_mah * clamp(self.reserve_percent, 0.0, 90.0) / 100.0

    @property
    def battery_voltage_v(self) -> float:
        return max(1, int(self.cell_count)) * max(0.1, self.cell_voltage)

    @property
    def max_total_current_a(self) -> float:
        return self.max_current_per_motor_a * max(1, int(self.rotor_count)) + self.avionics_current_a


def prop_disk_area_total(rotor_count: int, prop_diameter_in: float) -> float:
    diameter_m = prop_diameter_in * 0.0254
    radius_m = diameter_m / 2.0
    return max(1, int(rotor_count)) * math.pi * radius_m ** 2


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
    """Empirical amperage scaling model.

    This mirrors the richer estimator logic from the user's PyQt5 v3 estimator:
    I ≈ I_ref × (KV/KV_ref)^3 × (V/V_ref)^2 × (D/D_ref)^5 × density × λ^3 × pitch × blade correction.
    """
    kv_factor = (kv / max(ref_kv, 1e-6)) ** 3
    voltage_factor = (voltage / max(ref_voltage, 1e-6)) ** 2
    diameter_factor = (prop_diameter_in / max(ref_diameter_in, 1e-6)) ** 5
    density_factor = rho / max(ref_rho, 1e-6)
    lambda_factor = (loading_factor / max(ref_loading_factor, 1e-6)) ** 3
    pitch_ratio = (prop_pitch_in / max(prop_diameter_in, 1e-6)) / (ref_pitch_in / max(ref_diameter_in, 1e-6))
    pitch_factor = max(pitch_ratio, 0.05) ** pitch_exponent
    blade_factor = (max(1, int(blade_count)) / max(1, int(ref_blades))) ** blade_exponent
    return max(0.0, ref_current_a * kv_factor * voltage_factor * diameter_factor * density_factor * lambda_factor * pitch_factor * blade_factor)


def ideal_hover_power_w(total_mass_kg: float, rotor_count: int, prop_diameter_in: float, rho: float, figure_of_merit: float) -> float:
    weight_n = total_mass_kg * G
    area = prop_disk_area_total(rotor_count, prop_diameter_in)
    p_ideal = (weight_n ** 1.5) / math.sqrt(max(1e-9, 2.0 * rho * area))
    return p_ideal / clamp(figure_of_merit, 0.25, 0.95)


def rich_fuel_estimate(settings: FuelSettings) -> Dict[str, Any]:
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
    measured_max_thrust_g = settings.measured_max_thrust_g

    if measured_max_thrust_g > 0:
        thrust_fraction = clamp(hover_thrust_per_motor_g / measured_max_thrust_g, 0.02, 1.50)
        hover_throttle_equivalent = math.sqrt(thrust_fraction)
        hover_current_motor_cubic = static_current_motor * (thrust_fraction ** 1.5)
        thrust_margin_percent = ((measured_max_thrust_g * rotor_count) / (total_mass_kg * 1000.0) - 1.0) * 100.0
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
    max_total_current_a = settings.max_total_current_a
    usable_ah = max(0.001, settings.usable_mah / 1000.0)
    reserve_ah = max(0.0, settings.reserve_mah / 1000.0)

    flight_min_hover = usable_ah / max(total_hover_current_a, 0.001) * 60.0
    flight_min_cruise = usable_ah / max(cruise_current_a, 0.001) * 60.0
    to_reserve_hover = max(0.0, (usable_ah - reserve_ah) / max(total_hover_current_a, 0.001) * 60.0)
    to_reserve_cruise = max(0.0, (usable_ah - reserve_ah) / max(cruise_current_a, 0.001) * 60.0)

    warnings = []
    if measured_max_thrust_g > 0 and thrust_margin_percent < 50:
        warnings.append("LOW THRUST MARGIN: payload or AUW is high for this propulsion system.")
    if measured_max_thrust_g > 0 and hover_throttle_equivalent > 0.65:
        warnings.append("HIGH HOVER THROTTLE: limited control reserve may remain.")
    if static_current_motor > settings.esc_rating_a * 0.85:
        warnings.append("ESC CURRENT CAUTION: static current is close to ESC rating.")
    if min(flight_min_hover, flight_min_cruise) < 5:
        warnings.append("SHORT FLIGHT TIME: plan conservative flights and reserve battery.")

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
        "max_total_current_a": max_total_current_a,
        "flight_min_hover": flight_min_hover,
        "flight_min_cruise": flight_min_cruise,
        "to_reserve_hover": to_reserve_hover,
        "to_reserve_cruise": to_reserve_cruise,
        "warnings": warnings,
    }



def estimate_current_from_throttle(settings: FuelSettings, throttle_percent: float) -> float:
    """
    Practical fallback when telemetry current is missing.

    Throttle is not true RPM, but the cubic approximation is useful for live estimation:
    I_total ≈ I_motor_max × N × (throttle / max_throttle)^3 + avionics_current.
    """
    max_thr = clamp(settings.max_throttle_percent, 1.0, 100.0)
    thr = clamp(throttle_percent, 0.0, max_thr)
    ratio = thr / max_thr
    propulsion_current = settings.max_current_per_motor_a * max(1, int(settings.rotor_count)) * (ratio ** 3)
    return max(0.0, propulsion_current + settings.avionics_current_a + settings.telemetry_current_bias_a)


def estimate_planned_flight_time(settings: FuelSettings) -> Dict[str, float]:
    r = rich_fuel_estimate(settings)
    return {
        "cruise_current_a": r["cruise_current_a"],
        "usable_min": r["flight_min_cruise"],
        "to_reserve_min": r["to_reserve_cruise"],
        "max_total_current_a": r["max_total_current_a"],
    }


# ----------------------------- Low-latency video capture

# ----------------------------- Low-latency video capture -----------------------------

class CaptureThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    status = pyqtSignal(str)
    fps_measured = pyqtSignal(float)
    opened_info = pyqtSignal(str)

    def __init__(self, source: str, fps_target: int = 60, use_mjpg: bool = True, parent=None):
        super().__init__(parent)
        self.source = source
        self.fps_target = int(fps_target)
        self.use_mjpg = use_mjpg
        self.running = False
        self.cap = None

    def _open_capture(self):
        try:
            src = int(self.source)
        except ValueError:
            src = self.source

        if isinstance(src, int) and os.name == "nt":
            cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(src)

        if not cap.isOpened():
            return None

        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if self.use_mjpg:
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            except Exception:
                pass
        try:
            cap.set(cv2.CAP_PROP_FPS, self.fps_target)
        except Exception:
            pass
        return cap

    def run(self):
        self.running = True
        self.cap = self._open_capture()
        if self.cap is None:
            self.status.emit("Video: failed to open source")
            return

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        self.opened_info.emit(
            f"Opened {self.source} | {actual_w}x{actual_h} | requested {self.fps_target} FPS | "
            f"reported {actual_fps:.1f} FPS | fourcc {fourcc_str}"
        )
        self.status.emit("Video: running")

        frame_count = 0
        fps_t0 = time.time()
        while self.running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.msleep(1)
                continue
            self.frame_ready.emit(frame)
            frame_count += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                self.fps_measured.emit(frame_count / (now - fps_t0))
                frame_count = 0
                fps_t0 = now

        if self.cap:
            self.cap.release()
        self.status.emit("Video: stopped")

    def stop(self):
        self.running = False
        self.wait(1200)


class VideoLabel(QLabel):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(QSize(960, 540))
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: black; color: white; border-radius: 10px;")
        self.setText("No FPV video")
        self.last_qimage = None

    def set_frame(self, frame_bgr: np.ndarray):
        h, w = frame_bgr.shape[:2]
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        qimg = QImage(frame_rgb.data, w, h, frame_rgb.strides[0], QImage.Format_RGB888).copy()
        self.last_qimage = qimg
        self.update_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_pixmap()

    def update_pixmap(self):
        if self.last_qimage is None:
            return
        pixmap = QPixmap.fromImage(self.last_qimage)
        scaled = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(scaled)


# ----------------------------- MAVLink telemetry -----------------------------

class TelemetryThread(QThread):
    telemetry = pyqtSignal(dict)
    attitude = pyqtSignal(float, float, float)  # direct gyroball feed: roll, pitch, heading
    status = pyqtSignal(str)

    def __init__(self, connection_string: str, baud: int = 57600, parent=None):
        super().__init__(parent)
        self.connection_string = connection_string.strip()
        self.baud = int(baud)
        self.running = False
        self.master = None
        self.data: Dict[str, Any] = self.default_data()

    @staticmethod
    def default_data() -> Dict[str, Any]:
        return {
            "connected": False,
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
            "rc_channels": [0] * 14,
            "rc_rssi": None,
            "last_msg_time": None,
            "last_attitude_time": None,
            "last_attitude_source": "-",
            "attitude_msg_count": 0,
            "system_id": None,
            "component_id": None,
        }


    def connect_master(self):
        if not PYMAVLINK_AVAILABLE:
            raise RuntimeError("pymavlink is not installed. Install with: pip install pymavlink")
        if not self.connection_string:
            raise RuntimeError("Empty MAVLink connection string")

        # Serial ports are exclusive on Windows. If Mission Planner is already using COM9,
        # this app cannot also open COM9. Use UDP forwarding or close Mission Planner.
        if self.connection_string.upper().startswith("COM") or self.connection_string.startswith("/dev/"):
            return mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud,
                autoreconnect=False,
                source_system=255,
                source_component=190,
            )

        # For Mission Planner / MAVProxy forwarding, use UDP/TCP strings such as:
        # udp:127.0.0.1:14550, udp:0.0.0.0:14550, tcp:127.0.0.1:5760
        return mavutil.mavlink_connection(
            self.connection_string,
            autoreconnect=True,
            source_system=255,
            source_component=190,
        )


    def _close_master(self):
        try:
            if self.master is not None:
                self.master.close()
        except Exception:
            pass
        self.master = None

    def _wait_heartbeat_interruptible(self, timeout_s: float = 10.0):
        start = time.time()
        while self.running and time.time() - start < timeout_s:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.35)
            if msg is not None:
                self.parse_msg(msg)
                try:
                    self.master.target_system = msg.get_srcSystem()
                    self.master.target_component = msg.get_srcComponent()
                except Exception:
                    pass
                return msg
        raise TimeoutError("no heartbeat received")

    def _set_message_interval(self, msg_id: int, hz: float):
        if self.master is None or hz <= 0:
            return
        try:
            interval_us = int(1_000_000 / hz)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                int(msg_id),
                interval_us,
                0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    def _request_data_stream(self, stream_id: int, hz: int):
        if self.master is None:
            return
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream_id,
                int(hz),
                1,
            )
        except Exception:
            pass

    def request_telemetry_streams(self):
        """Request the exact messages needed by the dashboard.

        This fixes the common problem where only HEARTBEAT / flight mode appears,
        while ATTITUDE, GPS, battery and RC values stay frozen because no GCS has
        requested MAVLink streams yet.
        """
        if self.master is None:
            return

        # Newer ArduPilot/PX4 message interval requests.
        mav = mavutil.mavlink
        wanted = [
            (getattr(mav, "MAVLINK_MSG_ID_ATTITUDE", 30), 80),
            (getattr(mav, "MAVLINK_MSG_ID_ATTITUDE_QUATERNION", 31), 80),
            (getattr(mav, "MAVLINK_MSG_ID_AHRS2", 178), 25),
            (getattr(mav, "MAVLINK_MSG_ID_VFR_HUD", 74), 10),
            (getattr(mav, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33), 10),
            (getattr(mav, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32), 10),
            (getattr(mav, "MAVLINK_MSG_ID_GPS_RAW_INT", 24), 5),
            (getattr(mav, "MAVLINK_MSG_ID_SYS_STATUS", 1), 3),
            (getattr(mav, "MAVLINK_MSG_ID_BATTERY_STATUS", 147), 3),
            (getattr(mav, "MAVLINK_MSG_ID_RC_CHANNELS", 65), 20),
            (getattr(mav, "MAVLINK_MSG_ID_DISTANCE_SENSOR", 132), 10),
            (getattr(mav, "MAVLINK_MSG_ID_OPTICAL_FLOW", 100), 10),
            (getattr(mav, "MAVLINK_MSG_ID_OPTICAL_FLOW_RAD", 106), 10),
        ]
        for msg_id, hz in wanted:
            self._set_message_interval(msg_id, hz)

        # Older ArduPilot stream requests. These help when SET_MESSAGE_INTERVAL is ignored.
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_ALL", 0), 10)
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_RAW_SENSORS", 1), 5)
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_EXTENDED_STATUS", 2), 5)
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_RC_CHANNELS", 3), 20)
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_POSITION", 6), 10)
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_EXTRA1", 10), 80)  # ATTITUDE
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_EXTRA2", 11), 10)  # VFR_HUD
        self._request_data_stream(getattr(mav, "MAV_DATA_STREAM_EXTRA3", 12), 5)


    def run(self):
        self.running = True
        last_stream_request = 0.0
        try:
            self.status.emit("Telemetry: connecting...")
            self.master = self.connect_master()
            self._wait_heartbeat_interruptible(timeout_s=10.0)
            self.data["connected"] = True
            self.data["system_id"] = getattr(self.master, "target_system", None)
            self.data["component_id"] = getattr(self.master, "target_component", None)

            self.request_telemetry_streams()
            last_stream_request = time.time()

            self.status.emit(
                f"Telemetry: connected sys={self.data['system_id']} comp={self.data['component_id']} | requesting streams"
            )
        except PermissionError as e:
            self.status.emit(
                f"Telemetry: connection failed - Permission denied on {self.connection_string}. "
                "Close Mission Planner/other apps using this COM port, or use UDP forwarding."
            )
            self.data["connected"] = False
            self.telemetry.emit(dict(self.data))
            self._close_master()
            return
        except Exception as e:
            self.status.emit(f"Telemetry: connection failed - {e}")
            self.data["connected"] = False
            self.telemetry.emit(dict(self.data))
            self._close_master()
            return

        last_emit = 0.0
        last_status = 0.0

        try:
            while self.running:
                try:
                    msg = self.master.recv_match(blocking=True, timeout=0.02)
                    if msg is not None:
                        mtype = msg.get_type()
                        self.parse_msg(msg)
                        # Push attitude directly to the gyroball as fast as the COM/MAVLink stream provides it.
                        # This bypasses the slower dashboard telemetry refresh and removes smoothing lag.
                        if mtype in ("ATTITUDE", "AHRS2", "ATTITUDE_QUATERNION"):
                            self.attitude.emit(
                                float(self.data.get("roll_deg", 0.0)),
                                float(self.data.get("pitch_deg", 0.0)),
                                float(self.data.get("heading_deg", 0.0)),
                            )

                    now = time.time()

                    # Re-request streams periodically. Some autopilots/routers reset rates.
                    if now - last_stream_request >= 3.0:
                        self.request_telemetry_streams()
                        last_stream_request = now

                    if now - last_emit >= 0.016:  # ~60 Hz UI telemetry emission
                        self.data["last_msg_time"] = now
                        self.telemetry.emit(dict(self.data))
                        last_emit = now

                    if now - last_status >= 2.0:
                        age = None
                        if self.data.get("last_attitude_time") is not None:
                            age = now - self.data["last_attitude_time"]
                        if age is None:
                            self.status.emit(
                                f"Telemetry: connected sys={self.data['system_id']} comp={self.data['component_id']} | waiting ATTITUDE"
                            )
                        else:
                            self.status.emit(
                                f"Telemetry: connected | ATT {self.data['last_attitude_source']} age={age:.1f}s"
                            )
                        last_status = now

                except Exception as e:
                    if self.running:
                        self.status.emit(f"Telemetry error: {e}")
                    self.msleep(80)
        finally:
            self._close_master()
            self.data["connected"] = False
            self.telemetry.emit(dict(self.data))
            self.status.emit("Telemetry: stopped")


    def parse_msg(self, msg):
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            return

        self.data["connected"] = True

        if mtype == "HEARTBEAT":
            base_mode = getattr(msg, "base_mode", 0)
            armed_flag = 128  # MAV_MODE_FLAG_SAFETY_ARMED
            self.data["armed"] = bool(base_mode & armed_flag)
            try:
                self.data["mode"] = mavutil.mode_string_v10(msg)
            except Exception:
                self.data["mode"] = str(getattr(msg, "custom_mode", "-"))

        elif mtype == "ATTITUDE":
            self.data["roll_deg"] = math.degrees(getattr(msg, "roll", 0.0))
            self.data["pitch_deg"] = math.degrees(getattr(msg, "pitch", 0.0))
            yaw_deg = math.degrees(getattr(msg, "yaw", 0.0)) % 360.0
            self.data["yaw_deg"] = yaw_deg
            self.data["heading_deg"] = yaw_deg
            self.data["last_attitude_time"] = time.time()
            self.data["last_attitude_source"] = "ATTITUDE"
            self.data["attitude_msg_count"] = int(self.data.get("attitude_msg_count", 0)) + 1

        elif mtype == "AHRS2":
            # ArduPilot AHRS2 also carries roll/pitch/yaw in radians.
            self.data["roll_deg"] = math.degrees(getattr(msg, "roll", 0.0))
            self.data["pitch_deg"] = math.degrees(getattr(msg, "pitch", 0.0))
            yaw_deg = math.degrees(getattr(msg, "yaw", 0.0)) % 360.0
            self.data["yaw_deg"] = yaw_deg
            self.data["heading_deg"] = yaw_deg
            self.data["last_attitude_time"] = time.time()
            self.data["last_attitude_source"] = "AHRS2"
            self.data["attitude_msg_count"] = int(self.data.get("attitude_msg_count", 0)) + 1

        elif mtype == "ATTITUDE_QUATERNION":
            # Quaternion fallback for systems that stream quaternion but not Euler ATTITUDE.
            q1 = float(getattr(msg, "q1", 1.0))
            q2 = float(getattr(msg, "q2", 0.0))
            q3 = float(getattr(msg, "q3", 0.0))
            q4 = float(getattr(msg, "q4", 0.0))

            sinr_cosp = 2.0 * (q1 * q2 + q3 * q4)
            cosr_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            sinp = 2.0 * (q1 * q3 - q4 * q2)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)

            siny_cosp = 2.0 * (q1 * q4 + q2 * q3)
            cosy_cosp = 1.0 - 2.0 * (q3 * q3 + q4 * q4)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self.data["roll_deg"] = math.degrees(roll)
            self.data["pitch_deg"] = math.degrees(pitch)
            yaw_deg = math.degrees(yaw) % 360.0
            self.data["yaw_deg"] = yaw_deg
            self.data["heading_deg"] = yaw_deg
            self.data["last_attitude_time"] = time.time()
            self.data["last_attitude_source"] = "QUAT"
            self.data["attitude_msg_count"] = int(self.data.get("attitude_msg_count", 0)) + 1

        elif mtype == "VFR_HUD":
            self.data["airspeed_m_s"] = safe_float(getattr(msg, "airspeed", 0.0))
            self.data["groundspeed_m_s"] = safe_float(getattr(msg, "groundspeed", 0.0))
            self.data["gps_alt_m"] = safe_float(getattr(msg, "alt", 0.0))
            self.data["climb_m_s"] = safe_float(getattr(msg, "climb", 0.0))
            heading = getattr(msg, "heading", None)
            if heading is not None and int(heading) >= 0:
                self.data["heading_deg"] = safe_float(heading, self.data.get("heading_deg", 0.0)) % 360.0
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
            for i in range(1, 15):
                channels.append(int(getattr(msg, f"chan{i}_raw", 0)))
            self.data["rc_channels"] = channels
            self.data["rc_rssi"] = getattr(msg, "rssi", None)

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

        elif mtype == "LOCAL_POSITION_NED":
            z = safe_float(getattr(msg, "z", 0.0))
            vz = safe_float(getattr(msg, "vz", 0.0))
            if abs(z) > 0.001:
                self.data["rel_alt_m"] = -z
            self.data["climb_m_s"] = -vz


    def stop(self):
        self.running = False
        self._close_master()
        self.wait(1800)


class ArtificialHorizon(QWidget):
    """Direct artificial horizon.

    The gyroball no longer uses a fixed render FPS or exponential smoothing.
    Every fresh MAVLink attitude packet updates the displayed roll/pitch/heading
    immediately, so the visual rate follows whatever the COM port and autopilot
    can actually provide.
    """

    def __init__(self):
        super().__init__()
        self.setMinimumSize(420, 300)

        # Displayed values are now the latest raw attitude values, not smoothed targets.
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.heading_deg = 0.0

        self.last_target_time = time.monotonic()
        self.last_paint_time = time.monotonic()
        self.instrument_fps = 0.0
        self.attitude_age_s = 999.0

        # Backwards-compatible fields for older calls/debug displays.
        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_heading_deg = 0.0
        self.render_fps = 0
        self.render_interval_ms = 0

    def set_render_fps(self, fps: int):
        """Compatibility no-op.

        The gyroball is now event-driven: it renders whenever new attitude data
        arrives from MAVLink, regardless of whether the serial baudrate is 57600
        or 115200.
        """
        self.render_fps = 0
        self.render_interval_ms = 0

    def update_attitude(self, roll: float, pitch: float, heading: float):
        # Direct/no-smoothing attitude update. This prevents the old catch-up,
        # acceleration and lag-behind behaviour.
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

    def _draw_heading_tape(self, painter: QPainter, rect: QRectF, is_light: bool = False):
        """Short stable heading tape. The marks rotate correctly around a fixed centre slash."""
        tape_h = 30
        tape = QRectF(rect.left() + 8, rect.top() + 3, rect.width() - 16, tape_h)

        bg = QColor(238, 244, 252, 230) if is_light else QColor(10, 17, 28, 235)
        border = QColor(160, 174, 195) if is_light else QColor(82, 105, 142)
        text_col = QColor(15, 23, 42) if is_light else QColor(230, 242, 255)
        minor_col = QColor(65, 76, 96, 120) if is_light else QColor(230, 242, 255, 120)

        painter.setBrush(bg)
        painter.setPen(QPen(border, 1))
        painter.drawRoundedRect(tape, 6, 6)

        cx = tape.center().x()
        px_per_deg = tape.width() / 120.0

        def label_for(bearing: int) -> str:
            b = bearing % 360
            if b == 0:
                return "N"
            if b == 90:
                return "E"
            if b == 180:
                return "S"
            if b == 270:
                return "W"
            return f"{b:03d}"

        painter.save()
        painter.setClipRect(tape.adjusted(5, 1, -5, -1))
        painter.setFont(QFont("Arial", 7, QFont.Bold))

        # Draw every absolute 5-degree mark around the current heading.
        # Use floor instead of round to avoid text flicker near boundaries.
        centre = int(math.floor(self.heading_deg / 5.0) * 5)
        for bearing in range(centre - 180, centre + 181, 5):
            b = bearing % 360
            delta = self._angle_delta_deg(float(b), self.heading_deg)
            if abs(delta) > 65:
                continue
            x = cx + delta * px_per_deg
            major = b % 30 == 0
            mid = b % 10 == 0
            tick_h = 15 if major else (10 if mid else 6)
            painter.setPen(QPen(text_col if major else minor_col, 2 if major else 1))
            painter.drawLine(int(x), int(tape.bottom() - 4), int(x), int(tape.bottom() - 4 - tick_h))
            if major:
                painter.drawText(QRectF(x - 18, tape.top() + 3, 36, 13), Qt.AlignCenter, label_for(b))
        painter.restore()

        painter.setPen(QPen(QColor(255, 30, 30), 3))
        painter.drawLine(int(cx), int(tape.top() + 4), int(cx), int(tape.bottom() - 4))

        painter.setPen(QColor(220, 0, 0))
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        painter.drawText(QRectF(tape.right() - 78, tape.top() + 6, 70, 16), Qt.AlignRight, f"{self.heading_deg:03.0f}°")

    def _draw_roll_scale(self, painter: QPainter, cx: float, cy: float, radius: float, is_light: bool = False):
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll_deg)
        painter.setFont(QFont("Arial", 6, QFont.Bold))

        col_major = QColor(25, 35, 55, 150) if is_light else QColor(235, 244, 255, 150)
        col_minor = QColor(25, 35, 55, 80) if is_light else QColor(235, 244, 255, 80)

        for deg in range(-180, 181, 10):
            painter.save()
            painter.rotate(deg)
            major = deg % 30 == 0
            cardinal = deg % 90 == 0
            tick_outer = -radius - 3
            tick_inner = -radius - (14 if cardinal else 11 if major else 7)
            painter.setPen(QPen(col_major if major else col_minor, 1.5 if major else 1))
            painter.drawLine(0, int(tick_outer), 0, int(tick_inner))
            painter.restore()

        for deg in range(-180, 181, 30):
            a = math.radians(deg - 90)
            tx = math.cos(a) * (radius + 19)
            ty = math.sin(a) * (radius + 19)
            painter.setPen(QColor(25, 35, 55, 135) if is_light else QColor(230, 242, 255, 135))
            txt = str(abs(deg)) if deg != 0 else "0"
            painter.drawText(QRectF(tx - 14, ty - 7, 28, 14), Qt.AlignCenter, txt)
        painter.restore()

        marker = QPolygonF([
            QPointF(cx, cy - radius - 2),
            QPointF(cx - 8, cy - radius - 13),
            QPointF(cx + 8, cy - radius - 13),
        ])
        painter.setBrush(QBrush(QColor(255, 214, 65)))
        painter.setPen(QPen(QColor(10, 10, 10), 1))
        painter.drawPolygon(marker)

    def _draw_value_box(self, painter: QPainter, box: QRectF, is_light: bool):
        if box.width() < 120 or box.height() < 110:
            return
        bg = QColor(248, 251, 255, 235) if is_light else QColor(15, 24, 37, 225)
        border = QColor(140, 158, 182) if is_light else QColor(65, 84, 115)
        label_col = QColor(30, 41, 59) if is_light else QColor(210, 225, 245)
        value_col = QColor(215, 0, 0)

        painter.setBrush(bg)
        painter.setPen(QPen(border, 1))
        painter.drawRoundedRect(box, 9, 9)

        painter.setFont(QFont("Arial", 9, QFont.Bold))
        painter.setPen(label_col)
        painter.drawText(QRectF(box.left() + 10, box.top() + 8, box.width() - 20, 18), Qt.AlignLeft, "AHRS LIVE")

        status = "LIVE" if self.attitude_age_s < 0.25 else ("STALE" if self.attitude_age_s < 2.0 else "NO ATT")
        rows = [
            ("ROLL", f"{self.roll_deg:+.1f}°"),
            ("PITCH", f"{self.pitch_deg:+.1f}°"),
            ("HDG", f"{self.heading_deg:03.0f}°"),
            ("GYRO", f"{min(self.instrument_fps, 240):.0f} FPS"),
            ("STREAM", status),
        ]
        y = box.top() + 38
        for name, value in rows:
            row = QRectF(box.left() + 8, y, box.width() - 16, 26)
            painter.setBrush(QColor(235, 241, 250, 190) if is_light else QColor(20, 32, 50, 190))
            painter.setPen(QPen(border, 1))
            painter.drawRoundedRect(row, 5, 5)
            painter.setFont(QFont("Arial", 8, QFont.Bold))
            painter.setPen(label_col)
            painter.drawText(QRectF(row.left() + 8, row.top() + 4, row.width() * 0.42, 18), Qt.AlignLeft, name)
            painter.setPen(value_col)
            painter.setFont(QFont("Arial", 9, QFont.Bold))
            painter.drawText(QRectF(row.left() + row.width() * 0.42, row.top() + 4, row.width() * 0.55, 18), Qt.AlignRight, value)
            y += 31

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)

        now = time.monotonic()
        dt = max(0.001, now - self.last_paint_time)
        self.last_paint_time = now
        self.attitude_age_s = now - self.last_target_time
        inst = 1.0 / dt
        self.instrument_fps = inst if self.instrument_fps <= 0.1 else (0.85 * self.instrument_fps + 0.15 * inst)

        is_light = self.palette().color(self.backgroundRole()).lightness() > 150
        bg = self.palette().color(self.backgroundRole()) if is_light else QColor(8, 12, 20)
        painter.fillRect(self.rect(), bg)

        outer_rect = self.rect().adjusted(10, 8, -10, -10)
        self._draw_heading_tape(painter, QRectF(outer_rect), is_light=is_light)

        rect = outer_rect.adjusted(0, 38, 0, 0)
        side_w = max(145, min(210, rect.width() * 0.32))
        ball_area = QRectF(rect.left(), rect.top(), rect.width() - side_w - 12, rect.height())
        radius = min(ball_area.width() * 0.40, ball_area.height() * 0.43)
        cx = ball_area.left() + radius + 34
        if cx + radius > ball_area.right():
            cx = ball_area.center().x()
        cy = ball_area.center().y() - 2
        ball_rect = QRectF(cx - radius, cy - radius, radius * 2.0, radius * 2.0)
        value_box = QRectF(ball_area.right() + 10, rect.top() + 12, side_w - 12, rect.height() - 24)

        shadow = QRadialGradient(QPointF(cx, cy + radius * 0.20), radius * 1.35)
        shadow.setColorAt(0.0, QColor(0, 0, 0, 80 if is_light else 150))
        shadow.setColorAt(0.65, QColor(0, 0, 0, 40 if is_light else 90))
        shadow.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setBrush(QBrush(shadow))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QRectF(cx - radius * 1.26, cy - radius * 1.08, radius * 2.52, radius * 2.22))

        bezel = QRadialGradient(QPointF(cx - radius * 0.18, cy - radius * 0.22), radius * 1.25)
        bezel.setColorAt(0.00, QColor(185, 195, 210) if is_light else QColor(105, 118, 140))
        bezel.setColorAt(0.42, QColor(95, 110, 135) if is_light else QColor(35, 45, 62))
        bezel.setColorAt(0.82, QColor(42, 53, 72) if is_light else QColor(7, 11, 18))
        bezel.setColorAt(1.00, QColor(220, 226, 236) if is_light else QColor(175, 185, 200))
        painter.setBrush(QBrush(bezel))
        painter.setPen(QPen(QColor(190, 205, 225) if is_light else QColor(220, 228, 240), 2))
        painter.drawEllipse(QPointF(cx, cy), radius + 12, radius + 12)
        painter.setBrush(QColor(236, 241, 248) if is_light else QColor(5, 8, 14))
        painter.setPen(QPen(QColor(130, 145, 168) if is_light else QColor(28, 38, 55), 3))
        painter.drawEllipse(QPointF(cx, cy), radius + 4, radius + 4)

        ball_path = QPainterPath()
        ball_path.addEllipse(ball_rect)
        painter.save()
        painter.setClipPath(ball_path)

        painter.translate(cx, cy)
        painter.rotate(-self.roll_deg)
        pitch_px = self.pitch_deg * (radius / 28.0)

        sky_grad = QLinearGradient(0, -radius * 2.5 + pitch_px, 0, pitch_px)
        sky_grad.setColorAt(0.0, QColor(8, 40, 110))
        sky_grad.setColorAt(0.55, QColor(35, 122, 220))
        sky_grad.setColorAt(1.0, QColor(130, 198, 255))
        painter.setBrush(QBrush(sky_grad))
        painter.setPen(Qt.NoPen)
        painter.drawRect(QRectF(-radius * 2.5, -radius * 2.5 + pitch_px, radius * 5.0, radius * 2.5))

        ground_grad = QLinearGradient(0, pitch_px, 0, radius * 2.5 + pitch_px)
        ground_grad.setColorAt(0.0, QColor(170, 108, 50))
        ground_grad.setColorAt(0.55, QColor(92, 52, 25))
        ground_grad.setColorAt(1.0, QColor(35, 23, 16))
        painter.setBrush(QBrush(ground_grad))
        painter.drawRect(QRectF(-radius * 2.5, pitch_px, radius * 5.0, radius * 2.5))

        painter.setPen(QPen(QColor(0, 0, 0, 130), 6))
        painter.drawLine(int(-radius * 1.55), int(pitch_px + 1), int(radius * 1.55), int(pitch_px + 1))
        painter.setPen(QPen(QColor(250, 250, 250), 2))
        painter.drawLine(int(-radius * 1.55), int(pitch_px), int(radius * 1.55), int(pitch_px))

        painter.setFont(QFont("Arial", 8, QFont.Bold))
        for deg in range(-90, 91, 5):
            if deg == 0:
                continue
            y = pitch_px - deg * (radius / 28.0)
            if y < -radius * 1.20 or y > radius * 1.20:
                continue
            major = (deg % 10 == 0)
            length = radius * (0.42 if major else 0.23)
            gap = radius * 0.08
            pen_color = QColor(246, 250, 255, 225 if major else 165)
            painter.setPen(QPen(pen_color, 2 if major else 1))
            painter.drawLine(int(-length), int(y), int(-gap), int(y))
            painter.drawLine(int(gap), int(y), int(length), int(y))
            if major:
                painter.drawText(QRectF(length + 5, y - 9, 35, 18), Qt.AlignLeft | Qt.AlignVCenter, str(abs(deg)))
                painter.drawText(QRectF(-length - 40, y - 9, 35, 18), Qt.AlignRight | Qt.AlignVCenter, str(abs(deg)))
        painter.restore()

        painter.save()
        painter.setClipPath(ball_path)
        highlight = QRadialGradient(QPointF(cx - radius * 0.42, cy - radius * 0.50), radius * 1.05)
        highlight.setColorAt(0.0, QColor(255, 255, 255, 100))
        highlight.setColorAt(0.22, QColor(255, 255, 255, 45))
        highlight.setColorAt(0.58, QColor(255, 255, 255, 0))
        highlight.setColorAt(1.0, QColor(0, 0, 0, 60 if is_light else 82))
        painter.setBrush(QBrush(highlight))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(ball_rect)
        painter.restore()

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(245, 250, 255) if not is_light else QColor(80, 92, 112), 2))
        painter.drawEllipse(ball_rect)
        painter.setPen(QPen(QColor(72, 92, 125), 4))
        painter.drawEllipse(ball_rect.adjusted(-2, -2, 2, 2))
        self._draw_roll_scale(painter, cx, cy, radius, is_light=is_light)

        painter.setPen(QPen(QColor(0, 0, 0, 130), 7, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(int(cx - radius * 0.63), int(cy + 2), int(cx - radius * 0.13), int(cy + 2))
        painter.drawLine(int(cx + radius * 0.13), int(cy + 2), int(cx + radius * 0.63), int(cy + 2))
        painter.setPen(QPen(QColor(255, 208, 55), 5, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(int(cx - radius * 0.63), int(cy), int(cx - radius * 0.13), int(cy))
        painter.drawLine(int(cx + radius * 0.13), int(cy), int(cx + radius * 0.63), int(cy))
        painter.drawLine(int(cx), int(cy - radius * 0.11), int(cx), int(cy + radius * 0.11))
        painter.setPen(QPen(QColor(255, 235, 140), 2))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        self._draw_value_box(painter, value_box, is_light)

class TelemetryLabelGrid(QGroupBox):
    def __init__(self):
        super().__init__("Telemetry Rates / GPS / Altitude")
        self.labels: Dict[str, QLabel] = {}
        layout = QGridLayout(self)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)
        items = [
            ("mode", "Mode"), ("armed", "Armed"), ("gps", "GPS"),
            ("gps_alt", "GPS Alt"), ("rel_alt", "Rel Alt"), ("lidar", "Lidar Alt"),
            ("flow", "Optical Flow"), ("speed", "Ground Speed"), ("climb", "Climb Rate"),
            ("battery", "Battery"), ("current", "Current"), ("throttle", "Throttle"),
        ]
        for row, (key, name) in enumerate(items):
            title = QLabel(name + ":")
            title.setObjectName("telemetryTitle")
            value = QLabel("-")
            value.setObjectName("dynamicValue")
            value.setMinimumHeight(30)
            value.setAlignment(Qt.AlignCenter)
            layout.addWidget(title, row // 3, (row % 3) * 2)
            layout.addWidget(value, row // 3, (row % 3) * 2 + 1)
            self.labels[key] = value

    def update_data(self, d: Dict[str, Any]):
        self.labels["mode"].setText(str(d.get("mode", "-")))
        self.labels["armed"].setText("ARMED" if d.get("armed") else "DISARMED")
        self.labels["gps"].setText(f"fix {d.get('gps_fix_type', 0)} / {d.get('gps_sats', 0)} sats")
        self.labels["gps_alt"].setText(format_optional(d.get("gps_alt_m"), "m"))
        self.labels["rel_alt"].setText(format_optional(d.get("rel_alt_m"), "m"))
        self.labels["lidar"].setText(format_optional(d.get("lidar_alt_m"), "m"))
        flow_alt = d.get("optical_flow_alt_m")
        flow_q = d.get("optical_flow_quality")
        if flow_alt is not None:
            self.labels["flow"].setText(f"{flow_alt:.2f} m / q={flow_q}")
        else:
            self.labels["flow"].setText(f"q={flow_q}" if flow_q is not None else "-")
        self.labels["speed"].setText(f"{safe_float(d.get('groundspeed_m_s')):.2f} m/s")
        self.labels["climb"].setText(f"{safe_float(d.get('climb_m_s')):+.2f} m/s")
        self.labels["battery"].setText(format_optional(d.get("battery_voltage_v"), "V"))
        self.labels["current"].setText(format_optional(d.get("battery_current_a"), "A"))
        self.labels["throttle"].setText(f"{safe_float(d.get('throttle_percent')):.0f} %")


def format_optional(value: Any, unit: str, precision: int = 2) -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.{precision}f} {unit}"
    except Exception:
        return "-"


class RCChannelsWidget(QGroupBox):
    def __init__(self):
        super().__init__("TX / RC Channels 1-14")
        self.bars: List[QProgressBar] = []
        layout = QGridLayout(self)
        for i in range(14):
            lab = QLabel(f"CH{i+1}")
            bar = QProgressBar()
            bar.setRange(900, 2100)
            bar.setValue(1500)
            bar.setFormat("%v")
            bar.setFixedHeight(16)
            layout.addWidget(lab, i // 2, (i % 2) * 2)
            layout.addWidget(bar, i // 2, (i % 2) * 2 + 1)
            self.bars.append(bar)

    def update_channels(self, channels: List[int]):
        for i, bar in enumerate(self.bars):
            value = int(channels[i]) if i < len(channels) else 0
            if value <= 0:
                bar.setValue(900)
                bar.setFormat("-")
            else:
                bar.setValue(clamp(value, 900, 2100))
                bar.setFormat(str(value))


class MapWidget(QGroupBox):
    def __init__(self):
        super().__init__("Live GPS Map")
        layout = QVBoxLayout(self)
        self.last_lat = None
        self.last_lon = None
        self.ready = False
        if WEBENGINE_AVAILABLE:
            self.view = QWebEngineView()
            self.view.setHtml(self._html())
            layout.addWidget(self.view)
            self.ready = True
        else:
            self.view = QLabel("PyQtWebEngine not installed.\nInstall: pip install PyQtWebEngine")
            self.view.setAlignment(Qt.AlignCenter)
            self.view.setStyleSheet("background: #0d141f; color: white; font-size: 18px;")
            layout.addWidget(self.view)

    def _html(self):
        # Uses online OSM tiles. The dashboard still runs without internet, but map tiles may not load.
        return """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<style>
html, body, #map { height: 100%; margin: 0; background: #0d141f; }
.status { position:absolute; z-index:999; left:10px; top:10px; background:rgba(0,0,0,.65); color:white; padding:8px; border-radius:8px; font-family:Arial; }
</style>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
</head>
<body>
<div id="map"></div>
<div class="status" id="status">Waiting for GPS...</div>
<script>
var map = L.map('map').setView([1.3521, 103.8198], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OSM' }).addTo(map);
var droneIcon = L.divIcon({
    className: 'drone-heading-icon',
    html: '<div id="drone-shape" style="width:34px;height:34px;transform:rotate(0deg);transform-origin:center center;">' +
          '<svg width="34" height="34" viewBox="0 0 34 34" xmlns="http://www.w3.org/2000/svg">' +
          '<path d="M17 2 L26 29 L17 24 L8 29 Z" fill="#1f6feb" stroke="white" stroke-width="2"/>' +
          '<circle cx="17" cy="17" r="4" fill="#ff3030" stroke="white" stroke-width="1"/>' +
          '</svg></div>',
    iconSize: [34, 34],
    iconAnchor: [17, 17]
});
var marker = L.marker([1.3521, 103.8198], {icon: droneIcon}).addTo(map);
var path = L.polyline([], {color: '#ff3030', weight: 3, opacity: 0.85}).addTo(map);

function updateDrone(lat, lon, hdg, sats, speed, alt) {
    if (lat === null || lon === null) { return; }
    marker.setLatLng([lat, lon]);
    var el = marker.getElement();
    if (el) {
        var shape = el.querySelector('#drone-shape');
        if (shape) {
            shape.style.transform = 'rotate(' + hdg.toFixed(1) + 'deg)';
        }
    }
    path.addLatLng([lat, lon]);
    map.panTo([lat, lon], {animate: false});
    document.getElementById('status').innerHTML =
        'Lat: ' + lat.toFixed(7) + '<br>Lon: ' + lon.toFixed(7) +
        '<br>HDG: ' + hdg.toFixed(0) + '° | Sats: ' + sats +
        '<br>Speed: ' + speed.toFixed(2) + ' m/s | Alt: ' + alt.toFixed(1) + ' m';
}
</script>
</body>
</html>
"""

    def update_position(self, lat: Optional[float], lon: Optional[float], heading: float, sats: int, speed: float, alt: float):
        if not WEBENGINE_AVAILABLE or lat is None or lon is None:
            return
        self.last_lat, self.last_lon = lat, lon
        js = f"updateDrone({lat:.8f}, {lon:.8f}, {float(heading):.2f}, {int(sats)}, {float(speed):.2f}, {float(alt):.2f});"
        self.view.page().runJavaScript(js)



class FuelStatusWidget(QGroupBox):
    def __init__(self):
        super().__init__("Live Flight Time / Fuel")
        layout = QGridLayout(self)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)
        self.labels = {}
        fields = [
            ("timer", "Flight timer"), ("live_i", "Live current"), ("live_v", "Voltage"),
            ("mah", "Consumed"), ("left", "Estimated left"), ("source", "Current source"),
        ]
        for i, (k, name) in enumerate(fields):
            title = QLabel(name + ":")
            title.setObjectName("telemetryTitle")
            layout.addWidget(title, i, 0)
            lab = QLabel("-")
            lab.setObjectName("dynamicValue")
            lab.setMinimumHeight(30)
            lab.setAlignment(Qt.AlignCenter)
            layout.addWidget(lab, i, 1)
            self.labels[k] = lab

    def update_values(self, timer_s: float, live_i: Optional[float], live_v: Optional[float], consumed_mah: float, left_s: Optional[float], source: str):
        self.labels["timer"].setText(seconds_to_hms(timer_s))
        self.labels["live_i"].setText(format_optional(live_i, "A"))
        self.labels["live_v"].setText(format_optional(live_v, "V"))
        self.labels["mah"].setText(f"{consumed_mah:.0f} mAh")
        self.labels["left"].setText(seconds_to_hms(left_s) if left_s is not None else "-")
        self.labels["source"].setText(source)


class FuelEstimatorDialog(QDialog):
    settings_applied = pyqtSignal(object)

    def __init__(self, settings: FuelSettings, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Flight Time / Fuel Estimator")
        self.resize(680, 620)
        self.settings = FuelSettings(**asdict(settings))
        self.fields: Dict[str, QDoubleSpinBox] = {}
        self.build_ui()
        self.load_settings(settings)

    def build_ui(self):
        root = QVBoxLayout(self)
        title = QLabel("Open Flight Time Fuel Estimator")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        root.addWidget(title)

        form_box = QGroupBox("Estimator Inputs")
        form = QFormLayout(form_box)

        def add_float(key, label, minimum, maximum, step, decimals=2):
            box = QDoubleSpinBox()
            box.setRange(minimum, maximum)
            box.setSingleStep(step)
            box.setDecimals(decimals)
            self.fields[key] = box
            form.addRow(label, box)

        add_float("cell_count", "Cell count / S", 1, 12, 1, 0)
        add_float("capacity_mah", "Battery capacity (mAh)", 100, 50000, 50, 0)
        add_float("usable_percent", "Usable capacity (%)", 1, 100, 1, 1)
        add_float("reserve_percent", "Reserve warning (%)", 0, 90, 1, 1)
        add_float("max_throttle_percent", "Max throttle allowed (%)", 1, 100, 1, 1)
        add_float("cruise_throttle_percent", "Cruise throttle estimate (%)", 1, 100, 1, 1)
        add_float("max_current_per_motor_a", "Measured / estimated max current per motor (A)", 0.1, 300, 0.5, 2)
        add_float("rotor_count", "Number of motors", 1, 12, 1, 0)
        add_float("avionics_current_a", "Avionics current estimate (A)", 0, 50, 0.1, 2)
        add_float("telemetry_current_bias_a", "Telemetry current correction / bias (A)", -50, 50, 0.1, 2)
        root.addWidget(form_box)

        self.use_current_check = QCheckBox("Use telemetry current first when available")
        self.use_current_check.setChecked(True)
        root.addWidget(self.use_current_check)

        btn_row = QHBoxLayout()
        calc = QPushButton("Calculate")
        calc.clicked.connect(self.calculate)
        apply_btn = QPushButton("Apply to Dashboard")
        apply_btn.clicked.connect(self.apply_settings)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        btn_row.addWidget(calc)
        btn_row.addWidget(apply_btn)
        btn_row.addWidget(close_btn)
        root.addLayout(btn_row)

        self.result = QTextEdit()
        self.result.setReadOnly(True)
        root.addWidget(self.result, stretch=1)

    def load_settings(self, settings: FuelSettings):
        data = asdict(settings)
        for k, v in data.items():
            if k in self.fields:
                self.fields[k].setValue(float(v))
        self.use_current_check.setChecked(settings.use_telemetry_current_first)
        self.calculate()

    def read_settings(self) -> FuelSettings:
        s = FuelSettings()
        for k, box in self.fields.items():
            val = box.value()
            if k in ("cell_count", "rotor_count"):
                setattr(s, k, int(val))
            else:
                setattr(s, k, float(val))
        s.use_telemetry_current_first = self.use_current_check.isChecked()
        return s

    def calculate(self):
        s = self.read_settings()
        r = estimate_planned_flight_time(s)
        self.result.setPlainText(
            f"Planned cruise current estimate: {r['cruise_current_a']:.2f} A\n"
            f"Max total current estimate: {r['max_total_current_a']:.2f} A\n"
            f"Usable capacity: {s.usable_mah:.0f} mAh\n"
            f"Reserve capacity: {s.reserve_mah:.0f} mAh\n\n"
            f"Estimated flight time using usable capacity: {r['usable_min']:.1f} minutes\n"
            f"Estimated flight time until reserve: {r['to_reserve_min']:.1f} minutes\n\n"
            "Formula used when live current is unavailable:\n"
            "I_total ≈ I_max_total × (Throttle / MaxThrottle)^3 + avionics correction\n\n"
            "Important: throttle percentage is not the same as true RPM. Use telemetry current "
            "or bench-test current whenever available."
        )

    def apply_settings(self):
        self.settings_applied.emit(self.read_settings())
        self.accept()





class FuelEstimatorPanel(QGroupBox):
    settings_applied = pyqtSignal(object)

    def __init__(self, settings: FuelSettings, parent_dashboard=None):
        super().__init__("Detailed Flight Time / Fuel Estimator")
        self.parent_dashboard = parent_dashboard
        self.fields: Dict[str, QDoubleSpinBox] = {}
        self.use_current_check = QCheckBox("Use telemetry current first when available")
        self.build_ui()
        self.load_settings(settings)

    def add_float(self, form: QFormLayout, key: str, label: str, minimum: float, maximum: float, step: float, decimals: int = 2):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        box.setKeyboardTracking(False)
        self.fields[key] = box
        form.addRow(label, box)

    def add_section(self, form: QFormLayout, name: str):
        lab = QLabel(name)
        lab.setObjectName("sectionTitle")
        lab.setFont(QFont("Arial", 10, QFont.Bold))
        form.addRow(lab)

    def build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(6)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumHeight(360)
        inner = QWidget()
        form = QFormLayout(inner)
        form.setLabelAlignment(Qt.AlignRight)
        form.setFormAlignment(Qt.AlignTop)
        form.setVerticalSpacing(7)

        self.add_section(form, "Environment")
        self.add_float(form, "altitude_m", "Altitude above sea level (m)", -100, 6000, 10, 1)
        self.add_float(form, "temp_c", "Temperature (°C)", -20, 60, 1, 1)
        self.add_float(form, "humidity_percent", "Relative humidity (%)", 0, 100, 1, 1)

        self.add_section(form, "Battery")
        self.add_float(form, "cell_count", "Battery cell count / S", 1, 12, 1, 0)
        self.add_float(form, "cell_voltage", "Average cell voltage (V)", 2.8, 4.35, 0.01, 2)
        self.add_float(form, "capacity_mah", "Battery capacity (mAh)", 100, 50000, 50, 0)
        self.add_float(form, "usable_percent", "Usable capacity (%)", 1, 100, 1, 1)
        self.add_float(form, "reserve_percent", "Reserve warning (%)", 0, 90, 1, 1)

        self.add_section(form, "Drone + Payload")
        self.add_float(form, "rotor_count", "Number of rotors", 1, 12, 1, 0)
        self.add_float(form, "drone_mass_kg", "Drone weight without payload (kg)", 0.05, 100, 0.05, 3)
        self.add_float(form, "payload_mass_kg", "Payload weight (kg)", 0.0, 100, 0.05, 3)
        self.add_float(form, "avionics_power_w", "Electronics / avionics power (W)", 0, 1000, 1, 1)
        self.add_float(form, "avionics_current_a", "Electronics current fallback (A)", 0, 100, 0.1, 2)

        self.add_section(form, "Motor + Propeller")
        self.add_float(form, "motor_kv", "Motor KV", 10, 10000, 10, 0)
        self.add_float(form, "prop_diameter_in", "Prop diameter (inch)", 1, 40, 0.1, 2)
        self.add_float(form, "prop_pitch_in", "Prop pitch (inch)", 0.5, 20, 0.1, 2)
        self.add_float(form, "blade_count", "Blade count", 1, 8, 1, 0)
        self.add_float(form, "loading_factor", "Loaded RPM factor λ", 0.20, 1.00, 0.01, 2)
        self.add_float(form, "figure_of_merit", "Hover figure of merit", 0.25, 0.95, 0.01, 2)
        self.add_float(form, "measured_max_thrust_g", "Measured max thrust per motor (g)", 0, 50000, 50, 0)
        self.add_float(form, "max_current_per_motor_a", "Measured / estimated max current per motor (A)", 0.1, 300, 0.5, 2)
        self.add_float(form, "esc_rating_a", "ESC rating per motor (A)", 1, 300, 1, 1)

        self.add_section(form, "Mission / Live Fuel")
        self.add_float(form, "max_throttle_percent", "Max throttle allowed (%)", 1, 100, 1, 1)
        self.add_float(form, "cruise_throttle_percent", "Cruise throttle estimate (%)", 1, 100, 1, 1)
        self.add_float(form, "telemetry_current_bias_a", "Telemetry current correction / bias (A)", -50, 50, 0.1, 2)

        self.add_section(form, "Calibration Benchmark")
        self.add_float(form, "ref_current_a", "Reference static current per motor (A)", 0.1, 300, 0.5, 2)
        self.add_float(form, "ref_kv", "Reference KV", 10, 10000, 10, 0)
        self.add_float(form, "ref_voltage", "Reference voltage (V)", 1, 100, 0.1, 2)
        self.add_float(form, "ref_diameter_in", "Reference prop diameter (inch)", 1, 40, 0.1, 2)
        self.add_float(form, "ref_pitch_in", "Reference prop pitch (inch)", 0.5, 20, 0.1, 2)
        self.add_float(form, "ref_blades", "Reference blade count", 1, 8, 1, 0)
        self.add_float(form, "ref_loading_factor", "Reference loaded RPM factor λ", 0.20, 1.00, 0.01, 2)
        self.add_float(form, "pitch_exponent", "Pitch-ratio exponent", 0, 3, 0.05, 2)
        self.add_float(form, "blade_exponent", "Blade-count exponent", 0, 2, 0.05, 2)

        scroll.setWidget(inner)
        root.addWidget(scroll)

        self.use_current_check.setChecked(True)
        root.addWidget(self.use_current_check)

        btns = QHBoxLayout()
        calc = QPushButton("Calculate")
        calc.clicked.connect(self.calculate)
        apply_btn = QPushButton("Apply to Dashboard")
        apply_btn.clicked.connect(self.apply_settings)
        close_btn = QPushButton("Hide Estimator")
        close_btn.clicked.connect(self.hide)
        btns.addWidget(calc)
        btns.addWidget(apply_btn)
        btns.addWidget(close_btn)
        root.addLayout(btns)

        self.result = QTextEdit()
        self.result.setReadOnly(True)
        self.result.setMinimumHeight(220)
        root.addWidget(self.result)

    def load_settings(self, settings: FuelSettings):
        data = asdict(settings)
        for k, v in data.items():
            if k in self.fields:
                self.fields[k].setValue(float(v))
        self.use_current_check.setChecked(settings.use_telemetry_current_first)
        self.calculate()

    def read_settings(self) -> FuelSettings:
        s = FuelSettings()
        for k, box in self.fields.items():
            val = box.value()
            if k in ("cell_count", "rotor_count", "blade_count", "ref_blades"):
                setattr(s, k, int(val))
            else:
                setattr(s, k, float(val))
        s.use_telemetry_current_first = self.use_current_check.isChecked()
        return s

    def calculate(self):
        s = self.read_settings()
        r = rich_fuel_estimate(s)
        usable_ah = s.usable_mah / 1000.0
        reserve_ah = s.reserve_mah / 1000.0
        cruise_i = r["cruise_current_a"]
        hover_i = r["total_hover_current_a"]
        max_i = r["max_total_current_a"]
        mah_per_min_cruise = cruise_i * 1000.0 / 60.0
        mah_per_min_hover = hover_i * 1000.0 / 60.0
        mah_per_min_max = max_i * 1000.0 / 60.0

        live_block = ""
        if self.parent_dashboard is not None:
            live_i, source = self.parent_dashboard.get_live_current()
            voltage = self.parent_dashboard.telemetry.get("battery_voltage_v")
            consumed = self.parent_dashboard.consumed_mah
            left_s = self.parent_dashboard.estimate_time_left_seconds()
            live_block = (
                f"\nLIVE DASHBOARD DATA\n"
                f"Live current source: {source}\n"
                f"Live current: {live_i:.2f} A\n"
                f"Telemetry voltage: {format_optional(voltage, 'V')}\n"
                f"Consumed fuel: {consumed:.0f} mAh\n"
                f"Estimated live time left: {seconds_to_hms(left_s)}\n"
            )

        warnings = "\n".join("⚠ " + w for w in r["warnings"]) if r["warnings"] else "No major warning from the estimator."

        self.result.setPlainText(
            "ENVIRONMENT / RPM\n"
            f"Air density: {r['rho']:.3f} kg/m³\n"
            f"Battery voltage used: {r['voltage']:.2f} V\n"
            f"No-load RPM: {r['no_load_rpm']:.0f} rpm\n"
            f"Estimated loaded RPM: {r['loaded_rpm']:.0f} rpm\n\n"
            "MASS / THRUST\n"
            f"Total mass: {r['total_mass_kg']:.3f} kg\n"
            f"Hover thrust required per motor: {r['hover_thrust_per_motor_g']:.0f} g\n"
            f"Estimated thrust margin: {r['thrust_margin_percent']:.1f} %\n"
            f"Hover throttle equivalent: {r['hover_throttle_equivalent'] * 100:.1f} %\n\n"
            "CURRENT / POWER\n"
            f"Estimated static current per motor: {r['static_current_motor']:.2f} A\n"
            f"Estimated static total current: {r['static_total_current']:.2f} A\n"
            f"Estimated static total power: {r['static_total_power']:.0f} W\n"
            f"Ideal/induced hover power estimate: {r['p_hover_induced']:.0f} W\n"
            f"Estimated total hover current: {hover_i:.2f} A\n"
            f"Cruise throttle current estimate: {cruise_i:.2f} A\n"
            f"Max total current estimate: {max_i:.2f} A\n\n"
            "BATTERY / FLIGHT TIME\n"
            f"Usable capacity: {s.usable_mah:.0f} mAh ({usable_ah:.3f} Ah)\n"
            f"Reserve capacity: {s.reserve_mah:.0f} mAh ({reserve_ah:.3f} Ah)\n"
            f"Hover burn rate: {mah_per_min_hover:.1f} mAh/min\n"
            f"Cruise burn rate: {mah_per_min_cruise:.1f} mAh/min\n"
            f"Max burn rate: {mah_per_min_max:.1f} mAh/min\n"
            f"Estimated hover flight time: {r['flight_min_hover']:.1f} min ({seconds_to_hms(r['flight_min_hover'] * 60)})\n"
            f"Estimated cruise flight time: {r['flight_min_cruise']:.1f} min ({seconds_to_hms(r['flight_min_cruise'] * 60)})\n"
            f"Hover time until reserve: {r['to_reserve_hover']:.1f} min\n"
            f"Cruise time until reserve: {r['to_reserve_cruise']:.1f} min\n"
            f"{live_block}\n"
            "FALLBACK LIVE FORMULA\n"
            "I_total ≈ I_motor_max × motor_count × (throttle / max_throttle)^3 + avionics_current + bias\n\n"
            f"{warnings}"
        )

    def update_live(self):
        if self.isVisible():
            self.calculate()

    def apply_settings(self):
        self.settings_applied.emit(self.read_settings())
        self.calculate()


class FPVPanel(QGroupBox):
    """FPV panel with a non-native in-window control drawer.

    The old overlay used native combo popups, which can flicker/black-flash on
    some Windows + QtWebEngine + fullscreen/GPU combinations. This panel uses
    only embedded widgets, push buttons, and line edits inside the same window.
    """

    def __init__(self, parent_dashboard):
        super().__init__("FPV Live Feed")
        self.parent_dashboard = parent_dashboard
        self.fps_value = 60
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        self.video_container = QFrame()
        self.video_container.setObjectName("videoContainer")
        self.video_container.setMinimumSize(980, 560)
        self.video_container.setStyleSheet("#videoContainer { background: black; border-radius: 10px; }")
        layout.addWidget(self.video_container, stretch=1)

        self.video = VideoLabel()
        self.video.setParent(self.video_container)
        self.video.setText("No FPV video")

        self.dots_btn = QPushButton("⋮", self.video_container)
        self.dots_btn.setFixedSize(34, 34)
        self.dots_btn.setStyleSheet(
            "QPushButton { background: rgba(0,0,0,115); color: white; border: 1px solid rgba(255,255,255,80); "
            "border-radius: 17px; font-size: 20px; } QPushButton:hover { background: rgba(40,80,130,170); }"
        )
        self.dots_btn.clicked.connect(self.toggle_overlay)

        self.overlay = QFrame(self.video_container)
        self.overlay.setObjectName("fpvDrawer")
        if getattr(parent_dashboard, "theme_mode", "dark") == "light":
            self.overlay.setStyleSheet("""
                #fpvDrawer { background: rgba(248, 251, 255, 245); border: 1px solid #7f95b3; border-radius: 14px; }
                QLabel { color: #0f172a; font-weight: bold; }
                QLineEdit { background: #ffffff; color: #0f172a; border: 1px solid #7f95b3; border-radius: 6px; padding: 6px; }
                QPushButton { background: #2563eb; color: white; border: none; border-radius: 8px; padding: 7px 10px; font-weight: bold; }
                QPushButton:hover { background: #3b82f6; }
                QCheckBox { color: #0f172a; font-weight: bold; }
            """)
        else:
            self.overlay.setStyleSheet("""
                #fpvDrawer { background: rgba(8, 14, 24, 236); border: 1px solid #49617f; border-radius: 14px; }
                QLabel { color: #eef5ff; }
                QLineEdit { background: #111d2e; color: white; border: 1px solid #48607f; border-radius: 6px; padding: 6px; }
                QPushButton { background: #1f6feb; color: white; border: none; border-radius: 8px; padding: 7px 10px; font-weight: bold; }
                QPushButton:hover { background: #388bfd; }
                QCheckBox { color: #eef5ff; }
            """)
        self.overlay.hide()
        self.build_overlay()

    def build_overlay(self):
        layout = QVBoxLayout(self.overlay)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(9)

        title_row = QHBoxLayout()
        title = QLabel("FPV Source / Recording")
        title.setFont(QFont("Arial", 13, QFont.Bold))
        done = QPushButton("Done")
        done.setFixedWidth(92)
        done.clicked.connect(self.overlay.hide)
        title_row.addWidget(title)
        title_row.addStretch(1)
        title_row.addWidget(done)
        layout.addLayout(title_row)

        dash = self.parent_dashboard

        dash.source_edit = QLineEdit("0")
        dash.source_edit.setPlaceholderText("0, 1, 2, /dev/video0, or stream URL")
        preset_row = QHBoxLayout()
        for src in ["0", "1", "2", "3"]:
            btn = QPushButton(src)
            btn.clicked.connect(lambda _=False, s=src: dash.source_edit.setText(s))
            preset_row.addWidget(btn)

        fps_row = QHBoxLayout()
        dash.fps_value = 60
        dash.fps_30_btn = QPushButton("30 FPS")
        dash.fps_60_btn = QPushButton("60 FPS")
        dash.fps_60_btn.setCheckable(True)
        dash.fps_30_btn.setCheckable(True)
        dash.fps_60_btn.setChecked(True)

        def set_fps(value):
            dash.fps_value = value
            dash.fps_30_btn.setChecked(value == 30)
            dash.fps_60_btn.setChecked(value == 60)
            if hasattr(dash, "video_status_label"):
                dash.video_status_label.setText(f"Selected FPS: {value}")

        dash.fps_30_btn.clicked.connect(lambda: set_fps(30))
        dash.fps_60_btn.clicked.connect(lambda: set_fps(60))
        fps_row.addWidget(dash.fps_30_btn)
        fps_row.addWidget(dash.fps_60_btn)

        dash.mjpg_check = QCheckBox("MJPG mode for USB capture card")
        dash.mjpg_check.setChecked(True)

        dash.folder_edit = QLineEdit(str(dash.save_folder))
        dash.folder_edit.setPlaceholderText("Save folder path")
        apply_folder_btn = QPushButton("Apply Save Folder")
        apply_folder_btn.clicked.connect(dash.choose_folder)

        buttons_1 = QHBoxLayout()
        scan_btn = QPushButton("Scan Sources")
        scan_btn.clicked.connect(dash.scan_sources)
        start_btn = QPushButton("Start Video")
        start_btn.clicked.connect(dash.start_video)
        stop_btn = QPushButton("Stop Video")
        stop_btn.clicked.connect(dash.stop_video)
        buttons_1.addWidget(scan_btn)
        buttons_1.addWidget(start_btn)
        buttons_1.addWidget(stop_btn)

        buttons_2 = QHBoxLayout()
        shot_btn = QPushButton("Screenshot")
        shot_btn.clicked.connect(dash.save_screenshot)
        rec_btn = QPushButton("Start Recording")
        rec_btn.clicked.connect(dash.start_recording)
        stop_rec_btn = QPushButton("Stop Recording")
        stop_rec_btn.clicked.connect(dash.stop_recording)
        buttons_2.addWidget(shot_btn)
        buttons_2.addWidget(rec_btn)
        buttons_2.addWidget(stop_rec_btn)

        layout.addWidget(QLabel("Video source"))
        layout.addWidget(dash.source_edit)
        layout.addLayout(preset_row)
        layout.addWidget(QLabel("Capture FPS"))
        layout.addLayout(fps_row)
        layout.addWidget(dash.mjpg_check)
        layout.addLayout(buttons_1)
        layout.addLayout(buttons_2)
        layout.addWidget(QLabel("Save folder"))
        layout.addWidget(dash.folder_edit)
        layout.addWidget(apply_folder_btn)

        dash.video_status_label = QLabel("Video: stopped")
        dash.video_info_label = QLabel("Input: -")
        dash.measured_fps_label = QLabel("Measured FPS: -")
        dash.record_label = QLabel("Recording: stopped")
        dash.folder_label = QLabel(f"Save folder: {dash.save_folder}")
        for lab in [dash.video_status_label, dash.measured_fps_label, dash.video_info_label, dash.record_label, dash.folder_label]:
            lab.setWordWrap(True)
            layout.addWidget(lab)

        layout.addStretch(1)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.video.setGeometry(self.video_container.rect())
        margin = 12
        self.dots_btn.move(self.video_container.width() - self.dots_btn.width() - margin, margin)
        drawer_w = min(520, max(420, int(self.video_container.width() * 0.36)))
        self.overlay.setGeometry(
            self.video_container.width() - drawer_w - 18,
            18,
            drawer_w,
            self.video_container.height() - 36
        )

    def toggle_overlay(self):
        self.overlay.setVisible(not self.overlay.isVisible())
        self.overlay.raise_()
        self.dots_btn.raise_()


# ----------------------------- Main dashboard -----------------------------

class DroneDashboard(QWidget):
    def __init__(self, theme_mode: str = "dark", display_mode: str = "borderless"):
        super().__init__()
        self.theme_mode = theme_mode
        self.display_mode = display_mode
        self.setWindowTitle("Drone Dashboard v8 | Rebalanced Real-Time Telemetry Layout")
        self.resize(1920, 1080)
        if self.display_mode == "borderless":
            self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)

        self.save_folder = Path.home() / "Videos" / "DroneDashboard"
        self.save_folder.mkdir(parents=True, exist_ok=True)

        self.capture_thread: Optional[CaptureThread] = None
        self.telemetry_thread: Optional[TelemetryThread] = None
        self.last_frame: Optional[np.ndarray] = None
        self.recording = False
        self.writer = None
        self.record_path = None
        self.record_start = None

        self.telemetry: Dict[str, Any] = TelemetryThread.default_data()
        self.last_map_update_time = 0.0
        self.prev_armed = False
        self.arm_start_time: Optional[float] = None
        self.flight_timer_running = False

        self.fuel_settings = FuelSettings()
        self.consumed_mah = 0.0
        self.last_fuel_update_time: Optional[float] = None
        self.last_live_current_a: Optional[float] = None
        self.last_current_source = "-"

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_ui_loop)
        self.ui_timer.start(20)  # 50 Hz UI refresh for telemetry panels

        self.build_ui()
        self.apply_style()

    def build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 10)
        root.setSpacing(8)

        # ---------------- Top connection bar ----------------
        top = QHBoxLayout()
        title = QLabel("DRONE DASHBOARD v8")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        self.connection_label = QLabel("Telemetry: disconnected")
        self.connection_label.setStyleSheet("font-weight: bold; color: #ffdddd;")

        # Custom in-window MAVLink selector.
        # This avoids native QComboBox popup black-screen flashes on some Windows/GPU/fullscreen setups.
        self.mav_options = []
        self.mav_conn_edit = QLineEdit("COM7")
        self.mav_conn_edit.setFixedWidth(300)
        self.mav_conn_edit.setToolTip(
            "Select a detected COM port, or type manually.\\n"
            "Examples: COM7, udp:127.0.0.1:14550, tcp:127.0.0.1:5760, /dev/ttyACM0"
        )
        self.mav_dropdown_btn = QPushButton("▾")
        self.mav_dropdown_btn.setFixedWidth(34)
        self.mav_dropdown_btn.setToolTip("Open detected MAVLink / COM-port list")
        self.mav_dropdown_btn.clicked.connect(self.show_mavlink_port_selector)

        self.mav_port_popup = QFrame(self)
        self.mav_port_popup.setFrameShape(QFrame.StyledPanel)
        self.mav_port_popup.setWindowFlags(Qt.Widget)
        self.mav_port_popup.setStyleSheet(
            "QFrame { background: #101722; border: 1px solid #3d4b63; border-radius: 8px; }"
            "QPushButton { text-align: left; padding: 7px 10px; background: #182235; color: #eef4ff; border: 1px solid #2f3f58; border-radius: 6px; }"
            "QPushButton:hover { background: #263854; }"
            "QLabel { color: #d7e4ff; padding: 8px; }"
        )
        self.mav_port_popup_layout = QVBoxLayout(self.mav_port_popup)
        self.mav_port_popup_layout.setContentsMargins(8, 8, 8, 8)
        self.mav_port_popup_layout.setSpacing(5)
        self.mav_port_popup.hide()

        refresh_ports_btn = QPushButton("↻ Ports")
        refresh_ports_btn.setToolTip("Refresh COM / serial-port list")
        refresh_ports_btn.clicked.connect(self.refresh_mavlink_ports)
        self.mav_baud_edit = QLineEdit("57600")
        self.mav_baud_edit.setFixedWidth(80)

        connect_btn = QPushButton("Connect MAVLink")
        connect_btn.clicked.connect(self.start_telemetry)
        disconnect_btn = QPushButton("Disconnect")
        disconnect_btn.clicked.connect(self.stop_telemetry)
        border_btn = QPushButton("Fullscreen")
        border_btn.clicked.connect(self.toggle_fullscreen)
        close_btn = QPushButton("✕")
        close_btn.setFixedWidth(44)
        close_btn.clicked.connect(self.close)

        top.addWidget(title)
        top.addSpacing(20)
        top.addWidget(self.connection_label)
        top.addStretch(1)
        top.addWidget(QLabel("MAVLink:"))
        top.addWidget(self.mav_conn_edit)
        top.addWidget(self.mav_dropdown_btn)
        top.addWidget(refresh_ports_btn)
        top.addWidget(QLabel("Baud:"))
        top.addWidget(self.mav_baud_edit)
        top.addWidget(connect_btn)
        top.addWidget(disconnect_btn)
        top.addWidget(border_btn)
        top.addWidget(close_btn)
        root.addLayout(top)

        self.refresh_mavlink_ports()

        # ---------------- Main 4K layout ----------------
        main_split = QSplitter(Qt.Horizontal)
        root.addWidget(main_split, stretch=1)

        # LEFT: large FPV feed, then gyro + telemetry under it
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 4, 0)
        left_layout.setSpacing(8)

        self.fpv_panel = FPVPanel(self)
        left_layout.addWidget(self.fpv_panel, stretch=7)

        lower_flight = QHBoxLayout()
        lower_flight.setSpacing(8)

        gyro_box = QGroupBox("3D AHRS / Gyro Ball")
        gyro_layout = QVBoxLayout(gyro_box)
        self.horizon = ArtificialHorizon()
        self.horizon.setMinimumSize(520, 360)
        self.horizon.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        gyro_layout.addWidget(self.horizon)

        self.telemetry_grid = TelemetryLabelGrid()
        self.telemetry_grid.setMinimumWidth(560)

        lower_flight.addWidget(gyro_box, stretch=4)
        lower_flight.addWidget(self.telemetry_grid, stretch=5)
        left_layout.addLayout(lower_flight, stretch=4)

        main_split.addWidget(left)

        # RIGHT: map, timer, then RC + fuel
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 0, 0)
        right_layout.setSpacing(8)

        self.map_widget = MapWidget()
        right_layout.addWidget(self.map_widget, stretch=6)

        timer_box = QGroupBox("Arming / Auto Flight Timer")
        timer_layout = QVBoxLayout(timer_box)

        self.big_timer = QLabel("00:00:00")
        self.big_timer.setAlignment(Qt.AlignCenter)
        self.big_timer.setFont(QFont("Arial", 42, QFont.Bold))
        timer_layout.addWidget(self.big_timer)

        btn_row = QHBoxLayout()
        arm_btn = QPushButton("ARMING STARTED")
        arm_btn.clicked.connect(self.manual_start_timer)
        stop_timer = QPushButton("Stop Timer")
        stop_timer.clicked.connect(self.stop_flight_timer)
        reset_timer = QPushButton("Reset Timer")
        reset_timer.clicked.connect(self.reset_flight_timer)
        btn_row.addWidget(arm_btn)
        btn_row.addWidget(stop_timer)
        btn_row.addWidget(reset_timer)
        timer_layout.addLayout(btn_row)

        estimator_btn = QPushButton("Show / Hide Detailed Fuel Estimator")
        estimator_btn.clicked.connect(self.open_fuel_estimator)
        reset_fuel_btn = QPushButton("Reset Fuel Used")
        reset_fuel_btn.clicked.connect(self.reset_fuel)
        timer_layout.addWidget(estimator_btn)
        timer_layout.addWidget(reset_fuel_btn)

        right_layout.addWidget(timer_box, stretch=2)

        self.fuel_estimator_panel = FuelEstimatorPanel(self.fuel_settings, self)
        self.fuel_estimator_panel.settings_applied.connect(self.apply_fuel_settings)
        self.fuel_estimator_panel.hide()
        right_layout.addWidget(self.fuel_estimator_panel, stretch=5)

        bottom_right = QHBoxLayout()
        bottom_right.setSpacing(8)

        self.rc_widget = RCChannelsWidget()
        self.fuel_widget = FuelStatusWidget()

        bottom_right.addWidget(self.rc_widget, stretch=3)
        bottom_right.addWidget(self.fuel_widget, stretch=2)
        right_layout.addLayout(bottom_right, stretch=3)

        main_split.addWidget(right)
        main_split.setSizes([1320, 600])

        self.folder_label.setText(f"Save folder: {self.save_folder}")



    def apply_style(self):
        if getattr(self, "theme_mode", "dark") == "light":
            self.setStyleSheet("""
                QWidget { background: #eef4fb; color: #0f172a; font-family: Arial; font-size: 12px; }
                QGroupBox { border: 1px solid #a9bad1; border-radius: 10px; margin-top: 12px; padding: 8px; font-weight: bold; background: #f8fbff; color: #0f172a; }
                QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #1e3a5f; background: #f8fbff; }
                QPushButton { background: #2563eb; color: white; border: none; border-radius: 8px; padding: 8px 12px; font-weight: bold; }
                QPushButton:hover { background: #3b82f6; }
                QPushButton:pressed { background: #1d4ed8; }
                QLineEdit, QComboBox, QTextEdit, QDoubleSpinBox, QSpinBox { background: white; color: #0f172a; border: 1px solid #8ca4c0; border-radius: 6px; padding: 5px; }
                QScrollArea { background: #f8fbff; border: 1px solid #d1dbe8; }
                QProgressBar { background: #dbe7f5; border: 1px solid #8ca4c0; border-radius: 4px; text-align: center; color: #d00000; font-weight: bold; }
                QProgressBar::chunk { background: #8db7ff; border-radius: 4px; }
                QCheckBox, QRadioButton { color: #0f172a; }
                QLabel#dynamicValue { color: #d00000; background: #ffffff; border: 1px solid #d3deea; border-radius: 4px; padding: 5px; font-weight: bold; }
                QLabel#telemetryTitle { color: #0f172a; font-weight: bold; }
                QLabel#sectionTitle { color: #1e3a5f; padding-top: 8px; }
            """)
        else:
            self.setStyleSheet("""
                QWidget { background: #0d141f; color: #f2f6ff; font-family: Arial; font-size: 12px; }
                QGroupBox { border: 1px solid #2e4058; border-radius: 10px; margin-top: 12px; padding: 8px; font-weight: bold; }
                QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
                QPushButton { background: #1f6feb; color: white; border: none; border-radius: 8px; padding: 8px 12px; font-weight: bold; }
                QPushButton:hover { background: #388bfd; }
                QPushButton:pressed { background: #0b4aa2; }
                QLineEdit, QComboBox, QTextEdit, QDoubleSpinBox, QSpinBox { background: #111d2e; color: white; border: 1px solid #344966; border-radius: 6px; padding: 5px; }
                QScrollArea { background: #0d141f; border: 1px solid #26364d; }
                QProgressBar { background: #101826; border: 1px solid #344966; border-radius: 4px; text-align: center; color: #ff4040; font-weight: bold; }
                QProgressBar::chunk { background: #1f6feb; border-radius: 4px; }
                QCheckBox, QRadioButton { color: #f2f6ff; }
                QLabel#dynamicValue { color: #ff4040; background: #101a2a; border: 1px solid #4a5c78; border-radius: 4px; padding: 5px; font-weight: bold; }
                QLabel#telemetryTitle { color: #d7e4ff; font-weight: bold; }
                QLabel#sectionTitle { color: #d7e4ff; padding-top: 8px; }
            """)

    def scan_sources(self):
        found = []
        for idx in range(10):
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(idx)
            if cap.isOpened():
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    ok, _ = cap.read()
                    if ok:
                        found.append(str(idx))
                finally:
                    cap.release()
        if found:
            if hasattr(self, "source_edit"):
                self.source_edit.setText(found[0])
            msg = "Detected sources: " + ", ".join(found)
        else:
            msg = "No confirmed source. Type 0, 1, 2, 3, /dev/video0, or a stream URL manually."
        if hasattr(self, "video_status_label"):
            self.video_status_label.setText(msg)


    def start_video(self):
        self.stop_video()
        source = self.source_edit.text().strip() if hasattr(self, "source_edit") else "0"
        fps = int(getattr(self, "fps_value", 60))
        use_mjpg = self.mjpg_check.isChecked() if hasattr(self, "mjpg_check") else True
        self.capture_thread = CaptureThread(source, fps_target=fps, use_mjpg=use_mjpg)
        self.capture_thread.frame_ready.connect(self.on_frame)
        self.capture_thread.status.connect(self.video_status_label.setText)
        self.capture_thread.fps_measured.connect(lambda f: self.measured_fps_label.setText(f"Measured FPS: {f:.1f}"))
        self.capture_thread.opened_info.connect(self.video_info_label.setText)
        self.capture_thread.start()

    def stop_video(self):
        self.stop_recording()
        if self.capture_thread is not None:
            self.capture_thread.stop()
            self.capture_thread = None
        if hasattr(self, "video_status_label"):
            self.video_status_label.setText("Video: stopped")

    def on_frame(self, frame: np.ndarray):
        self.last_frame = frame
        self.fpv_panel.video.set_frame(frame)
        if self.recording and self.writer is not None:
            try:
                self.writer.write(frame)
            except Exception as e:
                self.record_label.setText(f"Recording write error: {e}")
                self.stop_recording()


    def choose_folder(self):
        # Non-native folder selection to avoid full-screen/GPU popup flicker.
        # Type or paste the folder path in the FPV drawer, then press Apply Save Folder.
        folder = self.folder_edit.text().strip() if hasattr(self, "folder_edit") else str(self.save_folder)
        if not folder:
            folder = str(self.save_folder)
        self.save_folder = Path(folder).expanduser()
        self.save_folder.mkdir(parents=True, exist_ok=True)
        if hasattr(self, "folder_edit"):
            self.folder_edit.setText(str(self.save_folder))
        if hasattr(self, "folder_label"):
            self.folder_label.setText(f"Save folder: {self.save_folder}")


    def save_screenshot(self):
        if self.last_frame is None:
            if hasattr(self, "video_status_label"):
                self.video_status_label.setText("Screenshot failed: start video first.")
            return
        path = self.save_folder / f"drone_screenshot_{time.strftime('%Y%m%d_%H%M%S')}.png"
        cv2.imwrite(str(path), self.last_frame)
        if hasattr(self, "record_label"):
            self.record_label.setText(f"Screenshot saved -> {path}")


    def start_recording(self):
        if self.last_frame is None:
            if hasattr(self, "record_label"):
                self.record_label.setText("Recording failed: start video first.")
            return
        if self.recording:
            return
        h, w = self.last_frame.shape[:2]
        fps = int(getattr(self, "fps_value", 60))
        self.record_path = self.save_folder / f"drone_recording_{time.strftime('%Y%m%d_%H%M%S')}.avi"
        self.writer = cv2.VideoWriter(str(self.record_path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
        if not self.writer.isOpened():
            self.writer = None
            if hasattr(self, "record_label"):
                self.record_label.setText("Recording error: could not open video writer.")
            return
        self.recording = True
        self.record_start = time.time()
        self.record_label.setText(f"Recording: started -> {self.record_path}")

    def stop_recording(self):
        if self.recording:
            self.recording = False
            if self.writer is not None:
                self.writer.release()
                self.writer = None
            self.record_label.setText(f"Recording: saved -> {self.record_path}")
        elif self.writer is not None:
            self.writer.release()
            self.writer = None

    # ----------------------------- Telemetry methods -----------------------------

    def refresh_mavlink_ports(self):
        """Refresh the custom in-window MAVLink port selector."""
        if not hasattr(self, "mav_conn_edit"):
            return
        current = clean_connection_string(self.mav_conn_edit.text())
        self.mav_options = discover_mavlink_connection_options()
        if (not current or current == "COM7") and self.mav_options:
            # Prefer a real detected COM port over UDP defaults on Windows.
            detected = [x for x in self.mav_options if x.upper().startswith("COM")]
            self.mav_conn_edit.setText(clean_connection_string(detected[0] if detected else self.mav_options[0]))
        self._rebuild_mavlink_popup()

    def _rebuild_mavlink_popup(self):
        if not hasattr(self, "mav_port_popup_layout"):
            return
        while self.mav_port_popup_layout.count():
            item = self.mav_port_popup_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        for opt in self.mav_options[:18]:
            btn = QPushButton(opt)
            btn.clicked.connect(lambda _=False, val=opt: self.select_mavlink_port(val))
            self.mav_port_popup_layout.addWidget(btn)
        if not self.mav_options:
            lab = QLabel("No ports detected. Type manually, e.g. COM7 or udp:127.0.0.1:14550")
            lab.setWordWrap(True)
            self.mav_port_popup_layout.addWidget(lab)

    def select_mavlink_port(self, value: str):
        self.mav_conn_edit.setText(clean_connection_string(value))
        self.mav_port_popup.hide()

    def show_mavlink_port_selector(self):
        self.refresh_mavlink_ports()
        pos = self.mav_conn_edit.mapTo(self, self.mav_conn_edit.rect().bottomLeft())
        width = self.mav_conn_edit.width() + self.mav_dropdown_btn.width() + 8
        self.mav_port_popup.setGeometry(pos.x(), pos.y() + 4, width, min(430, 42 + 34 * max(1, len(self.mav_options[:10]))))
        self.mav_port_popup.raise_()
        self.mav_port_popup.show()


    def start_telemetry(self):
        if not PYMAVLINK_AVAILABLE:
            QMessageBox.critical(self, "Missing pymavlink", "Install pymavlink first:\n\npip install pymavlink")
            return

        # Avoid stacking two telemetry threads. This was one cause of reconnect failure.
        self.stop_telemetry()

        conn = clean_connection_string(self.mav_conn_edit.text())
        baud = int(safe_float(self.mav_baud_edit.text(), 57600))

        self.connection_label.setText(
            f"Telemetry: opening connection... | gyro direct COM-rate render | baud {baud}"
        )
        self.telemetry_thread = TelemetryThread(conn, baud=baud)
        self.telemetry_thread.telemetry.connect(self.on_telemetry)
        self.telemetry_thread.attitude.connect(self.horizon.update_attitude)
        self.telemetry_thread.status.connect(self.connection_label.setText)
        self.telemetry_thread.finished.connect(lambda: setattr(self, "telemetry_thread", None))
        self.telemetry_thread.start()


    def stop_telemetry(self):
        if self.telemetry_thread is not None:
            thread = self.telemetry_thread
            self.telemetry_thread = None
            try:
                thread.telemetry.disconnect(self.on_telemetry)
            except Exception:
                pass
            try:
                thread.status.disconnect(self.connection_label.setText)
            except Exception:
                pass
            thread.stop()
        self.connection_label.setText("Telemetry: disconnected")


    def on_telemetry(self, data: Dict[str, Any]):
        self.telemetry.update(data)

        # Gyroball attitude is updated by TelemetryThread.attitude directly.
        # Keep this slower path for labels, battery, GPS, RC and map data only.
        armed = bool(self.telemetry.get("armed"))
        if armed and not self.prev_armed:
            self.auto_start_timer_from_arm()
            self.reset_fuel()
        elif not armed and self.prev_armed:
            self.stop_flight_timer()
        self.prev_armed = armed

    def manual_start_timer(self):
        self.arm_start_time = time.time()
        self.flight_timer_running = True
        self.reset_fuel()

    def auto_start_timer_from_arm(self):
        self.arm_start_time = time.time()
        self.flight_timer_running = True

    def stop_flight_timer(self):
        self.flight_timer_running = False

    def reset_flight_timer(self):
        self.flight_timer_running = False
        self.arm_start_time = None
        self.big_timer.setText("00:00:00")

    def reset_fuel(self):
        self.consumed_mah = 0.0
        self.last_fuel_update_time = time.time()


    def open_fuel_estimator(self):
        if hasattr(self, "fuel_estimator_panel"):
            self.fuel_estimator_panel.setVisible(not self.fuel_estimator_panel.isVisible())
            if self.fuel_estimator_panel.isVisible():
                self.fuel_estimator_panel.load_settings(self.fuel_settings)
                self.fuel_estimator_panel.calculate()


    def apply_fuel_settings(self, settings: FuelSettings):
        self.fuel_settings = settings
        self.reset_fuel()
        if hasattr(self, "fuel_estimator_panel"):
            self.fuel_estimator_panel.load_settings(settings)

    def get_live_current(self) -> (Optional[float], str):
        current = self.telemetry.get("battery_current_a")
        throttle = safe_float(self.telemetry.get("throttle_percent"), 0.0)
        if self.fuel_settings.use_telemetry_current_first and current is not None and current > 0.1:
            return max(0.0, float(current) + self.fuel_settings.telemetry_current_bias_a), "telemetry current"
        return estimate_current_from_throttle(self.fuel_settings, throttle), "throttle cubic estimate"

    def update_fuel_integration(self):
        now = time.time()
        if self.last_fuel_update_time is None:
            self.last_fuel_update_time = now
            return
        dt = now - self.last_fuel_update_time
        self.last_fuel_update_time = now
        if not self.flight_timer_running:
            return
        current, source = self.get_live_current()
        if current is None:
            return
        self.last_live_current_a = current
        self.last_current_source = source
        self.consumed_mah += current * dt * 1000.0 / 3600.0

    def estimate_time_left_seconds(self) -> Optional[float]:
        settings = self.fuel_settings
        current, _ = self.get_live_current()
        if current is None or current <= 0.1:
            return None

        usable_mah = settings.usable_mah
        remaining_mah = max(0.0, usable_mah - self.consumed_mah)

        # If MAVLink battery_remaining is available, use it as an additional cap.
        batt_rem = self.telemetry.get("battery_remaining_percent")
        if batt_rem is not None and batt_rem >= 0:
            remaining_from_fc = settings.capacity_mah * safe_float(batt_rem) / 100.0
            remaining_mah = min(remaining_mah, remaining_from_fc)
        else:
            # Voltage-based fallback can reduce overly optimistic estimates under low voltage.
            voltage = self.telemetry.get("battery_voltage_v")
            if voltage is not None and settings.cell_count > 0:
                soc_v = lipo_soc_from_cell_voltage(float(voltage) / settings.cell_count)
                remaining_from_v = settings.capacity_mah * soc_v / 100.0
                remaining_mah = min(remaining_mah, remaining_from_v)

        reserve_mah = settings.reserve_mah
        safe_remaining_mah = max(0.0, remaining_mah - reserve_mah)
        return (safe_remaining_mah / 1000.0) / current * 3600.0

    # ----------------------------- UI loop -----------------------------

    def update_ui_loop(self):
        d = self.telemetry
        # Do not drive the gyroball from this fixed UI timer; it now follows live attitude packets directly.
        self.telemetry_grid.update_data(d)
        self.rc_widget.update_channels(d.get("rc_channels", [0] * 14))

        lat = d.get("lat")
        lon = d.get("lon")
        map_alt = d.get("rel_alt_m") if d.get("rel_alt_m") is not None else d.get("gps_alt_m")
        now_for_map = time.time()
        if now_for_map - self.last_map_update_time >= 0.25:
            self.last_map_update_time = now_for_map
            self.map_widget.update_position(
                lat, lon,
                safe_float(d.get("heading_deg")),
                int(d.get("gps_sats", 0)),
                safe_float(d.get("groundspeed_m_s")),
                safe_float(map_alt),
            )

        if self.flight_timer_running and self.arm_start_time is not None:
            elapsed = time.time() - self.arm_start_time
            self.big_timer.setText(seconds_to_hms(elapsed))
        elif self.arm_start_time is None:
            elapsed = 0.0
        else:
            elapsed = time.time() - self.arm_start_time

        self.update_fuel_integration()
        left_s = self.estimate_time_left_seconds()
        voltage = d.get("battery_voltage_v")
        live_i, source = self.get_live_current()
        self.fuel_widget.update_values(
            elapsed,
            live_i,
            voltage,
            self.consumed_mah,
            left_s,
            source,
        )
        if hasattr(self, "fuel_estimator_panel") and self.fuel_estimator_panel.isVisible():
            self.fuel_estimator_panel.update_live()

        if self.recording and self.record_start is not None:
            rec_elapsed = int(time.time() - self.record_start)
            self.record_label.setText(f"Recording: {rec_elapsed}s -> {self.record_path}")

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.showNormal()
        elif event.key() == Qt.Key_F11:
            self.toggle_fullscreen()
        else:
            super().keyPressEvent(event)


    def mousePressEvent(self, event):
        if hasattr(self, "mav_port_popup") and self.mav_port_popup.isVisible():
            gp = event.pos()
            if (not self.mav_port_popup.geometry().contains(gp)) and (not self.mav_dropdown_btn.geometry().contains(gp)):
                self.mav_port_popup.hide()
        super().mousePressEvent(event)

    def closeEvent(self, event):
        self.stop_video()
        self.stop_telemetry()
        event.accept()




class BootLauncher(QWidget):
    """Small boot screen to choose theme/window mode and show loading progress."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Dashboard Boot")
        self.resize(680, 420)
        self.dashboard = None
        self.boot_step = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 24)
        root.setSpacing(14)

        title = QLabel("Drone Dashboard Startup")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        subtitle = QLabel("Choose display mode before loading MAVLink, FPV, and GPS map modules.")
        subtitle.setAlignment(Qt.AlignCenter)
        root.addWidget(subtitle)

        theme_box = QGroupBox("Theme")
        theme_row = QHBoxLayout(theme_box)
        self.dark_radio = QRadioButton("Dark mode")
        self.light_radio = QRadioButton("Light mode")
        self.dark_radio.setChecked(True)
        theme_row.addWidget(self.dark_radio)
        theme_row.addWidget(self.light_radio)
        root.addWidget(theme_box)

        mode_box = QGroupBox("Window mode")
        mode_row = QHBoxLayout(mode_box)
        self.borderless_radio = QRadioButton("Fullscreen borderless")
        self.windowed_radio = QRadioButton("Windowed mode")
        self.borderless_radio.setChecked(True)
        mode_row.addWidget(self.borderless_radio)
        mode_row.addWidget(self.windowed_radio)
        root.addWidget(mode_box)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        root.addWidget(self.progress)

        self.status = QLabel("Ready.")
        self.status.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status)

        self.boot_btn = QPushButton("Boot Dashboard")
        self.boot_btn.clicked.connect(self.start_boot)
        root.addWidget(self.boot_btn)

        self.setStyleSheet("""
            QWidget { background: #0d141f; color: #f2f6ff; font-family: Arial; font-size: 13px; }
            QGroupBox { border: 1px solid #2e4058; border-radius: 10px; margin-top: 10px; padding: 10px; font-weight: bold; }
            QPushButton { background: #1f6feb; color: white; border: none; border-radius: 8px; padding: 10px 14px; font-weight: bold; }
            QPushButton:hover { background: #388bfd; }
            QProgressBar { background: #101826; border: 1px solid #344966; border-radius: 5px; text-align: center; color: white; height: 24px; }
            QProgressBar::chunk { background: #1f6feb; border-radius: 5px; }
        """)

    def start_boot(self):
        self.boot_btn.setEnabled(False)
        self.progress.setValue(5)
        self.status.setText("Preparing Qt / OpenGL / dashboard modules...")
        self.boot_step = 0
        QTimer.singleShot(150, self.next_boot_step)

    def next_boot_step(self):
        theme = "light" if self.light_radio.isChecked() else "dark"
        display = "windowed" if self.windowed_radio.isChecked() else "borderless"

        if self.boot_step == 0:
            self.progress.setValue(20)
            self.status.setText("Loading dashboard layout...")
            self.boot_step += 1
            QTimer.singleShot(120, self.next_boot_step)
        elif self.boot_step == 1:
            self.progress.setValue(40)
            self.status.setText("Loading GPS map during startup...")
            QApplication.processEvents()
            self.dashboard = DroneDashboard(theme_mode=theme, display_mode=display)
            self.boot_step += 1
            QTimer.singleShot(120, self.next_boot_step)
        elif self.boot_step == 2:
            self.progress.setValue(75)
            self.status.setText("Initialising FPV controls, gyro, telemetry tables, and fuel estimator...")
            QApplication.processEvents()
            self.boot_step += 1
            QTimer.singleShot(180, self.next_boot_step)
        else:
            self.progress.setValue(100)
            self.status.setText("Ready.")
            QApplication.processEvents()
            if display == "borderless":
                self.dashboard.showFullScreen()
            else:
                self.dashboard.show()
            self.close()



def main():
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
