#!/usr/bin/env python3
"""MR60BHA2 / HLK-LD6002 (Andar ADT6101P) – Advanced Radar Console.

A full-featured live dashboard with manual controls for all meaningful
parameters and an automatic mode.

Data source:
  * COM port of the XIAO USB bridge (TinyFrame, 0x0A32/0x0A34 = complex
    range-Doppler raw data, 8/16/32 rows x 32 Doppler bins)
  * --demo    synthetic raw data (no radar required)
  * --replay  recorded NPZ file

The dashboard is intentionally split into three layers:
  * LIVE  – commands the radar understands at runtime (zones, raw/track,
            echo, flag, version, custom TinyFrames)
  * SOFT  – PC-side processing/visualization (calibration, contrast,
            clutter filter, detection, auto mode, recording)
  * FW    – only possible via firmware patch (TIA/VGA/PA/HPF, window size,
            UART baud rate). These values only generate an image; it is
            never flashed automatically.

Start:
  python advanced_dashboard.py --port COM3
  python advanced_dashboard.py --demo
  python advanced_dashboard.py --replay raw_session.npz
  python advanced_dashboard.py --selftest

Browser: http://127.0.0.1:8770
"""

from __future__ import annotations

import argparse
import collections
import copy
import csv
import hashlib
import json
import math
import os
import queue
import re
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import numpy as np

try:
    import serial
except Exception:  # pragma: no cover - pyserial should be installed; demo must still work
    serial = None

HERE = Path(__file__).resolve().parent
# Keep the vendored protocol helper importable even in isolated mode (python -I).
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import radar_tf as d  # noqa: E402  (TFParser, build_frame, decode, COMMANDS)
from firmware_vault import FirmwareVault, inspect_image  # noqa: E402

HTML_PATH = HERE / "advanced_dashboard.html"
FLASHER_PATH = HERE / "webflasher.html"
CONFIG_PATH = HERE / "advanced_dashboard_config.json"
REC_DIR = HERE / "recordings"
SNAP_DIR = HERE / "snapshots"
TUNING_DIR = HERE / "tuning" / "dashboard"
VAULT_DIR = HERE / "firmware_vault"
FIRMWARE_OUT_DIR = HERE / "firmware_out"
MAX_FIRMWARE_UPLOAD = 8 * 1024 * 1024 + 64 * 1024
RAW_TYPES = (0x0A32, 0x0A34)
BLOCKED_TYPES = {0x3000, 0x4000}
APP_VERSION = "1.0.0"

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DEFAULT_CONFIG: dict[str, object] = {
    "config_version": 1,
    # Connection
    "conn_port": "COM3",
    "conn_baud": 115200,
    "conn_auto_connect": True,
    "conn_auto_reconnect": True,
    "conn_reconnect_s": 2.0,
    "conn_on_connect_version": True,
    "conn_on_connect_mode": "raw",       # raw | track | keep
    "conn_on_connect_zone_read": True,
    # Radar live commands (stored arguments only; sent via button)
    "radar_zone_min": 0,
    "radar_zone_max": 1500,
    "radar_zone_width": 1500,
    "radar_echo_value": 12345678,
    "radar_flag_value": 1,
    "cmd_custom_type": "0x0A20",
    "cmd_custom_hex": "",
    # Raw data / DSP
    "dsp_rows": 0,                       # 0 = derive from payload
    "dsp_range_start_m": 1.0,
    "dsp_range_step_m": 0.125,
    "dsp_doppler_bin_mps": 0.05,
    "dsp_doppler_zero_bin": 16,
    "dsp_iq_view": "magnitude",          # magnitude | real | imag | phase | power
    "dsp_clutter_remove": False,
    "dsp_clutter_alpha": 0.02,
    "dsp_smooth_alpha": 0.0,
    "dsp_display_gain_db": 0.0,
    "dsp_auto_contrast": True,
    "dsp_contrast_low_pct": 5.0,
    "dsp_contrast_high_pct": 99.5,
    "dsp_db_min": 38.0,
    "dsp_db_max": 58.0,
    "dsp_colormap": "magma",
    "dsp_interp": True,
    "dsp_show_grid": True,
    "dsp_stream_hz": 10,
    "dsp_waterfall_rows": 200,
    "dsp_waterfall_mode": "auto",        # auto | max | manual
    "dsp_waterfall_row": 4,
    "dsp_3d_source": "combined",         # combined | rx1 | rx2
    "dsp_heat_mode": "magnitude",        # magnitude | fusion | motion | snr | difference
    "dsp_heat_gamma": 1.0,
    "dsp_heat_equalize": False,
    "dsp_persistence": 0.0,
    "dsp_det_overlay": True,
    "dsp_peak_overlay": True,
    "dsp_3d_rotate": True,
    "dsp_3d_light": True,
    "dsp_3d_wireframe": False,
    "dsp_phase_unwrap": True,
    "dsp_vital_history_s": 120,
    "dsp_vital_smooth": 0.2,
    "dsp_vital_fft_enabled": True,
    "dsp_vital_fft_window_s": 20,
    "dsp_vital_fft_n": 1024,
    "dsp_vital_breath_low_hz": 0.10,
    "dsp_vital_breath_high_hz": 0.60,
    "dsp_vital_heart_low_hz": 0.80,
    "dsp_vital_heart_high_hz": 2.00,
    # Detection
    "det_enabled": True,
    "det_channel": "mean",               # mean | rx1 | rx2 | max
    "det_method": "localmax",            # localmax | cfar
    "det_noise_mode": "auto",            # auto | manual
    "det_manual_noise_db": 42.0,
    "det_noise_percentile": 10.0,
    "det_threshold_db": 8.0,
    "det_cfar_guard": 1,
    "det_cfar_train": 2,
    "det_cfar_bias_db": 6.0,
    "det_min_range_m": 0.9,
    "det_max_range_m": 2.2,
    "det_min_doppler_bin": 0,
    "det_max_doppler_bin": 31,
    "det_exclude_dc_bins": 1,
    "det_min_peak_distance": 1,
    "det_max_peaks": 12,
    "det_min_power_db": 0.0,
    "det_target_hold_s": 2.0,
    "det_association_m": 1.0,
    "det_smoothing": 0.3,
    "det_show_threshold": True,
    # Presentation
    "ui_theme": "dark",
    "ui_show_rx1": True,
    "ui_show_rx2": True,
    "ui_show_range": True,
    "ui_show_doppler": True,
    "ui_show_waterfall": True,
    "ui_show_map": True,
    "ui_show_fusion": True,
    "ui_show_rti": True,
    "ui_show_3d": True,
    "ui_show_vitals": True,
    "ui_show_detections": True,
    "ui_show_target": True,
    "ui_show_trail": True,
    "ui_map_size_m": 3.0,
    "ui_auto_map_size": True,
    "ui_point_age_s": 120.0,
    "ui_max_points": 4000,
    "ui_trail_length": 300,
    "ui_view_min_m": 0.8,
    "ui_view_max_m": 2.2,
    "ui_3d_height": 1.0,
    "ui_3d_decimate": 1,
    # Auto mode
    "auto_enabled": True,
    "auto_contrast": True,
    "auto_threshold": True,
    "auto_view": True,
    "auto_view_span_m": 1.5,
    "auto_map_size": True,
    "auto_peak_target": True,
    "auto_gain": False,
    "auto_gain_target_db": 50.0,
    "auto_mode_switch": False,
    "auto_raw_hold_s": 2.0,
    "auto_track_after_s": 8.0,
    "auto_presence_source": "either",    # either | radar | raw
    "auto_motion_threshold_db": 1.5,
    "auto_record": False,
    "auto_record_post_s": 5.0,
    # Recording
    "rec_path": "recordings/advanced_session.npz",
    "rec_max_frames": 20000,
    "rec_include_detections": True,
    # Firmware tuning (image generation only, no flashing)
    "fw_rows": 8,
    "fw_tia": 6,
    "fw_vga": 15,
    "fw_pa": 10,
    "fw_hpf": 150,
    "fw_allow_rf": False,
    "fw_allow_unsafe": False,
    "fw_out_dir": "tuning/dashboard",
}

CLAMPS: dict[str, tuple[float, float]] = {
    "conn_baud": (9600, 3_000_000),
    "conn_reconnect_s": (0.2, 60.0),
    "radar_zone_min": (0, 100000),
    "radar_zone_max": (0, 100000),
    "radar_zone_width": (0, 100000),
    "radar_echo_value": (0, 0xFFFFFFFF),
    "radar_flag_value": (0, 0xFFFFFFFF),
    "dsp_range_start_m": (0.05, 20.0),
    "dsp_range_step_m": (0.005, 2.0),
    "dsp_doppler_bin_mps": (0.0001, 2.0),
    "dsp_doppler_zero_bin": (0, 63),
    "dsp_clutter_alpha": (0.0001, 0.9),
    "dsp_smooth_alpha": (0.0, 0.99),
    "dsp_display_gain_db": (-60.0, 60.0),
    "dsp_contrast_low_pct": (0.0, 60.0),
    "dsp_contrast_high_pct": (40.0, 100.0),
    "dsp_db_min": (-200.0, 200.0),
    "dsp_db_max": (-200.0, 200.0),
    "dsp_stream_hz": (2, 30),
    "dsp_heat_gamma": (0.2, 4.0),
    "dsp_persistence": (0.0, 0.95),
    "dsp_waterfall_rows": (20, 1000),
    "dsp_waterfall_row": (0, 63),
    "dsp_vital_history_s": (5, 900),
    "dsp_vital_smooth": (0.0, 0.99),
    "dsp_vital_fft_window_s": (3, 300),
    "dsp_vital_fft_n": (128, 8192),
    "dsp_vital_breath_low_hz": (0.01, 2.0),
    "dsp_vital_breath_high_hz": (0.02, 3.0),
    "dsp_vital_heart_low_hz": (0.2, 4.0),
    "dsp_vital_heart_high_hz": (0.3, 5.0),
    "det_manual_noise_db": (-200.0, 200.0),
    "det_noise_percentile": (0.0, 80.0),
    "det_threshold_db": (0.0, 80.0),
    "det_cfar_guard": (0, 10),
    "det_cfar_train": (1, 20),
    "det_cfar_bias_db": (-20.0, 40.0),
    "det_min_range_m": (0.0, 20.0),
    "det_max_range_m": (0.0, 20.0),
    "det_min_doppler_bin": (0, 63),
    "det_max_doppler_bin": (0, 63),
    "det_exclude_dc_bins": (0, 20),
    "det_min_peak_distance": (0, 20),
    "det_max_peaks": (1, 100),
    "det_min_power_db": (-200.0, 200.0),
    "det_target_hold_s": (0.0, 120.0),
    "det_association_m": (0.0, 20.0),
    "det_smoothing": (0.0, 0.99),
    "ui_map_size_m": (0.5, 30.0),
    "ui_point_age_s": (1.0, 3600.0),
    "ui_max_points": (50, 50000),
    "ui_trail_length": (2, 5000),
    "ui_view_min_m": (0.0, 20.0),
    "ui_view_max_m": (0.0, 20.0),
    "ui_3d_height": (0.1, 5.0),
    "ui_3d_decimate": (1, 8),
    "auto_view_span_m": (0.1, 20.0),
    "auto_gain_target_db": (-100.0, 150.0),
    "auto_raw_hold_s": (0.0, 60.0),
    "auto_track_after_s": (0.0, 300.0),
    "auto_motion_threshold_db": (0.0, 30.0),
    "auto_record_post_s": (0.0, 300.0),
    "rec_max_frames": (0, 500000),
    "fw_rows": (8, 32),
    "fw_tia": (6, 18),
    "fw_vga": (15, 20),
    "fw_pa": (10, 15),
    "fw_hpf": (150, 400),
}

CHOICES: dict[str, tuple[str, ...]] = {
    "conn_on_connect_mode": ("raw", "track", "keep"),
    "dsp_iq_view": ("magnitude", "real", "imag", "phase", "power"),
    "dsp_colormap": ("magma", "viridis", "inferno", "turbo", "grayscale", "hsv", "icefire"),
    "dsp_waterfall_mode": ("auto", "max", "manual"),
    "dsp_3d_source": ("combined", "rx1", "rx2"),
    "dsp_heat_mode": ("magnitude", "fusion", "motion", "snr", "difference"),
    "det_channel": ("mean", "rx1", "rx2", "max"),
    "det_method": ("localmax", "cfar"),
    "det_noise_mode": ("auto", "manual"),
    "ui_theme": ("dark", "light"),
    "auto_presence_source": ("either", "radar", "raw"),
    "fw_rows": ("8", "16", "32"),
    "fw_tia": ("6", "12", "18"),
    "fw_vga": ("15", "18", "20"),
    "fw_pa": ("10", "12", "15"),
    "fw_hpf": ("150", "400"),
}

BOOL_KEYS = {k for k, v in DEFAULT_CONFIG.items() if isinstance(v, bool)}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def safe_float(v, default=0.0) -> float:
    try:
        f = float(v)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default


def rounded_list(a, nd=1) -> list[float]:
    arr = np.asarray(a, dtype=np.float64)
    arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
    return np.round(arr, nd).tolist()


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def resolve_under_here(value: str, base: Path = HERE) -> Path:
    """Allow relative paths only below base (no escape)."""
    p = Path(str(value)).expanduser()
    if not p.is_absolute():
        p = base / p
    p = p.resolve()
    base = base.resolve()
    if p != base and base not in p.parents:
        raise ValueError(f"Path is outside of {base}: {value}")
    return p


def unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    stem, suffix = path.stem, path.suffix
    stamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = path.with_name(f"{stem}_{stamp}{suffix}")
    i = 2
    while candidate.exists():
        candidate = path.with_name(f"{stem}_{stamp}_{i}{suffix}")
        i += 1
    return candidate


def iso_time(ts: float | None = None) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(ts or time.time()))


def load_config(path: Path = CONFIG_PATH) -> dict:
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if path.exists():
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
            for k, v in raw.items():
                if k in cfg:
                    cfg[k] = coerce_value(k, v)
        except Exception as exc:
            print(f"[WARN] Configuration {path} not readable: {exc}", flush=True)
    return cfg


def coerce_value(key: str, value):
    default = DEFAULT_CONFIG[key]
    if key in BOOL_KEYS:
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on", "ja")
        return bool(value)
    if isinstance(default, int) and not isinstance(default, bool):
        value = int(round(safe_float(value, default)))
    elif isinstance(default, float):
        value = safe_float(value, default)
    else:
        value = str(value)[:500]
    if key in CLAMPS:
        lo, hi = CLAMPS[key]
        value = clamp(value, lo, hi)
        if isinstance(default, int):
            value = int(round(value))
    if key in CHOICES and str(value) not in CHOICES[key]:
        raise ValueError(f"{key}: allowed values are {', '.join(CHOICES[key])}")
    if key == "conn_port":
        value = str(value).strip()[:64]
    if key in ("cmd_custom_type",):
        value = str(value).strip()[:32]
    if key in ("cmd_custom_hex",):
        value = re.sub(r"[^0-9a-fA-F ]", "", str(value))[:2048]
    if key in ("rec_path", "fw_out_dir"):
        # Validation happens on access; here only length and no control characters.
        value = str(value).replace("\x00", "")[:300]
    return value


def save_config(cfg: dict, path: Path = CONFIG_PATH) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------

PRESETS: dict[str, dict] = {
    "default": {
        "label": "Default (balanced)",
        "updates": {
            "dsp_auto_contrast": True, "auto_enabled": True, "auto_contrast": True,
            "auto_threshold": True, "auto_view": True, "auto_map_size": True,
            "auto_peak_target": True, "auto_gain": False, "auto_mode_switch": False,
            "dsp_clutter_remove": False, "dsp_smooth_alpha": 0.0,
            "dsp_stream_hz": 10, "det_method": "localmax", "det_threshold_db": 8.0,
            "det_noise_percentile": 10.0, "det_min_range_m": 0.9,
            "det_max_range_m": 2.2, "det_max_peaks": 12,
        },
    },
    "sensitive": {
        "label": "Sensitive (weak motion)",
        "updates": {
            "auto_enabled": True, "auto_contrast": True, "auto_threshold": True,
            "auto_view": True, "auto_map_size": True, "auto_peak_target": True,
            "dsp_clutter_remove": True, "dsp_clutter_alpha": 0.015,
            "dsp_smooth_alpha": 0.25, "det_method": "localmax",
            "det_threshold_db": 4.0, "det_noise_percentile": 5.0,
            "det_min_power_db": 30.0, "det_max_peaks": 20,
            "dsp_db_min": 32.0, "dsp_db_max": 54.0, "dsp_stream_hz": 15,
        },
    },
    "far": {
        "label": "Long range / room",
        "updates": {
            "auto_enabled": True, "auto_contrast": True, "auto_threshold": True,
            "auto_view": True, "auto_map_size": True, "auto_peak_target": True,
            "dsp_range_start_m": 0.5, "dsp_range_step_m": 0.125,
            "ui_view_min_m": 0.4, "ui_view_max_m": 5.5,
            "auto_view_span_m": 4.0, "det_min_range_m": 0.4,
            "det_max_range_m": 5.5, "ui_map_size_m": 6.0,
            "det_max_peaks": 20, "dsp_stream_hz": 10,
        },
    },
    "raw": {
        "label": "Raw data only (maximum detail)",
        "updates": {
            "auto_enabled": False, "auto_contrast": False, "auto_threshold": False,
            "auto_view": False, "auto_map_size": False, "auto_peak_target": False,
            "auto_gain": False, "auto_mode_switch": False,
            "dsp_clutter_remove": False, "dsp_smooth_alpha": 0.0,
            "dsp_auto_contrast": True, "dsp_stream_hz": 20,
            "dsp_iq_view": "magnitude", "dsp_colormap": "magma",
            "det_enabled": False, "ui_show_detections": False,
            "ui_view_min_m": 0.8, "ui_view_max_m": 2.2,
        },
    },
    "vital": {
        "label": "Vitals / phase",
        "updates": {
            "auto_enabled": True, "auto_contrast": True, "auto_threshold": True,
            "auto_view": True, "auto_map_size": True, "auto_peak_target": True,
            "det_enabled": False, "ui_show_detections": False,
            "dsp_vital_fft_enabled": True, "dsp_vital_fft_window_s": 20,
            "dsp_vital_smooth": 0.25, "dsp_stream_hz": 15,
        },
    },
    "debug": {
        "label": "Debug / show everything",
        "updates": {
            "auto_enabled": True, "auto_contrast": True, "auto_threshold": True,
            "auto_view": True, "auto_map_size": True, "auto_peak_target": True,
            "det_enabled": True, "det_method": "cfar", "det_max_peaks": 40,
            "det_min_power_db": 20.0, "ui_show_detections": True,
            "dsp_stream_hz": 15, "dsp_waterfall_rows": 400,
        },
    },
}


# ---------------------------------------------------------------------------
# SSE hub
# ---------------------------------------------------------------------------

class Hub:
    def __init__(self) -> None:
        self.clients: set[queue.Queue] = set()
        self.lock = threading.Lock()

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=4)
        with self.lock:
            self.clients.add(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self.lock:
            self.clients.discard(q)

    def publish(self, message: str) -> None:
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(message)
                except queue.Full:
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        q.put_nowait(message)
                    except queue.Full:
                        pass


# ---------------------------------------------------------------------------
# Synthetic data (demo + selftest)
# ---------------------------------------------------------------------------

def synthetic_raw_payload(rng: np.random.Generator, t: float, rows: int = 8,
                          cols: int = 32, rx: int = 1,
                          targets: list[dict] | None = None) -> bytes:
    """Generate a plausible complex range-Doppler image as an int16 payload."""
    noise = (rng.normal(size=(rows, cols)) + 1j * rng.normal(size=(rows, cols))) * 220.0
    z = noise.astype(np.complex128)
    rr, cc = np.mgrid[0:rows, 0:cols]
    if targets is None:
        targets = [
            {"row": rows * 0.45 + 1.4 * math.sin(t * 0.7),
             "col": cols * 0.55 + 4.0 * math.sin(t * 0.35),
             "amp": 2600.0, "spread_r": 0.55, "spread_c": 1.8, "phase": t * 1.3},
            {"row": rows * 0.72 + 0.5 * math.sin(t * 0.23 + 1.0),
             "col": cols * 0.30 + 2.0 * math.sin(t * 0.17 + 2.0),
             "amp": 1300.0, "spread_r": 0.75, "spread_c": 2.4, "phase": t * 0.4},
        ]
    for i, tg in enumerate(targets):
        r0 = float(tg.get("row", 3.0))
        c0 = float(tg.get("col", 16.0))
        amp = float(tg.get("amp", 2000.0)) * (1.0 + 0.08 * math.sin(t * (0.5 + i * 0.13) + i))
        sr = float(tg.get("spread_r", 0.6))
        sc = float(tg.get("spread_c", 2.0))
        phase = float(tg.get("phase", 0.0)) + (0.0 if rx == 1 else 0.7)
        env = np.exp(-((rr - r0) ** 2 / (2 * sr ** 2) + (cc - c0) ** 2 / (2 * sc ** 2)))
        z += amp * env * np.exp(1j * phase)
    # slight, slow background drift
    z += 120.0 * np.exp(1j * (t * 0.1 + rr * 0.3))
    iq = np.empty((rows, cols, 2), dtype=np.int16)
    iq[..., 0] = np.clip(np.real(z), -32760, 32760).astype(np.int16)
    iq[..., 1] = np.clip(np.imag(z), -32760, 32760).astype(np.int16)
    return iq.reshape(-1).astype("<i2").tobytes()


def make_point_frame(points: list[dict]) -> bytes:
    out = bytearray(struct.pack("<i", len(points)))
    for p in points:
        out += struct.pack("<ffiI", float(p.get("x", 0)), float(p.get("y", 0)),
                           int(p.get("v", 0)), int(p.get("id", 0)))
    return bytes(out)


# ---------------------------------------------------------------------------
# Engine
# ---------------------------------------------------------------------------

class Engine:
    def __init__(self, cfg: dict, source: str = "serial", replay_path: Path | None = None,
                 replay_speed: float = 1.0, config_path: Path = CONFIG_PATH,
                 start_threads: bool = True, vault_dir: Path | None = None) -> None:
        self.cfg = copy.deepcopy(cfg)
        self.defaults = copy.deepcopy(DEFAULT_CONFIG)
        self.config_path = config_path
        self.source = source
        self.replay_path = Path(replay_path) if replay_path else None
        self.replay_speed = max(0.05, float(replay_speed))
        self.vault = FirmwareVault(Path(vault_dir) if vault_dir else VAULT_DIR)

        self.lock = threading.RLock()
        self.dsp_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.hub = Hub()
        self.stop_evt = threading.Event()
        self.conn_stop = threading.Event()
        self.worker: threading.Thread | None = None
        self.stream_thread: threading.Thread | None = None
        self.demo_thread: threading.Thread | None = None
        self.replay_thread: threading.Thread | None = None
        self.ser = None
        self.parser = None
        self.connected = False
        self.port = str(self.cfg["conn_port"])
        self.radar_mode = "unknown"
        self.source_label = {"serial": "Serial", "demo": "Demo", "replay": "Replay",
                             "selftest": "Selftest"}.get(source, source)

        self.t0 = time.time()
        self.bytes_total = 0
        self.frames_total = 0
        self.counts: collections.Counter = collections.Counter()
        self.type_times: dict[int, collections.deque] = collections.defaultdict(
            lambda: collections.deque(maxlen=120))
        self.last_frame_t: dict[int, float] = {}
        self.parser_stats = {"frames": 0, "head_err": 0, "data_err": 0, "text_bytes": 0}
        self.rate_marks: collections.deque = collections.deque(maxlen=120)

        self.raw: dict[int, tuple[float, float, bytes]] = {}
        self.version = "–"
        self.status = None
        self.presence = None
        self.distance_cm = None
        self.target = {"x": None, "y": None, "t": 0.0}
        self.zone = {"min_cm": None, "max_cm": None, "width_cm": None}
        self.vitals = {"total": None, "breath": None, "heart": None,
                       "breath_rate": None, "heart_rate": None, "t": 0.0,
                       "seq": 0}
        self.phase_hist: collections.deque = collections.deque(maxlen=20000)
        self.points: collections.deque = collections.deque(maxlen=50000)
        self.logs: collections.deque = collections.deque(maxlen=1000)
        self.log_seq = 0
        self.text_last = ""
        self.recording = False
        self.rec_limit_pending = False
        self.rec_source = ""
        self.rec_started = 0.0
        self.record: list[tuple[float, int, bytes]] = []
        self.rec_detections: list[dict] = []
        self.last_saved_path: str | None = None
        self.last_snapshot_path: str | None = None
        # Re-request raw mode after boot if needed (the first 0x0A14 can be lost)
        self.raw_last_rx_t = 0.0
        self.raw_reassert_until = 0.0
        self.raw_reassert_next = 0.0
        self.raw_reassert_tries = 0
        self.cfg_seq = 1
        self.snap_seq = 0
        self.last_sent_cfg_seq = 0
        self.last_detections: list[dict] = []
        self.detection_meta: dict = {}
        self.det_history: collections.deque = collections.deque(maxlen=20000)

        # DSP state
        self.bg: dict[str, np.ndarray | None] = {"rx1": None, "rx2": None}
        self.mag_smooth: dict[str, np.ndarray | None] = {"rx1": None, "rx2": None,
                                                         "combined": None}
        self.prev_mag_comb: np.ndarray | None = None
        self.motion_db = 0.0
        self.vital_fft_state: dict = {}
        self.auto_state: dict = {
            "contrast": {"lo": float(self.cfg["dsp_db_min"]),
                         "hi": float(self.cfg["dsp_db_max"])},
            "gain_db": float(self.cfg["dsp_display_gain_db"]),
            "view": {"min_m": float(self.cfg["ui_view_min_m"]),
                     "max_m": float(self.cfg["ui_view_max_m"])},
            "map_size_m": float(self.cfg["ui_map_size_m"]),
            "peak": None,
            "presence": False,
            "presence_source": "–",
            "mode_switch": {"state": "idle", "since": time.time()},
            "record": {"active": False, "reason": ""},
        }
        self.auto_present_since: float | None = None
        self.auto_absent_since: float | None = None
        self.auto_last_present_t: float | None = None
        self.last_mode_cmd_t = 0.0
        self.last_auto_record_t = 0.0
        self.last_patch: dict | None = None

        self.log("info", f"Advanced Radar Console {APP_VERSION} started "
                         f"(source: {self.source_label})")
        if start_threads:
            self.stream_thread = threading.Thread(target=self._stream_loop,
                                                  name="stream", daemon=True)
            self.stream_thread.start()
            if source == "demo":
                self.demo_thread = threading.Thread(target=self._demo_loop,
                                                    name="demo", daemon=True)
                self.demo_thread.start()
            elif source == "replay":
                self.replay_thread = threading.Thread(target=self._replay_loop,
                                                      name="replay", daemon=True)
                self.replay_thread.start()
            elif source == "serial" and self.cfg.get("conn_auto_connect", True):
                threading.Thread(target=self._auto_connect, name="auto-connect",
                                 daemon=True).start()

    # ------------------------------------------------------------------
    # Logging / statistics
    # ------------------------------------------------------------------
    def log(self, level: str, msg: str) -> None:
        msg = str(msg).replace("\x00", "")
        # Radar debug text can contain raw bytes; control characters would
        # otherwise break the Windows console and abort the serial loop.
        msg = "".join(ch if ch.isprintable() else " " for ch in msg)[:800]
        with self.lock:
            self.log_seq += 1
            entry = {"seq": self.log_seq, "t": time.time(), "level": level, "msg": msg}
            self.logs.append(entry)
        line = f"[{level.upper():5}] {msg}"
        out = sys.stdout
        if out is None:  # e.g. pythonw.exe without a console
            return
        try:
            print(line, file=out, flush=True)
        except (UnicodeEncodeError, OSError, AttributeError):
            enc = getattr(out, "encoding", None) or "ascii"
            safe = line.encode(enc, "replace").decode(enc, "replace")
            try:
                print(safe, file=out, flush=True)
            except (OSError, AttributeError):
                pass

    def _rate(self, values: collections.deque, window: float = 2.0) -> float:
        if len(values) < 2:
            return 0.0
        now = time.time()
        recent = [t for t in values if now - t <= window]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def stats(self) -> dict:
        with self.lock:
            self.rate_marks.append((time.time(), self.bytes_total, self.frames_total))
            while len(self.rate_marks) > 2 and time.time() - self.rate_marks[0][0] > 3.0:
                self.rate_marks.popleft()
            bps = 0.0
            fps = 0.0
            if len(self.rate_marks) >= 2:
                t0, b0, f0 = self.rate_marks[0]
                t1, b1, f1 = self.rate_marks[-1]
                dt = t1 - t0
                if dt > 0:
                    bps = (b1 - b0) / dt
                    fps = (f1 - f0) / dt
            type_rates = {}
            for ftype, times in list(self.type_times.items()):
                r = self._rate(times)
                if r > 0:
                    type_rates[f"{ftype:#06x}"] = round(r, 2)
            counts = {f"{k:#06x}": v for k, v in self.counts.items()}
            last_age = None
            if self.last_frame_t:
                last_age = round(time.time() - max(self.last_frame_t.values()), 2)
            raw_rate = {f"{t:#06x}": round(self._rate(self.type_times[t]), 2)
                        for t in RAW_TYPES}
            return {
                "connected": self.connected,
                "port": self.port,
                "baud": self.cfg["conn_baud"],
                "mode": self.radar_mode,
                "version": self.version,
                "source": self.source_label,
                "bytes_total": self.bytes_total,
                "bytes_per_s": round(bps, 1),
                "frames_total": self.frames_total,
                "frames_per_s": round(fps, 2),
                "raw_rate_hz": raw_rate,
                "type_rates": type_rates,
                "counts": counts,
                "parser": dict(self.parser_stats),
                "last_frame_age_s": last_age,
                "uptime_s": round(time.time() - self.t0, 1),
                "recording": self.recording,
                "recording_frames": len(self.record),
                "recording_source": self.rec_source,
            }

    # ------------------------------------------------------------------
    # Frame input
    # ------------------------------------------------------------------
    def on_text(self, line: str) -> None:
        line = line.strip()
        if not line or line == self.text_last:
            return
        # 0x0100 also contains raw binary data during boot (decoded as Latin-1).
        # Only show mostly readable text; otherwise it floods the log.
        if len(line) >= 4:
            readable = sum(1 for ch in line if ch.isascii() and ch.isprintable())
            if readable < len(line) * 0.7:
                return
        self.text_last = line
        self.log("radar", line[:400])

    def on_frame(self, _fid: int, ftype: int, payload: bytes, _raw: bytes) -> None:
        now = time.time()
        with self.lock:
            self.frames_total += 1
            self.counts[ftype] += 1
            self.type_times[ftype].append(now)
            self.last_frame_t[ftype] = now
            if ftype in RAW_TYPES:
                self.raw_last_rx_t = now
                self.raw[ftype] = (time.perf_counter(), now, payload)
                if self.recording:
                    self.record.append((now, ftype, bytes(payload)))
                    if self.cfg["rec_max_frames"] and len(self.record) >= int(self.cfg["rec_max_frames"]):
                        self._rec_limit_hit()
                return
            try:
                info = d.decode(ftype, payload)
            except Exception:
                info = {}
            if ftype == 0xFFFF and len(payload) >= 4:
                self.version = ".".join(str(b) for b in payload[:4])
            elif ftype == 0x0A04:
                pts = info.get("points", []) if info else []
                for p in pts[:5000]:
                    self.points.append({
                        "t": now, "x": float(p.get("x", 0)), "y": float(p.get("y", 0)),
                        "v": int(p.get("v", 0)), "id": int(p.get("id", 0)),
                    })
                if len(self.points) > int(self.cfg["ui_max_points"]):
                    for _ in range(len(self.points) - int(self.cfg["ui_max_points"])):
                        self.points.popleft()
            elif ftype == 0x0A13 and info:
                self.vitals.update({
                    "total": info.get("total"), "breath": info.get("breath"),
                    "heart": info.get("heart"), "t": now,
                    "seq": self.vitals.get("seq", 0) + 1,
                })
                self.phase_hist.append((now, safe_float(info.get("total")),
                                        safe_float(info.get("breath")),
                                        safe_float(info.get("heart"))))
            elif ftype == 0x0A14 and info:
                self.vitals["breath_rate"] = info.get("rate")
                self.vitals["t"] = now
            elif ftype == 0x0A15 and info:
                self.vitals["heart_rate"] = info.get("rate")
                self.vitals["t"] = now
            elif ftype == 0x0A16 and info:
                self.distance_cm = info.get("cm")
            elif ftype == 0x0A17 and info:
                self.target = {"x": info.get("x"), "y": info.get("y"), "t": now}
            elif ftype == 0x0F09 and info:
                self.presence = bool(info.get("present"))
            elif ftype == 0x0A28 and info:
                self.zone = {"min_cm": info.get("min_cm"), "max_cm": info.get("max_cm"),
                             "width_cm": info.get("width_cm")}
            elif ftype == 0x0A29 and info:
                self.status = info.get("status")
            elif ftype == 0x0A20 and info:
                self.log("info", f"Echo response: {info.get('value')}")
            elif ftype == 0x0100 and info:
                self.on_text(info.get("text", ""))
            elif ftype in BLOCKED_TYPES:
                self.log("warn", f"Frame {ftype:#06x} received (bootloader/update) – ignored")

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------
    def _auto_connect(self) -> None:
        time.sleep(0.7)
        if not self.stop_evt.is_set():
            self.connect()

    def connect(self) -> tuple[bool, str]:
        if self.source != "serial":
            return False, f"No COM port is opened in {self.source_label} mode."
        if serial is None:
            return False, "pyserial is not installed."
        with self.lock:
            if self.worker and self.worker.is_alive():
                return False, "Connection is already running."
            self.conn_stop = threading.Event()
            self.port = str(self.cfg["conn_port"])
            self.worker = threading.Thread(target=self._serial_loop, name="serial",
                                           daemon=True)
            self.worker.start()
        self.log("info", f"Connecting to {self.port} …")
        return True, f"Connecting to {self.port} …"

    def disconnect(self) -> tuple[bool, str]:
        self.conn_stop.set()
        ser = self.ser
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        worker = self.worker
        if worker and worker.is_alive() and worker is not threading.current_thread():
            worker.join(timeout=1.0)
        self.connected = False
        self.log("info", "Connection closed.")
        return True, "Connection closed."

    def _serial_loop(self) -> None:
        backoff = 0.5
        conn_stop = self.conn_stop  # bind locally: disconnect()/connect() replace the event
        while not conn_stop.is_set() and not self.stop_evt.is_set():
            port = str(self.cfg["conn_port"])
            baud = int(self.cfg["conn_baud"])
            try:
                ser = serial.Serial(port, baud, timeout=0.05)
                with self.lock:
                    self.ser = ser
                    self.connected = True
                    self.port = port
                    self.parser = d.TFParser(self.on_frame, self.on_text)
                self.log("info", f"{port} opened (USB-CDC; baud {baud} is practically irrelevant).")
                backoff = 0.5
                time.sleep(0.15)
                self._on_connect_actions()
                while not conn_stop.is_set() and not self.stop_evt.is_set():
                    data = ser.read(65536)
                    if data:
                        with self.lock:
                            self.bytes_total += len(data)
                        try:
                            self.parser.feed(data)
                        except Exception as exc:
                            self.log("warn", f"Parser error: {type(exc).__name__}: {exc}")
                        with self.lock:
                            self.parser_stats = dict(self.parser.stats)
                        self._maybe_reassert_raw()
                break
            except Exception as exc:
                if conn_stop.is_set() or self.stop_evt.is_set():
                    break
                self.log("warn", f"Serial error: {type(exc).__name__}: {exc}")
            finally:
                with self.lock:
                    self.connected = False
                    self.ser = None
                try:
                    ser.close()
                except Exception:
                    pass
            if conn_stop.is_set() or self.stop_evt.is_set():
                break
            if not self.cfg.get("conn_auto_reconnect", True):
                self.log("warn", "Auto-reconnect is off – connection stays closed.")
                break
            delay = max(0.2, float(self.cfg["conn_reconnect_s"]))
            self.log("info", f"Reconnecting in {delay:.1f} s …")
            end = time.time() + delay
            while time.time() < end and not conn_stop.is_set() and not self.stop_evt.is_set():
                time.sleep(0.1)

    def _on_connect_actions(self) -> None:
        try:
            if self.cfg.get("conn_on_connect_version", True):
                self.send_command("version")
                time.sleep(0.08)
            mode = str(self.cfg.get("conn_on_connect_mode", "raw")).lower()
            if mode == "raw":
                self.send_command("raw_on")
                self.radar_mode = "raw"
                # The firmware may discard the first command during boot.
                self.raw_reassert_until = time.time() + 9.0
                self.raw_reassert_next = time.time() + 2.0
                self.raw_reassert_tries = 0
            elif mode == "track":
                self.send_command("raw_off")
                self.radar_mode = "track"
            if self.cfg.get("conn_on_connect_zone_read", True):
                time.sleep(0.08)
                self.send_command("zone_read")
        except Exception as exc:
            self.log("warn", f"Initialization after connect failed: {exc}")

    def _maybe_reassert_raw(self) -> None:
        """Repeat 0x0A14 in case the radar was not ready yet when connecting."""
        if self.source != "serial" or self.radar_mode != "raw":
            return
        now = time.time()
        if not self.raw_reassert_until or now > self.raw_reassert_until:
            return
        if now - self.raw_last_rx_t < 1.0:
            self.raw_reassert_until = 0.0
            return
        if now < self.raw_reassert_next or self.raw_reassert_tries >= 2:
            return
        self.raw_reassert_tries += 1
        self.raw_reassert_next = now + 2.5
        ok, _msg = self.send_command("raw_on")
        if ok:
            self.log("info", "Raw data not active yet – 0x0A14 sent again "
                             f"({self.raw_reassert_tries}/2).")

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------
    def send_raw(self, ftype: int, payload: bytes = b"", note: str = "") -> tuple[bool, str]:
        ftype = int(ftype) & 0xFFFF
        if ftype in BLOCKED_TYPES:
            msg = f"Frame {ftype:#06x} is blocked (bootloader/update)."
            self.log("error", msg)
            return False, msg
        if self.source != "serial":
            msg = f"Sending is not possible in {self.source_label} mode."
            self.log("warn", msg)
            return False, msg
        with self.serial_lock:
            ser = self.ser
            if not self.connected or ser is None:
                msg = "Not connected – command not sent."
                self.log("warn", msg)
                return False, msg
            try:
                frame = d.build_frame(ftype, payload)
                ser.write(frame)
                self.log("tx", f"{note or 'Frame'} {ftype:#06x} "
                               f"({len(payload)} B): {frame.hex(' ')[:160]}")
                return True, f"Sent: {ftype:#06x}, {len(payload)} bytes"
            except Exception as exc:
                msg = f"Send failed: {type(exc).__name__}: {exc}"
                self.log("error", msg)
                return False, msg

    def send_command(self, name: str, value=None) -> tuple[bool, str]:
        if name not in d.COMMANDS:
            return False, f"Unknown command: {name}"
        ftype, arg, desc = d.COMMANDS[name]
        payload = b""
        if arg == "u32":
            payload = struct.pack("<I", int(value if value is not None else 0) & 0xFFFFFFFF)
        ok, msg = self.send_raw(ftype, payload, note=name)
        if ok:
            if name == "raw_on":
                self.radar_mode = "raw"
            elif name == "raw_off":
                self.radar_mode = "track"
        return ok, msg

    def send_custom(self, type_text: str, hex_text: str) -> tuple[bool, str]:
        try:
            ftype = int(str(type_text), 0)
        except ValueError:
            return False, f"Invalid frame type: {type_text!r}"
        try:
            payload = bytes.fromhex(re.sub(r"[^0-9a-fA-F]", "", str(hex_text)))
        except ValueError as exc:
            return False, f"Invalid hex payload: {exc}"
        if len(payload) > 4096:
            return False, "Payload too large (max. 4096 bytes)."
        return self.send_raw(ftype, payload, note="custom")

    # ------------------------------------------------------------------
    # Demo / Replay
    # ------------------------------------------------------------------
    def _inject_raw(self, ftype: int, payload: bytes) -> None:
        with self.lock:
            now = time.time()
            self.frames_total += 1
            self.counts[ftype] += 1
            self.type_times[ftype].append(now)
            self.last_frame_t[ftype] = now
            self.raw[ftype] = (time.perf_counter(), now, payload)
            if self.recording:
                self.record.append((now, ftype, payload))
                if self.cfg["rec_max_frames"] and len(self.record) >= int(self.cfg["rec_max_frames"]):
                    self._rec_limit_hit()

    def _demo_loop(self) -> None:
        rng = np.random.default_rng(20261019)
        with self.lock:
            self.connected = True
            self.radar_mode = "raw"
            self.version = "DEMO-1.6.12"
            self.presence = True
            self.distance_cm = 145.0
            self.zone = {"min_cm": 0, "max_cm": 1500, "width_cm": 1500}
            self.status = 0
            self.parser_stats = {"frames": 0, "head_err": 0, "data_err": 0, "text_bytes": 0}
        self.log("info", "Demo mode: synthetic range-Doppler data, 20 Hz per channel.")
        t0 = time.time()
        i = 0
        while not self.stop_evt.is_set():
            t = time.time() - t0
            for ftype, rx in ((0x0A32, 1), (0x0A34, 2)):
                payload = synthetic_raw_payload(rng, t, 8, 32, rx)
                self._inject_raw(ftype, payload)
            if i % 4 == 0:
                phase = 0.8 * math.sin(t * 2 * math.pi * 0.22) + 0.15 * math.sin(t * 2 * math.pi * 1.1)
                self.on_frame(0, 0x0A13, struct.pack("<fff", phase,
                                                     phase * 0.7, phase * 0.2), b"")
                self.on_frame(0, 0x0A14, struct.pack("<f", 13.2 + 1.0 * math.sin(t * 0.2)), b"")
                self.on_frame(0, 0x0A15, struct.pack("<f", 68.0 + 3.0 * math.sin(t * 0.13)), b"")
                self.on_frame(0, 0x0F09, struct.pack("<H", 1), b"")
                self.on_frame(0, 0x0A16, struct.pack("<If", 1, 145.0 + 8.0 * math.sin(t * 0.4)), b"")
                tx = 0.35 * math.sin(t * 0.5)
                ty = 1.35 + 0.35 * math.sin(t * 0.7)
                self.on_frame(0, 0x0A17, struct.pack("<ff", tx, ty), b"")
                pts = [
                    {"x": tx, "y": ty, "v": int(40 * math.cos(t * 0.7)), "id": 1},
                    {"x": 0.55 * math.sin(t * 0.31 + 1.0), "y": 1.9 + 0.2 * math.sin(t * 0.2), "v": 12, "id": 2},
                ]
                self.on_frame(0, 0x0A04, make_point_frame(pts), b"")
                self.on_frame(0, 0x0A28, struct.pack("<HHH", 0, 1500, 1500), b"")
                self.on_frame(0, 0x0A29, struct.pack("<H", 0), b"")
                self.on_frame(0, 0xFFFF, bytes([1, 6, 12, 0]), b"")
            i += 1
            time.sleep(max(0.0, 0.05 - ((time.time() - t0) % 0.05)))

    def _replay_loop(self) -> None:
        path = self.replay_path
        if path is None or not path.exists():
            self.log("error", f"Replay file not found: {path}")
            return
        self.log("info", f"Replay: {path}")
        try:
            z = np.load(path, allow_pickle=False)
            frames: list[tuple[float, int, bytes]] = []
            if "a32" in z or "a34" in z:
                for key, ftype in (("a32", 0x0A32), ("a34", 0x0A34)):
                    if key not in z:
                        continue
                    arr = z[key]
                    if arr.ndim == 1:
                        arr = arr.reshape(1, -1)
                    for i, row in enumerate(arr):
                        frames.append((i / 19.58, ftype, bytes(row.tobytes())))
            elif "data" in z:
                data = z["data"]
                types = z["type"] if "type" in z else None
                times = z["t"] if "t" in z else None
                lengths = z["length"] if "length" in z else None
                if data.ndim == 2:
                    for i in range(len(data)):
                        ftype = int(types[i]) if types is not None else 0x0A32
                        frames.append((float(times[i]) if times is not None else i / 19.58,
                                       ftype, bytes(data[i].tobytes())))
                else:
                    offset = 0
                    for i in range(len(data)):
                        ln = int(lengths[i]) if lengths is not None else 1024
                        payload = bytes(data[offset:offset + ln].tobytes())
                        offset += ln
                        ftype = int(types[i]) if types is not None else 0x0A32
                        frames.append((float(times[i]) if times is not None else i / 19.58,
                                       ftype, payload))
            else:
                self.log("error", "Replay: no known arrays (a32/a34 or data/type/t).")
                return
            frames.sort(key=lambda x: x[0])
            self.log("info", f"Replay: {len(frames)} frames loaded.")
            with self.lock:
                self.connected = True
                self.radar_mode = "replay"
                self.version = "REPLAY"
                self.parser_stats = {"frames": 0, "head_err": 0, "data_err": 0, "text_bytes": 0}
            t_start_wall = time.time()
            t_first = frames[0][0] if frames else 0.0
            for t_rel, ftype, payload in frames:
                if self.stop_evt.is_set():
                    break
                due = t_start_wall + max(0.0, (t_rel - t_first) / self.replay_speed)
                while True:
                    delay = due - time.time()
                    if delay <= 0 or self.stop_evt.is_set():
                        break
                    time.sleep(min(delay, 0.25))
                if ftype in RAW_TYPES:
                    self._inject_raw(ftype, payload)
                else:
                    self.on_frame(0, ftype, payload, payload)
            self.log("info", "Replay finished.")
        except Exception as exc:
            self.log("error", f"Replay error: {type(exc).__name__}: {exc}")
        finally:
            with self.lock:
                self.connected = False

    # ------------------------------------------------------------------
    # DSP
    # ------------------------------------------------------------------
    def _decode_iq(self, payload: bytes) -> np.ndarray | None:
        a = np.frombuffer(payload, dtype="<i2")
        if a.size == 0 or a.size % (32 * 2) != 0:
            return None
        rows = a.size // (32 * 2)
        iq = a.reshape(rows, 32, 2).astype(np.float64)
        return iq[..., 0] + 1j * iq[..., 1]

    def _apply_clutter(self, rx: str, z: np.ndarray, cfg: dict) -> np.ndarray:
        if not cfg.get("dsp_clutter_remove"):
            self.bg[rx] = None
            return z
        bg = self.bg.get(rx)
        if bg is None or bg.shape != z.shape:
            self.bg[rx] = z.copy()
            return np.zeros_like(z)
        alpha = float(cfg["dsp_clutter_alpha"])
        bg = (1.0 - alpha) * bg + alpha * z
        self.bg[rx] = bg
        return z - bg

    def _smooth(self, key: str, m: np.ndarray, cfg: dict) -> np.ndarray:
        a = float(cfg.get("dsp_smooth_alpha", 0.0))
        prev = self.mag_smooth.get(key)
        if a <= 0.0:
            self.mag_smooth[key] = m
            return m
        if prev is None or prev.shape != m.shape:
            self.mag_smooth[key] = m
            return m
        out = (1.0 - a) * prev + a * m
        self.mag_smooth[key] = out
        return out

    def _view_matrix(self, z: np.ndarray, view: str) -> np.ndarray:
        if view == "real":
            return np.real(z)
        if view == "imag":
            return np.imag(z)
        if view == "phase":
            return np.degrees(np.angle(z))
        if view == "power":
            return 10.0 * np.log10(np.abs(z) ** 2 + 1e-9)
        return 20.0 * np.log10(np.abs(z) + 1e-9)

    def _cfar_threshold(self, mag: np.ndarray, guard: int, train: int, bias_db: float) -> np.ndarray:
        rows, cols = mag.shape
        pad = max(1, guard + train)
        p = np.pad(mag, pad, mode="edge")
        out = np.empty_like(mag)
        # Training window without guard cells; mean power.
        for r in range(rows):
            for c in range(cols):
                win = p[r:r + 2 * pad + 1, c:c + 2 * pad + 1]
                mask = np.ones_like(win, dtype=bool)
                g0 = max(0, pad - guard)
                g1 = min(2 * pad + 1, pad + guard + 1)
                mask[g0:g1, g0:g1] = False
                vals = win[mask]
                if vals.size == 0:
                    out[r, c] = float(np.max(win)) + bias_db
                else:
                    power = np.mean(10.0 ** (vals / 10.0))
                    out[r, c] = 10.0 * math.log10(power + 1e-12) + bias_db
        return out

    def detect(self, mag: np.ndarray, cfg: dict, range_m: np.ndarray,
               doppler_mps: np.ndarray, channel: str) -> tuple[list[dict], dict]:
        rows, cols = mag.shape
        if not cfg.get("det_enabled"):
            return [], {"enabled": False, "noise_db": None, "threshold_db": None,
                        "method": cfg.get("det_method"), "channel": channel, "count": 0}
        if cfg.get("det_noise_mode") == "manual":
            noise = float(cfg["det_manual_noise_db"])
        else:
            noise = float(np.percentile(mag, float(cfg["det_noise_percentile"])))
        method = str(cfg.get("det_method", "localmax"))
        if method == "cfar":
            th_map = self._cfar_threshold(mag, int(cfg["det_cfar_guard"]),
                                          int(cfg["det_cfar_train"]),
                                          float(cfg["det_cfar_bias_db"]))
            threshold = float(np.median(th_map))
        else:
            threshold = noise + float(cfg["det_threshold_db"])
            th_map = np.full_like(mag, threshold)

        mask = np.zeros((rows, cols), dtype=bool)
        rmin = float(cfg["det_min_range_m"])
        rmax = float(cfg["det_max_range_m"])
        if rmax < rmin:
            rmin, rmax = rmax, rmin
        for r in range(rows):
            if rmin <= range_m[r] <= rmax:
                mask[r, :] = True
        cmin = int(cfg["det_min_doppler_bin"])
        cmax = int(cfg["det_max_doppler_bin"])
        if cmax < cmin:
            cmin, cmax = cmax, cmin
        mask[:, :max(0, cmin)] = False
        mask[:, min(cols, cmax + 1):] = False
        dc = int(cfg["det_exclude_dc_bins"])
        if dc > 0:
            zbin = int(cfg["dsp_doppler_zero_bin"])
            mask[:, max(0, zbin - dc):min(cols, zbin + dc + 1)] = False

        cand = (mag > th_map) & mask & (mag >= float(cfg["det_min_power_db"]))
        coords = np.argwhere(cand)
        if coords.size:
            order = np.argsort([mag[r, c] for r, c in coords])[::-1]
            coords = coords[order]
        d = max(1, int(cfg["det_min_peak_distance"]))
        selected: list[tuple[int, int]] = []
        max_peaks = int(cfg["det_max_peaks"])
        for r, c in coords:
            r = int(r); c = int(c)
            r0, r1 = max(0, r - d), min(rows, r + d + 1)
            c0, c1 = max(0, c - d), min(cols, c + d + 1)
            if mag[r, c] < float(np.max(mag[r0:r1, c0:c1])) - 1e-9:
                continue
            if any(abs(r - rr) <= d and abs(c - cc) <= d for rr, cc in selected):
                continue
            selected.append((r, c))
            if len(selected) >= max_peaks:
                break

        dets: list[dict] = []
        for r, c in selected:
            power = float(mag[r, c])
            local_noise = float(th_map[r, c] - (float(cfg["det_cfar_bias_db"]) if method == "cfar"
                                                else float(cfg["det_threshold_db"])))
            dets.append({
                "row": int(r), "col": int(c),
                "range_m": round(float(range_m[r]), 3),
                "doppler_bin": int(c),
                "velocity_mps": round(float(doppler_mps[c]), 3),
                "power_db": round(power, 2),
                "noise_db": round(local_noise, 2),
                "snr_db": round(power - local_noise, 2),
                "channel": channel,
            })
        meta = {
            "enabled": True, "noise_db": round(noise, 2),
            "threshold_db": round(threshold, 2), "method": method,
            "channel": channel, "count": len(dets),
            "guard": int(cfg["det_cfar_guard"]), "train": int(cfg["det_cfar_train"]),
        }
        return dets, meta

    def _compute_vital_fft(self, cfg: dict) -> dict | None:
        if not cfg.get("dsp_vital_fft_enabled"):
            return None
        with self.lock:
            hist = list(self.phase_hist)
        if len(hist) < 32:
            return None
        t = np.array([h[0] for h in hist], dtype=np.float64)
        x = np.array([h[1] for h in hist], dtype=np.float64)
        window = float(cfg["dsp_vital_fft_window_s"])
        keep = t >= (t[-1] - window)
        t, x = t[keep], x[keep]
        if len(t) < 32:
            return None
        # Interpolate onto a uniform 20 Hz grid.
        fs = 20.0
        n_uniform = max(32, int((t[-1] - t[0]) * fs) + 1)
        tu = np.linspace(t[0], t[-1], n_uniform)
        xu = np.interp(tu, t, x)
        if cfg.get("dsp_phase_unwrap", True):
            xu = np.unwrap(xu)
        xu = xu - np.mean(xu)
        xu = xu * np.hanning(len(xu))
        n = int(max(int(cfg["dsp_vital_fft_n"]), len(xu)))
        n = 1 << int(math.ceil(math.log2(max(128, n))))
        spec = np.fft.rfft(xu, n=n)
        freqs = np.fft.rfftfreq(n, 1.0 / fs)
        mag = 20.0 * np.log10(np.abs(spec) + 1e-9)
        lo = 0.05
        hi = min(3.0, freqs[-1])
        sel = (freqs >= lo) & (freqs <= hi)
        f = freqs[sel]
        m = mag[sel]
        if f.size < 4:
            return None
        # Decimate to about 256 points to keep the SSE packet small.
        if f.size > 256:
            idx = np.linspace(0, f.size - 1, 256).astype(int)
            f, m = f[idx], m[idx]
        def peak(flo, fhi):
            band = (f >= flo) & (f <= fhi)
            if not np.any(band):
                return None
            ii = np.argmax(m[band])
            ff = float(f[band][ii])
            return {"hz": round(ff, 4), "bpm": round(ff * 60.0, 1),
                    "power_db": round(float(m[band][ii]), 2)}
        br = peak(float(cfg["dsp_vital_breath_low_hz"]), float(cfg["dsp_vital_breath_high_hz"]))
        hr = peak(float(cfg["dsp_vital_heart_low_hz"]), float(cfg["dsp_vital_heart_high_hz"]))
        return {
            "freqs": rounded_list(f, 4),
            "mag_db": rounded_list(m, 2),
            "breath": br, "heart": hr,
            "fs": fs, "n": n, "window_s": round(float(t[-1] - t[0]), 2),
            "points": int(len(hist)),
        }

    def _auto_tick(self, now: float, cfg: dict, context: dict) -> None:
        auto = self.auto_state
        enabled = bool(cfg.get("auto_enabled", True))
        # Contrast / gain
        mag = context.get("mag_comb")
        if mag is not None and mag.size:
            if enabled and cfg.get("auto_gain"):
                target = float(cfg["auto_gain_target_db"])
                p99 = float(np.percentile(mag, 99.0))
                desired = clamp(target - p99, -30.0, 30.0)
                auto["gain_db"] = 0.85 * float(auto.get("gain_db", 0.0)) + 0.15 * desired
            contrast_on = bool(cfg.get("dsp_auto_contrast")) or (
                enabled and bool(cfg.get("auto_contrast")))
            if contrast_on:
                lo = float(np.percentile(mag, float(cfg["dsp_contrast_low_pct"])))
                hi = float(np.percentile(mag, float(cfg["dsp_contrast_high_pct"])))
                if hi - lo < 3.0:
                    hi = lo + 3.0
                old = auto.get("contrast") or {}
                a = 0.12 if old else 1.0
                auto["contrast"] = {
                    "lo": round((1 - a) * float(old.get("lo", lo)) + a * lo, 2),
                    "hi": round((1 - a) * float(old.get("hi", hi)) + a * hi, 2),
                }
        # Automatic display range
        range_comb = context.get("range_comb")
        range_m = context.get("range_m")
        if range_comb is not None and range_m is not None and len(range_m):
            if enabled and cfg.get("auto_view"):
                rmin = float(cfg["det_min_range_m"]); rmax = float(cfg["det_max_range_m"])
                if rmax < rmin:
                    rmin, rmax = rmax, rmin
                band = (range_m >= rmin) & (range_m <= rmax)
                if np.any(band):
                    idx = np.argmax(np.where(band, range_comb, -1e9))
                    peak_r = float(range_m[int(idx)])
                    span = float(cfg["auto_view_span_m"])
                    lo = peak_r - span / 2.0
                    hi = peak_r + span / 2.0
                    old = auto.get("view") or {}
                    a = 0.12 if old else 1.0
                    auto["view"] = {
                        "min_m": round((1 - a) * float(old.get("min_m", lo)) + a * lo, 3),
                        "max_m": round((1 - a) * float(old.get("max_m", hi)) + a * hi, 3),
                    }
        # Auto-scale the map
        if enabled and cfg.get("auto_map_size"):
            with self.lock:
                pts = list(self.points)[-800:]
            vals = []
            for p in pts:
                if now - p["t"] <= float(cfg["ui_point_age_s"]):
                    vals.extend((abs(float(p["x"])), abs(float(p["y"]))))
            if vals:
                desired = clamp(max(vals) * 1.25 + 0.25, 1.0, 30.0)
                old = float(auto.get("map_size_m", desired))
                auto["map_size_m"] = round(0.9 * old + 0.1 * desired, 3)
        # Peak info
        if context.get("range_comb") is not None and range_m is not None:
            idx = int(np.argmax(context["range_comb"]))
            auto["peak"] = {"range_m": round(float(range_m[idx]), 3),
                            "power_db": round(float(context["range_comb"][idx]), 2)}
        # Presence
        radar_present = self.presence
        motion_present = float(context.get("motion_db", 0.0)) >= float(cfg["auto_motion_threshold_db"])
        source_mode = str(cfg.get("auto_presence_source", "either"))
        if source_mode == "radar":
            present = bool(radar_present) if radar_present is not None else motion_present
            psource = "Radar"
        elif source_mode == "raw":
            present = motion_present
            psource = "Raw data"
        else:
            present = bool(radar_present) or motion_present
            psource = "Radar/Raw data" if radar_present is not None else "Raw data"
        auto["presence"] = bool(present)
        auto["presence_source"] = psource
        if present:
            if self.auto_present_since is None:
                self.auto_present_since = now
            self.auto_absent_since = None
            self.auto_last_present_t = now
        else:
            if self.auto_absent_since is None:
                self.auto_absent_since = now
            self.auto_present_since = None
        # Mode switch with hysteresis
        if enabled and cfg.get("auto_mode_switch") and self.source == "serial":
            if present and self.radar_mode != "raw" and self.auto_present_since is not None:
                if now - self.auto_present_since >= float(cfg["auto_raw_hold_s"]) and now - self.last_mode_cmd_t > 1.5:
                    ok, _ = self.send_command("raw_on")
                    if ok:
                        auto["mode_switch"] = {"state": "raw", "since": now}
                        self.last_mode_cmd_t = now
            elif (not present) and self.radar_mode == "raw" and self.auto_last_present_t is not None:
                if now - self.auto_last_present_t >= float(cfg["auto_track_after_s"]) and now - self.last_mode_cmd_t > 1.5:
                    ok, _ = self.send_command("raw_off")
                    if ok:
                        auto["mode_switch"] = {"state": "track", "since": now}
                        self.last_mode_cmd_t = now
        # Automatic recording
        if enabled and cfg.get("auto_record"):
            if present and not self.recording:
                self.start_recording(source="auto", reason="presence detected")
                auto["record"] = {"active": self.recording, "reason": "presence"}
            elif (not present) and self.recording and self.rec_source == "auto":
                if self.auto_last_present_t is not None and now - self.auto_last_present_t >= float(cfg["auto_record_post_s"]):
                    self.stop_recording(reason="presence ended")
                    auto["record"] = {"active": False, "reason": "post-roll finished"}
        else:
            auto["record"] = {"active": self.recording, "reason": "manual/off"}

    def process_raw(self, cfg: dict) -> dict:
        with self.dsp_lock:
            with self.lock:
                raw_copy = {t: (v[1], v[2]) for t, v in self.raw.items()}
            z1 = self._decode_iq(raw_copy[0x0A32][1]) if 0x0A32 in raw_copy else None
            z2 = self._decode_iq(raw_copy[0x0A34][1]) if 0x0A34 in raw_copy else None
            if z1 is None and z2 is None:
                return {"ok": False, "reason": "no raw data"}
            if z1 is None:
                z1 = np.zeros_like(z2)
            if z2 is None:
                z2 = np.zeros_like(z1)
            if z1.shape != z2.shape:
                rows = min(z1.shape[0], z2.shape[0])
                z1, z2 = z1[:rows], z2[:rows]
            rows, cols = z1.shape
            cfg_rows = int(cfg.get("dsp_rows", 0))
            if cfg_rows and cfg_rows != rows:
                # Payload wins; the firmware determines the row count.
                pass
            z1 = self._apply_clutter("rx1", z1, cfg)
            z2 = self._apply_clutter("rx2", z2, cfg)
            gain_db = (float(self.auto_state.get("gain_db", 0.0)) if cfg.get("auto_gain")
                       else float(cfg.get("dsp_display_gain_db", 0.0)))
            mag1 = self._view_matrix(z1, str(cfg.get("dsp_iq_view", "magnitude")))
            mag2 = self._view_matrix(z2, str(cfg.get("dsp_iq_view", "magnitude")))
            # Profiles/detection always use power dB, regardless of the view.
            db1 = 20.0 * np.log10(np.abs(z1) + 1e-9) + gain_db
            db2 = 20.0 * np.log10(np.abs(z2) + 1e-9) + gain_db
            db_mean = 10.0 * np.log10((np.abs(z1) ** 2 + np.abs(z2) ** 2) / 2.0 + 1e-9) + gain_db
            db_max = np.maximum(db1, db2)
            db1 = self._smooth("rx1", db1, cfg)
            db2 = self._smooth("rx2", db2, cfg)
            db_mean = self._smooth("combined", db_mean, cfg)
            db_max = np.maximum(db1, db2)
            # Smooth view matrices if needed (only meaningful for magnitude)
            view = str(cfg.get("dsp_iq_view", "magnitude"))
            if view == "magnitude":
                heat1, heat2 = db1, db2
                heat_comb = db_mean
            elif view == "power":
                heat1 = 10.0 * np.log10(np.abs(z1) ** 2 + 1e-9)
                heat2 = 10.0 * np.log10(np.abs(z2) ** 2 + 1e-9)
                heat_comb = 10.0 * np.log10((np.abs(z1) ** 2 + np.abs(z2) ** 2) / 2.0 + 1e-9)
            else:
                heat1, heat2 = mag1, mag2
                zc = (z1 + z2) / 2.0
                heat_comb = self._view_matrix(zc, view)
            # Noise and contrast values
            contrast_on = bool(cfg.get("dsp_auto_contrast")) or (
                bool(cfg.get("auto_enabled")) and bool(cfg.get("auto_contrast")))
            power_color = (dict(self.auto_state.get("contrast") or {}) if contrast_on
                           else {"lo": float(cfg["dsp_db_min"]), "hi": float(cfg["dsp_db_max"])})
            if power_color.get("hi", 0) - power_color.get("lo", 0) < 1.0:
                power_color = {"lo": float(cfg["dsp_db_min"]), "hi": float(cfg["dsp_db_max"])}
            if view == "phase":
                heat_color = {"lo": -180.0, "hi": 180.0}
                heat_cmap = "hsv"
                unit = "°"
            elif view in ("real", "imag"):
                lim = float(np.percentile(np.abs(heat_comb), 99.5)) or 1.0
                heat_color = {"lo": -lim, "hi": lim}
                heat_cmap = "icefire"
                unit = "I" if view == "real" else "Q"
            elif view == "power":
                heat_color = power_color
                heat_cmap = str(cfg.get("dsp_colormap", "magma"))
                unit = "dB"
            else:
                heat_color = power_color
                heat_cmap = str(cfg.get("dsp_colormap", "magma"))
                unit = "dB"
            # Profiles
            range1 = 10.0 * np.log10(np.sum(np.abs(z1) ** 2, axis=1) + 1e-9)
            range2 = 10.0 * np.log10(np.sum(np.abs(z2) ** 2, axis=1) + 1e-9)
            range_comb = 10.0 * np.log10(np.sum(np.abs(z1) ** 2 + np.abs(z2) ** 2, axis=1) / 2.0 + 1e-9)
            dop1 = 10.0 * np.log10(np.sum(np.abs(z1) ** 2, axis=0) + 1e-9)
            dop2 = 10.0 * np.log10(np.sum(np.abs(z2) ** 2, axis=0) + 1e-9)
            dop_comb = 10.0 * np.log10(np.sum(np.abs(z1) ** 2 + np.abs(z2) ** 2, axis=0) / 2.0 + 1e-9)
            range_start = float(cfg["dsp_range_start_m"])
            range_step = float(cfg["dsp_range_step_m"])
            range_m = range_start + range_step * np.arange(rows)
            doppler_mps = (np.arange(cols) - float(cfg["dsp_doppler_zero_bin"])) * float(cfg["dsp_doppler_bin_mps"])
            # Detection
            det_channel = str(cfg.get("det_channel", "mean"))
            det_map = {"mean": db_mean, "rx1": db1, "rx2": db2, "max": db_max}.get(det_channel, db_mean)
            dets, det_meta = self.detect(det_map, cfg, range_m, doppler_mps, det_channel)
            # Motion (frame-to-frame difference) and SNR map
            motion_m = np.zeros_like(db_mean)
            if self.prev_mag_comb is not None and self.prev_mag_comb.shape == db_mean.shape:
                motion_m = np.abs(db_mean - self.prev_mag_comb)
            self.prev_mag_comb = db_mean.copy()
            motion = float(np.mean(motion_m))
            self.motion_db = 0.85 * self.motion_db + 0.15 * motion if self.motion_db else motion
            noise_floor = float(det_meta.get("noise_db", np.percentile(db_mean, 10.0)))
            snr_m = db_mean - noise_floor
            power_db1 = db1          # includes display gain, independent of dsp_iq_view
            power_db2 = db2
            power_db_comb = db_mean
            # Waterfall row
            wf_mode = str(cfg.get("dsp_waterfall_mode", "auto"))
            if wf_mode == "manual":
                wf_row = int(clamp(int(cfg["dsp_waterfall_row"]), 0, rows - 1))
                wf_source = "manual"
            elif wf_mode == "max":
                wf_row = int(np.argmax(range_comb))
                wf_source = "max"
            else:
                wf_row = int(np.argmax(range_comb))
                wf_source = "auto"
            wf_db = 10.0 * np.log10(np.abs((z1[wf_row] + z2[wf_row]) / 2.0) ** 2 + 1e-9) + gain_db
            # Auto tick (uses the current values)
            context = {
                "mag_comb": db_mean, "range_comb": range_comb, "range_m": range_m,
                "motion_db": self.motion_db,
            }
            self._auto_tick(time.time(), cfg, context)
            # View range
            phys_min = float(range_m[0] - range_step / 2.0)
            phys_max = float(range_m[-1] + range_step / 2.0)
            if cfg.get("auto_enabled") and cfg.get("auto_view"):
                view_min = float(self.auto_state.get("view", {}).get("min_m", cfg["ui_view_min_m"]))
                view_max = float(self.auto_state.get("view", {}).get("max_m", cfg["ui_view_max_m"]))
            else:
                view_min = float(cfg["ui_view_min_m"])
                view_max = float(cfg["ui_view_max_m"])
            view_min = clamp(view_min, phys_min, phys_max)
            view_max = clamp(view_max, phys_min, phys_max)
            if view_max - view_min < max(0.05, range_step):
                view_min, view_max = phys_min, phys_max
            result = {
                "ok": True,
                "rows": rows, "cols": cols,
                "range_m": rounded_list(range_m, 4),
                "doppler_mps": rounded_list(doppler_mps, 4),
                "physical_min_m": round(phys_min, 4), "physical_max_m": round(phys_max, 4),
                "view": {"min_m": round(view_min, 4), "max_m": round(view_max, 4)},
                "view_mode": "auto" if (cfg.get("auto_enabled") and cfg.get("auto_view")) else "manual",
                "heat_color": {**heat_color, "unit": unit, "cmap": heat_cmap},
                "power_color": {**power_color, "unit": "dB", "cmap": str(cfg.get("dsp_colormap", "magma"))},
                "rx1": {"m": rounded_list(heat1, 2), "peak_range_m": round(float(range_m[int(np.argmax(range1))]), 3),
                        "peak_power_db": round(float(np.max(range1)), 2)},
                "rx2": {"m": rounded_list(heat2, 2), "peak_range_m": round(float(range_m[int(np.argmax(range2))]), 3),
                        "peak_power_db": round(float(np.max(range2)), 2)},
                "combined": {"m": rounded_list(heat_comb, 2),
                             "peak_range_m": round(float(range_m[int(np.argmax(range_comb))]), 3),
                             "peak_power_db": round(float(np.max(range_comb)), 2)},
                "power": {
                    "rx1": rounded_list(power_db1, 1),
                    "rx2": rounded_list(power_db2, 1),
                    "combined": rounded_list(power_db_comb, 1),
                },
                "motion_m": rounded_list(motion_m, 2),
                "snr_m": rounded_list(snr_m, 2),
                "difference_m": rounded_list(power_db1 - power_db2, 2),
                "noise_db": round(noise_floor, 2),
                "profiles": {
                    "range_rx1": rounded_list(range1, 2), "range_rx2": rounded_list(range2, 2),
                    "range_combined": rounded_list(range_comb, 2),
                    "doppler_rx1": rounded_list(dop1, 2), "doppler_rx2": rounded_list(dop2, 2),
                    "doppler_combined": rounded_list(dop_comb, 2),
                },
                "waterfall": {"db": rounded_list(wf_db, 2), "row": wf_row,
                              "range_m": round(float(range_m[wf_row]), 3),
                              "source": wf_source},
                "motion_db": round(float(self.motion_db), 3),
                "gain_db": round(gain_db, 2),
                "detections": dets,
                "detection_meta": det_meta,
            }
            self.last_detections = dets
            self.detection_meta = det_meta
            self.det_history.append({"t": time.time(), "top": dets[0] if dets else None,
                                     "count": len(dets)})
            if self.recording and cfg.get("rec_include_detections"):
                for det in dets:
                    self.rec_detections.append({"t": time.time(), **det})
            return result

    # ------------------------------------------------------------------
    # Snapshot / SSE
    # ------------------------------------------------------------------
    def build_snapshot(self) -> dict:
        with self.lock:
            cfg = copy.deepcopy(self.cfg)
        raw = self.process_raw(cfg)
        now = time.time()
        with self.lock:
            pts = []
            for p in self.points:
                if now - p["t"] <= float(cfg["ui_point_age_s"]):
                    pts.append({"x": round(p["x"], 3), "y": round(p["y"], 3),
                                "v": p["v"], "id": p["id"],
                                "age_s": round(now - p["t"], 1)})
            if len(pts) > 1200:
                pts = pts[-1200:]
            phase = dict(self.vitals)
            target = dict(self.target)
            target["age_s"] = round(now - target["t"], 1) if target.get("t") else None
            presence = self.presence
            distance_cm = self.distance_cm
            zone = dict(self.zone)
            version = self.version
            status = self.status
            logs = list(self.logs)[-10:]
            rec_frames = len(self.record)
            rec_source = self.rec_source
            rec_started = self.rec_started
            cfg_seq = self.cfg_seq
            include_cfg = cfg_seq != self.last_sent_cfg_seq
            if include_cfg:
                self.last_sent_cfg_seq = cfg_seq
            self.snap_seq += 1
            snap_seq = self.snap_seq
        # Vitals FFT outside the lock
        try:
            vital_fft = self._compute_vital_fft(cfg)
        except Exception as exc:
            vital_fft = None
            self.log("warn", f"Vital FFT error: {exc}")
        # Target: radar target or raw peak
        pseudo_target = None
        if raw.get("ok") and cfg.get("auto_enabled") and cfg.get("auto_peak_target"):
            peak = raw.get("combined", {})
            if peak.get("peak_power_db") is not None:
                pseudo_target = {"x": 0.0, "y": float(peak.get("peak_range_m", 0.0)),
                                 "source": "raw_peak", "power_db": peak.get("peak_power_db")}
        auto_view = self.auto_state.get("view", {})
        eff_map = (float(self.auto_state.get("map_size_m", cfg["ui_map_size_m"]))
                   if cfg.get("auto_enabled") and cfg.get("auto_map_size")
                   else float(cfg["ui_map_size_m"]))
        snap = {
            "seq": snap_seq,
            "t": now,
            "app": APP_VERSION,
            "source": self.source_label,
            "conn": {
                "connected": self.connected, "port": self.port,
                "baud": cfg["conn_baud"], "mode": self.radar_mode,
                "version": version, "status": status,
                "auto_reconnect": bool(cfg["conn_auto_reconnect"]),
            },
            "stats": self.stats(),
            "raw": raw,
            "points": pts,
            "target": target,
            "pseudo_target": pseudo_target,
            "presence": presence,
            "distance_cm": distance_cm,
            "zone": zone,
            "vitals": {
                "phase": {"total": phase.get("total"), "breath": phase.get("breath"),
                          "heart": phase.get("heart")},
                "seq": phase.get("seq", 0),
                "breath_rate": phase.get("breath_rate"),
                "heart_rate": phase.get("heart_rate"),
                "motion_db": round(float(self.motion_db), 3),
                "presence": presence,
                "distance_cm": distance_cm,
                "fft_breath_bpm": (vital_fft or {}).get("breath", {}).get("bpm") if vital_fft else None,
                "fft_heart_bpm": (vital_fft or {}).get("heart", {}).get("bpm") if vital_fft else None,
            },
            "vital_fft": vital_fft,
            "auto": {
                "enabled": bool(cfg.get("auto_enabled")),
                "contrast": self.auto_state.get("contrast"),
                "gain_db": round(float(self.auto_state.get("gain_db", 0.0)), 2),
                "view": auto_view,
                "map_size_m": round(eff_map, 3),
                "peak": self.auto_state.get("peak"),
                "presence": self.auto_state.get("presence"),
                "presence_source": self.auto_state.get("presence_source"),
                "mode_switch": self.auto_state.get("mode_switch"),
                "record": self.auto_state.get("record"),
            },
            "rec": {
                "recording": self.recording, "frames": rec_frames,
                "source": rec_source,
                "seconds": round(now - rec_started, 1) if self.recording and rec_started else 0,
                "max_frames": cfg["rec_max_frames"],
                "last_saved": self.last_saved_path,
                "last_snapshot": self.last_snapshot_path,
            },
            "logs": logs,
            "cfg_seq": cfg_seq,
        }
        if include_cfg:
            snap["config"] = cfg
        return snap

    def _stream_loop(self) -> None:
        last_err = 0.0
        hz = 10.0
        while not self.stop_evt.is_set():
            t0 = time.perf_counter()
            try:
                with self.lock:
                    hz = clamp(float(self.cfg["dsp_stream_hz"]), 2.0, 30.0)
                snap = self.build_snapshot()
                msg = json.dumps(snap, separators=(",", ":"), ensure_ascii=False,
                                 allow_nan=False)
                self.hub.publish(msg)
            except Exception as exc:
                now = time.time()
                if now - last_err > 2.0:
                    last_err = now
                    self.log("error", f"Stream error: {type(exc).__name__}: {exc}")
                    traceback.print_exc()
                time.sleep(0.3)
            elapsed = time.perf_counter() - t0
            sleep = max(0.0, 1.0 / hz - elapsed)
            self.stop_evt.wait(sleep)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------
    def config_snapshot(self) -> dict:
        with self.lock:
            return copy.deepcopy(self.cfg)

    def update_config(self, updates: dict) -> tuple[bool, str, list[str]]:
        if not isinstance(updates, dict):
            return False, "updates must be an object.", []
        changed = []
        with self.lock:
            for key, value in updates.items():
                if key not in DEFAULT_CONFIG:
                    return False, f"Unknown parameter: {key}", changed
                try:
                    new = coerce_value(key, value)
                except Exception as exc:
                    return False, f"{key}: {exc}", changed
                if self.cfg.get(key) != new:
                    self.cfg[key] = new
                    changed.append(key)
            if changed:
                self.cfg_seq += 1
                if any(k.startswith("dsp_") for k in changed):
                    # Reset DSP buffers on fundamental changes.
                    if any(k in changed for k in ("dsp_rows", "dsp_iq_view")):
                        self.bg = {"rx1": None, "rx2": None}
                        self.mag_smooth = {"rx1": None, "rx2": None, "combined": None}
                if "dsp_clutter_remove" in changed:
                    self.bg = {"rx1": None, "rx2": None}
        if changed:
            try:
                save_config(self.cfg, self.config_path)
            except Exception as exc:
                self.log("warn", f"Could not save configuration: {exc}")
            self.log("info", "Parameters changed: " + ", ".join(changed[:12]) +
                     (" …" if len(changed) > 12 else ""))
        return True, "OK", changed

    def apply_preset(self, name: str) -> tuple[bool, str, list[str]]:
        preset = PRESETS.get(name)
        if not preset:
            return False, f"Unknown preset: {name}", []
        return self.update_config(preset["updates"])

    def reset_config(self) -> tuple[bool, str, list[str]]:
        # Keep connection and recording path; reset everything else to defaults.
        keep = {k: self.cfg[k] for k in ("conn_port", "conn_baud", "rec_path",
                                          "fw_out_dir") if k in self.cfg}
        with self.lock:
            self.cfg = copy.deepcopy(DEFAULT_CONFIG)
            self.cfg.update(keep)
            self.cfg_seq += 1
        save_config(self.cfg, self.config_path)
        self.log("info", "Configuration reset to defaults.")
        return True, "Defaults loaded.", list(DEFAULT_CONFIG.keys())

    # ------------------------------------------------------------------
    # Recording / export
    # ------------------------------------------------------------------
    def start_recording(self, source: str = "manual", reason: str = "") -> tuple[bool, str]:
        with self.lock:
            if self.recording:
                return False, "Recording already running."
            self.record.clear()
            self.rec_detections.clear()
            self.recording = True
            self.rec_source = source
            self.rec_started = time.time()
        self.log("info", f"Recording started ({source}{': ' + reason if reason else ''}).")
        return True, "Recording started."

    def _record_path(self) -> Path:
        path = resolve_under_here(str(self.cfg["rec_path"]), base=HERE)
        if path.exists() and path.is_dir():
            path = path / f"advanced_session_{time.strftime('%Y%m%d_%H%M%S')}.npz"
        if not path.suffix:
            path = path.with_suffix(".npz")
        path.parent.mkdir(parents=True, exist_ok=True)
        return unique_path(path)

    def _rec_limit_hit(self) -> None:
        """Recording limit reached: collect data under the lock and save asynchronously."""
        if not self.recording or self.rec_limit_pending:
            return
        self.rec_limit_pending = True
        self.recording = False
        rec = list(self.record)
        dets = list(self.rec_detections)
        self.record.clear()
        self.rec_detections.clear()
        source = self.rec_source
        self.log("warn", f"Recording limit of {self.cfg['rec_max_frames']} frames reached – "
                         f"saving {len(rec)} frames …")
        threading.Thread(target=self._save_recording_async, args=(rec, dets, source),
                         name="rec-save", daemon=True).start()

    def _save_recording_async(self, rec: list[tuple[float, int, bytes]],
                              dets: list[dict], source: str) -> None:
        try:
            path = self._record_path()
            self._save_recording(path, rec, dets, source)
            self.last_saved_path = str(path)
            self.log("info", f"Recording saved (frame limit): {path.name} "
                             f"({len(rec)} frames, {path.stat().st_size / 1e6:.2f} MB)")
        except Exception as exc:
            self.log("error", f"Failed to save recording: {exc}")
        finally:
            self.rec_limit_pending = False

    def stop_recording(self, reason: str = "") -> tuple[bool, str, str | None]:
        with self.lock:
            if not self.recording:
                return False, "No recording active.", None
            self.recording = False
            rec = list(self.record)
            dets = list(self.rec_detections)
            self.record.clear()
            self.rec_detections.clear()
            source = self.rec_source
        if not rec:
            self.log("warn", "Recording was empty.")
            return False, "Recording was empty.", None
        try:
            path = self._record_path()
            self._save_recording(path, rec, dets, source)
            self.last_saved_path = str(path)
            self.log("info", f"Recording saved: {path.name} "
                             f"({len(rec)} frames, {path.stat().st_size / 1e6:.2f} MB)")
            return True, f"Recording saved: {path.name}", str(path)
        except Exception as exc:
            self.log("error", f"Failed to save recording: {exc}")
            return False, f"Save failed: {exc}", None

    def _save_recording(self, path: Path, rec: list[tuple[float, int, bytes]],
                        dets: list[dict], source: str) -> None:
        t = np.array([r[0] for r in rec], dtype=np.float64)
        types = np.array([r[1] for r in rec], dtype=np.uint16)
        lengths = np.array([len(r[2]) for r in rec], dtype=np.uint32)
        data = np.frombuffer(b"".join(r[2] for r in rec), dtype=np.uint8)
        meta = {
            "app": APP_VERSION, "created": iso_time(), "source": source,
            "version": self.version, "port": self.port,
            "config": self.config_snapshot(),
            "frames": len(rec), "dtypes": {f"{k:#06x}": v for k, v in
                                           collections.Counter(types.tolist()).items()},
        }
        arrays = {
            "t": t, "type": types, "length": lengths, "data": data,
            "meta": np.array(json.dumps(meta, ensure_ascii=False)),
        }
        if dets:
            arrays.update({
                "det_t": np.array([d.get("t", 0.0) for d in dets], dtype=np.float64),
                "det_range": np.array([d.get("range_m", 0.0) for d in dets], dtype=np.float32),
                "det_doppler": np.array([d.get("doppler_bin", 0) for d in dets], dtype=np.int16),
                "det_power": np.array([d.get("power_db", 0.0) for d in dets], dtype=np.float32),
                "det_snr": np.array([d.get("snr_db", 0.0) for d in dets], dtype=np.float32),
                "det_channel": np.array([str(d.get("channel", "")) for d in dets]),
            })
        np.savez_compressed(path, **arrays)

    def save_snapshot(self) -> tuple[bool, str, str | None]:
        with self.lock:
            raw = {t: v[2] for t, v in self.raw.items()}
            meta = {
                "app": APP_VERSION, "created": iso_time(), "version": self.version,
                "port": self.port, "mode": self.radar_mode,
                "config": copy.deepcopy(self.cfg),
                "detections": self.last_detections[:50],
                "zone": dict(self.zone), "target": dict(self.target),
                "presence": self.presence, "distance_cm": self.distance_cm,
            }
        if not raw:
            return False, "No raw data for a snapshot.", None
        SNAP_DIR.mkdir(parents=True, exist_ok=True)
        path = unique_path(SNAP_DIR / f"snapshot_{time.strftime('%Y%m%d_%H%M%S')}.npz")
        arrays = {}
        for ftype, payload in raw.items():
            arrays[f"a{ftype:04x}"] = np.frombuffer(payload, np.uint8).reshape(1, -1)
            arrays[f"t{ftype:04x}"] = np.array([time.time()])
        arrays["meta"] = np.array(json.dumps(meta, ensure_ascii=False))
        np.savez_compressed(path, **arrays)
        self.last_snapshot_path = str(path)
        self.log("info", f"Snapshot saved: {path.name}")
        return True, f"Snapshot: {path.name}", str(path)

    def export_csv(self) -> tuple[bool, str, str | None]:
        with self.lock:
            dets = list(self.rec_detections) if self.rec_detections else list(self.last_detections)
        if not dets:
            return False, "No detections to export.", None
        REC_DIR.mkdir(parents=True, exist_ok=True)
        path = unique_path(REC_DIR / f"detections_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        fields = ["t_iso", "t_unix", "range_m", "doppler_bin", "velocity_mps",
                  "power_db", "noise_db", "snr_db", "channel"]
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for det in dets:
                w.writerow({
                    "t_iso": iso_time(det.get("t", time.time())),
                    "t_unix": f"{det.get('t', time.time()):.6f}",
                    "range_m": det.get("range_m", ""),
                    "doppler_bin": det.get("doppler_bin", ""),
                    "velocity_mps": det.get("velocity_mps", ""),
                    "power_db": det.get("power_db", ""),
                    "noise_db": det.get("noise_db", ""),
                    "snr_db": det.get("snr_db", ""),
                    "channel": det.get("channel", ""),
                })
        self.log("info", f"Detections saved as CSV: {path.name} ({len(dets)} rows)")
        return True, f"CSV: {path.name}", str(path)

    def list_files(self) -> list[dict]:
        out = []
        for folder in (REC_DIR, SNAP_DIR, TUNING_DIR):
            if not folder.exists():
                continue
            for p in sorted(folder.glob("*"), key=lambda x: x.stat().st_mtime, reverse=True):
                if not p.is_file() or p.suffix.lower() not in (".npz", ".csv", ".bin", ".json", ".txt"):
                    continue
                try:
                    stat = p.stat()
                    rel = p.relative_to(HERE).as_posix()
                    out.append({
                        "name": p.name, "path": rel, "size": stat.st_size,
                        "mtime": iso_time(stat.st_mtime),
                        "kind": folder.name,
                        "url": "/api/download?file=" + rel,
                    })
                except Exception:
                    pass
                if len(out) >= 200:
                    return out
        return out

    def clear_log(self) -> tuple[bool, str]:
        with self.lock:
            self.logs.clear()
            self.log_seq += 1
            entry = {"seq": self.log_seq, "t": time.time(), "level": "info",
                     "msg": "Log cleared."}
            self.logs.append(entry)
        return True, "Log cleared."

    # ------------------------------------------------------------------
    # Firmware patch (generate an image only, never flash)
    # ------------------------------------------------------------------
    def generate_patch(self, base_id: str | None = None,
                       allow_unknown_base: bool = False,
                       baud: str = "auto",
                       out_root: Path | None = None) -> tuple[bool, str, dict]:
        with self.lock:
            cfg = copy.deepcopy(self.cfg)
        rows = int(cfg["fw_rows"]); tia = int(cfg["fw_tia"]); vga = int(cfg["fw_vga"])
        pa = int(cfg["fw_pa"]); hpf = int(cfg["fw_hpf"])
        rf = (tia != 6 or vga != 15 or pa != 10 or hpf != 150)
        if tia != 6 and vga != 15:
            msg = ("TIA and VGA must not be increased at the same time - both "
                   "gain stages add up. Test one stage first.")
            self.log("warn", msg)
            return False, msg, {"ok": False, "error": msg}
        if rows == 32 and not cfg.get("fw_allow_unsafe"):
            msg = ("32 rows at ~20 Hz overload the UART (about 160 kB/s > 139 kB/s). "
                   "Patch the frame rate first or enable 'allow unsafe rate'.")
            self.log("warn", msg)
            return False, msg, {"ok": False, "error": msg}
        if rf and not cfg.get("fw_allow_rf"):
            msg = ("RF patches (TIA/VGA/PA/HPF) must be allowed explicitly. "
                   "They are untested and can change reception and range.")
            self.log("warn", msg)
            return False, msg, {"ok": False, "error": msg}
        if baud not in ("auto", "keep", "1382400"):
            baud = "auto"

        legacy_available = (
            (HERE / "MR60BHA2_eeprom_1.6.12_baud1382400.bin").is_file()
            and (HERE / "mr60bha2_v1.6.12_app@0x8000_baud1382400.bin").is_file())
        base_entry: dict | None = None
        base_path: Path | None = None
        if base_id:
            try:
                base_entry, base_path = self.vault.get(str(base_id))
            except Exception as exc:
                msg = f"Firmware base not available: {exc}"
                self.log("warn", msg)
                return False, msg, {"ok": False, "error": msg}
        elif not legacy_available:
            # The sidebar button has no base selector. If no legacy files are
            # present, use the newest flashable image from the vault.
            try:
                images = [e for e in self.vault.list()
                          if e.get("buildable") and e.get("exists")]
            except Exception as exc:
                msg = f"Firmware vault not readable: {exc}"
                self.log("error", msg)
                return False, msg, {"ok": False, "error": msg}
            if not images:
                msg = ("No build base available. Import your original full image "
                       "(16-byte header + app) in the Firmware tab first.")
                self.log("warn", msg)
                return False, msg, {"ok": False, "error": msg}
            base_entry = images[0]
            base_id = str(base_entry.get("id"))
            try:
                _, base_path = self.vault.get(base_id)
            except Exception as exc:
                msg = f"Firmware base not available: {exc}"
                self.log("warn", msg)
                return False, msg, {"ok": False, "error": msg}

        if base_entry is not None:
            if not base_entry.get("buildable"):
                msg = (f"{base_entry.get('original_name', base_id)} is stored as a "
                       f"{base_entry.get('kind', 'backup')} backup. Only a full image "
                       "(16-byte OTA header + app) can be used as a build base.")
                self.log("warn", msg)
                return False, msg, {"ok": False, "error": msg}
            if not base_entry.get("known_base") and not allow_unknown_base:
                msg = ("This image is not a recognised official base. Verify every "
                       "patch offset yourself, then enable 'allow unknown base'.")
                self.log("warn", msg)
                return False, msg, {"ok": False, "error": msg}
            already = base_entry.get("inspection", {}).get("patches") or {}
            defaults = {"rows": 8, "tia_db": 6, "vga": 15, "pa": 10, "hpf_khz": 150}
            changed = [k for k, v in defaults.items()
                       if already.get(k) is not None and already.get(k) != v]
            if changed:
                msg = ("This image already contains tuning patches "
                       f"({', '.join(changed)}). Use an unmodified original image "
                       "as the build base.")
                self.log("warn", msg)
                return False, msg, {"ok": False, "error": msg}

        if base_entry is not None:
            safe_id = re.sub(r"[^A-Za-z0-9._-]+", "_", str(base_id))[:80] or "base"
            root = Path(out_root) if out_root else FIRMWARE_OUT_DIR
            out_dir = root / f"{time.strftime('%Y%m%d-%H%M%S')}_{safe_id}"
            counter = 2
            while out_dir.exists():
                out_dir = root / f"{time.strftime('%Y%m%d-%H%M%S')}_{safe_id}_{counter}"
                counter += 1
        else:
            try:
                out_dir = resolve_under_here(str(cfg["fw_out_dir"]), base=HERE)
            except Exception as exc:
                return False, str(exc), {"ok": False, "error": str(exc)}
        out_dir.mkdir(parents=True, exist_ok=True)

        cmd = [
            sys.executable, "-I", str(HERE / "patch_raw_window.py"),
            "--rows", str(rows), "--tia", str(tia), "--vga", str(vga),
            "--pa", str(pa), "--hpf", str(hpf), "--out-dir", str(out_dir),
            "--baud", baud,
        ]
        if base_path is not None:
            cmd.extend(["--base-full", str(base_path)])
            if base_entry is not None and not base_entry.get("known_base"):
                cmd.append("--allow-unknown-base")
        if rf:
            cmd.append("--allow-rf")
        if rows == 32:
            cmd.append("--allow-unsafe-rate")
        pretty = " ".join(shlex.quote(str(x)) for x in cmd)
        self.log("info", "Generating firmware tuning image (no flash): " + pretty)
        try:
            res = subprocess.run(cmd, cwd=str(HERE), capture_output=True, text=True,
                                 timeout=180, encoding="utf-8", errors="replace")
        except Exception as exc:
            msg = f"Could not start patch script: {exc}"
            self.log("error", msg)
            return False, msg, {"ok": False, "error": msg, "command": pretty}
        output = (res.stdout or "") + (("\n[stderr]\n" + res.stderr) if res.stderr else "")
        manifests = sorted(out_dir.glob("patch_manifest_*.json"),
                           key=lambda p: p.stat().st_mtime, reverse=True)
        manifest = None
        if manifests:
            try:
                manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
                manifest["_path"] = str(manifests[0])
            except Exception as exc:
                self.log("warn", f"Manifest not readable: {exc}")
        download_file = None
        flash_cmd = None
        if manifest:
            for item in manifest.get("outputs", {}).values():
                try:
                    rel = Path(item["path"]).resolve().relative_to(HERE.resolve())
                    item["download_file"] = str(rel).replace("\\", "/")
                except Exception:
                    item["download_file"] = None
            full = manifest.get("outputs", {}).get("full", {})
            download_file = full.get("download_file")
            if download_file:
                # The dashboard never flashes. Point the user to the browser-based
                # Web Flasher, which sends the image only after explicit confirmation.
                flash_cmd = (f"Open /flasher in this dashboard and select "
                             f"{Path(full['path']).name} - nothing is flashed automatically.")
        result = {
            "ok": res.returncode == 0,
            "returncode": res.returncode,
            "command": pretty,
            "stdout": output[-20000:],
            "stderr": (res.stderr or "")[-10000:],
            "manifest": manifest,
            "manifest_path": str(manifests[0]) if manifests else None,
            "download_file": download_file,
            "flash_command": flash_cmd,
            "base": None if base_entry is None else {
                "id": base_entry.get("id"),
                "name": base_entry.get("original_name"),
                "kind": base_entry.get("kind"),
                "known_base": base_entry.get("known_base"),
                "sha256": base_entry.get("sha256"),
            },
            "baud": baud,
            "warning": "NOT FLASHED. Flash manually only after explicit confirmation.",
        }
        self.last_patch = result
        if res.returncode == 0:
            full_path = (manifest.get("outputs", {}).get("full", {}).get("path")
                         if manifest else None)
            self.log("info", f"Tuning image generated: {full_path or out_dir}")
        else:
            self.log("error", f"Patch script error (rc={res.returncode}): {output[-500:]}")
        return (res.returncode == 0,
                "Patch image generated." if res.returncode == 0 else "Patch image failed.",
                result)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        self.stop_evt.set()
        self.conn_stop.set()
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        if self.recording:
            try:
                self.stop_recording(reason="Shutdown")
            except Exception:
                pass
        for thread in (self.worker, self.stream_thread, self.demo_thread, self.replay_thread):
            if thread and thread.is_alive():
                thread.join(timeout=1.5)
        self.log("info", "Engine stopped.")


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, engine: Engine):
        super().__init__(addr, handler)
        self.engine = engine


class Handler(BaseHTTPRequestHandler):
    server: Server

    def log_message(self, fmt, *args):
        # Suppress access logs; the engine logs the relevant events.
        return

    def _send_json(self, code: int, obj: dict) -> None:
        data = json.dumps(obj, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, code: int, data: bytes, content_type: str,
                    filename: str | None = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length > 2_000_000:
            raise ValueError("Request too large")
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw.decode("utf-8") or "{}")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ("/", "/index.html", "/advanced_dashboard.html"):
            try:
                html = HTML_PATH.read_bytes()
            except Exception as exc:
                msg = f"<h1>advanced_dashboard.html is missing</h1><pre>{exc}</pre>"
                self._send_bytes(500, msg.encode("utf-8"), "text/html; charset=utf-8")
                return
            self._send_bytes(200, html, "text/html; charset=utf-8")
            return
        if path in ("/flasher", "/flasher.html", "/webflasher.html"):
            try:
                html = FLASHER_PATH.read_bytes()
            except Exception as exc:
                msg = f"<h1>webflasher.html is missing</h1><pre>{exc}</pre>"
                self._send_bytes(500, msg.encode("utf-8"), "text/html; charset=utf-8")
                return
            self._send_bytes(200, html, "text/html; charset=utf-8")
            return
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        if path == "/api/state":
            eng = self.server.engine
            cfg = eng.config_snapshot()
            self._send_json(200, {
                "ok": True, "app": APP_VERSION,
                "config": cfg, "defaults": eng.defaults,
                "presets": PRESETS,
                "connection": eng.stats(),
                "recordings": eng.list_files(),
                "source": eng.source_label,
                "html_path": str(HTML_PATH),
                "last_patch": eng.last_patch,
                "logs": list(eng.logs)[-500:],
            })
            return
        if path == "/api/stream":
            self._stream()
            return
        if path == "/api/download":
            self._download(parsed)
            return
        if path == "/api/firmware/list":
            eng = self.server.engine
            try:
                images = eng.vault.list()
                self._send_json(200, {"ok": True, "images": images,
                                      "vault_dir": str(eng.vault.root)})
            except Exception as exc:
                self._send_json(200, {"ok": False, "images": [],
                                      "error": str(exc),
                                      "vault_dir": str(eng.vault.root)})
            return
        if path == "/api/firmware/image":
            qs = parse_qs(parsed.query)
            image_id = (qs.get("id") or [""])[0]
            try:
                entry, image_path = self.server.engine.vault.get(image_id)
                self._send_bytes(200, image_path.read_bytes(),
                                 "application/octet-stream",
                                 filename=image_path.name)
            except Exception as exc:
                self._send_json(404, {"ok": False, "error": str(exc)})
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/firmware/import":
            self._firmware_import(parsed)
            return
        try:
            req = self._read_json()
        except Exception as exc:
            self._send_json(400, {"ok": False, "error": f"JSON error: {exc}"})
            return
        eng = self.server.engine
        if parsed.path == "/api/config":
            updates = req.get("updates") if isinstance(req.get("updates"), dict) else req
            ok, msg, changed = eng.update_config(updates)
            self._send_json(200 if ok else 400, {
                "ok": ok, "message": msg, "changed": changed,
                "config": eng.config_snapshot(),
            })
            return
        if parsed.path == "/api/firmware/verify":
            image_id = str(req.get("id", "")).strip()
            try:
                self._send_json(200, eng.vault.verify(image_id))
            except Exception as exc:
                self._send_json(404, {"ok": False, "error": str(exc)})
            return
        if parsed.path == "/api/action":
            self._action(req)
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def _firmware_import(self, parsed) -> None:
        eng = self.server.engine
        try:
            length = int(self.headers.get("Content-Length", "0") or "0")
        except ValueError:
            length = 0
        if length <= 0:
            self._send_json(400, {"ok": False, "error": "empty upload"})
            return
        if length > MAX_FIRMWARE_UPLOAD:
            self._send_json(413, {"ok": False, "error":
                                  f"image too large (max {MAX_FIRMWARE_UPLOAD} bytes)"})
            return
        data = self.rfile.read(length)
        if len(data) != length:
            self._send_json(400, {"ok": False, "error": "incomplete upload"})
            return
        qs = parse_qs(parsed.query)
        source = (qs.get("source") or [self.headers.get("X-Firmware-Source", "other")])[0]
        name = (qs.get("name") or [self.headers.get("X-Filename", "firmware.bin")])[0]
        note = (qs.get("note") or [""])[0]
        try:
            entry = eng.vault.import_bytes(data, unquote(name), source=source, note=note)
        except Exception as exc:
            eng.log("warn", f"Firmware import rejected: {exc}")
            self._send_json(400, {"ok": False, "error": str(exc)})
            return
        eng.log("info", f"Firmware image stored in vault: {entry['id']} "
                        f"({entry['kind']}, {entry['size']} B)")
        self._send_json(200, {"ok": True, "image": entry,
                              "duplicate": bool(entry.get("duplicate"))})

    def _action(self, req: dict) -> None:
        eng = self.server.engine
        action = str(req.get("action", "")).strip()
        cfg = eng.config_snapshot()
        try:
            if action == "connect":
                ok, msg = eng.connect()
                self._send_json(200, {"ok": ok, "message": msg, "config": eng.config_snapshot()})
            elif action == "disconnect":
                ok, msg = eng.disconnect()
                self._send_json(200, {"ok": ok, "message": msg})
            elif action == "reconnect":
                eng.disconnect()
                time.sleep(0.2)
                ok, msg = eng.connect()
                self._send_json(200, {"ok": ok, "message": msg})
            elif action == "version":
                self._send_json(200, self._cmd_result(eng.send_command("version")))
            elif action == "raw_on":
                self._send_json(200, self._cmd_result(eng.send_command("raw_on")))
            elif action == "raw_off":
                self._send_json(200, self._cmd_result(eng.send_command("raw_off")))
            elif action == "zone_set":
                ok1, m1 = eng.send_command("zone_min", req.get("min", cfg["radar_zone_min"]))
                ok2, m2 = eng.send_command("zone_max", req.get("max", cfg["radar_zone_max"]))
                ok3, m3 = eng.send_command("zone_width", req.get("width", cfg["radar_zone_width"]))
                self._send_json(200, {"ok": ok1 and ok2 and ok3,
                                      "message": f"{m1}; {m2}; {m3}"})
            elif action == "zone_read":
                self._send_json(200, self._cmd_result(eng.send_command("zone_read")))
            elif action == "zone_reset":
                self._send_json(200, self._cmd_result(eng.send_command("zone_reset")))
            elif action == "echo":
                self._send_json(200, self._cmd_result(
                    eng.send_command("echo", req.get("value", cfg["radar_echo_value"]))))
            elif action == "flag":
                self._send_json(200, self._cmd_result(
                    eng.send_command("flag_a13", req.get("value", cfg["radar_flag_value"]))))
            elif action == "custom_frame":
                ok, msg = eng.send_custom(req.get("type", cfg["cmd_custom_type"]),
                                          req.get("hex", cfg["cmd_custom_hex"]))
                self._send_json(200, {"ok": ok, "message": msg})
            elif action == "record_start":
                ok, msg = eng.start_recording(source="manual", reason="Button")
                self._send_json(200, {"ok": ok, "message": msg})
            elif action == "record_stop":
                ok, msg, path = eng.stop_recording(reason="Button")
                self._send_json(200, {"ok": ok, "message": msg, "path": path,
                                      "recordings": eng.list_files()})
            elif action == "snapshot":
                ok, msg, path = eng.save_snapshot()
                self._send_json(200, {"ok": ok, "message": msg, "path": path,
                                      "recordings": eng.list_files()})
            elif action == "export_csv":
                ok, msg, path = eng.export_csv()
                self._send_json(200, {"ok": ok, "message": msg, "path": path,
                                      "recordings": eng.list_files()})
            elif action == "apply_preset":
                ok, msg, changed = eng.apply_preset(str(req.get("name", "default")))
                self._send_json(200, {"ok": ok, "message": msg, "changed": changed,
                                      "config": eng.config_snapshot()})
            elif action == "reset_config":
                ok, msg, changed = eng.reset_config()
                self._send_json(200, {"ok": ok, "message": msg, "changed": changed,
                                      "config": eng.config_snapshot()})
            elif action == "auto_preset":
                ok, msg, changed = eng.apply_preset("default")
                eng.update_config({"auto_enabled": True, "auto_mode_switch": False,
                                   "auto_record": False})
                self._send_json(200, {"ok": ok, "message": "Auto preset active: " + msg,
                                      "changed": changed, "config": eng.config_snapshot()})
            elif action == "auto_off":
                eng.update_config({"auto_enabled": False, "auto_mode_switch": False,
                                   "auto_record": False})
                self._send_json(200, {"ok": True, "message": "Auto mode off.",
                                      "config": eng.config_snapshot()})
            elif action == "generate_patch":
                ok, msg, result = eng.generate_patch(
                    base_id=str(req.get("base_id") or "") or None,
                    allow_unknown_base=bool(req.get("allow_unknown_base")),
                    baud=str(req.get("baud") or "auto"),
                )
                self._send_json(200, {"ok": ok, "message": msg,
                                      "error": None if ok else msg,
                                      "patch": result,
                                      "config": eng.config_snapshot()})
            elif action == "clear_log":
                ok, msg = eng.clear_log()
                self._send_json(200, {"ok": ok, "message": msg})
            else:
                self._send_json(400, {"ok": False, "error": f"Unknown action: {action}"})
        except Exception as exc:
            eng.log("error", f"Action {action} failed: {type(exc).__name__}: {exc}")
            self._send_json(500, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})

    @staticmethod
    def _cmd_result(result: tuple[bool, str]) -> dict:
        ok, msg = result
        return {"ok": ok, "message": msg}

    def _download(self, parsed) -> None:
        qs = parse_qs(parsed.query)
        rel = (qs.get("file") or [""])[0]
        if not rel:
            self._send_json(400, {"ok": False, "error": "file missing"})
            return
        try:
            path = resolve_under_here(rel, base=HERE)
            if not path.exists() or not path.is_file():
                raise FileNotFoundError(str(path))
            data = path.read_bytes()
            ext = path.suffix.lower()
            ctype = {
                ".csv": "text/csv; charset=utf-8",
                ".json": "application/json; charset=utf-8",
                ".txt": "text/plain; charset=utf-8",
                ".npz": "application/octet-stream",
                ".bin": "application/octet-stream",
            }.get(ext, "application/octet-stream")
            self._send_bytes(200, data, ctype, filename=path.name)
        except Exception as exc:
            self._send_json(404, {"ok": False, "error": str(exc)})

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        q = self.server.engine.hub.subscribe()
        try:
            # Send a complete state event immediately.
            self.wfile.write(b"event: hello\ndata: {\"ok\":true}\n\n")
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=10.0)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    continue
                self.wfile.write(b"data: " + msg.encode("utf-8") + b"\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            self.server.engine.hub.unsubscribe(q)


# ---------------------------------------------------------------------------
# Selftest
# ---------------------------------------------------------------------------

def make_synthetic_full_image(rows: int = 8) -> bytes:
    """Synthetic, hardware-free full image with valid vectors and patch sites.

    Used only by the selftest and the test suite. It contains no vendor code.
    """
    app = bytearray(42340)
    struct.pack_into("<I", app, 0, 0x20007218)   # initial stack pointer
    struct.pack_into("<I", app, 4, 0x80D1)       # reset vector (Thumb)
    struct.pack_into("<I", app, 0x998C, 0x05F5E100)  # stock UART clock literal
    row_bytes = {8: b"\x4f\xf4\x80\x62",
                 16: b"\x4f\xf4\x00\x62",
                 32: b"\x4f\xf4\x80\x52"}[rows]
    app[0x8470:0x8474] = row_bytes
    app[0x8492:0x8496] = row_bytes
    app[0x3024:0x3026] = b"\x23\x69"   # TIA 6 dB
    app[0x3028:0x302A] = b"\xa1\x68"
    app[0x302A:0x302C] = b"\x08\x46"
    app[0x33AE:0x33B0] = b"\x0f\x20"   # VGA 15
    app[0x33B6:0x33B8] = b"\x0f\x20"
    app[0x339E:0x33A0] = b"\x0a\x20"   # PA 10
    app[0x33BE:0x33C0] = b"\x96\x20"   # HPF 150
    header = bytes.fromhex("64a564a5 5a100000 02000000 00000000")
    return header + bytes(app)


def selftest() -> int:
    print("Selftest - Advanced Radar Console")
    tmp_root = Path(tempfile.mkdtemp(prefix="radar_selftest_"))
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    cfg["auto_enabled"] = True
    cfg["auto_contrast"] = True
    cfg["det_enabled"] = True
    eng = Engine(cfg, source="selftest", start_threads=False,
                 config_path=tmp_root / "config.json",
                 vault_dir=tmp_root / "vault")
    rng = np.random.default_rng(42)
    t = 1.0
    p1 = synthetic_raw_payload(rng, t, 8, 32, 1)
    p2 = synthetic_raw_payload(rng, t, 8, 32, 2)
    eng.on_frame(0, 0x0A32, p1, p1)
    eng.on_frame(0, 0x0A34, p2, p2)
    eng.on_frame(0, 0xFFFF, bytes([1, 6, 12, 0]), b"")
    eng.on_frame(0, 0x0F09, struct.pack("<H", 1), b"")
    eng.on_frame(0, 0x0A17, struct.pack("<ff", 0.2, 1.4), b"")
    eng.on_frame(0, 0x0A16, struct.pack("<If", 1, 140.0), b"")
    eng.on_frame(0, 0x0A13, struct.pack("<fff", 0.5, 0.35, 0.1), b"")
    eng.on_frame(0, 0x0A04, make_point_frame([{"x": 0.2, "y": 1.4, "v": 10, "id": 1}]), b"")
    snap = eng.build_snapshot()
    assert snap["raw"]["ok"], snap["raw"]
    assert len(snap["raw"]["rx1"]["m"]) == 8
    assert len(snap["raw"]["rx1"]["m"][0]) == 32
    assert len(snap["raw"]["range_m"]) == 8
    assert len(snap["raw"]["doppler_mps"]) == 32
    assert "detections" in snap["raw"]
    assert snap["config"] if "config" in snap else True
    assert snap["stats"]["frames_total"] >= 8
    assert snap["auto"]["enabled"] is True
    # JSON round trip
    payload = json.dumps(snap, allow_nan=False)
    assert len(payload) > 1000

    # Firmware vault and build pipeline (synthetic image, no vendor code)
    vault_ok = False
    firmware_build = False
    firmware_error = None
    try:
        sample = make_synthetic_full_image(8)
        entry = eng.vault.import_bytes(sample, "synthetic-full.bin", source="other")
        assert entry["buildable"] and entry["kind"] == "full", entry
        duplicate = eng.vault.import_bytes(sample, "again.bin", source="other")
        assert duplicate["duplicate"] and duplicate["id"] == entry["id"], duplicate
        assert eng.vault.verify(entry["id"])["ok"]
        raw = bytes(range(256)) * 16
        raw_entry = eng.vault.import_bytes(raw, "swd-dump.bin", source="swd-dump")
        assert raw_entry["kind"] == "raw" and not raw_entry["buildable"], raw_entry
        try:
            eng.vault.import_bytes(raw, "not-vendor.bin", source="vendor-download")
            raise AssertionError("raw image accepted as a vendor image")
        except ValueError:
            pass
        vault_ok = True

        eng.update_config({"fw_rows": 16})
        ok_build, build_msg, build_result = eng.generate_patch(
            base_id=entry["id"], allow_unknown_base=True, baud="auto",
            out_root=tmp_root / "out")
        assert ok_build, f"{build_msg}: {build_result.get('stdout', '')[-500:]}"
        full_path = Path(build_result["manifest"]["outputs"]["full"]["path"])
        built_info = inspect_image(full_path.read_bytes())
        assert built_info["patches"]["rows"] == 16, built_info["patches"]
        assert built_info["patches"]["uart_baud"] == 1388889, built_info["patches"]
        reject_ok, _, _ = eng.generate_patch(
            base_id=entry["id"], allow_unknown_base=False,
            out_root=tmp_root / "out")
        assert not reject_ok, "unknown base accepted without confirmation"
        firmware_build = True
    except Exception as exc:
        firmware_error = f"{type(exc).__name__}: {exc}"
        print(f"Firmware studio selftest failed: {firmware_error}")

    # Check HTML
    html_ok = HTML_PATH.exists()
    html_markers = False
    if html_ok:
        html_text = HTML_PATH.read_text(encoding="utf-8")
        html_markers = all(m in html_text for m in (
            'id="heat1"', 'id="heat2"', 'id="waterfall"', 'id="map"',
            'id="surface3d"', 'id="fwOutput"', 'id="fwVaultList"',
            'id="fwBase"', 'id="fwBuild"', 'id="fwImport"'))
    # Web Flasher page
    flasher_ok = FLASHER_PATH.exists()
    flasher_markers = False
    if flasher_ok:
        flasher_text = FLASHER_PATH.read_text(encoding="utf-8")
        flasher_markers = all(m in flasher_text for m in ('id="flashFile"',
                                                          'id="flashStart"',
                                                          'id="flashLog"'))
    # Patch script and vault module presence (without generating an image)
    patch_script = (HERE / "patch_raw_window.py").exists()
    vault_module = (HERE / "firmware_vault.py").exists()
    eng.shutdown()
    shutil.rmtree(tmp_root, ignore_errors=True)
    ok = (html_ok and html_markers and flasher_ok and flasher_markers
          and patch_script and vault_module and vault_ok and firmware_build)
    print(json.dumps({
        "ok": ok,
        "rows": len(snap["raw"]["rx1"]["m"]),
        "cols": len(snap["raw"]["rx1"]["m"][0]),
        "detections": len(snap["raw"]["detections"]),
        "json_bytes": len(payload),
        "html_exists": html_ok,
        "html_markers": html_markers,
        "flasher_exists": flasher_ok,
        "flasher_markers": flasher_markers,
        "patch_script": patch_script,
        "vault_module": vault_module,
        "vault_ok": vault_ok,
        "firmware_build": firmware_build,
        "firmware_error": firmware_error,
        "version": snap["conn"]["version"],
    }, indent=2, ensure_ascii=False))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    try:  # Windows console: unexpected bytes in radar debug text must never crash
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(
        description="Advanced Radar Console for MR60BHA2 / HLK-LD6002",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", default=None, help="COM port (overrides the configuration)")
    ap.add_argument("--baud", type=int, default=None, help="COM baud rate (usually irrelevant for USB-CDC)")
    ap.add_argument("--http", type=int, default=8770, help="HTTP port (default 8770)")
    ap.add_argument("--host", default="127.0.0.1", help="Bind address (default 127.0.0.1)")
    ap.add_argument("--demo", action="store_true", help="synthetic data instead of radar")
    ap.add_argument("--replay", type=Path, default=None, help="play back an NPZ recording")
    ap.add_argument("--replay-speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser automatically")
    ap.add_argument("--no-auto-connect", action="store_true", help="do not connect automatically")
    ap.add_argument("--config", type=Path, default=CONFIG_PATH, help="configuration file")
    ap.add_argument("--selftest", action="store_true", help="run the selftest and exit")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    cfg = load_config(args.config)
    if args.port:
        cfg["conn_port"] = args.port
    if args.baud:
        cfg["conn_baud"] = args.baud
    if args.no_auto_connect:
        cfg["conn_auto_connect"] = False
    source = "demo" if args.demo else ("replay" if args.replay else "serial")
    if source == "replay" and (args.replay is None or not args.replay.exists()):
        print(f"Replay file not found: {args.replay}", file=sys.stderr)
        return 2

    for folder in (REC_DIR, SNAP_DIR, TUNING_DIR, VAULT_DIR, FIRMWARE_OUT_DIR):
        folder.mkdir(parents=True, exist_ok=True)

    engine = Engine(cfg, source=source, replay_path=args.replay,
                    replay_speed=args.replay_speed, config_path=args.config)
    try:
        server = Server((args.host, args.http), Handler, engine)
    except OSError as exc:
        print(f"HTTP server could not bind {args.host}:{args.http}: {exc}", file=sys.stderr)
        engine.shutdown()
        return 3

    url = f"http://{args.host}:{args.http}/"
    print("=" * 72)
    print(f"Advanced Radar Console {APP_VERSION}")
    print(f"Source : {engine.source_label}" + (f" ({args.replay})" if args.replay else ""))
    print(f"URL    : {url}")
    print("Stop   : Ctrl+C")
    print("=" * 72, flush=True)
    if not args.no_open:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nStopping …", flush=True)
    finally:
        engine.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
