#!/usr/bin/env python3
"""MR60BHA2 / ADT6101P: prepare tuning images - **without flashing**.

Two ways to supply the base image:

1. Vault/custom base (recommended):
     --base-full PATH
   PATH is a full image (16-byte OTA header + application). The vault stores
   one for you; see firmware_vault.py. The official 1.6.12 image and the
   project's baud-patched 1.6.12 image are recognised by SHA-256. Other images
   need --allow-unknown-base after you have verified the byte sites.

2. Legacy files in this directory:
     mr60bha2_v1.6.12_app@0x8000_baud1382400.bin   (app without the header)
     MR60BHA2_eeprom_1.6.12_baud1382400.bin        (full image)

The script patches only known, previously verified bytes. It does not open a
COM port and contains no flash code. An image must only be flashed after
explicit confirmation and through the 115200-baud bridge.

Examples:
  python -I patch_raw_window.py --rows 16
  python -I patch_raw_window.py --base-full firmware_vault/IMAGE.bin --rows 16
  python -I patch_raw_window.py --rows 16 --tia 12 --allow-rf
  python -I patch_raw_window.py --rows 16 --vga 18 --allow-rf
  python -I patch_raw_window.py --rows 16 --pa 12 --hpf 400 --allow-rf

The official image runs the radar UART at 115200 baud. This project's bridge
expects 1,388,889 baud, so --baud auto (the default) applies the known UART
clock literal patch when the stock value is present. Use --baud keep to leave
the base untouched.

By default only the safe 16-row window is generated. RF changes
(TIA gain, VGA/LNA, PA bias, HPF) additionally require --allow-rf.
TIA and VGA must not be increased at the same time.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
APP_SRC = HERE / "mr60bha2_v1.6.12_app@0x8000_baud1382400.bin"
FULL_SRC = HERE / "MR60BHA2_eeprom_1.6.12_baud1382400.bin"

APP_SHA256 = "18b4a4401ee9d271230123655b163526aa24ec1ac7bc360c25b7f456c062f355"
FULL_SHA256 = "bab8e4a9c09582a08710b2e14ff3ac4f7cb4788885faa881ce6f2cd7cba39ddc"
FULL_HEADER = 0x10

# Recognised full images (full-image SHA-256).
KNOWN_FULL_SHA256 = {
    "official-1.6.12":
        "10906046140495b54d854b3630a27503c1adb36c614febba8279c854d284ec3d",
    "official-1.6.12-baud1382400":
        "bab8e4a9c09582a08710b2e14ff3ac4f7cb4788885faa881ce6f2cd7cba39ddc",
}
KNOWN_APP_SHA256 = {
    "official-1.6.12":
        "b8fef2d50086a745e9f13bb8db8106fff369501872d33b7e2d2152d13c569247",
    "official-1.6.12-baud1382400":
        "18b4a4401ee9d271230123655b163526aa24ec1ac7bc360c25b7f456c062f355",
}

# UART clock literal in uartInit: hclk / baud divisor. The app calls
# uartInit(UART0, 115200); patching the 100 MHz literal to 8,294,400 makes the
# integer divisor 72, giving 100 MHz / 72 = 1,388,889 baud.
BAUD_LITERAL_OFF = 0x998C
BAUD_LITERAL_STOCK = struct.pack("<I", 0x05F5E100)
BAUD_LITERAL_FAST = struct.pack("<I", 0x007E9000)

# App offsets (file without header). Full image offset = app offset + 0x10.
SEND_LEN_SITES = (
    ("RX1 tf_send length (0x0A32)", 0x8470, b"\x4f\xf4\x80\x62"),  # mov.w r2,#0x400
    ("RX2 tf_send length (0x0A34)", 0x8492, b"\x4f\xf4\x80\x62"),
)
SEND_LEN_BYTES = {
    8: b"\x4f\xf4\x80\x62",   # 0x400 = 1024 B = 8 rows
    16: b"\x4f\xf4\x00\x62",  # 0x800 = 2048 B = 16 rows
    32: b"\x4f\xf4\x80\x52",  # 0x1000 = 4096 B = 32 rows
}

TIA_SITES = (
    ("TIA1 Q gain (config +0x10)", 0x3024, b"\x23\x69"),  # ldr r3,[r4,#0x10]
    ("TIA0 Q gain (config +0x08)", 0x3028, b"\xa1\x68"),  # ldr r1,[r4,#8]
    ("TIA0 I gain (mov r0,r1)", 0x302A, b"\x08\x46"),     # mov r0,r1
)
TIA_BYTES = {
    12: (b"\x02\x23", b"\x02\x21", b"\x02\x20"),  # movs r3/r1/r0,#2 -> 12 dB
    18: (b"\x03\x23", b"\x03\x21", b"\x03\x20"),  # movs r3/r1/r0,#3 -> 18 dB
}

VGA_SITES = (
    ("VGA/LNA channel 1 (+0x0c)", 0x33AE, b"\x0f\x20"),  # movs r0,#15
    ("VGA/LNA channel 2 (+0x14)", 0x33B6, b"\x0f\x20"),
)
VGA_BYTES = {18: b"\x12\x20", 20: b"\x14\x20"}  # 18/20 = 36/40 dB at 2 dB/bit

PA_SITE = ("PA bias (+0x34/+0x38)", 0x339E, b"\x0a\x20")  # movs r0,#10
PA_BYTES = {12: b"\x0c\x20", 15: b"\x0f\x20"}

HPF_SITE = ("HPF selection (+0x28)", 0x33BE, b"\x96\x20")  # movs r0,#150
HPF_BYTES = {400: b"\x97\x20"}  # comparison != 150 -> different branch


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def patch(buf: bytearray, off: int, old: bytes, new: bytes, label: str) -> dict:
    if len(old) != len(new):
        raise SystemExit(f"{label}: internal error, old/new lengths differ")
    got = bytes(buf[off:off + len(old)])
    if got != old:
        raise SystemExit(
            f"{label}: expected {old.hex(' ')}, found {got.hex(' ')} "
            f"at app offset 0x{off:04X} - wrong base image or already patched"
        )
    buf[off:off + len(new)] = new
    return {
        "label": label,
        "app_offset": f"0x{off:04X}",
        "full_offset": f"0x{off + FULL_HEADER:04X}",
        "old": old.hex(" "),
        "new": new.hex(" "),
    }


def _safe_stem(name: str) -> str:
    stem = re.sub(r"[^A-Za-z0-9._@+-]+", "_", Path(name).stem).strip("._")
    return (stem or "base")[:90]


def load_base(args) -> tuple[bytes, bytes, dict, bool]:
    """Return (full_src, app_src, source_info, custom)."""
    if args.base_full is not None:
        full_path = args.base_full
        full_src = full_path.read_bytes()
        if len(full_src) < FULL_HEADER + 64:
            raise SystemExit(f"{full_path}: file is too small for a full image")
        if full_src[:4] != bytes.fromhex("64a564a5"):
            raise SystemExit(f"{full_path}: OTA header magic 64a564a5 is missing. "
                             "Use a full image (16-byte header + app), not an "
                             "app-only image or a raw chip dump.")
        app_src = full_src[FULL_HEADER:]
        app_sha = sha256(app_src)
        full_sha = sha256(full_src)
        known = None
        for label, digest in KNOWN_FULL_SHA256.items():
            if full_sha == digest:
                known = label
                break
        if known is None:
            for label, digest in KNOWN_APP_SHA256.items():
                if app_sha == digest:
                    known = label + "-app"
                    break
        if not known and not (args.allow_unknown_base or args.force):
            raise SystemExit(
                f"{full_path}: base image SHA-256 {full_sha} is not a recognised "
                "official image. Verify all patch offsets yourself, then rerun "
                "with --allow-unknown-base. The byte signatures are still checked."
            )
        source = {
            "mode": "base-full",
            "full": {"path": str(full_path), "sha256": full_sha},
            "app": {"path": f"{full_path} (offset 0x{FULL_HEADER:X})",
                    "sha256": app_sha},
            "known_base": known,
        }
        return full_src, app_src, source, True

    full_src = FULL_SRC.read_bytes()
    app_src = APP_SRC.read_bytes()
    if len(full_src) != FULL_HEADER + len(app_src):
        raise SystemExit("Full image length does not match the app image")
    if full_src[FULL_HEADER:] != app_src:
        raise SystemExit("App area of the full image differs from the app image")
    if not args.force:
        if sha256(app_src) != APP_SHA256:
            raise SystemExit("App base image does not have the expected SHA-256 "
                             "(--force only if the addresses were verified)")
        if sha256(full_src) != FULL_SHA256:
            raise SystemExit("Full image does not have the expected SHA-256 "
                             "(--force only if the addresses were verified)")
    source = {
        "mode": "legacy-files",
        "full": {"path": str(FULL_SRC), "sha256": sha256(full_src)},
        "app": {"path": str(APP_SRC), "sha256": sha256(app_src)},
        "known_base": "official-1.6.12-baud1382400",
    }
    return full_src, app_src, source, False


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Prepare a tuning image for MR60BHA2 (no flashing)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--base-full", type=Path,
                    help="full base image (16-byte header + app); default: the "
                         "legacy files in this directory")
    ap.add_argument("--allow-unknown-base", action="store_true",
                    help="allow a full base image whose SHA-256 is not recognised "
                         "(byte signatures are still verified)")
    ap.add_argument("--baud", choices=("auto", "keep", "1382400"), default="auto",
                    help="UART clock patch: auto applies the known patch when the "
                         "stock literal is present; keep leaves the base untouched")
    ap.add_argument("--rows", type=int, choices=(8, 16, 32), default=16,
                    help="rows in the raw window starting at row 8 (default: 16)")
    ap.add_argument("--tia", type=int, choices=(6, 12, 18), default=6,
                    help="TIA gain in dB (6=current, 12=+6 dB, 18=+12 dB)")
    ap.add_argument("--vga", type=int, choices=(15, 18, 20), default=15,
                    help="VGA/LNA gain steps (15=30 dB, 18=36 dB, 20=40 dB)")
    ap.add_argument("--pa", type=int, choices=(10, 12, 15), default=10,
                    help="PA bias, 4 bit (default 10)")
    ap.add_argument("--hpf", type=int, choices=(150, 400), default=150,
                    help="high-pass filter 150 or 400 kHz")
    ap.add_argument("--out-dir", type=Path, default=HERE / "tuning",
                    help="output directory (default: tuning/)")
    ap.add_argument("--allow-rf", action="store_true",
                    help="explicitly allow RF changes (TIA/VGA/PA/HPF)")
    ap.add_argument("--allow-unsafe-rate", action="store_true",
                    help="allow 32 rows at 20 Hz (UART overload!)")
    ap.add_argument("--force", action="store_true",
                    help="skip the base image hash check (not recommended)")
    args = ap.parse_args()

    rf_requested = (args.tia != 6 or args.vga != 15 or args.pa != 10
                    or args.hpf != 150)
    if rf_requested and not args.allow_rf:
        raise SystemExit("RF change requested but --allow-rf is missing. "
                         "These patches are not recommended without measurement.")
    if args.tia != 6 and args.vga != 15:
        raise SystemExit("Do not increase TIA and VGA gain at the same time: both "
                         "stages add up (+12 dB or more). Test one stage first, "
                         "then decide. --allow-rf alone does not permit this "
                         "combination.")
    if args.rows == 32 and not args.allow_unsafe_rate:
        raise SystemExit(
            "32 rows at 20 Hz produce about 160.7 kB/s, but the UART only "
            "handles about 138.9 kB/s. Do not use without a frame-rate patch "
            "(15-16 Hz). Use --allow-unsafe-rate to generate it anyway "
            "(analysis only)."
        )

    full_src, app_src, source, custom = load_base(args)
    app = bytearray(app_src)
    patches: list[dict] = []

    # 0) UART clock literal (radar <-> bridge baud rate)
    baud_state = struct.unpack_from("<I", app, BAUD_LITERAL_OFF)[0]
    baud_applied = False
    if args.baud == "keep":
        if baud_state == struct.unpack("<I", BAUD_LITERAL_STOCK)[0]:
            print("WARNING: base runs the radar UART at the stock 115200 baud; "
                  "this project's bridge expects 1,388,889 baud. Raw streaming "
                  "will not work until you apply --baud 1382400.", file=sys.stderr)
    else:
        if baud_state == struct.unpack("<I", BAUD_LITERAL_STOCK)[0]:
            patches.append(patch(app, BAUD_LITERAL_OFF, BAUD_LITERAL_STOCK,
                                 BAUD_LITERAL_FAST,
                                 "UART clock literal uartInit (1,388,889 baud)"))
            baud_applied = True
        elif baud_state != struct.unpack("<I", BAUD_LITERAL_FAST)[0]:
            if args.baud == "1382400":
                raise SystemExit(
                    "UART clock literal is neither the stock value nor the known "
                    "patched value at app offset 0x998C; this base is not "
                    "supported for the baud patch.")
            print("WARNING: unknown UART clock literal at app offset 0x998C; "
                  "baud rate left unchanged.", file=sys.stderr)

    # 1) Window size: 8 -> 16/32 rows
    if args.rows != 8:
        for label, off, old in SEND_LEN_SITES:
            patches.append(patch(app, off, old, SEND_LEN_BYTES[args.rows], label))

    # 2) TIA gain (optional, before VGA)
    if args.tia != 6:
        for (label, off, old), new in zip(TIA_SITES, TIA_BYTES[args.tia]):
            patches.append(patch(app, off, old, new, label))

    # 3) VGA/LNA (optional)
    if args.vga != 15:
        for label, off, old in VGA_SITES:
            patches.append(patch(app, off, old, VGA_BYTES[args.vga], label))

    # 4) PA bias (optional)
    if args.pa != 10:
        label, off, old = PA_SITE
        patches.append(patch(app, off, old, PA_BYTES[args.pa], label))

    # 5) HPF (optional)
    if args.hpf != 150:
        label, off, old = HPF_SITE
        patches.append(patch(app, off, old, HPF_BYTES[args.hpf], label))

    tag_parts = [f"win{args.rows}"]
    if args.tia != 6:
        tag_parts.append(f"tia{args.tia}")
    if args.vga != 15:
        tag_parts.append(f"vga{args.vga}")
    if args.pa != 10:
        tag_parts.append(f"pa{args.pa}")
    if args.hpf != 150:
        tag_parts.append(f"hpf{args.hpf}")
    tag = "_".join(tag_parts)

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    if custom:
        stem = _safe_stem(args.base_full.name)
        if baud_applied and "baud1382400" not in stem:
            stem += "_baud1382400"
        app_out = out_dir / f"{stem}_app_{tag}.bin"
        full_out = out_dir / f"{stem}_{tag}.bin"
    else:
        app_out = out_dir / f"mr60bha2_v1.6.12_app@0x8000_baud1382400_{tag}.bin"
        full_out = out_dir / f"MR60BHA2_eeprom_1.6.12_baud1382400_{tag}.bin"
    full = full_src[:FULL_HEADER] + bytes(app)
    app_out.write_bytes(app)
    full_out.write_bytes(full)

    payload = args.rows * 128
    frame_bytes = payload + 9  # 8-byte header + 1-byte data checksum
    rate_hz = 19.583
    load = 2 * frame_bytes * rate_hz / 138_888.9 * 100.0
    manifest = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source,
        "options": {
            "rows": args.rows,
            "tia": args.tia,
            "vga": args.vga,
            "pa": args.pa,
            "hpf": args.hpf,
            "baud": args.baud,
        },
        "patches": patches,
        "outputs": {
            "app": {"path": str(app_out), "sha256": sha256(bytes(app)),
                    "size": len(app)},
            "full": {"path": str(full_out), "sha256": sha256(full),
                     "size": len(full)},
        },
        "expected": {
            "payload_bytes_per_frame": payload,
            "frame_bytes_per_frame": frame_bytes,
            "raw_rate_hz_per_channel": rate_hz,
            "uart_load_percent": round(load, 1),
            "range_centers_m": [1.0, 1.0 + (args.rows - 1) * 0.125],
            "range_window_edges_m": [1.0 - 0.0625,
                                     1.0 + (args.rows - 0.5) * 0.125],
            "raw_outer_radius_m": 1.0 + args.rows * 0.125,
        },
        "warning": "NOT FLASHED. RF patches only after measurement and confirmation. "
                   "Flash only through the 115200-baud bridge (bootloader).",
    }
    manifest_path = out_dir / f"patch_manifest_{tag}.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False),
                             encoding="utf-8")

    print("MR60BHA2 tuning image prepared (not flashed)")
    print(f"  Base        : {source['known_base'] or 'unrecognised'} "
          f"({source['full']['sha256']})")
    print(f"  Rows        : {args.rows}  ({payload} B/frame, "
          f"row centers {1.0:.3f}-{1.0 + (args.rows - 1) * 0.125:.3f} m, "
          f"window {1.0 - 0.0625:.3f}-{1.0 + (args.rows - 0.5) * 0.125:.3f} m)")
    print(f"  UART load   : {load:.1f} % at {rate_hz:.3f} Hz and 138.889 kBaud")
    print(f"  TIA/VGA/PA/HPF: {args.tia}/{args.vga}/{args.pa}/{args.hpf}")
    print(f"  UART baud   : {'patched to 1,388,889' if baud_applied else 'kept as in base'}")
    print(f"  App         : {app_out}")
    print(f"                SHA-256 {sha256(bytes(app))}")
    print(f"  Full image  : {full_out}")
    print(f"                SHA-256 {sha256(full)}")
    print(f"  Manifest    : {manifest_path}")
    if patches:
        print("  Patches     :")
        for p in patches:
            print(f"    {p['label']}: app {p['app_offset']} / full {p['full_offset']}  "
                  f"{p['old']} -> {p['new']}")
    else:
        print("  Patches     : none (copy of the base image)")
    print("\nDo not flash without confirmation. RF patches are untested.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
