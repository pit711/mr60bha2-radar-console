#!/usr/bin/env python3
"""Firmware vault for MR60BHA2 / HLK-LD6002 (Andar ADT6101P).

Stores immutable copies of firmware images, validates their headers, and
tracks SHA-256 hashes. It never opens a serial port and never flashes.

What the vault accepts
----------------------
* **full**  – 16-byte OTA header + application. Flashable and usable as a
  base for tuning patches.
* **app**   – application only (starts with the vector table). Stored as a
  backup; not flashable because the OTA header is missing.
* **raw**   – any other dump (for example a raw SWD readout). Stored as a
  backup only; not flashable and not patchable.

The radar bootloader is write-only over UART. A true chip dump requires an
external SWD programmer; this tool cannot read the radar through COM.

CLI:
  python -I firmware_vault.py inspect IMAGE
  python -I firmware_vault.py import IMAGE --source vendor-download
  python -I firmware_vault.py import DUMP.bin --source swd-dump --note "SWD readout"
  python -I firmware_vault.py list
  python -I firmware_vault.py verify IMAGE_ID
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import struct
import sys
import threading
import time
from pathlib import Path

HEADER_SIZE = 16
HEADER_MAGIC = bytes.fromhex("64a564a5")
SP_MIN, SP_MAX = 0x20000000, 0x20100000
APP_BASE = 0x8000
MAX_IMAGE = 8 * 1024 * 1024
SOURCES = ("vendor-download", "swd-dump", "other")

KNOWN_BASES = {
    "official-1.6.12": {
        "label": "Seeed MR60BHA2 1.6.12 (official, 115200 baud)",
        "full_sha256": "10906046140495b54d854b3630a27503c1adb36c614febba8279c854d284ec3d",
        "app_sha256": "b8fef2d50086a745e9f13bb8db8106fff369501872d33b7e2d2152d13c569247",
    },
    "official-1.6.12-baud1382400": {
        "label": "MR60BHA2 1.6.12 with 1,388,889 baud patch",
        "full_sha256": "bab8e4a9c09582a08710b2e14ff3ac4f7cb4788885faa881ce6f2cd7cba39ddc",
        "app_sha256": "18b4a4401ee9d271230123655b163526aa24ec1ac7bc360c25b7f456c062f355",
    },
}

# Patch sites are mirrored from patch_raw_window.py for read-only inspection.
ROW_SITES = (0x8470, 0x8492)
ROW_BYTES = {
    b"\x4f\xf4\x80\x62": 8,
    b"\x4f\xf4\x00\x62": 16,
    b"\x4f\xf4\x80\x52": 32,
}
TIA_SITES = (
    (0x3024, {b"\x23\x69": 6, b"\x02\x23": 12, b"\x03\x23": 18}),
    (0x3028, {b"\xa1\x68": 6, b"\x02\x21": 12, b"\x03\x21": 18}),
    (0x302A, {b"\x08\x46": 6, b"\x02\x20": 12, b"\x03\x20": 18}),
)
VGA_SITES = (
    (0x33AE, {b"\x0f\x20": 15, b"\x12\x20": 18, b"\x14\x20": 20}),
    (0x33B6, {b"\x0f\x20": 15, b"\x12\x20": 18, b"\x14\x20": 20}),
)
PA_SITE = (0x339E, {b"\x0a\x20": 10, b"\x0c\x20": 12, b"\x0f\x20": 15})
HPF_SITE = (0x33BE, {b"\x96\x20": 150, b"\x97\x20": 400})
BAUD_OFF = 0x998C
BAUD_BYTES = {
    0x05F5E100: 115200,
    0x007E9000: 1388889,
}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _plausible_vector_table(app: bytes) -> bool:
    if len(app) < 8:
        return False
    sp = _u32(app, 0)
    reset = _u32(app, 4)
    reset_addr = reset & ~1
    return (SP_MIN <= sp < SP_MAX and sp % 8 == 0
            and (reset & 1) == 1
            and APP_BASE <= reset_addr < APP_BASE + len(app))


def _read_sites(app: bytes, sites) -> int | None:
    if isinstance(sites, tuple) and sites and isinstance(sites[0], int):
        sites = (sites,)
    values = []
    for site in sites:
        if isinstance(site, tuple):
            offset, mapping = site
        else:
            offset, mapping = site, ROW_BYTES
        if offset + 2 > len(app):
            values.append(None)
            continue
        values.append(mapping.get(bytes(app[offset:offset + 2])))
    if values and all(v is not None for v in values) and len(set(values)) == 1:
        return values[0]
    return None


def _read_rows(app: bytes) -> int | None:
    values = []
    for offset in ROW_SITES:
        if offset + 4 > len(app):
            values.append(None)
            continue
        values.append(ROW_BYTES.get(bytes(app[offset:offset + 4])))
    if values and all(v is not None for v in values) and len(set(values)) == 1:
        return values[0]
    return None


def inspect_image(data: bytes) -> dict:
    """Validate and describe an image. Raises ValueError on invalid input."""
    if len(data) < 32:
        raise ValueError("Image is too small (at least 32 bytes required).")
    if len(data) > MAX_IMAGE:
        raise ValueError(f"Image is larger than {MAX_IMAGE} bytes.")

    has_header = data[:4] == HEADER_MAGIC
    if has_header:
        kind = "full"
        header = data[:HEADER_SIZE]
        app = data[HEADER_SIZE:]
        if len(app) < 16:
            raise ValueError("OTA header is present but the application is empty.")
    else:
        header = b""
        app = data
        kind = "app" if _plausible_vector_table(app) else "raw"

    sp = reset = None
    if kind in ("full", "app"):
        if len(app) < 8:
            raise ValueError("Application is too small to contain a vector table.")
        sp = _u32(app, 0)
        reset = _u32(app, 4)
        reset_addr = reset & ~1
        if not (SP_MIN <= sp < SP_MAX):
            raise ValueError(
                f"Stack pointer 0x{sp:08X} is outside SRAM "
                f"(0x{SP_MIN:08X}-0x{SP_MAX:08X}).")
        if sp % 8:
            raise ValueError(f"Stack pointer 0x{sp:08X} is not 8-byte aligned.")
        if not (reset & 1):
            raise ValueError(f"Reset vector 0x{reset:08X} has no Thumb bit.")
        if not (APP_BASE <= reset_addr < APP_BASE + len(app)):
            raise ValueError(
                f"Reset vector 0x{reset:08X} points outside the application "
                f"(0x{APP_BASE:08X}-0x{APP_BASE + len(app):08X}).")

    full_sha = sha256(data)
    app_sha = sha256(app)
    known = None
    for key, entry in KNOWN_BASES.items():
        if kind == "full" and full_sha == entry["full_sha256"]:
            known = key
            break
        if kind == "app" and app_sha == entry["app_sha256"]:
            known = key + "-app"
            break

    patches = None
    if kind in ("full", "app"):
        baud = (struct.unpack_from("<I", app, BAUD_OFF)[0]
                if BAUD_OFF + 4 <= len(app) else None)
        patches = {
            "rows": _read_rows(app),
            "tia_db": _read_sites(app, TIA_SITES),
            "vga": _read_sites(app, VGA_SITES),
            "pa": _read_sites(app, PA_SITE),
            "hpf_khz": _read_sites(app, HPF_SITE),
            "uart_baud": BAUD_BYTES.get(baud),
        }

    info = {
        "kind": kind,
        "buildable": kind == "full",
        "size": len(data),
        "app_size": len(app),
        "sha256": full_sha,
        "app_sha256": app_sha,
        "known_base": known,
        "sp": None if sp is None else f"0x{sp:08X}",
        "reset": None if reset is None else f"0x{reset:08X}",
        "reset_addr": None if reset is None else f"0x{(reset & ~1):08X}",
        "patches": patches,
    }
    if kind == "full":
        info["header"] = {
            "magic": header[:4].hex(),
            "field1": f"0x{_u32(header, 4):08X}",
            "field2": f"0x{_u32(header, 8):08X}",
            "field3": f"0x{_u32(header, 12):08X}",
        }
    if kind == "raw":
        info["warning"] = ("No OTA header and no vector table detected. "
                           "Stored as a raw backup only; not flashable or patchable.")
    elif kind == "app":
        info["warning"] = ("Application only. The 16-byte OTA header is missing, "
                           "so this image is not directly flashable.")
    return info


def _safe_stem(name: str) -> str:
    stem = Path(name or "firmware").stem
    stem = re.sub(r"[^A-Za-z0-9._@+-]+", "_", stem).strip("._")
    return (stem or "firmware")[:80]


class FirmwareVault:
    """Local, append-only vault with a JSON manifest."""

    def __init__(self, root: Path):
        self.root = Path(root)
        self.manifest_path = self.root / "manifest.json"
        self.lock = threading.Lock()
        self.root.mkdir(parents=True, exist_ok=True)

    # -- manifest ---------------------------------------------------------
    def _load(self) -> dict:
        if not self.manifest_path.exists():
            return {"version": 1, "images": []}
        try:
            data = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise ValueError(f"Vault manifest is unreadable: {exc}") from exc
        if not isinstance(data, dict) or not isinstance(data.get("images"), list):
            raise ValueError("Vault manifest has an unexpected structure.")
        return data

    def _save(self, data: dict) -> None:
        tmp = self.manifest_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
        os.replace(tmp, self.manifest_path)

    # -- operations -------------------------------------------------------
    def import_bytes(self, data: bytes, original_name: str,
                     source: str = "other", note: str = "") -> dict:
        if source not in SOURCES:
            raise ValueError(f"Unknown source {source!r}; allowed: {', '.join(SOURCES)}")
        info = inspect_image(data)
        if source == "vendor-download" and info["kind"] != "full":
            raise ValueError("A vendor image must be the full image "
                             "(16-byte OTA header + application).")
        digest = info["sha256"]
        with self.lock:
            manifest = self._load()
            for entry in manifest["images"]:
                if entry.get("sha256") == digest:
                    return {**entry, "duplicate": True}
            image_id = time.strftime("%Y%m%d-%H%M%S") + "-" + digest[:8]
            filename = f"{image_id}_{_safe_stem(original_name)}.bin"
            path = self.root / filename
            suffix = 1
            while path.exists():
                filename = f"{image_id}_{suffix}_{_safe_stem(original_name)}.bin"
                path = self.root / filename
                suffix += 1
            with open(path, "xb") as handle:
                handle.write(data)
            entry = {
                "id": image_id,
                "file": filename,
                "original_name": str(original_name or "firmware.bin")[:200],
                "sha256": digest,
                "app_sha256": info["app_sha256"],
                "size": info["size"],
                "kind": info["kind"],
                "buildable": info["buildable"],
                "known_base": info["known_base"],
                "imported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "source": source,
                "note": str(note or "")[:500],
                "inspection": info,
            }
            manifest["images"].append(entry)
            self._save(manifest)
            return {**entry, "duplicate": False}

    def list(self) -> list[dict]:
        with self.lock:
            manifest = self._load()
        images = []
        for entry in reversed(manifest["images"]):
            row = dict(entry)
            row["exists"] = (self.root / entry.get("file", "")).is_file()
            images.append(row)
        return images

    def get(self, image_id: str) -> tuple[dict, Path]:
        with self.lock:
            manifest = self._load()
        for entry in manifest["images"]:
            if entry.get("id") == image_id:
                path = (self.root / entry.get("file", "")).resolve()
                root = self.root.resolve()
                if root not in path.parents:
                    raise ValueError("Vault entry points outside the vault directory.")
                if not path.is_file():
                    raise FileNotFoundError(f"Vault file is missing: {path.name}")
                return dict(entry), path
        raise KeyError(f"Unknown firmware image: {image_id}")

    def verify(self, image_id: str) -> dict:
        entry, path = self.get(image_id)
        digest = sha256(path.read_bytes())
        return {
            "id": image_id,
            "ok": digest == entry.get("sha256"),
            "expected": entry.get("sha256"),
            "actual": digest,
            "size": path.stat().st_size,
        }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_human(info: dict) -> None:
    print(f"Kind        : {info['kind']} "
          f"({'flashable' if info['buildable'] else 'backup only'})")
    print(f"Size        : {info['size']} B (app {info['app_size']} B)")
    print(f"SHA-256     : {info['sha256']}")
    print(f"App SHA-256 : {info['app_sha256']}")
    if info.get("sp"):
        print(f"SP / reset  : {info['sp']} / {info['reset']} "
              f"(reset address {info['reset_addr']})")
    if info.get("known_base"):
        print(f"Known base  : {info['known_base']}")
    if info.get("patches"):
        p = info["patches"]
        print("Patch state : " + ", ".join(
            f"{k}={v}" for k, v in p.items()))
    if info.get("warning"):
        print(f"Warning     : {info['warning']}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Validate, store, and verify MR60BHA2 firmware images "
                    "(never flashes)")
    ap.add_argument("--vault", type=Path,
                    default=Path(__file__).resolve().parent / "firmware_vault",
                    help="vault directory (default: ./firmware_vault)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_inspect = sub.add_parser("inspect", help="validate an image and show details")
    p_inspect.add_argument("image", type=Path)

    p_import = sub.add_parser("import", help="store an immutable copy")
    p_import.add_argument("image", type=Path)
    p_import.add_argument("--source", choices=SOURCES, default="other")
    p_import.add_argument("--note", default="")

    sub.add_parser("list", help="list stored images")

    p_verify = sub.add_parser("verify", help="recompute the SHA-256 of a stored image")
    p_verify.add_argument("id")

    args = ap.parse_args()
    vault = FirmwareVault(args.vault)

    if args.cmd == "inspect":
        try:
            info = inspect_image(args.image.read_bytes())
        except Exception as exc:
            print(f"Invalid image: {exc}", file=sys.stderr)
            return 2
        _print_human(info)
        return 0

    if args.cmd == "import":
        try:
            entry = vault.import_bytes(args.image.read_bytes(), args.image.name,
                                       source=args.source, note=args.note)
        except Exception as exc:
            print(f"Import failed: {exc}", file=sys.stderr)
            return 2
        state = "already stored" if entry.get("duplicate") else "stored"
        print(f"{state}: {entry['id']}")
        print(f"  File    : {vault.root / entry['file']}")
        print(f"  Kind    : {entry['kind']} "
              f"({'flashable' if entry['buildable'] else 'backup only'})")
        print(f"  SHA-256 : {entry['sha256']}")
        return 0

    if args.cmd == "list":
        images = vault.list()
        if not images:
            print("Vault is empty.")
            return 0
        for entry in images:
            flag = "flashable" if entry.get("buildable") else "backup"
            missing = "" if entry.get("exists") else "  [file missing]"
            print(f"{entry['id']}  {entry['size']:>8} B  {flag:<9} "
                  f"{entry.get('source', '?'):<15} {entry.get('original_name', '')}"
                  f"{missing}")
        return 0

    if args.cmd == "verify":
        try:
            result = vault.verify(args.id)
        except Exception as exc:
            print(f"Verify failed: {exc}", file=sys.stderr)
            return 2
        print(("OK  " if result["ok"] else "MISMATCH  ") + args.id)
        print(f"  expected: {result['expected']}")
        print(f"  actual  : {result['actual']}")
        return 0 if result["ok"] else 1

    return 2


if __name__ == "__main__":
    sys.exit(main())
