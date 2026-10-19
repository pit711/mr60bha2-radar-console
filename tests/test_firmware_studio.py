#!/usr/bin/env python3
"""Tests for the firmware vault and the tuning-image build pipeline.

All images are synthetic and generated inside the test. No vendor firmware,
SDK code or flash backup is required or read.

Run from the project directory:
    python -I -m unittest discover -s tests -v
"""

from __future__ import annotations

import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from firmware_vault import (  # noqa: E402
    APP_BASE,
    BAUD_OFF,
    FirmwareVault,
    HEADER_MAGIC,
    HPF_SITE,
    PA_SITE,
    ROW_BYTES,
    ROW_SITES,
    TIA_SITES,
    VGA_SITES,
    inspect_image,
)

APP_SIZE = 42340
SP = 0x20007218
RESET = APP_BASE + 0xD1


def make_app(rows: int = 8, baud: int = 115200) -> bytes:
    """Build a synthetic application with all patch signatures in place."""
    app = bytearray(APP_SIZE)
    struct.pack_into("<I", app, 0, SP)
    struct.pack_into("<I", app, 4, RESET)

    row_bytes = next(k for k, v in ROW_BYTES.items() if v == rows)
    for offset in ROW_SITES:
        app[offset:offset + 4] = row_bytes

    for offset, mapping in TIA_SITES:
        value = next(k for k, v in mapping.items() if v == 6)
        app[offset:offset + 2] = value
    for offset, mapping in VGA_SITES:
        value = next(k for k, v in mapping.items() if v == 15)
        app[offset:offset + 2] = value
    app[PA_SITE[0]:PA_SITE[0] + 2] = next(
        k for k, v in PA_SITE[1].items() if v == 10)
    app[HPF_SITE[0]:HPF_SITE[0] + 2] = next(
        k for k, v in HPF_SITE[1].items() if v == 150)

    clock = 0x05F5E100 if baud == 115200 else 0x007E9000
    struct.pack_into("<I", app, BAUD_OFF, clock)
    return bytes(app)


def make_full(rows: int = 8, baud: int = 115200) -> bytes:
    return HEADER_MAGIC + b"\x00" * 12 + make_app(rows, baud)


class InspectTests(unittest.TestCase):
    def test_full_image_is_buildable_and_reports_defaults(self):
        info = inspect_image(make_full())
        self.assertEqual(info["kind"], "full")
        self.assertTrue(info["buildable"])
        self.assertEqual(info["size"], 16 + APP_SIZE)
        self.assertEqual(info["app_size"], APP_SIZE)
        self.assertEqual(info["patches"]["rows"], 8)
        self.assertEqual(info["patches"]["tia_db"], 6)
        self.assertEqual(info["patches"]["vga"], 15)
        self.assertEqual(info["patches"]["pa"], 10)
        self.assertEqual(info["patches"]["hpf_khz"], 150)
        self.assertEqual(info["patches"]["uart_baud"], 115200)
        self.assertEqual(info["reset_addr"], f"0x{RESET & ~1:08X}")

    def test_patched_image_is_reported_as_such(self):
        info = inspect_image(make_full(rows=16, baud=1_388_889))
        self.assertEqual(info["patches"]["rows"], 16)
        self.assertEqual(info["patches"]["uart_baud"], 1_388_889)

    def test_app_only_image_is_backup_only(self):
        info = inspect_image(make_app())
        self.assertEqual(info["kind"], "app")
        self.assertFalse(info["buildable"])
        self.assertIn("Application only", info["warning"])

    def test_raw_dump_is_backup_only(self):
        info = inspect_image(b"\xAA" * 4096)
        self.assertEqual(info["kind"], "raw")
        self.assertFalse(info["buildable"])
        self.assertIsNone(info["patches"])
        self.assertIn("raw backup", info["warning"])

    def test_too_small_is_rejected(self):
        with self.assertRaises(ValueError):
            inspect_image(b"\x00" * 10)

    def test_full_image_with_bad_stack_pointer_is_rejected(self):
        data = bytearray(make_full())
        struct.pack_into("<I", data, 16, 0x10000000)
        with self.assertRaises(ValueError):
            inspect_image(bytes(data))


class VaultTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.vault = FirmwareVault(Path(self.tmp.name) / "vault")
        self.data = make_full()

    def tearDown(self):
        self.tmp.cleanup()

    def test_import_list_get_verify(self):
        entry = self.vault.import_bytes(
            self.data, "my original.bin", source="vendor-download",
            note="official download")
        self.assertFalse(entry["duplicate"])
        self.assertTrue(entry["buildable"])
        self.assertEqual(entry["kind"], "full")
        self.assertTrue((self.vault.root / entry["file"]).is_file())

        images = self.vault.list()
        self.assertEqual(len(images), 1)
        self.assertTrue(images[0]["exists"])

        result = self.vault.verify(entry["id"])
        self.assertTrue(result["ok"])

        _, path = self.vault.get(entry["id"])
        self.assertEqual(path.read_bytes(), self.data)

    def test_duplicate_import_returns_existing_entry(self):
        first = self.vault.import_bytes(self.data, "a.bin", source="vendor-download")
        second = self.vault.import_bytes(self.data, "b.bin", source="other")
        self.assertTrue(second["duplicate"])
        self.assertEqual(first["id"], second["id"])
        self.assertEqual(len(self.vault.list()), 1)

    def test_verify_detects_tampering(self):
        entry = self.vault.import_bytes(self.data, "a.bin", source="vendor-download")
        path = self.vault.root / entry["file"]
        raw = bytearray(path.read_bytes())
        raw[-1] ^= 0xFF
        path.write_bytes(bytes(raw))
        self.assertFalse(self.vault.verify(entry["id"])["ok"])

    def test_vendor_source_requires_full_image(self):
        with self.assertRaises(ValueError):
            self.vault.import_bytes(b"\xAA" * 4096, "dump.bin",
                                    source="vendor-download")

    def test_swd_dump_is_stored_as_raw_backup(self):
        entry = self.vault.import_bytes(b"\x55" * 2048, "dump.bin",
                                        source="swd-dump", note="SWD readout")
        self.assertEqual(entry["kind"], "raw")
        self.assertFalse(entry["buildable"])
        self.assertEqual(entry["source"], "swd-dump")

    def test_unknown_source_is_rejected(self):
        with self.assertRaises(ValueError):
            self.vault.import_bytes(self.data, "a.bin", source="internet")

    def test_path_traversal_id_is_rejected(self):
        self.vault.import_bytes(self.data, "a.bin", source="other")
        with self.assertRaises(KeyError):
            self.vault.get("../manifest.json")


class PatchPipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.base = self.root / "synthetic_full.bin"
        self.base.write_bytes(make_full())

    def tearDown(self):
        self.tmp.cleanup()

    def _run(self, out_dir: Path, *extra: str) -> subprocess.CompletedProcess:
        cmd = [
            sys.executable, "-I", str(ROOT / "patch_raw_window.py"),
            "--base-full", str(self.base), "--rows", "16",
            "--tia", "6", "--vga", "15", "--pa", "10", "--hpf", "150",
            "--baud", "auto", "--out-dir", str(out_dir),
            *extra,
        ]
        return subprocess.run(cmd, cwd=str(ROOT), capture_output=True,
                              text=True, encoding="utf-8", errors="replace",
                              timeout=120)

    def test_unknown_base_requires_explicit_flag(self):
        res = self._run(self.root / "out_blocked")
        self.assertNotEqual(res.returncode, 0)
        self.assertIn("unknown", (res.stdout + res.stderr).lower())

    def test_build_applies_rows_and_baud_patch(self):
        out_dir = self.root / "out_ok"
        res = self._run(out_dir, "--allow-unknown-base")
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)

        manifests = list(out_dir.glob("patch_manifest_*.json"))
        self.assertEqual(len(manifests), 1)
        manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
        full_path = Path(manifest["outputs"]["full"]["path"])
        self.assertTrue(full_path.is_file())

        info = inspect_image(full_path.read_bytes())
        self.assertEqual(info["kind"], "full")
        self.assertEqual(info["patches"]["rows"], 16)
        self.assertEqual(info["patches"]["uart_baud"], 1_388_889)
        self.assertEqual(info["size"], 16 + APP_SIZE)

    def test_double_baud_patch_is_a_no_op(self):
        # A base that already carries the fast clock must not be patched again.
        fast = self.root / "fast_full.bin"
        fast.write_bytes(make_full(baud=1_388_889))
        out_dir = self.root / "out_fast"
        cmd = [
            sys.executable, "-I", str(ROOT / "patch_raw_window.py"),
            "--base-full", str(fast), "--allow-unknown-base",
            "--rows", "16", "--tia", "6", "--vga", "15", "--pa", "10",
            "--hpf", "150", "--baud", "auto", "--out-dir", str(out_dir),
        ]
        res = subprocess.run(cmd, cwd=str(ROOT), capture_output=True,
                             text=True, encoding="utf-8", errors="replace",
                             timeout=120)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        manifest = json.loads(
            next(out_dir.glob("patch_manifest_*.json")).read_text(encoding="utf-8"))
        info = inspect_image(
            Path(manifest["outputs"]["full"]["path"]).read_bytes())
        self.assertEqual(info["patches"]["uart_baud"], 1_388_889)


class CliTests(unittest.TestCase):
    def test_inspect_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "test.bin"
            image.write_bytes(make_full())
            res = subprocess.run(
                [sys.executable, "-I", str(ROOT / "firmware_vault.py"),
                 "--vault", str(Path(tmp) / "vault"), "inspect", str(image)],
                cwd=str(ROOT), capture_output=True, text=True,
                encoding="utf-8", errors="replace", timeout=60)
            self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
            self.assertIn("full (flashable)", res.stdout)
            self.assertIn("uart_baud=115200", res.stdout)


if __name__ == "__main__":
    unittest.main()
