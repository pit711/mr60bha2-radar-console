#!/usr/bin/env python3
"""Tests for the firmware vault, inbox watcher and tuning-image pipeline.

All images are synthetic and generated inside the test. No vendor firmware,
SDK code or flash backup is required or read.

Run from the project directory:
    python -I -m unittest discover -s tests -v
"""

from __future__ import annotations

import copy
import json
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from advanced_dashboard import DEFAULT_CONFIG, Engine  # noqa: E402
from firmware_vault import (  # noqa: E402
    APP_BASE,
    BAUD_OFF,
    FirmwareVault,
    HEADER_MAGIC,
    HPF_SITE,
    KNOWN_BASES,
    PA_SITE,
    ROW_BYTES,
    ROW_SITES,
    TIA_SITES,
    VGA_SITES,
    device_version,
    inspect_image,
    match_device_version,
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


class VersionTests(unittest.TestCase):
    def test_device_reply_is_normalised(self):
        info = device_version(bytes([4, 1, 6, 12]))
        self.assertEqual(info["version"], "1.6.12")
        self.assertEqual(info["tuple"], [1, 6, 12])
        self.assertEqual(info["family"], 4)
        self.assertEqual(info["raw"], "0401060C")
        self.assertEqual(info["full"], "4.1.6.12")

    def test_three_byte_reply_is_accepted(self):
        info = device_version(bytes([1, 6, 12]))
        self.assertEqual(info["version"], "1.6.12")
        self.assertIsNone(info["family"])

    def test_short_or_empty_reply_is_rejected(self):
        self.assertIsNone(device_version(b""))
        self.assertIsNone(device_version(None))
        self.assertIsNone(device_version(b"\x01\x06"))

    def test_known_base_versions_match_their_replies(self):
        for key, meta in KNOWN_BASES.items():
            info = device_version(bytes.fromhex(meta["version_raw"]))
            self.assertEqual(tuple(info["tuple"]), tuple(meta["version"]), key)

    def _entry(self, **updates):
        entry = {
            "exists": True, "buildable": True, "known_base": "official-1.6.12",
            "source": "other", "imported_at": "2026-01-01T00:00:00",
            "id": "a", "sha256": "x",
        }
        entry.update(updates)
        return entry

    def test_match_prefers_vendor_source(self):
        vendor = self._entry(id="vendor", source="vendor-download",
                             imported_at="2026-01-01T00:00:00")
        other = self._entry(id="other", source="other",
                            imported_at="2026-02-01T00:00:00")
        matched = match_device_version([other, vendor], bytes([4, 1, 6, 12]))
        self.assertEqual(matched["id"], "vendor")

    def test_match_prefers_unpatched_variant(self):
        patched = self._entry(id="patched", source="vendor-download",
                              known_base="official-1.6.12-baud1382400")
        plain = self._entry(id="plain", source="vendor-download")
        matched = match_device_version([patched, plain], bytes([4, 1, 6, 12]))
        self.assertEqual(matched["id"], "plain")

    def test_match_uses_newest_import_on_equal_rank(self):
        old = self._entry(id="old", source="vendor-download",
                          imported_at="2026-01-01T00:00:00")
        new = self._entry(id="new", source="vendor-download",
                          imported_at="2026-02-01T00:00:00")
        matched = match_device_version([old, new], bytes([4, 1, 6, 12]))
        self.assertEqual(matched["id"], "new")

    def test_match_ignores_backups_missing_files_and_other_versions(self):
        backup = self._entry(id="backup", buildable=False)
        missing = self._entry(id="missing", exists=False)
        unknown = self._entry(id="unknown", known_base=None)
        self.assertIsNone(match_device_version(
            [backup, missing, unknown], bytes([4, 1, 6, 12])))
        self.assertIsNone(match_device_version(
            [self._entry()], bytes([4, 1, 6, 13])))


class AutoIntakeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.inbox = self.root / "inbox"
        self.inbox.mkdir()
        self.vault = FirmwareVault(self.root / "vault")

    def tearDown(self):
        self.tmp.cleanup()

    def test_import_folder_stores_full_images_only(self):
        (self.inbox / "original.bin").write_bytes(make_full())
        (self.inbox / "app-only.bin").write_bytes(make_app())
        (self.inbox / "notes.txt").write_text("ignore me", encoding="utf-8")
        results = self.vault.import_folder(self.inbox)
        self.assertEqual(len(results), 2)  # .txt is ignored
        by_name = {r["file"]: r for r in results}
        self.assertTrue(by_name["original.bin"]["ok"])
        self.assertFalse(by_name["app-only.bin"]["ok"])
        self.assertIn("not a full image", by_name["app-only.bin"]["error"])
        self.assertEqual(len(self.vault.list()), 1)

    def test_second_scan_is_deduplicated(self):
        (self.inbox / "original.bin").write_bytes(make_full())
        first = self.vault.import_folder(self.inbox)
        second = self.vault.import_folder(self.inbox)
        self.assertTrue(first[0]["ok"])
        self.assertTrue(second[0]["duplicate"])
        self.assertEqual(len(self.vault.list()), 1)

    def test_import_folder_source_override(self):
        (self.inbox / "original.bin").write_bytes(make_full())
        result = self.vault.import_folder(self.inbox,
                                          source="vendor-download")[0]
        self.assertEqual(result["source"], "vendor-download")
        self.assertEqual(self.vault.list()[0]["source"], "vendor-download")

    def test_missing_folder_is_empty(self):
        self.assertEqual(self.vault.import_folder(self.root / "missing"), [])

    def test_safeguard_receipt_round_trip(self):
        entry = self.vault.import_bytes(make_full(), "original.bin",
                                        source="vendor-download")
        receipt = self.vault.record_safeguard(
            {"port": "COM3", "connected": True, "version": "1.6.12",
             "version_raw": "0401060C", "reply": "4.1.6.12"},
            entry, "version-match", note="test")
        self.assertTrue(self.vault.safeguard_path.is_file())
        loaded = self.vault.get_safeguard()
        self.assertEqual(loaded["device"]["version"], "1.6.12")
        self.assertEqual(loaded["matched"]["sha256"], entry["sha256"])
        self.assertEqual(loaded["basis"], "version-match")
        self.assertEqual(receipt["matched"]["id"], entry["id"])

    def test_safeguard_without_image(self):
        receipt = self.vault.record_safeguard(
            {"port": "COM3", "version": "1.6.12"}, None, "no-matching-image")
        self.assertIsNone(receipt["matched"])
        self.assertEqual(self.vault.get_safeguard()["basis"],
                         "no-matching-image")

    def test_no_receipt_returns_none(self):
        self.assertIsNone(self.vault.get_safeguard())


class InboxWatchTests(unittest.TestCase):
    def test_image_dropped_after_startup_is_imported(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            inbox = root / "inbox"
            inbox.mkdir()
            cfg = copy.deepcopy(DEFAULT_CONFIG)
            cfg["conn_auto_connect"] = False
            cfg["dsp_stream_hz"] = 2
            engine = Engine(cfg, source="serial", start_threads=True,
                            config_path=root / "config.json",
                            vault_dir=root / "vault",
                            firmware_in_dir=inbox)
            try:
                self.assertEqual(
                    engine.firmware_auto_status()["imported_count"], 0)
                (inbox / "late.bin").write_bytes(make_full())
                deadline = time.time() + 12.0
                status = {}
                while time.time() < deadline:
                    status = engine.firmware_auto_status()
                    if status.get("last_reason") == "inbox-change":
                        break
                    time.sleep(0.2)
                self.assertEqual(status.get("last_reason"), "inbox-change")
                self.assertEqual(status.get("imported_count"), 1)
                self.assertEqual(len(engine.vault.list()), 1)
            finally:
                engine.shutdown()


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

    def test_scan_and_safeguard_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            inbox = root / "inbox"
            inbox.mkdir()
            (inbox / "original.bin").write_bytes(make_full())
            vault_dir = root / "vault"
            base_cmd = [sys.executable, "-I", str(ROOT / "firmware_vault.py"),
                        "--vault", str(vault_dir)]
            res = subprocess.run(base_cmd + ["scan", str(inbox)],
                                 cwd=str(ROOT), capture_output=True, text=True,
                                 encoding="utf-8", errors="replace", timeout=60)
            self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
            self.assertIn("stored:", res.stdout)
            res = subprocess.run(base_cmd + ["safeguard"],
                                 cwd=str(ROOT), capture_output=True, text=True,
                                 encoding="utf-8", errors="replace", timeout=60)
            self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
            self.assertIn("No safeguard receipt yet", res.stdout)


if __name__ == "__main__":
    unittest.main()
