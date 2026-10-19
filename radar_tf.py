"""MR60BHA2 / HLK-LD6002 (Andar ADT6101P) – TinyFrame protocol helper.

Reads the radar's UART stream (through the XIAO USB bridge), splits TinyFrames,
decodes all known frame types and forwards them over Server-Sent Events to a
web UI. Commands to the radar are sent via POST /cmd.

This module is imported by advanced_dashboard.py (TFParser, build_frame, decode,
COMMANDS). It can also run as a small standalone frame monitor:

    python radar_tf.py --port COM3
    python radar_tf.py --replay capture.bin     (replay a capture, no device)
    Browser: http://127.0.0.1:8765
"""

import argparse
import base64
import json
import queue
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Frame types (from firmware analysis of v1.6.12)
# ---------------------------------------------------------------------------
FRAME_NAMES = {
    0x0A04: "Point cloud / targets",
    0x0A13: "Phase (total, breathing, heart)",
    0x0A14: "Breathing rate",
    0x0A15: "Heart rate",
    0x0A16: "Distance",
    0x0A17: "Target position x/y",
    0x0A20: "Echo (response to 0x0A12)",
    0x0A28: "Zone configuration",
    0x0A29: "Target status",
    0x0A32: "Raw data channel 1 (1024 B)",
    0x0A34: "Raw data channel 2 (1024 B)",
    0x0F09: "Presence",
    0x0100: "Debug text",
    0x3000: "Update request acknowledged",
    0xFFFF: "Version info",
}

# Host -> radar commands (dispatcher @0xFCF4). Bootloader commands 0x3000/0x4000 are intentionally not exposed.
COMMANDS = {
    "version": (0xFFFF, None, "Query version"),
    "echo": (0x0A12, "u32", "Echo test (response 0x0A20)"),
    "flag_a13": (0x0A13, "u32", "Set unknown flag @0x20000011 (value != 0)"),
    "raw_on": (0x0A14, None, "Raw data mode ON (0x0A32/0x0A34)"),
    "raw_off": (0x0A15, None, "Raw data mode OFF (vital signs)"),
    "zone_min": (0x0A16, "u32", "Zone: minimum distance (cm)"),
    "zone_max": (0x0A17, "u32", "Zone: maximum distance (cm)"),
    "zone_width": (0x0A18, "u32", "Zone: width (cm, ±half to each side)"),
    "zone_reset": (0x0A19, None, "Reset zone to default {0, 1500, 1500}"),
    "zone_read": (0x0A20, None, "Read zone configuration (response 0x0A28)"),
}
BLOCKED_TYPES = {0x3000, 0x4000}


def cksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return (~c) & 0xFF


def build_frame(ftype: int, payload: bytes = b"", frame_id: int = 0x8000) -> bytes:
    head = struct.pack(">BHHH", 0x01, frame_id & 0xFFFF, len(payload), ftype & 0xFFFF)
    out = head + bytes([cksum(head)])
    if payload:
        out += payload + bytes([cksum(payload)])
    return out


def f32(b, off=0):
    return struct.unpack_from("<f", b, off)[0]


def decode(ftype: int, d: bytes):
    """Return a dict of decoded fields (or {} if the type is unknown)."""
    try:
        if ftype == 0x0A13 and len(d) >= 12:
            return {"total": f32(d, 0), "breath": f32(d, 4), "heart": f32(d, 8)}
        if ftype in (0x0A14, 0x0A15) and len(d) >= 4:
            return {"rate": f32(d)}
        if ftype == 0x0A16 and len(d) >= 4:
            flag = struct.unpack_from("<I", d)[0]
            return {"flag": flag, "cm": f32(d, 4) if len(d) >= 8 else None}
        if ftype == 0x0100:
            return {"text": d.decode("latin-1").strip()}
        if ftype == 0x0A17 and len(d) >= 8:
            return {"x": f32(d, 0), "y": f32(d, 4)}
        if ftype == 0x0F09 and len(d) >= 2:
            return {"present": struct.unpack_from("<H", d)[0]}
        if ftype == 0x0A29 and len(d) >= 2:
            return {"status": struct.unpack_from("<H", d)[0]}
        if ftype == 0x0A28 and len(d) >= 6:
            mn, mx, w = struct.unpack_from("<HHH", d)
            return {"min_cm": mn, "max_cm": mx, "width_cm": w}
        if ftype == 0x0A20 and len(d) >= 4:
            return {"value": struct.unpack_from("<I", d)[0]}
        if ftype == 0xFFFF and len(d) >= 4:
            return {"bytes": list(d[:4]), "text": ".".join(str(x) for x in d[:4])}
        if ftype == 0x0A04 and len(d) >= 4:
            n = struct.unpack_from("<i", d)[0]
            pts = []
            for i in range(max(0, min(n, (len(d) - 4) // 16))):
                o = 4 + i * 16
                x, y = struct.unpack_from("<ff", d, o)
                v, tid = struct.unpack_from("<iI", d, o + 8)
                pts.append({"x": x, "y": y, "v": v, "id": tid})
            return {"n": n, "points": pts}
        if ftype in (0x0A32, 0x0A34):
            return {"b64": base64.b64encode(d).decode()}
    except struct.error:
        pass
    return {}


class TFParser:
    """TinyFrame parser. Bytes that do not belong to a valid frame are emitted as text."""

    def __init__(self, on_frame, on_text):
        self.buf = bytearray()
        self.on_frame = on_frame
        self.on_text = on_text
        self.text = bytearray()
        self.stats = {"frames": 0, "head_err": 0, "data_err": 0, "text_bytes": 0}

    def _flush_text(self, force=False):
        while True:
            nl = self.text.find(b"\n")
            if nl < 0:
                break
            line = self.text[:nl].rstrip(b"\r")
            del self.text[: nl + 1]
            if line:
                self.on_text(line.decode("latin-1"))
        if force and self.text:
            self.on_text(self.text.decode("latin-1"))
            self.text.clear()
        if len(self.text) > 400:
            self.on_text(self.text.decode("latin-1"))
            self.text.clear()

    def _junk(self, b):
        self.text.append(b)
        self.stats["text_bytes"] += 1

    def feed(self, data: bytes):
        self.buf += data
        while self.buf:
            if self.buf[0] != 0x01:
                self._junk(self.buf.pop(0))
                continue
            if len(self.buf) < 8:
                break
            if cksum(self.buf[:7]) != self.buf[7]:
                self.stats["head_err"] += 1
                self._junk(self.buf.pop(0))
                continue
            fid, ln, ftype = struct.unpack_from(">HHH", self.buf, 1)
            if ln > 4096:
                self.stats["head_err"] += 1
                self._junk(self.buf.pop(0))
                continue
            total = 8 + (ln + 1 if ln else 0)
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[8 : 8 + ln])
            if ln and cksum(payload) != self.buf[8 + ln]:
                self.stats["data_err"] += 1
                self._junk(self.buf.pop(0))
                continue
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            self.stats["frames"] += 1
            self._flush_text(force=True)
            self.on_frame(fid, ftype, payload, raw)
        self._flush_text()


# ---------------------------------------------------------------------------
# Distribution to the browser (SSE)
# ---------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self.clients = set()
        self.lock = threading.Lock()
        self.counts = {}
        self.last = {}

    def add(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            self.clients.add(q)
        return q

    def remove(self, q):
        with self.lock:
            self.clients.discard(q)

    def publish(self, msg: dict):
        s = json.dumps(msg)
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(s)
                except queue.Full:
                    pass


HUB = Hub()


class Radar:
    def __init__(self, args):
        self.args = args
        self.ser = None
        self.record = None
        self.parser = TFParser(self.on_frame, self.on_text)
        self.t0 = time.time()
        self.lock = threading.Lock()
        if args.record:
            p = Path(args.record)
            p.parent.mkdir(parents=True, exist_ok=True)
            self.record = open(p, "ab")

    def on_frame(self, fid, ftype, payload, raw):
        HUB.counts[ftype] = HUB.counts.get(ftype, 0) + 1
        msg = {
            "kind": "frame",
            "t": round(time.time() - self.t0, 3),
            "id": fid,
            "type": ftype,
            "name": FRAME_NAMES.get(ftype, "unknown"),
            "len": len(payload),
            "data": decode(ftype, payload),
            "hex": raw[:48].hex(" ") + (" …" if len(raw) > 48 else ""),
        }
        HUB.publish(msg)

    def on_text(self, line):
        HUB.publish({"kind": "text", "t": round(time.time() - self.t0, 3), "line": line})

    def run_serial(self):
        import serial  # pyserial

        while True:
            try:
                self.ser = serial.Serial(self.args.port, self.args.baud, timeout=0.05)
                HUB.publish({"kind": "status", "msg": f"connected to {self.args.port}"})
                while True:
                    data = self.ser.read(4096)
                    if data:
                        if self.record:
                            self.record.write(data)
                            self.record.flush()
                        self.parser.feed(data)
            except Exception as e:  # port gone, cable unplugged, ...
                HUB.publish({"kind": "status", "msg": f"Serial error: {e} – retrying in 2 s"})
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(2)

    def run_replay(self):
        data = Path(self.args.replay).read_bytes()
        HUB.publish({"kind": "status", "msg": f"Replay {self.args.replay} ({len(data)} B)"})
        chunk = max(1, int(self.args.replay_rate / 50))
        while True:
            for i in range(0, len(data), chunk):
                self.parser.feed(data[i : i + chunk])
                time.sleep(0.02)
            time.sleep(1)

    def send(self, ftype, payload=b""):
        if ftype in BLOCKED_TYPES:
            raise ValueError("Bootloader/flash commands are blocked in the dashboard")
        frame = build_frame(ftype, payload)
        with self.lock:
            if not self.ser:
                raise RuntimeError("no serial connection")
            self.ser.write(frame)
        HUB.publish({"kind": "tx", "t": round(time.time() - self.t0, 3), "type": ftype, "hex": frame.hex(" ")})
        return frame


RADAR: Radar = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        b = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/meta":
            meta = {
                "frames": {f"{k:#06x}": v for k, v in FRAME_NAMES.items()},
                "commands": {k: {"type": f"{v[0]:#06x}", "arg": v[1], "desc": v[2]} for k, v in COMMANDS.items()},
                "stats": RADAR.parser.stats,
                "counts": {f"{k:#06x}": v for k, v in HUB.counts.items()},
            }
            self._send(200, json.dumps(meta))
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = HUB.add()
            try:
                while True:
                    try:
                        s = q.get(timeout=5)
                        self.wfile.write(f"data: {s}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                HUB.remove(q)
        else:
            self._send(404, '{"error":"not found"}')

    def do_POST(self):
        if self.path != "/cmd":
            return self._send(404, '{"error":"not found"}')
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if "name" in req:
                ftype, arg, _ = COMMANDS[req["name"]]
                payload = b""
                if arg == "u32":
                    payload = struct.pack("<I", int(req.get("value", 0)) & 0xFFFFFFFF)
            else:  # custom frame: {"type":"0x0A20","hex":"01 00 00 00"}
                ftype = int(str(req["type"]), 0)
                payload = bytes.fromhex(req.get("hex", "").replace(" ", ""))
            frame = RADAR.send(ftype, payload)
            self._send(200, json.dumps({"ok": True, "sent": frame.hex(" ")}))
        except Exception as e:
            self._send(400, json.dumps({"ok": False, "error": str(e)}))


def main():
    global RADAR
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200, help="USB-CDC effectively ignores the baud rate")
    ap.add_argument("--http", type=int, default=8765)
    ap.add_argument("--record", help="append the raw stream to a file")
    ap.add_argument("--replay", help="play back a capture instead of using the device")
    ap.add_argument("--replay-rate", type=int, default=20000, help="bytes/s during replay")
    args = ap.parse_args()

    RADAR = Radar(args)
    target = RADAR.run_replay if args.replay else RADAR.run_serial
    threading.Thread(target=target, daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    print(f"Dashboard: http://127.0.0.1:{args.http}  (source: {args.replay or args.port})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
