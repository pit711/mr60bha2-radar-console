# Advanced Radar Console

Local live dashboard for the **MR60BHA2 / HLK-LD6002** (Andar ADT6101P) via the
XIAO-ESP32-USB bridge. It displays the complex range-Doppler raw data of both
receive channels (RX1/RX2), processes it further on the PC side, and allows
manual configuration of virtually all relevant parameters — plus an automatic mode.

```
advanced_dashboard.py     Backend: serial reader, DSP, HTTP+SSE, recording/patch generation
advanced_dashboard.html   Frontend: canvas renderer, user interface, automation UI
webflasher.html           Browser-based firmware flasher (Web Serial, XMODEM-CRC)
radar_tf.py               TinyFrame protocol helper and command-line probe
firmware_vault.py         Validates and stores original images, SHA-256 (never flashes)
patch_raw_window.py       Generates tuning images (never flashes)
tests/                    Unit tests for vault and patch pipeline (synthetic images)
advanced_dashboard_config.json   Saved configuration (created automatically)
```

> **No firmware in this repository.** Radar firmware, the vendor SDK,
> decompiled code, and flash backups are third-party copyrighted material and
> are not distributed here. You need your own legally obtained full firmware
> image to use the Web Flasher. Never publish a flash backup: it can contain
> NVS data such as Wi-Fi credentials.

---

## Start

```bat
:: Live on the radar (COM3), browser opens automatically
python -I advanced_dashboard.py --port COM3

:: without radar, synthetic data
python -I advanced_dashboard.py --demo

:: play back a recorded NPZ session
python -I advanced_dashboard.py --replay recordings/session.npz --replay-speed 2

:: only check that backend and frontend are OK
python -I advanced_dashboard.py --selftest

:: different HTTP port / no browser / no automatic connection
python -I advanced_dashboard.py --http 8771 --no-open --no-auto-connect
```

Default address: **http://127.0.0.1:8770** (binds only to `127.0.0.1`).

| Option | Meaning |
|---|---|
| `--port COMx` | overrides the COM port from the configuration |
| `--baud N` | baud rate (practically irrelevant for USB-CDC) |
| `--http N` | HTTP port (default 8770) |
| `--host ADDR` | bind address (default 127.0.0.1) |
| `--demo` | synthetic raw data without hardware |
| `--replay FILE` | play back an NPZ recording |
| `--replay-speed F` | playback speed (default 1.0) |
| `--config FILE` | alternative configuration file |
| `--no-open` | do not open the browser |
| `--no-auto-connect` | do not connect automatically |
| `--firmware-in DIR` | folder watched for full firmware images (default `./firmware_in`) |
| `--no-auto-safeguard` | disable automatic intake and version matching |
| `--selftest` | run the self-test and exit |

### Keyboard Shortcuts

| Key | Action |
|---|---|
| `1` … `7` | Live, Analysis, Room Map, Vitals, Recordings, Firmware, Log |
| `A` | toggle auto mode |
| `C` | connect/disconnect |

---

## Architecture

```
COM3 ──pyserial──► RadarReader ──TinyFrame──► TFParser
                                               │
                         ┌─────────────────────┴─────────────────────┐
                         │                 RadarEngine               │
                         │  • buffers 0x0A32/0x0A34 (8×32 complex)   │
                         │  • DSP: dB, clutter, smoothing, detection │
                         │  • tracking, vitals, auto mode            │
                         │  • recording / snapshot / CSV             │
                         └─────────────────────┬─────────────────────┘
                                               │  build_snapshot()
                        HTTP server :8770 ─────┴──► /api/state (JSON)
                                               └──► /api/stream (SSE)
                                                        │
                                            Browser: canvas renderer
```

- **Server:** `ThreadingHTTPServer`, static HTML + JSON + SSE.
- **Stream:** `GET /api/stream` sends a snapshot as a `data:` event on every
  change; the browser frame rate is governed by `dsp_stream_hz`.
- **Task distribution:** the serial reader, snapshot construction, and recording
  run in separate threads with locks; the frontend renders exclusively in the browser.
- **Demo/replay** produce the same snapshot structure as live operation.
  Replay processes frames with accurate timing (`--replay-speed`).

### HTTP Endpoints

| Endpoint | Purpose |
|---|---|
| `GET /` | dashboard |
| `GET /flasher` | Web Flasher page (Web Serial, Chrome/Edge desktop) |
| `GET /api/state` | full snapshot including the latest log lines |
| `GET /api/stream` | server-sent events with snapshots |
| `POST /api/config` | `{updates:{...}}` – set parameters (with clamping) |
| `POST /api/action` | `{action:"...", ...}` – connection, recording, presets, patch |
| `GET /api/download?file=...` | file below the project directory (recordings, snapshots, tuning, firmware output) |
| `GET /api/firmware/list` | vault entries with kind, buildable flag, hashes, source |
| `GET /api/firmware/image?id=...` | raw bytes of a stored image |
| `POST /api/firmware/import` | binary body; `X-Filename`, `X-Firmware-Source` headers or query parameters |
| `POST /api/firmware/verify` | `{id}` – recompute and compare SHA-256 |

The backend clamps numeric values to the ranges defined in `CLAMPS` and accepts
only known keys from `DEFAULT_CONFIG`; paths are resolved below the project directory.

---

## The Seven Views

| Tab | Content |
|---|---|
| **Live** | RX1 and RX2 heatmap, combined multispectrum, range/Doppler profile, waterfall, range-time map (RTI), interactive 3D surface, status cards |
| **Analysis** | large heatmap (current multispectrum mode) and large RTI |
| **Room Map** | x/y point cloud, target markers, trail, grid, adjustable map size |
| **Vitals** | respiration/heart rate, phase trace, dedicated FFT, spectra |
| **Recordings** | start/stop recording, snapshot, CSV export, file list |
| **Firmware** | three-step studio: import the original into the vault, build a tuned image, download it and flash it manually |
| **Log** | event log, parser statistics, frame type table, radar debug text |

On the right is the parameter drawer with the groups **Connection**,
**Radar Parameters**, **Raw Data / Calibration**, **Filters / Display**,
**Stream / Waterfall / 3D**, **Detection**, **Tracking**, **Vitals**,
**Visibility**, **Map / View**, **Automation**, and **Recording**.
**Firmware Patch Parameters** are located in the **Firmware** tab.

---

## Modern Visualization

### Multispectrum Modes (`dsp_heat_mode`)

The combined heatmap can display five things:

| Mode | Presentation |
|---|---|
| `magnitude` | mean of RX1+RX2 in dB |
| `fusion` | RGB fusion: RX1 → blue/cyan, RX2 → red/magenta, overlap bright |
| `motion` | magnitude of the frame-to-frame change (motion map) |
| `snr` | level above the estimated noise floor |
| `difference` | RX1 − RX2, symmetric color scale around 0 dB |

The legend and footer adapt automatically; detections and the global peak are
marked in the image.

### Tone Mapping and Persistence

| Parameter | Effect |
|---|---|
| `dsp_heat_gamma` | gamma 0.2–4; >1 brightens weak targets, <1 emphasizes peaks |
| `dsp_heat_equalize` | histogram equalization (rank distribution instead of linear) |
| `dsp_persistence` | afterglow 0–0.95; 0 = off, 0.8 = long motion trails |
| `dsp_auto_contrast` + percentiles | color range from the current image instead of fixed dB limits |

Persistence is maintained separately per view/image source (RX1, RX2, fusion,
and each mode have their own buffers), so switching modes does not show ghost
images from another mode.

### Range-Time Heatmap (RTI)

`pushRTI()` takes up to ~12 rows per second from the combined range profile and
retains 240 rows (~20 s). The newest row is at the top; range runs from left to
right. When auto contrast is active, the RTI receives its **own** 2%/98%
contrast with a minimum span of 0.5 dB, so weak targets are not washed out by a
dominant near target.

### Interactive 3D Surface

Range × Doppler × level (not true height-based 3D — the hardware has only two
RX antennas).

- **Drag:** rotate yaw/pitch
- **Mouse wheel:** zoom 0.35–3.5
- **Double-click:** reset view
- **Auto-rotation** (`dsp_3d_rotate`)
- **Lambert lighting** (`dsp_3d_light`) shades according to the surface normal
- **Wireframe** (`dsp_3d_wireframe`), floor grid, depth sorting, perspective
- **Hover** shows the range/Doppler/level of the surface under the cursor
- Source RX1/RX2/combined (`dsp_3d_source`), height (`ui_3d_height`), and
  decimation (`ui_3d_decimate`) are adjustable

### Hover Readout

Heatmaps, RTI, and 3D use a shared info box anchored to the mouse pointer
(`#hoverTip`):

- Heatmap: range, Doppler velocity, value in dB
- RTI: range, time offset, value in dB
- 3D: range, Doppler, level of the hit grid segment

### Detection Overlay

Peaks are drawn as yellow diamonds with a ring and label (at most five labels,
alternating offset, semi-transparent background). This can be disabled via
`dsp_det_overlay`; the global peak can be disabled separately via
`dsp_peak_overlay`.

---

## Parameters

All values are applied immediately and saved to `advanced_dashboard_config.json`.
A restart restores the last state.

### Connection

`conn_port`, `conn_baud`, `conn_auto_connect`, `conn_auto_reconnect`,
`conn_reconnect_s`, `conn_on_connect_version`, `conn_on_connect_mode` (raw/track/keep),
`conn_on_connect_zone_read`.

### Live Radar Commands

| Parameter | Frame | Meaning |
|---|---|---|
| `radar_zone_min` / `_max` / `_width` | `0x0A16` / `0x0A17` / `0x0A18` | presence zone in cm |
| `radar_echo_value` | `0x0A12` | echo test value (response `0x0A20`) |
| `radar_flag_value` | `0x0A13` | unknown u32 flag, for testing only |
| `cmd_custom_type` / `cmd_custom_hex` | any | arbitrary TinyFrame |

Additional actions: version (`0xFFFF`), raw on (`0x0A14`), raw off/track
(`0x0A15`), read zone (`0x0A20`), reset zone (`0x0A19`).
**Locked:** `0x3000` and `0x4000` (bootloader/flash) — the dashboard never sends
these types.

### Raw Data and Display

`dsp_rows` (auto/8/16/32), `dsp_range_start_m`, `dsp_range_step_m`,
`dsp_doppler_zero_bin`, `dsp_doppler_bin_mps`, `dsp_iq_view`
(magnitude/real/imag/phase/power), `dsp_clutter_remove`, `dsp_clutter_alpha`,
`dsp_smooth_alpha`, `dsp_display_gain_db`, dB limits, colormap
(magma/viridis/inferno/turbo/grayscale/hsv/icefire), interpolation, grid.

> FFT processing is **post-FFT**: `0x0A32`/`0x0A34` already provide
> range-Doppler complex values (int16 I/Q), not a raw ADC stream.

### Detection

Channel (`mean/rx1/rx2/max`), method (`localmax`/`cfar`), noise mode
(auto/manual), percentile, threshold above noise, guard/training cells,
CFAR bias, range and Doppler limits, DC exclusion, minimum peak spacing,
maximum peaks, minimum level.

### Tracking

`det_target_hold_s`, `det_association_m`, `det_smoothing`.

### Vitals

`dsp_vital_history_s`, `dsp_vital_smooth`, `dsp_vital_fft_enabled`,
`dsp_vital_fft_window_s`, `dsp_vital_fft_n`, respiration/heart bands,
`dsp_phase_unwrap`.

### Visualization

Visibility of all tiles, `ui_theme`, `ui_map_size_m`, `ui_auto_map_size`,
`ui_point_age_s`, `ui_max_points`, `ui_trail_length`, `ui_view_min_m`,
`ui_view_max_m`, `dsp_waterfall_rows`, `dsp_waterfall_mode`, `dsp_waterfall_row`,
`dsp_stream_hz`.

---

## Automation Mode

Activate via the **Auto Mode** switch or the `A` key.

| Function | Parameter |
|---|---|
| Auto contrast | `auto_contrast`, complements `dsp_auto_contrast` |
| Auto threshold | `auto_threshold` – adjusts the detection threshold based on peak count |
| Auto view range | `auto_view`, `auto_view_span_m` |
| Auto map size | `auto_map_size` |
| Raw peak as fallback target | `auto_peak_target` |
| Auto gain | `auto_gain`, `auto_gain_target_db` |
| Raw/track switching | `auto_mode_switch`, `auto_raw_hold_s`, `auto_track_after_s` |
| Presence source | `auto_presence_source` (either/radar/raw) |
| Motion threshold | `auto_motion_threshold_db` |
| Auto recording | `auto_record`, `auto_record_post_s` |

The auto mode changes only **PC-side** display and detection parameters as well
as the Raw/Track runtime modes. It does not generate or flash any firmware.

### Presets

`default`, `sensitive` (weak motion), `far` (long range/room),
`raw` (raw data only, maximum detail), `vital` (vitals/phase),
`debug` (everything on, CFAR, 40 peaks). Each preset sets a defined group of
keys; `reset_config` restores the defaults.

---

## Recording and Export

| Action | Result |
|---|---|
| Start recording | writes NPZ to `recordings/` (path from `rec_path`, relative to the project directory) |
| Max frames | automatic stop and asynchronous save |
| Detections | optionally recorded per frame (`rec_include_detections`) |
| Snapshot | `snapshots/snapshot_YYYYMMDD_HHMMSS.npz` |
| CSV export | range/Doppler/detection data of the current state |
| Download | files from `recordings/`, `snapshots/`, `tuning/`, `firmware_out/` via `/api/download` |

Existing files are never overwritten (`unique_path`). Recordings can be played
back directly with `--replay`.

---

## Firmware Studio (Back Up, Build, Flash)

The Firmware tab is a three-step studio. Step 1 stores the original image in a
local vault, step 2 builds a tuned image from a full image, step 3 hands the
result to the Web Flasher. The same operations are available as CLIs.

### Step 1 — Back up the original

> **The ADT6101P UART bootloader is write-only.** No command reads the flash
> back, and the vendor OTA tool has no dump function. A true chip backup needs
> an external **SWD programmer** (read protection is unknown and may be
> locked). The practical alternative is a legally obtained **vendor full
> image**, which can serve as the build base.

`firmware_vault.py` stores immutable copies under `firmware_vault/`:

| Kind | Detection | Flashable | Build base |
|---|---|---|---|
| **full** | OTA magic `64 A5 64 A5` + 16-byte header | yes | yes |
| **app** | plausible vector table, no header | no | no |
| **raw** | anything else (for example an SWD readout) | no | no |

Every image is validated (SP in SRAM, 8-byte alignment, Thumb reset vector
inside the image), stored under a timestamped ID, and deduplicated by SHA-256.
`vendor-download` imports must be full images; app-only and raw dumps are
accepted as backups only.

```bat
python -I firmware_vault.py inspect IMAGE.bin
python -I firmware_vault.py import IMAGE.bin --source vendor-download
python -I firmware_vault.py import DUMP.bin --source swd-dump --note "SWD readout"
python -I firmware_vault.py list
python -I firmware_vault.py verify IMAGE_ID
```

#### Automatic intake (`firmware_in/`)

The dashboard polls `firmware_in/` once per second — no extra dependency, no
restart, no button. A `.bin`/`.img` file that appears after startup (or is
copied over while the dashboard runs) is taken over automatically as soon as
its size and mtime have stayed unchanged for 1.5 s, so a copy that is still in
progress is not read half-written.

- The same intake runs at startup and on every folder change: full images are
  stored in the vault and deduplicated by SHA-256; app-only and raw files are
  reported as skipped.
- Once the radar reports its version (`0xFFFF`), it is matched against the
  verified images and `firmware_vault/safeguard.json` is written. This confirms
  a **version match** — it does not read the chip.
- The Firmware tab refreshes by itself: status line, vault list, and build base.
- Disable it with **Automatic safeguard** in the tab or
  `--no-auto-safeguard`; `--firmware-in DIR` points the watcher at another
  folder.

### Step 2 — Build a tuned image

In the tab, select a full vault image, set the tuning values in the sidebar
**Firmware** panel, choose the UART baud mode (below), and press **Build tuning
image**. The dashboard invokes `patch_raw_window.py` and writes to
`firmware_out/<timestamp>_<base>/` together with a manifest of every offset,
old/new byte, output SHA-256, and expected UART load.

| Baud mode | Effect |
|---|---|
| `auto` (default) | patches the stock 115200 baud clock literal to 1,388,889; already-fast images are left unchanged |
| `keep` | keeps the stock baud rate |
| `1382400` | requires the fast literal and refuses an unknown state |

Safety rules:

- Only full images can be bases; backups are rejected.
- A base that already contains tuning patches is rejected — use an unmodified
  original.
- An image whose SHA-256 is not a recognised official base is rejected until
  **Allow unknown base** is enabled. Only enable it after verifying every patch
  offset yourself.
- TIA and VGA must not be increased at the same time.
- 32 rows require the explicit unsafe-rate switch; RF patches require
  `fw_allow_rf`.

CLI equivalents:

```bat
python -I patch_raw_window.py --base-full YOUR_FULL_IMAGE.bin --rows 16
python -I patch_raw_window.py --base-full custom.bin --allow-unknown-base --rows 16 --baud auto
python -I patch_raw_window.py --rows 16 --tia 12 --allow-rf
```

The first two forms accept any full image. The legacy form without `--base-full`
expects these files in the project directory (not bundled):

```
mr60bha2_v1.6.12_app@0x8000_baud1382400.bin
MR60BHA2_eeprom_1.6.12_baud1382400.bin
```

### Step 3 — Flash it yourself

Download the generated full image from the manifest and open `/flasher`. The
flasher validates the header and SHA-256 and sends data only after two explicit
confirmations.

**The dashboard never flashes by itself.** `0x3000`/`0x4000` remain locked in
the dashboard runtime; only the Web Flasher sends them. Flash a firmware image
only after deliberate review; use risky RF patches (TIA/VGA/PA/HPF) only after
making your own risk assessment.

### Patch parameters

| Parameter | Values | Note |
|---|---|---|
| `fw_rows` | 8 / 16 / 32 | 32 rows overload the UART at ~20 Hz |
| `fw_tia` | 6 / 12 / 18 dB | gain, untested |
| `fw_vga` | 15 / 18 / 20 dB | gain, untested |
| `fw_pa` | 10 / 12 / 15 | PA bias, **RF-relevant** |
| `fw_hpf` | 150 / 400 kHz | high-pass filter, untested |
| `fw_allow_rf` | bool | enables TIA/VGA/PA/HPF |
| `fw_allow_unsafe` | bool | enables 32 rows |

Known throughputs at 1,388,889 baud (138.9 kB/s):

| Rows | Payload | Approx. rate |
|---|---|---|
| 8 | 1.0 m | 40.5 kB/s |
| 16 | 2.0 m | 80.6 kB/s |
| 32 | 4.0 m | 160.7 kB/s (UART overload) |

---

## Web Flasher

`webflasher.html` is served by the dashboard at **http://127.0.0.1:8770/flasher**
(also linked from the dashboard header and the Firmware tab). It runs entirely
in the browser through the **Web Serial API** — no Python flasher, no server-side
port access.

**Requirements**

- Chrome or Edge on desktop (Web Serial is not available in Firefox or Safari)
- The page must be opened from `http://127.0.0.1:8770` or HTTPS
- Close the dashboard connection to COM3 first; only one program can hold the port
- Your own legally obtained **full** firmware image (16-byte header + app)

**Protocol**

1. Validate the image header: magic `64 A5 64 A5`, stack pointer in SRAM,
   Thumb reset vector inside the image. SHA-256 is computed and shown.
2. Open the port at **115200 baud**, 8N1, no flow control.
3. Send TinyFrame type `0x3000` (update request) and wait for the bootloader's
   `C` prompts.
4. Transfer with **XMODEM-CRC**: 128-byte blocks, `0x1A` padding, CRC16-XMODEM
   (big-endian), 16 retries per block, 60 s block timeout.
5. Send `EOT` (`0x04`) and query the version (`0xFFFF`) up to five times.

**Safety behaviour**

- The flash button stays disabled until a port is open, the image is valid, and
  the acknowledgement checkbox is ticked.
- A second confirmation panel must be accepted before the first byte is sent.
- An abort button is available during the transfer. Aborting may leave the
  device in the bootloader — reconnect and flash again.
- No firmware is bundled. The page never selects an image automatically.

---

## Troubleshooting

| Problem | Cause / Solution |
|---|---|
| `COM3 in use` | terminate the other program (old dashboard, Serial Monitor) |
| no raw data | radar in Track mode → send **Raw On** (`0x0A14`); after connecting, the dashboard automatically retries the command up to two times if the firmware was not yet ready at boot |
| no frames | wrong COM port, XIAO not connected, check the bridge baud rate |
| `numpy/pyserial module missing` | `python -m pip install numpy pyserial` |
| Port 8770 in use | use `--http 8771` |
| empty maps in the demo | the demo generates synthetic targets — wait a few seconds |
| RTI stays empty | is raw data arriving? check `raw.ok` and `profiles.range_combined` |
| 3D hover without a hit | the mouse must be over a drawn surface (not over the background) |
| Firmware: no build base | import a full image into the vault first; images are not bundled |
| Firmware: unknown base rejected | verify the patch offsets, then enable **Allow unknown base** |
| Firmware: already patched | use an unmodified original as the base |
| Firmware: need a true chip dump | the UART bootloader cannot read flash — use SWD or the vendor image |

Diagnosis without a browser:

```bat
python -I advanced_dashboard.py --selftest
```

Expected output (example):

```json
{"ok": true, "rows": 8, "cols": 32, "detections": 12, "json_bytes": 24039,
 "html_exists": true, "html_markers": true, "flasher_exists": true,
 "flasher_markers": true, "patch_script": true, "vault_module": true,
 "vault_ok": true, "firmware_build": true, "firmware_error": null,
 "version": "1.6.12.0"}
```

The self-test builds a synthetic image in a temporary directory; it needs no
vendor firmware and does not touch `firmware_vault/`. Unit tests:

```bat
python -I -m unittest discover -s tests -v
```

---

## Files and Limits

- All artifacts remain under the project directory.
- `recordings/`, `snapshots/`, `tuning/dashboard/`, `firmware_in/`,
  `firmware_vault/`, and `firmware_out/` are working directories; all of them
  are gitignored.
- The radar provides **no height information**; the “3D” view is
  range × Doppler × level.
- The raw data is already range/Doppler-processed; a complete pre-FFT raw cube
  is not available over the UART.
- This repository ships **no** firmware, SDK, decompiled code, or flash
  backups. Do not add them to a fork or release.
