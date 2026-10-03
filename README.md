# E88 Pro Web Controller (`e88pro_web.py`)

A single-file Python program that lets you fly an **E88 Pro** (RC UFO / `com.cooingdv.rcufo` family) Wi-Fi drone from a browser on your PC: two on-screen joysticks, live video from both cameras (one at a time), takeoff/land, and emergency buttons.

> **Protocol credit:** the byte formats below come from the reverse-engineering notes in
> [CraxCurl/RC-Swamp](https://github.com/CraxCurl/RC-Swamp) (`working.md`), which documents the official Android app. This program implements that spec; it is not an independent capture. Items marked **(spec)** come from that document, items marked **(code)** describe what this program does.

> **Safety first:** test with the propellers removed. Fly somewhere open. Keep a hand on **Emergency Land** (Space). The drone obeys whatever this program sends, including bugs.

---

## Contents

1. [Quick start](#1-quick-start)
2. [Big picture: how the pieces fit](#2-big-picture)
3. [The network: IPs and ports](#3-the-network)
4. [UDP packets, byte by byte](#4-udp-packets-byte-by-byte)
5. [Checksum and worked examples](#5-checksum-and-worked-examples)
6. [Stick values, speed gears, trim](#6-stick-values-and-gears)
7. [Timing and threads](#7-timing-and-threads)
8. [Telemetry and protocol auto-detection](#8-telemetry-and-protocol-auto-detection)
9. [Commands (takeoff, land, kill, gyro, headless)](#9-commands)
10. [Safety mechanisms](#10-safety-mechanisms)
11. [Video pipeline](#11-video-pipeline)
12. [The HTTP server and its API](#12-the-http-server-and-api)
13. [The web UI](#13-the-web-ui)
14. [Code map](#14-code-map)
15. [Command-line options](#15-command-line-options)
16. [Troubleshooting](#16-troubleshooting)
17. [Known limitations](#17-known-limitations)

---

## 1. Quick start

```bash
pip install opencv-python numpy
# 1. Power on the drone, join its Wi-Fi network from your PC
# 2. Run:
python e88pro_web.py
# 3. Open http://localhost:8080
```

No other dependencies: the web server uses Python's standard library (`http.server`).

---

## 2. Big picture

A browser **cannot** send raw UDP packets or open an RTSP video stream, so a small local server sits in the middle:

```
 ┌────────────────────┐   HTTP (localhost:8080)    ┌────────────────────────┐   UDP 7099    ┌─────────┐
 │  Browser (UI)      │ ─────────────────────────► │  e88pro_web.py         │ ────────────► │         │
 │  • 2 joysticks     │   POST /api/ctl  (25 Hz)   │                        │  heartbeat    │  Drone  │
 │  • buttons         │   POST /api/cmd            │  Drone class           │  control pkts │ 192.168 │
 │  • <img> video     │ ◄───────────────────────── │  (builds + sends       │ ◄──────────── │  .1.1   │
 │                    │   MJPEG /video, /snap/N    │   packets)             │  telemetry    │         │
 │                    │   GET /api/status          │                        │               │         │
 └────────────────────┘                            │  Stream class          │  RTSP 7070    │         │
                                                   │  (reads video)         │ ◄──────────── │         │
                                                   └────────────────────────┘  UDP/MJPEG    └─────────┘
```

- **Control path:** browser joystick → JSON over HTTP → `Drone.inp` → `Drone.packet()` → UDP to the drone.
- **Video path:** drone RTSP → OpenCV decodes frames → server re-encodes as JPEG → browser shows them in `<img>` tags.
- **Feedback path:** drone sends UDP telemetry back to the same socket → used to auto-detect the protocol variant.

---

## 3. The network

When powered on, the drone creates its own Wi-Fi access point. Your PC joins it and gets an address like `192.168.1.x`. **(spec)**

| Item | Value | Used for |
|---|---|---|
| Drone IP | `192.168.1.1` | everything |
| UDP port | **7099** | heartbeat, flight control, camera switch, stop, telemetry (both directions) |
| RTSP | `rtsp://192.168.1.1:7070/webcam` | live camera (UDP-transported MJPEG) |
| HTTP :80 / FTP :21 | `192.168.1.1` | photos/videos on the drone's SD card (not used by this program) |

Your PC has no internet while connected to the drone's Wi-Fi. That's normal.

The drone replies to whatever **source address and port** your UDP socket sends from. That's why this program uses **one socket for everything** (sending and receiving): replies come back to the same ephemeral port.

---

## 4. UDP packets, byte by byte

Every UDP datagram sent to port 7099 starts with a **command ID** byte. All values are hex unless noted. **(spec)**

| Cmd ID | Name | Full packet | Length | Sent by this program |
|---|---|---|---|---|
| `01` | Heartbeat | `01 01` | 2 | every 1 s |
| `03` | Flight control | see below | 9 or 21 | every 40 ms (25 Hz) |
| `06` | Switch camera | `06 01` (front) / `06 02` (bottom) | 2 | when you switch camera |
| `08` | Stop flight | `08 01` | 2 | on shutdown |
| `09` | Media ACK | `09 01` (photo) / `09 02` (video) | 2 | only if the drone reports a shutter press |
| `0A` | Set Wi-Fi password | `0A d0 … d7` (8 ASCII digits) | 9 | **never** (not implemented) |

### 4.1 Heartbeat: `01 01`

A keep-alive. The official app sends it once per second. This program sends one immediately at startup (which also makes the OS bind the UDP socket so telemetry can come back) and then every second (`Drone._hb`).

### 4.2 Flight control: Legacy format (9 bytes, "Type 10")

Used by simple/classic drones. This is the default in the code until telemetry says otherwise (and likely what an E88 Pro uses).

```
Offset:   0     1     2      3      4         5     6      7     8
        ┌─────┬─────┬──────┬──────┬─────────┬─────┬───────┬─────┬─────┐
        │ 03  │ 66  │ Roll │Pitch │Throttle │ Yaw │ Flags │ CS  │ 99  │
        └─────┴─────┴──────┴──────┴─────────┴─────┴───────┴─────┴─────┘
         cmd   head   ←── axes (1..255, 128 = centre) ──→  bitmask  XOR   tail
```

| Offset | Field | Meaning |
|---|---|---|
| 0 | `03` | command ID: flight control |
| 1 | `66` | start-of-frame marker |
| 2 | Roll | 1 = full left, 128 = centre, 255 = full right |
| 3 | Pitch | 1 = full back, 128 = centre, 255 = full forward |
| 4 | Throttle | 1 (or 0) = down, 128 = centre/hover, 255 = full climb |
| 5 | Yaw | 1 = rotate left, 128 = centre, 255 = rotate right |
| 6 | Flags | **additive** bitmask, see table below |
| 7 | Checksum | XOR of bytes 2–6, see [section 5](#5-checksum-and-worked-examples) |
| 8 | `99` | end-of-frame marker |

**Legacy flag values (byte 6)** (spec). They are *added*, which is equivalent to OR since each uses its own bit:

| Value | Meaning | Does this program use it? |
|---|---|---|
| `0x01` (1) | Takeoff | yes |
| `0x02` (2) | Land | yes |
| `0x04` (4) | Emergency stop (cuts motors) | yes |
| `0x08` (8) | 360° flip | no |
| `0x10` (16) | Headless mode | yes |
| `0x20` (32) | Return-to-home | no |
| `0x80` (128) | Gyro calibration | yes |

### 4.3 Flight control: GL format (21 bytes, "Type 2")

Used by newer HD/4K models with optical flow. The program switches to this if the drone's telemetry identifies it as GL.

```
Offset:  0    1    2    3     4      5        6    7      8     9 … 18      19   20
       ┌────┬────┬────┬─────┬─────┬────────┬─────┬──────┬──────┬──────────┬────┬────┐
       │ 03 │ 66 │ 14 │Roll │Pitch│Throttle│ Yaw │Flags1│Flags2│ 00 ×10   │ CS │ 99 │
       └────┴────┴────┴─────┴─────┴────────┴─────┴──────┴──────┴──────────┴────┴────┘
```

| Offset | Field | Meaning |
|---|---|---|
| 0 | `03` | command ID |
| 1 | `66` | start marker |
| 2 | `14` | inner payload length = 20 decimal |
| 3–6 | Roll, Pitch, Throttle, Yaw | same ranges as Legacy |
| 7 | Flags1 | bitmask (below) |
| 8 | Flags2 | bitmask (below) |
| 9–18 | zeros | reserved, 10 bytes |
| 19 | Checksum | see section 5 |
| 20 | `99` | end marker |

**Flags1 (byte 7):** `0x01` takeoff **or** land (same bit for both, per spec) · `0x02` emergency stop · `0x04` gyro calibration · `0x08` flip · `0x40` gesture mode.
**Flags2 (byte 8):** `0x01` headless · `0x02` altitude hold. This program always sets altitude hold (`f2 = headless | 2`).

> In code, the "inner" 20 bytes are built in a `bytearray(20)` (`inner[0]=0x66`, `inner[1]=0x14`, axes at `[2:6]`, flags at `[6]`/`[7]`, checksum at `[18]`, tail at `[19]`), then `0x03` is prepended, giving 21 bytes.

### 4.4 Camera switch: `06 01` / `06 02`

`01` selects the front camera, `02` the bottom camera. After switching, the video stream must be reopened; `switch_camera()` does this (see [section 11](#11-video-pipeline)).

### 4.5 Stop flight: `08 01`

Sent once at shutdown, after the emergency-land sequence, to disarm flight controls.

### 4.6 Media ACK: `09 01` / `09 02`

If a paired remote's photo/video button is pressed, the drone sends telemetry with byte 2 = `0x4D` ('M', photo) or `0x58` ('X', video). The app acknowledges with `09 01` or `09 02`. The program does this automatically in `_rx`; you'll probably never see it.

---

## 5. Checksum and worked examples

**Legacy:** `CS = roll ^ pitch ^ throttle ^ yaw ^ flags` (XOR, masked to 8 bits)
**GL:** `CS = flags1 ^ pitch ^ roll ^ throttle ^ yaw ^ flags2`

(XOR is commutative, so the order doesn't matter.) The drone recomputes this and ignores packets that don't match.

### Example A: Legacy, everything centred, nothing pressed
roll = pitch = throttle = yaw = 128 (`0x80`), flags = 0
`0x80 ^ 0x80 = 0x00`, `^ 0x80 = 0x80`, `^ 0x80 = 0x00`, `^ 0x00 = 0x00` → **CS = 00**

```
03 66 80 80 80 80 00 00 99
```

### Example B: same, with takeoff flag
flags = `0x01` → CS = `0x00 ^ 0x01` = `01`

```
03 66 80 80 80 80 01 01 99
```
This is held for 1 second (25 packets), then the flag clears.

### Example C: Legacy, right stick pushed fully forward in speed gear 1
Gear 1 → deflection ±40, so pitch = 128 + 40 = 168 = `0xA8`
`0x80 ^ 0xA8 = 0x28`, `^ 0x80 = 0xA8`, `^ 0x80 = 0x28` → **CS = 28**

```
03 66 80 A8 80 80 00 28 99
```

### Example D: GL, neutral
flags1 = 0, flags2 = `0x02` (altitude hold). Axes XOR to 0, so CS = `0 ^ 0x02` = `02`

```
03 66 14 80 80 80 80 00 02 00 00 00 00 00 00 00 00 00 00 02 99
└┬┘└┬┘└┬┘ └─ axes ───┘ f1 f2 └────── 10 zeros ──────┘ CS └tail
```

---

## 6. Stick values and gears

**The browser sends normalised values** in the range −1.0 … +1.0 (see [section 13](#13-the-web-ui)). The server turns them into bytes in `Drone.compute_axes()`:

```
byte = 128 + clamp(value, -1, 1) × deflection
```

| Axis | Deflection used | Comes from |
|---|---|---|
| Roll | gear value (40 / 60 / 127) | right stick, left/right |
| Pitch | gear value (40 / 60 / 127) | right stick, up/down |
| Throttle | always 127 | left stick, up/down |
| Yaw | always 127 | left stick, left/right |

**Speed gears** (`GEARS = {1: 40, 2: 60, 3: 127}`) match the official app's 30 % / 60 % / 100 % modes **(spec)**:

| Gear | Button | Deflection | Roll/pitch byte range |
|---|---|---|---|
| 1 | Speed 30 % | ±40 | 88 … 168 |
| 2 | Speed 60 % | ±60 | 68 … 188 |
| 3 | Speed 100 % | ±127 | 1 … 255 |

Gears scale **only roll and pitch**, exactly as the app does. Throttle and yaw always use full range, so be gentle with the left stick.

Clamping before sending (`packet()`): roll, pitch, yaw are kept ≥ 1. Throttle may reach 0. Max is 255 everywhere.

**Trim:** the official app adds a trim offset to each axis. This program has no trim: if the drone drifts when centred, recalibrate the gyro (button) on a flat surface.

---

## 7. Timing and threads

| Rate | What | Where |
|---|---|---|
| 1 Hz | heartbeat `01 01` | `Drone._hb` thread |
| 25 Hz (40 ms) | flight control packet | `Drone._ctl` thread |
| 25 Hz | browser posts stick values | JS `setInterval(…, 40)` |
| ~25 Hz | MJPEG frames to browser | `/video` handler loop, `sleep(0.04)` |
| 2 Hz | UI status refresh | JS, 500 ms |
| 0.4 Hz | other-camera snapshot refresh | JS, 2500 ms |

`_ctl` uses a drift-free schedule: `n += 0.04; sleep(max(0, n - time.time()))`, so packets stay evenly spaced even when a send takes a few ms.

**Threads in the program:**
1. *main*: runs the HTTP server (`serve_forever`)
2. *HTTP handler threads*: one per browser connection (`ThreadingHTTPServer`, `daemon_threads = True`). The live video stream holds one open for as long as the page is open.
3. `Drone._hb`, `Drone._ctl`, `Drone._rx`
4. `Stream._loop`: reads RTSP frames

Shared state (`drone.inp`, `drone.once`, `stream.frame`, …) is read/written from several threads without locks. Assignments of simple values are atomic enough in CPython for this purpose, but it is not strictly thread-safe design.

---

## 8. Telemetry and protocol auto-detection

The drone sends status datagrams back to the socket that sent the heartbeat. `Drone._rx` reads them (socket timeout 0.5 s so the thread can notice shutdown).

- **First packet** is printed in the console as hex: `[telemetry] first packet: …`. Keep this if you need to debug.
- **Byte 0 is the device ID.** If it is in 90–101, or equals 82, 85, 88 or 103, the drone is **GL** (21-byte control); otherwise **Legacy** (9-byte). **(spec)** In the code: `new = 2 if (90 <= d[0] <= 101 or d[0] in (103, 82, 85, 88)) else 10`.
- The detected type replaces `drone.dtype` live. The packet builder reads `dtype` on every packet, so the switch takes effect immediately.
- `--type 2` or `--type 10` disables auto-detection (`self.auto = False`).
- Until the first telemetry packet arrives, the type is **Legacy (10)**.
- The UI shows `● link OK` once at least one telemetry packet has arrived (`rx > 0`); `○ no telemetry` otherwise.

Other telemetry fields in the spec (camera resolution code, photo/video counters, Wi-Fi password nibbles) are not decoded by this program.

---

## 9. Commands

Most commands are **one-shot flags**: the flag is raised for N seconds and then clears itself. Implementation: `pulse(name, sec)` stores an expiry time in `drone.once[name]`; `on(name)` is true until that time passes. Because the control loop sends a packet every 40 ms, a 1-second pulse is 25 identical packets, which is how the official app does it **(spec)**, and gives the drone many chances to catch the command over lossy Wi-Fi.

| Button / key | `cmd` sent | Effect in the packet |
|---|---|---|
| TAKEOFF | `takeoff` | takeoff flag for **1.0 s** (Legacy `+1`, GL Flags1 `0x01`) |
| LAND | `land` | land flag for **1.0 s** (Legacy `+2`, GL Flags1 `0x01`) |
| ⚠ EMERGENCY LAND / Space | `emland` | land flag for **4.0 s** + sticks locked to neutral for 4.0 s |
| KILL MOTORS | `estop` | emergency-stop flag for **1.0 s** (Legacy `+4`, GL `0x02`); motors cut |
| Gyro calibrate | `gyro` | gyro flag for **2.0 s** (Legacy `+128`, GL `0x04`); put the drone on a flat surface and don't touch it |
| Headless | `headless` | toggles a persistent flag (Legacy `+16`, GL Flags2 `0x01`) |
| Speed 30/60/100 % | `gear` (v = 1/2/3) | changes roll/pitch scaling |
| Camera tile / Switch | `cam` (v = 1/2) | sends `06 01`/`06 02`, restarts video |

**Headless mode:** the drone treats "forward" as the direction it faced at power-on, regardless of its current heading. It is a drone-side feature; this program only sets the flag.

---

## 10. Safety mechanisms

| Mechanism | What it does | Limits |
|---|---|---|
| **Dead-man timeout** | If the server gets no `/api/ctl` message for `STALE = 0.4 s` (tab closed or hidden, browser frozen, page crash), `compute_axes()` returns all-neutral. | Neutral = hover, **not** land. The drone keeps hovering until the battery runs low or you act. |
| **Emergency Land** | Sets `lock_until = now + 4 s` and pulses land for 4 s. While locked, `compute_axes()` ignores the sticks and returns neutral, so a shaky joystick can't fight the landing. | Relies on the drone obeying the land flag. UI shows "EMERGENCY LANDING". |
| **Kill Motors** | Emergency-stop flag: instant motor cut. The drone falls. | Separate red button, kept away from Emergency Land. |
| **Shutdown (Ctrl+C)** | `finally:` block runs `emergency_land()`, waits 1.2 s, sends `08 01`, then stops threads. | A hard kill of the process (closing the terminal, power loss) skips this. |
| **Localhost only** | The server binds `127.0.0.1` by default. | `--host 0.0.0.0` exposes it to everyone on the drone's Wi-Fi network, who could then fly the drone. |

> The README for the original reverse-engineering project and the spec describe the emergency-stop flag as cutting power to all motors; this program has **not** been tested on hardware for that, and neither has the land flag's timing on every model. Treat both as "best effort".

---

## 11. Video pipeline

### Source
`rtsp://192.168.1.1:7070/webcam`, transported over **UDP** and carrying **MJPEG** (each frame is a standalone JPEG). **(spec)**

### Reading it: `Stream` class
```python
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay"
```
This must be set **before** `import cv2`. It tells OpenCV's FFmpeg backend to use UDP transport, disable input buffering and use low-delay decoding.

`Stream._loop` runs in its own thread: open the URL → if it fails, wait 0.5 s and retry → read frames continuously → store each into `self.frame` (latest) and `last[cam]` (latest per camera) → if a read fails, release and reconnect. Always keeping only the *newest* frame means the display never lags behind a growing queue.

### Two cameras, one stream
The drone streams **one camera at a time**. `switch_camera(cam)`:
1. `drone.cam = cam`
2. send `06 cam`
3. close the old `Stream`, wait 0.3 s
4. start a new `Stream(cam, last)` (the spec says to reconnect after switching)

`last = {1: frame, 2: frame}` keeps the most recent frame from each camera, so the inactive tile can show a "last frame" snapshot.

### Serving to the browser
- **`/video`** is an MJPEG stream: HTTP header `Content-Type: multipart/x-mixed-replace; boundary=f`, then an endless series of parts (`--f`, JPEG headers, JPEG bytes). Browsers render this natively inside a plain `<img>` tag, so no JavaScript video code is needed. Each frame is re-encoded with `cv2.imencode(".jpg", …, quality 70)`. If there's no frame yet, a black placeholder with "waiting for video…" is sent.
- **`/snap/N`** returns a single JPEG of the last frame seen from camera N (or a placeholder).

---

## 12. The HTTP server and API

Built on `ThreadingHTTPServer` + a `BaseHTTPRequestHandler` subclass (`H`) using HTTP/1.1 keep-alive. All responses carry `Cache-Control: no-store`.

| Method & path | Body | Response | Purpose |
|---|---|---|---|
| `GET /` | none | HTML page | the UI (the `PAGE` string embedded in the script) |
| `GET /video` | none | MJPEG stream | live camera |
| `GET /snap/1`, `/snap/2` | none | JPEG | last frame of that camera |
| `GET /api/status` | none | JSON | `tx` (packets sent), `rx` (telemetry packets received), `type` (2 or 10), `cam`, `gear`, `headless`, `axes` (the 4 bytes currently being sent), `locked` (emergency landing active) |
| `POST /api/ctl` | `{"roll","pitch","thr","yaw"}` each −1…1 | `{}` | stick positions; refreshes the dead-man timer |
| `POST /api/cmd` | `{"cmd": …, "v": …}` | `{}` | `takeoff`, `land`, `emland`, `estop`, `gyro`, `headless`, `gear` (v=1..3), `cam` (v=1..2) |

You can drive the drone from any script. For example, a 1-second takeoff from the command line (the control loop does the rest):
```bash
curl -X POST localhost:8080/api/cmd -d '{"cmd":"takeoff"}'
```

---

## 13. The web UI

One HTML page with inline CSS and JavaScript, no frameworks.

### Layout
- **Top:** two camera tiles. The active one has a green border and `LIVE` tag and shows `/video`; the other shows `/snap/N` refreshed every 2.5 s. Click a tile (or "Switch camera") to change.
- **Status bar:** link state, protocol (GL/Legacy), `tx`/`rx` counters, and the four axis bytes currently being sent.
- **Bottom:** left joystick, centre buttons, right joystick.

### Joysticks
Built from a circular `div` (200 px) and a draggable knob (radius limit `R = 61 px`) using **Pointer Events**, so mouse, touch and multi-touch all work, and both sticks can be held at once.

```
dx, dy = pointer position relative to the stick's centre, clamped to radius R
value  = ( dx / R ,  -dy / R )     # y inverted: up is positive
```
Release → knob springs to centre and the value becomes (0, 0).

| Stick | x axis | y axis |
|---|---|---|
| **Left** (`v.L`) | yaw: rotate left/right | throttle: up/down |
| **Right** (`v.R`) | roll: strafe left/right | pitch: forward/back |

### Sending sticks
Every 40 ms the page POSTs `{yaw: L.x, thr: L.y, roll: R.x, pitch: R.y}` to `/api/ctl`. Keyboard input is **added** to the joystick values (`W/S` throttle, `A/D` yaw, arrow keys pitch/roll); the server clamps the sum to ±1. If the tab is hidden, browsers throttle timers → messages stop → the server's dead-man timer zeroes the sticks.

**Space** triggers `emland` and calls `preventDefault()` so the page doesn't scroll.

### Status polling
Every 500 ms the page GETs `/api/status`, updates the status bar, highlights the active gear and headless button, and shows "EMERGENCY LANDING" while `locked` is true. If the fetch fails it shows "server offline".

---

## 14. Code map

| Piece | Responsibility |
|---|---|
| `Drone.__init__` | UDP socket, state: stick inputs, gear, flags, counters |
| `Drone.start` | send first heartbeat, launch `_hb`, `_ctl`, `_rx` threads |
| `Drone.raw` | `sendto(…, (192.168.1.1, 7099))`, errors ignored |
| `Drone.pulse` / `on` | time-limited one-shot flags |
| `Drone.emergency_land` | land flag 4 s + stick lock 4 s |
| `Drone.compute_axes` | normalised inputs → bytes; neutral if stale/locked |
| `Drone.packet` | build the Legacy or GL packet with flags and checksum |
| `Drone._hb`, `_ctl`, `_rx` | heartbeat, 25 Hz control, telemetry receiver |
| `Stream` | RTSP reader thread, keeps latest frame per camera |
| `switch_camera` | send `06 cam`, restart `Stream` |
| `jpeg` | frame → JPEG bytes (placeholder if `None`) |
| `H` | HTTP handler: page, video, snapshots, status, control, commands |
| `PAGE` | the whole UI (HTML + CSS + JS) |
| `main` | argument parsing, start everything, run server, shutdown sequence |

---

## 15. Command-line options

| Option | Default | Meaning |
|---|---|---|
| `--type 2\|10` | auto | force GL (21-byte) or Legacy (9-byte) control packets |
| `--port N` | 8080 | web UI port |
| `--host ADDR` | 127.0.0.1 | bind address; `0.0.0.0` = reachable from other devices (insecure) |

---

## 16. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Status shows `○ no telemetry` | PC isn't on the drone's Wi-Fi; or the drone doesn't send telemetry on this model. Controls may still work. Try `--type 2` / `--type 10`. |
| Video says "waiting for video…" | Wi-Fi not joined, wrong camera, or RTSP blocked. Test `rtsp://192.168.1.1:7070/webcam` in VLC. |
| Takeoff does nothing | Wrong packet format: try the other `--type`. Check that the drone is calibrated and the battery is charged. |
| Drone drifts when sticks are centred | Press **Gyro calibrate** with the drone on a flat surface, motionless. |
| Camera switch shows a frozen frame | Wait a second for the stream to reconnect; click the other tile again. |
| Sticks "stick" or feel laggy | Close other tabs using the page; Wi-Fi congestion; only one browser tab should control the drone. |
| Page can't reach the server | Server not running, or you used a different `--port`. |
| Firewall prompt on Windows | Allow Python on private networks (needed for UDP replies). |

---

## 17. Known limitations

- **Only what the spec documents:** flips, return-to-home, trim, gesture mode, Wi-Fi password change and telemetry decoding are not implemented.
- **Throttle and yaw ignore the speed gear** (matches the app's behaviour, but is easy to over-do).
- **Neutral throttle = 128 (hover)** per the spec. The drone's own firmware decides how it behaves at that value.
- **Dead-man = hover, not land.** Use Emergency Land.
- **Video latency:** typically around 100 ms or more through RTSP → OpenCV → JPEG → browser; don't fly beyond line of sight.
- **Other-camera tile is a snapshot,** not live: the drone only streams one camera at a time.
- **The `/video` handler re-encodes the latest frame every 40 ms** even if no new frame arrived, which wastes some CPU.
- **No authentication:** anyone who can reach the server's port can control the drone.
- **Not tested across models:** the E88 Pro is listed only as "suspected compatible" by related projects; your own tests (RC-Swamp working on your drone) are the best evidence.
