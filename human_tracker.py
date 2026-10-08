"""
E88 Pro / RC UFO web controller  (protocol: github.com/CraxCurl/RC-Swamp working.md)
+ YOLO human detection, human-follow autopilot, search mode, and motion prediction

    pip install opencv-python numpy ultralytics
    1) join the drone's Wi-Fi   2) python e88pro_web.py   3) open http://localhost:8080
Options: --type 2|10 (force protocol)  --port 8080  --host 127.0.0.1
         --model yolo11n.pt  --imgsz 416  --conf 0.35
TEST WITH PROPELLERS REMOVED FIRST.

Autopilot safety rules (built in):
  * Browser tab must stay open: if stick messages stop, everything goes neutral.
  * Moving either on-screen stick / WASD / arrows takes over instantly (autopilot pauses).
  * No fresh video frame -> drone hovers (autopilot sends neutral sticks, no turning).
  * Person lost -> PREDICT (if enabled): keeps turning toward where they were moving,
    for at most PRED_MAX_AGE seconds, never pushing forward on a prediction.
  * Prediction over (or off) -> SEARCH (if enabled): turns toward the side the person
    last went, in small steps. Otherwise the drone hovers.
  * SPACE / EMERGENCY LAND turns the autopilot off and lands.
  * Autopilot only steers yaw (+ optional altitude) and pushes forward. It does not
    take off or land by itself, and has no obstacle avoidance. Use a big open space.
"""
import argparse, json, math, os, socket, threading, time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay"
import cv2
import numpy as np

DRONE_IP, CTRL_PORT = "192.168.1.1", 7099
RTSP_URL = f"rtsp://{DRONE_IP}:7070/webcam"
C = 128
GEARS = {1: 40, 2: 60, 3: 127}
STALE = 0.8      # no stick message for this long -> sticks go neutral (dead-man)
TAKEOVER = 0.15  # manual stick deflection above this pauses the autopilot
SLEW_RATE = 0.2  # max change per update cycle; slows acceleration (0..1 range)

# ---- follow tuning ----
KP_YAW, MAX_YAW = 0.9, 0.45     # yaw gain / max stick (0..1)
KP_THR, MAX_THR = 0.8, 0.40     # altitude gain / max stick (only if altitude align is on)
MIN_CMD = 0.10                  # smallest stick value worth sending when correcting
FWD = 0.7                       # forward push as fraction of the selected speed gear
RELEASE = 1.6                   # leave FORWARD and re-align if target is > RELEASE*radius from center

# ---- frame-health tuning ----
FRAME_STALE = 0.4     # no new video frame for this long -> autopilot hovers (neutral sticks)
FRAME_MIN_FPS = 3.0   # frames slower than this are treated as a broken stream
YAW_SCALE_MIN = 0.35  # yaw is scaled down to this when frames are arriving slowly
DETECT_CONFIRM = 2    # consecutive detections needed before the autopilot moves again after a loss

# ---- search tuning ----
SEARCH_YAW = 0.10     # turn stick magnitude; the direction is picked automatically
SEARCH_TURN = 0.35    # seconds turning per step
SEARCH_PAUSE = 1.0    # seconds holding still so the camera can look
SEARCH_MOVE_MIN = 0.05  # motion (fraction of frame per second) needed to trust the direction

# ---- prediction tuning ----
PRED_WINDOW = 0.8     # seconds of past detections used to estimate motion
PRED_LEAD = 1.0       # max seconds to extrapolate ahead of the last sighting
PRED_MAX_AGE = 1.5    # stop predicting this long after the person was last seen
PRED_GAIN = 0.6       # predicted turns are gentler than live tracking

LOST_STATES = ("LOST - HOVER", "SEARCH", "PREDICT", "NEED FRONT CAM", "NO VIDEO - HOVER", "-")


class Drone:
    def __init__(self, dtype=None):
        self.auto, self.dtype = dtype is None, dtype or 10
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.addr = (DRONE_IP, CTRL_PORT)
        self.inp = (0.0, 0.0, 0.0, 0.0)          # roll, pitch, throttle, yaw in -1..1
        self.inp_t = 0.0
        self.auto_inp = None                     # autopilot sticks (roll, pitch, thr, yaw) or None
        self.auto_t = 0.0
        self.slewed = (0.0, 0.0, 0.0, 0.0)       # smoothed sticks (slew-rate limited)
        self.gear, self.headless, self.cam = 1, False, 1
        self.once, self.lock_until = {}, 0.0
        self.sent = self.rx = 0
        self.axes = (C, C, C, C)
        self.run = True

    def start(self):
        self.raw(b"\x01\x01")
        for f in (self._hb, self._ctl, self._rx):
            threading.Thread(target=f, daemon=True).start()

    def raw(self, d):
        try: self.sock.sendto(d, self.addr)
        except OSError: pass

    def pulse(self, name, sec): self.once[name] = time.time() + sec
    def on(self, name): return self.once.get(name, 0) > time.time()

    def emergency_land(self):
        """Sticks forced neutral + land flag re-sent for 4 s (ignores UI input)."""
        self.auto_inp = None
        self.lock_until = time.time() + 4.0
        self.pulse("land", 4.0)

    def compute_axes(self):
        now = time.time()
        if now < self.lock_until or now - self.inp_t > STALE:   # lock or browser gone -> neutral
            self.slewed = (0.0, 0.0, 0.0, 0.0)
            return (C, C, C, C)
        r, p, t, y = self.inp
        manual = max(abs(r), abs(p), abs(t), abs(y)) > TAKEOVER
        if not manual and self.auto_inp is not None and now - self.auto_t <= STALE:
            r, p, t, y = self.auto_inp
        # Slew-rate limiting: smooth stick movements
        sr, sp, st, sy = self.slewed
        r = sr + max(-SLEW_RATE, min(SLEW_RATE, r - sr))
        p = sp + max(-SLEW_RATE, min(SLEW_RATE, p - sp))
        t = st + max(-SLEW_RATE, min(SLEW_RATE, t - st))
        y = sy + max(-SLEW_RATE, min(SLEW_RATE, y - sy))
        self.slewed = (r, p, t, y)
        d = GEARS[self.gear]
        cl = lambda v: max(-1.0, min(1.0, v))
        return (int(C + cl(r) * d), int(C + cl(p) * d), int(C + cl(t) * 127), int(C + cl(y) * 127))

    def packet(self):
        r, p, t, y = self.axes = self.compute_axes()
        r, p, y = max(1, r), max(1, p), max(1, y)
        fly, drop, estop, gyro = self.on("takeoff"), self.on("land"), self.on("estop"), self.on("gyro")
        if self.dtype != 10:   # GL, 21 bytes
            f1 = (1 if fly or drop else 0) | (2 if estop else 0) | (4 if gyro else 0)
            f2 = (1 if self.headless else 0) | 2
            cs = (f1 ^ (((p ^ r) ^ t) ^ y)) ^ f2
            inner = bytearray(20)
            inner[0], inner[1] = 0x66, 0x14
            inner[2:6] = bytes([r, p, t, y]); inner[6], inner[7] = f1, f2
            inner[18], inner[19] = cs & 255, 0x99
            return b"\x03" + bytes(inner)
        fl = fly + 2 * drop + 4 * estop + 16 * self.headless + 128 * gyro
        cs = (((r ^ p) ^ t) ^ y) ^ fl
        return bytes([3, 0x66, r, p, t, y, fl & 255, cs & 255, 0x99])

    def _hb(self):
        while self.run: self.raw(b"\x01\x01"); time.sleep(1)

    def _ctl(self):
        n = time.time()
        while self.run:
            self.raw(self.packet()); self.sent += 1
            n += 0.04; time.sleep(max(0, n - time.time()))

    def _rx(self):
        while self.run:
            try: d, _ = self.sock.recvfrom(1024)
            except (socket.timeout, OSError): continue
            if not d: continue
            if self.rx == 0: print("[telemetry] first packet:", d.hex(" "))
            self.rx += 1
            if self.auto:
                new = 2 if (90 <= d[0] <= 101 or d[0] in (103, 82, 85, 88)) else 10
                if new != self.dtype: self.dtype = new; print("[auto] protocol type ->", new)
            if len(d) > 4 and d[2] in (0x4D, 0x58): self.raw(bytes([9, 1 if d[2] == 0x4D else 2]))


class Stream:
    def __init__(self, cam, last):
        self.cam, self.last, self.frame, self.run = cam, last, None, True
        self.frame_t = 0.0        # wall-clock time the current frame was read
        self.frame_n = 0          # increments on every new frame
        self.fps = 0.0            # smoothed incoming frame rate
        self.connected = False
        threading.Thread(target=self._loop, daemon=True).start()

    def age(self):
        """Seconds since the last frame arrived (inf if none yet)."""
        return time.time() - self.frame_t if self.frame_n else float("inf")

    def _loop(self):
        while self.run:
            cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.connected = False
                time.sleep(0.5); continue
            self.connected = True
            t_prev, fails = time.time(), 0
            while self.run:
                ok, f = cap.read()
                if not ok or f is None:
                    fails += 1                      # count failed reads, reconnect after 15 in a row
                    if fails >= 15: break
                    time.sleep(0.02); continue
                fails = 0
                now = time.time()
                self.fps = 0.8 * self.fps + 0.2 / max(1e-3, now - t_prev) if self.frame_n else 0.0
                t_prev = now
                self.frame = self.last[self.cam] = f
                self.frame_t = now
                self.frame_n += 1
            cap.release()
            self.connected = False
            self.frame_n = 0   # detector sees "no fresh frame" until reconnected
            self.frame_t = 0.0

    def close(self): self.run = False


class Detector:
    """YOLO person detector (+ follow, search and prediction autopilot). modes: off | detect | follow"""

    def __init__(self, model_path, imgsz, conf):
        self.model_path, self.imgsz, self.conf = model_path, imgsz, conf
        self.mode = "off"
        self.state = "-"
        self.loading = False
        self.boxes = []                 # [(x1,y1,x2,y2,conf,is_target)] normalized 0..1
        self.persons = 0
        self.fps = 0.0
        self.radius = 0.12              # circle radius as fraction of min(frame w,h)
        self.stop = 0.60                # stop advancing when person height >= this fraction of frame
        self.alt = False                # also align vertically with throttle
        self.confirm = 0                # consecutive detections since last loss
        self.search = False             # search mode on/off (GUI toggle)
        self.search_phase = "turn"      # "turn" or "pause"
        self.search_until = 0.0
        self.search_dir = -1            # -1 = left, +1 = right; set when a search starts
        self.predict_on = False         # prediction mode on/off (GUI toggle)
        self.hist = deque(maxlen=12)    # recent (time, cx, cy, size) sightings, normalized
        self.seen = None                # last sighting (cx, cy), normalized
        self.seen_t = 0.0               # when the person was last seen
        self.pred_px = None             # predicted point in pixels, for the overlay
        self.stale = True
        threading.Thread(target=self._loop, daemon=True).start()

    def set_mode(self, m):
        self.mode = m
        self.state = "ALIGN" if m == "follow" else "-"
        self.confirm = 0
        self.reset_track()
        if m != "follow": drone.auto_inp = None
        if m == "off": self.boxes, self.persons = [], 0

    def reset_track(self):
        self.hist.clear()
        self.seen, self.pred_px = None, None

    def set_search(self, on):
        self.search = on
        if not on:
            if self.state == "SEARCH": self.state = "LOST - HOVER"
            drone.auto_inp = None

    def set_predict(self, on):
        self.predict_on = on
        if not on:
            self.pred_px = None
            if self.state == "PREDICT":
                self.state = "LOST - HOVER"
                drone.auto_inp, drone.auto_t = (0, 0, 0, 0), time.time()

    def _load(self):
        self.loading = True
        print("[yolo] loading", self.model_path, "...")
        import torch
        from ultralytics import YOLO
        self.dev = 0 if torch.cuda.is_available() else "cpu"
        self.half = self.dev != "cpu"
        m = YOLO(self.model_path)
        print(f"[yolo] inference device: {'GPU' if self.half else 'CPU'}")
        self.loading = False
        return m

    def _loop(self):
        model, prev, t_prev = None, None, time.time()
        while True:
            if self.mode == "off":
                time.sleep(0.1); continue
            if model is None:
                try: model = self._load()
                except Exception as e:
                    print("[yolo] load failed:", e); self.loading = False
                    self.set_mode("off"); continue
            f = stream.frame
            if f is None or f is prev:
                # no new frame yet: don't let the follower act on an old picture
                if self.mode == "follow" and stream.age() > FRAME_STALE:
                    self.hold("NO VIDEO - HOVER")
                time.sleep(0.01); continue
            prev = f
            try:
                res = model.predict(f, imgsz=self.imgsz, conf=self.conf, classes=[0],
                                    device=self.dev, half=self.half, verbose=False)[0]
                xy = res.boxes.xyxy.cpu().numpy(); cf = res.boxes.conf.cpu().numpy()
            except Exception as e:
                print("[yolo] inference error:", e); time.sleep(0.2); continue
            H, W = f.shape[:2]
            dets = [(float(a), float(b), float(c), float(d), float(s)) for (a, b, c, d), s in zip(xy, cf)]
            tgt = max(dets, key=lambda b: (b[2] - b[0]) * (b[3] - b[1])) if dets else None
            self.boxes = [(b[0] / W, b[1] / H, b[2] / W, b[3] / H, b[4], b is tgt) for b in dets]
            self.persons = len(dets)
            now = time.time()
            self.fps = 0.8 * self.fps + 0.2 / max(1e-3, now - t_prev); t_prev = now
            if self.mode == "follow":
                if stream.age() > FRAME_STALE or (0 < stream.fps < FRAME_MIN_FPS):
                    # frames are missing or too slow: hover, do not steer on stale data
                    self.hold("NO VIDEO - HOVER")
                else:
                    self.follow(tgt, W, H, now)

    def hold(self, state):
        """Neutral sticks, do not turn or push."""
        self.state = state
        self.stale = True
        self.pred_px = None
        drone.auto_inp, drone.auto_t = (0.0, 0.0, 0.0, 0.0), time.time()

    @staticmethod
    def _p(e, kp, mx, dz):
        if abs(e) < dz: return 0.0
        v = max(-mx, min(mx, kp * e))
        return math.copysign(max(abs(v), MIN_CMD), v)

    def record(self, now, cx, cy, size):
        """Remember a sighting so the motion can be estimated later."""
        self.hist.append((now, cx, cy, size))
        self.seen, self.seen_t = (cx, cy), now

    def velocity(self):
        """Motion (dx/dt, dy/dt) in normalized units per second, from recent sightings."""
        pts = [p for p in self.hist if self.seen_t - p[0] <= PRED_WINDOW]
        if len(pts) < 2: return None
        t0, x0, y0, _ = pts[0]
        t1, x1, y1, _ = pts[-1]
        span = t1 - t0
        if span < 0.15: return None
        return ((x1 - x0) / span, (y1 - y0) / span)

    def predict_point(self, now):
        """Where the person probably is now, or None if we can't or shouldn't guess."""
        if self.seen is None: return None
        dt = now - self.seen_t
        if dt > PRED_MAX_AGE: return None
        v = self.velocity()
        if v is None: return None
        lead = min(dt, PRED_LEAD)
        cx, cy = self.seen
        return (min(1.0, max(0.0, cx + v[0] * lead)),
                min(1.0, max(0.0, cy + v[1] * lead)))

    def predict_steer(self, W, H, now):
        """Keep turning toward the predicted position. Returns False if there is no prediction."""
        p = self.predict_point(now) if self.predict_on else None
        if p is None: return False
        px, py = p[0] * W, p[1] * H
        self.pred_px = (px, py)
        dx, dy = px - W / 2, py - H / 2
        R = self.radius * min(W, H)
        fps_scale = 1.0
        if stream.fps > 0:
            fps_scale = max(YAW_SCALE_MIN, min(1.0, stream.fps / 15.0))
        g = PRED_GAIN * fps_scale
        yaw = self._p(dx / (W / 2), KP_YAW * PRED_GAIN, MAX_YAW * g, 0.4 * R / (W / 2))
        thr = self._p(-dy / (H / 2), KP_THR * PRED_GAIN, MAX_THR * PRED_GAIN, 0.4 * R / (H / 2)) if self.alt else 0.0
        self.state = "PREDICT"
        drone.auto_inp, drone.auto_t = (0.0, 0.0, thr, yaw), now   # no forward push on a guess
        return True

    def last_direction(self):
        """Which way to search: -1 (left) or +1 (right), from the person's last motion.
        Falls back to the side of the frame they were last seen on, then to left."""
        v = self.velocity()
        if v is not None and abs(v[0]) >= SEARCH_MOVE_MIN:
            return -1 if v[0] < 0 else 1       # moving left in the image -> turn left
        if self.seen is not None and abs(self.seen[0] - 0.5) > 0.02:
            return -1 if self.seen[0] < 0.5 else 1
        return -1

    def run_search(self, now):
        """Small turn toward the last known side, pause to look, repeat."""
        if self.state != "SEARCH":
            self.state = "SEARCH"
            self.search_dir = self.last_direction()   # pick the side once, when search starts
            self.search_phase = "turn"
            self.search_until = now + SEARCH_TURN
        if now >= self.search_until:
            if self.search_phase == "turn":
                self.search_phase, self.search_until = "pause", now + SEARCH_PAUSE
            else:
                self.search_phase, self.search_until = "turn", now + SEARCH_TURN
        yaw = self.search_dir * SEARCH_YAW if self.search_phase == "turn" else 0.0
        drone.auto_inp, drone.auto_t = (0.0, 0.0, 0.0, yaw), now

    def follow(self, tgt, W, H, now):
        if drone.cam != 1:                       # follow only works with the front camera
            self.state = "NEED FRONT CAM"; drone.auto_inp = None; return
        self.stale = False

        if tgt is None:                          # nobody in view
            self.confirm = 0
            if self.predict_steer(W, H, now):    # 1) keep going the way they were moving
                return
            self.pred_px = None
            if self.search:
                self.run_search(now)             # 2) look around, toward their last side
            else:
                self.state = "LOST - HOVER"      # 3) nothing enabled: hover
                drone.auto_inp, drone.auto_t = (0, 0, 0, 0), now
            return

        x1, y1, x2, y2, _ = tgt
        self.record(now, (x1 + x2) / 2 / W, (y1 + y2) / 2 / H, (y2 - y1) / H)

        # person seen: require a few consecutive hits before moving again after a loss
        self.confirm += 1
        if self.confirm < DETECT_CONFIRM and self.state in LOST_STATES:
            if self.state == "PREDICT" and self.predict_steer(W, H, now):
                pass                             # keep the predicted turn until confirmed
            elif self.state == "SEARCH":
                self.run_search(now)             # keep searching until confirmed
            else:
                drone.auto_inp, drone.auto_t = (0, 0, 0, 0), now
            return

        self.pred_px = None
        dx, dy = (x1 + x2) / 2 - W / 2, (y1 + y2) / 2 - H / 2
        R = self.radius * min(W, H)
        dist = math.hypot(dx, dy) if self.alt else abs(dx)
        size = (y2 - y1) / H

        if self.state in LOST_STATES:
            self.state = "ALIGN"
        if self.state == "ALIGN" and dist <= R:
            self.state = "FORWARD"
        elif self.state in ("FORWARD", "ARRIVED") and dist > R * RELEASE:
            self.state = "ALIGN"
        if self.state in ("FORWARD", "ARRIVED"):
            if size >= self.stop: self.state = "ARRIVED"
            elif self.state == "ARRIVED" and size < self.stop * 0.85: self.state = "FORWARD"

        # slow the turn down when the video frame rate is low
        fps_scale = 1.0
        if stream.fps > 0:
            fps_scale = max(YAW_SCALE_MIN, min(1.0, stream.fps / 15.0))
        yaw = self._p(dx / (W / 2), KP_YAW, MAX_YAW * fps_scale, 0.4 * R / (W / 2))
        thr = self._p(-dy / (H / 2), KP_THR, MAX_THR, 0.4 * R / (H / 2)) if self.alt else 0.0
        pitch = FWD * fps_scale if self.state == "FORWARD" else 0.0
        drone.auto_inp, drone.auto_t = (0.0, pitch, thr, yaw), now


drone = None
det = None
last = {1: None, 2: None}
stream = None


def switch_camera(cam):
    global stream
    drone.cam = cam
    drone.raw(bytes([6, cam]))
    stream.close(); time.sleep(0.3)
    stream = Stream(cam, last)


def overlay(f):
    """Draw detections, follow circle and predicted point on a copy of the frame."""
    if f is None or det.mode == "off": return f
    f = f.copy(); H, W = f.shape[:2]
    for x1, y1, x2, y2, c, is_t in det.boxes:
        col = (0, 220, 0) if is_t else (0, 200, 255)
        p1, p2 = (int(x1 * W), int(y1 * H)), (int(x2 * W), int(y2 * H))
        cv2.rectangle(f, p1, p2, col, 2)
        cv2.putText(f, f"person {c:.2f}", (p1[0], max(14, p1[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        if is_t and det.mode == "follow":
            cv2.line(f, (W // 2, H // 2), ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2), col, 1)
            cv2.circle(f, ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2), 5, col, -1)
    if det.mode == "follow":
        col = {"FORWARD": (0, 220, 0), "ARRIVED": (255, 160, 0), "SEARCH": (255, 0, 255),
               "PREDICT": (255, 255, 0)}.get(det.state, (0, 200, 255))
        cv2.circle(f, (W // 2, H // 2), int(det.radius * min(W, H)), col, 2)
        cv2.drawMarker(f, (W // 2, H // 2), col, cv2.MARKER_CROSS, 14, 1)
        if det.state == "PREDICT" and det.pred_px is not None:
            px, py = int(det.pred_px[0]), int(det.pred_px[1])
            cv2.drawMarker(f, (px, py), col, cv2.MARKER_TILTED_CROSS, 18, 2)
    label = "loading model..." if det.loading else (f"FOLLOW: {det.state}" if det.mode == "follow" else "DETECT")
    cv2.putText(f, f"{label}  persons:{det.persons}  {det.fps:.0f} fps  video:{stream.fps:.0f} fps",
                (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return f


def jpeg(frame, text=None):
    if frame is None:
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(frame, text or "waiting for video...", (150, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, *a): pass

    def send_body(self, body, ctype, code=200):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/":
            self.send_body(PAGE.encode(), "text/html; charset=utf-8")
        elif url.path == "/api/status":
            self.send_body(json.dumps(dict(tx=drone.sent, rx=drone.rx, type=drone.dtype, cam=drone.cam,
                gear=drone.gear, headless=drone.headless, axes=drone.axes,
                locked=time.time() < drone.lock_until,
                mode=det.mode, state=det.state, persons=det.persons, fps=round(det.fps, 1),
                search=det.search, predict=det.predict_on,
                video_fps=round(stream.fps, 1), video_age=round(min(stream.age(), 99), 2),
                connected=stream.connected, loading=det.loading)).encode(), "application/json")
        elif url.path == "/frame":
            # one JPEG per request: a stalled request just times out and the page asks again
            n = int(parse_qs(url.query).get("cam", ["1"])[0])
            f = stream.frame if n == stream.cam else last.get(n)
            if n == 1: f = overlay(f)
            self.send_body(jpeg(f, f"camera {n}: no frame yet"), "image/jpeg")
        else:
            self.send_body(b"not found", "text/plain", 404)

    def do_POST(self):
        data = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        if self.path == "/api/ctl":
            drone.inp = (data.get("roll", 0), data.get("pitch", 0), data.get("thr", 0), data.get("yaw", 0))
            drone.inp_t = time.time()
        elif self.path == "/api/cmd":
            c = data.get("cmd")
            if c == "takeoff": drone.pulse("takeoff", 1.0)
            elif c == "land": det.set_mode("off"); drone.pulse("land", 1.0)
            elif c == "emland": det.set_mode("off"); drone.emergency_land(); print("EMERGENCY LANDING")
            elif c == "estop": det.set_mode("off"); drone.pulse("estop", 1.0); print("MOTOR KILL")
            elif c == "gyro": drone.pulse("gyro", 2.0)
            elif c == "headless": drone.headless = not drone.headless
            elif c == "gear": drone.gear = int(data.get("v", 1))
            elif c == "cam":
                det.set_mode("off") if det.mode == "follow" and int(data.get("v", 1)) != 1 else None
                switch_camera(int(data.get("v", 1)))
            elif c == "mode":
                m = data.get("v", "off")
                if m in ("off", "detect", "follow"): det.set_mode(m); print("[mode]", m)
            elif c == "search":
                det.set_search(bool(data.get("v", False))); print("[search]", det.search)
            elif c == "predict":
                det.set_predict(bool(data.get("v", False))); print("[predict]", det.predict_on)
            elif c == "cfg":
                det.radius = max(0.03, min(0.45, float(data.get("radius", det.radius * 100)) / 100))
                det.stop = max(0.2, min(0.95, float(data.get("stop", det.stop * 100)) / 100))
                det.alt = bool(data.get("alt", det.alt))
        self.send_body(b"{}", "application/json")


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>E88 Pro</title><style>
*{box-sizing:border-box;user-select:none;-webkit-user-select:none;touch-action:none}
body{margin:0;background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;display:flex;flex-direction:column;height:100vh}
#cams{display:flex;gap:8px;padding:8px;flex:1;min-height:0}
.tile{position:relative;flex:1;background:#000;border:2px solid #30363d;border-radius:8px;overflow:hidden;cursor:pointer}
.tile.active{border-color:#2ea043}.tile img{width:100%;height:100%;object-fit:contain}
.tag{position:absolute;top:6px;left:8px;background:#000a;padding:2px 8px;border-radius:4px;font-size:13px}
#bar{display:flex;align-items:center;justify-content:space-between;padding:4px 12px;font-size:13px;color:#8b949e}
#ctrl{display:flex;align-items:center;justify-content:space-around;padding:6px 8px 14px;gap:8px}
.joy{position:relative;width:200px;height:200px;border-radius:50%;background:#161b22;border:2px solid #30363d;flex:none}
.joy .knob{position:absolute;width:78px;height:78px;border-radius:50%;background:#388bfd;left:61px;top:61px;box-shadow:0 0 12px #0008}
.joy small{position:absolute;color:#6e7681;font-size:12px}
.mid{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:340px}
button{padding:12px 10px;border:0;border-radius:8px;font-size:15px;font-weight:600;color:#fff;background:#30363d;cursor:pointer}
button:active{filter:brightness(1.3)}.g{background:#238636}.o{background:#d9730d;grid-column:span 2;font-size:19px;padding:16px}.r{background:#b62324}
.sel{background:#1f6feb}.ai{background:#6e40c9}.ai.sel{background:#2ea043}
.cfg{grid-column:span 2;display:grid;grid-template-columns:auto 1fr auto;gap:4px 8px;align-items:center;font-size:12px;color:#8b949e;background:#161b22;border:1px solid #30363d;border-radius:8px;padding:6px 10px}
.cfg input[type=range]{width:100%;touch-action:auto}.cfg label.chk{grid-column:span 3;display:flex;gap:6px;align-items:center;color:#c9d1d9}
.cfg input[type=checkbox]{touch-action:auto}
</style></head><body>
<div id="cams">
 <div class="tile" id="t1" onclick="setCam(1)"><span class="tag" id="g1">CAM 1 front</span><img id="i1"></div>
 <div class="tile" id="t2" onclick="setCam(2)"><span class="tag" id="g2">CAM 2 bottom</span><img id="i2"></div>
</div>
<div id="bar"><span id="st">connecting…</span><span>Keys: WASD = left stick · arrows = right stick · Space = emergency land</span></div>
<div id="ctrl">
 <div class="joy" id="jL"><small style="top:6px;left:84px">UP</small><small style="bottom:6px;left:74px">DOWN</small><small style="top:90px;left:6px">⟲</small><small style="top:90px;right:6px">⟳</small><div class="knob"></div></div>
 <div class="mid">
  <button class="o" onclick="cmd('emland')">⚠ EMERGENCY LAND</button>
  <button class="g" onclick="cmd('takeoff')">TAKEOFF</button><button onclick="cmd('land')">LAND</button>
  <button class="ai" id="bd" onclick="toggleMode('detect')">👤 DETECT HUMAN</button>
  <button class="ai" id="bf" onclick="toggleMode('follow')">🎯 FOLLOW HUMAN</button>
  <button class="ai" id="bs" onclick="toggleSearch()">🔍 SEARCH: OFF</button>
  <button class="ai" id="bp" onclick="togglePredict()">🔮 PREDICT: OFF</button>
  <div class="cfg">
   <span>Circle</span><input type="range" id="rad" min="3" max="45" value="12" oninput="cfg()"><b id="radv">12%</b>
   <span>Stop at</span><input type="range" id="stp" min="20" max="95" value="60" oninput="cfg()"><b id="stpv">60%</b>
   <label class="chk"><input type="checkbox" id="alt" onchange="cfg()">Also align vertically (altitude)</label>
  </div>
  <button id="b1" onclick="gear(1)">Speed 30%</button><button id="b2" onclick="gear(2)">Speed 60%</button>
  <button id="b3" onclick="gear(3)">Speed 100%</button><button onclick="setCam(cam==1?2:1)">⇄ Switch camera</button>
  <button onclick="cmd('gyro')">Gyro calibrate</button><button id="hl" onclick="cmd('headless')">Headless</button>
  <button class="r" style="grid-column:span 2" onclick="cmd('estop')">KILL MOTORS (drone will fall)</button>
 </div>
 <div class="joy" id="jR"><small style="top:6px;left:72px">FORWARD</small><small style="bottom:6px;left:78px">BACK</small><small style="top:90px;left:8px">LEFT</small><small style="top:90px;right:4px">RIGHT</small><div class="knob"></div></div>
</div>
<script>
let cam=1,curMode='off',searchOn=false,predictOn=false,v={L:[0,0],R:[0,0]},prevUrl={1:null,2:null};
const post=(u,b)=>fetch(u,{method:'POST',body:JSON.stringify(b),keepalive:true}).catch(()=>{});
const cmd=(c,x)=>post('/api/cmd',{cmd:c,v:x});
const gear=n=>cmd('gear',n);
function toggleMode(m){ if(m=='follow'&&curMode!='follow'&&cam!=1)setCam(1); cmd('mode',curMode==m?'off':m) }
function toggleSearch(){ cmd('search',!searchOn) }
function togglePredict(){ cmd('predict',!predictOn) }
function cfg(){
 const r=+document.getElementById('rad').value,s=+document.getElementById('stp').value,a=document.getElementById('alt').checked;
 document.getElementById('radv').textContent=r+'%';document.getElementById('stpv').textContent=s+'%';
 post('/api/cmd',{cmd:'cfg',radius:r,stop:s,alt:a})}
function setCam(n){cam=n;cmd('cam',n);render()}

// Live view: request one JPEG at a time with a timeout. A stalled request is
// aborted and retried, so the picture can't freeze the way a long MJPEG stream can.
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
async function pump(){
 while(true){
  const n=cam, im=document.getElementById('i'+n);
  try{
   const ctl=new AbortController(), to=setTimeout(()=>ctl.abort(),1500);
   const r=await fetch('/frame?cam='+n+'&t='+Date.now(),{cache:'no-store',signal:ctl.signal});
   clearTimeout(to);
   if(r.ok){
    const u=URL.createObjectURL(await r.blob());
    if(n===cam){ im.src=u; if(prevUrl[n]) URL.revokeObjectURL(prevUrl[n]); prevUrl[n]=u; }
    else URL.revokeObjectURL(u);
   }
  }catch(e){ await sleep(300); }
  await sleep(50);
 }
}
function refreshInactive(){
 const o=cam==1?2:1, im=document.getElementById('i'+o);
 if(prevUrl[o]){ URL.revokeObjectURL(prevUrl[o]); prevUrl[o]=null; }
 im.src='/frame?cam='+o+'&t='+Date.now();
}
function render(){
 for(const n of [1,2]){
  document.getElementById('t'+n).classList.toggle('active',n==cam);
  document.getElementById('g'+n).textContent='CAM '+n+(n==1?' front':' bottom')+(n==cam?' · LIVE':' · last frame');
 }
 refreshInactive();
}
setInterval(refreshInactive,2500);

function joy(id,key){
 const el=document.getElementById(id),k=el.querySelector('.knob'),R=61;let pid=null;
 const mv=e=>{const r=el.getBoundingClientRect();let dx=e.clientX-(r.left+r.width/2),dy=e.clientY-(r.top+r.height/2);
  const m=Math.hypot(dx,dy);if(m>R){dx*=R/m;dy*=R/m}
  k.style.transform=`translate(${dx}px,${dy}px)`;v[key]=[dx/R,-dy/R]};
 const end=()=>{pid=null;k.style.transform='';v[key]=[0,0]};
 el.addEventListener('pointerdown',e=>{pid=e.pointerId;el.setPointerCapture(pid);mv(e)});
 el.addEventListener('pointermove',e=>{if(e.pointerId===pid)mv(e)});
 el.addEventListener('pointerup',end);el.addEventListener('pointercancel',end);
}
joy('jL','L');joy('jR','R');
const keys={};
addEventListener('keydown',e=>{if(e.code=='Space'){cmd('emland');e.preventDefault();return}keys[e.code]=1});
addEventListener('keyup',e=>delete keys[e.code]);
const kk=(a,b)=>(keys[a]?1:0)-(keys[b]?1:0);
setInterval(()=>{ // 25 Hz stick stream; server zeroes sticks if this stops (also required for autopilot)
 let [yl,tl]=[v.L[0],v.L[1]],[rr,pr]=[v.R[0],v.R[1]];
 yl+=kk('KeyD','KeyA');tl+=kk('KeyW','KeyS');rr+=kk('ArrowRight','ArrowLeft');pr+=kk('ArrowUp','ArrowDown');
 post('/api/ctl',{yaw:yl,thr:tl,roll:rr,pitch:pr})},40);
setInterval(async()=>{try{const s=await (await fetch('/api/status')).json();
 curMode=s.mode; searchOn=s.search; predictOn=s.predict;
 let ai=s.mode=='off'?'':` · AI ${s.mode}${s.loading?' (loading model…)':''} ${s.mode=='follow'?s.state+' ':''}· ${s.persons} person · ${s.fps} fps · video ${s.video_fps} fps${s.video_age>0.4?' (stale '+s.video_age+'s)':''}`;
 document.getElementById('st').textContent=(s.rx?'● link OK':'○ no telemetry')+` · ${s.type==2?'GL':'Legacy'} · tx ${s.tx} rx ${s.rx} · R${s.axes[0]} P${s.axes[1]} T${s.axes[2]} Y${s.axes[3]}`+ai+(s.locked?' · EMERGENCY LANDING':'');
 [1,2,3].forEach(n=>document.getElementById('b'+n).classList.toggle('sel',s.gear==n));
 document.getElementById('bd').classList.toggle('sel',s.mode=='detect');
 document.getElementById('bf').classList.toggle('sel',s.mode=='follow');
 const bs=document.getElementById('bs');
 bs.classList.toggle('sel',s.search);
 bs.textContent='🔍 SEARCH: '+(s.search?'ON':'OFF');
 const bp=document.getElementById('bp');
 bp.classList.toggle('sel',s.predict);
 bp.textContent='🔮 PREDICT: '+(s.predict?'ON':'OFF');
 document.getElementById('hl').classList.toggle('sel',s.headless)}catch(e){document.getElementById('st').textContent='server offline'}},500);
render();cfg();pump();
</script></body></html>"""


def main():
    global drone, stream, det
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", type=int, choices=(2, 10))
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--conf", type=float, default=0.35)
    a = ap.parse_args()
    drone = Drone(a.type); drone.start()
    stream = Stream(1, last)
    det = Detector(a.model, a.imgsz, a.conf)
    srv = ThreadingHTTPServer((a.host, a.port), H)
    srv.daemon_threads = True
    print(f"Open http://localhost:{a.port}  (Ctrl+C to quit)")
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        det.set_mode("off")
        drone.emergency_land(); time.sleep(1.2)
        drone.raw(b"\x08\x01"); drone.run = False; stream.close()


if __name__ == "__main__":
    main()
