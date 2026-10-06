"""
E88 Pro / RC UFO web controller  (protocol: github.com/CraxCurl/RC-Swamp working.md)
+ YOLO human detection and human-follow autopilot

    pip install opencv-python numpy ultralytics
    1) join the drone's Wi-Fi   2) python e88pro_web.py   3) open http://localhost:8080
Options: --type 2|10 (force protocol)  --port 8080  --host 127.0.0.1
         --model yolo11n.pt  --imgsz 416  --conf 0.35
TEST WITH PROPELLERS REMOVED FIRST.

Autopilot safety rules (built in):
  * Browser tab must stay open: if stick messages stop, everything goes neutral.
  * Moving either on-screen stick / WASD / arrows takes over instantly (autopilot pauses).
  * No person in frame -> drone hovers (all sticks neutral).
  * SPACE / EMERGENCY LAND turns the autopilot off and lands.
  * Autopilot only steers yaw (+ optional altitude) and pushes forward. It does not
    take off or land by itself, and has no obstacle avoidance. Use a big open space.
"""
import argparse, json, math, os, socket, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay"
import cv2
import numpy as np

DRONE_IP, CTRL_PORT = "192.168.1.1", 7099
RTSP_URL = f"rtsp://{DRONE_IP}:7070/webcam"
C = 128
GEARS = {1: 40, 2: 60, 3: 127}
STALE = 0.4      # no stick message for this long -> sticks go neutral (dead-man)
TAKEOVER = 0.15  # manual stick deflection above this pauses the autopilot

# ---- follow tuning ----
KP_YAW, MAX_YAW = 0.9, 0.45     # yaw gain / max stick (0..1)
KP_THR, MAX_THR = 0.8, 0.40     # altitude gain / max stick (only if altitude align is on)
MIN_CMD = 0.10                  # smallest stick value worth sending when correcting
FWD = 0.7                       # forward push as fraction of the selected speed gear
RELEASE = 1.6                   # leave FORWARD and re-align if target is > RELEASE*radius from center


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
            return (C, C, C, C)
        r, p, t, y = self.inp
        manual = max(abs(r), abs(p), abs(t), abs(y)) > TAKEOVER
        if not manual and self.auto_inp is not None and now - self.auto_t <= STALE:
            r, p, t, y = self.auto_inp
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
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.run:
            cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
            if not cap.isOpened(): time.sleep(0.5); continue
            while self.run:
                ok, f = cap.read()
                if not ok: break
                self.frame = self.last[self.cam] = f
            cap.release()

    def close(self): self.run = False


class Detector:
    """YOLO person detector (+ follow autopilot). modes: off | detect | follow"""

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
        threading.Thread(target=self._loop, daemon=True).start()

    def set_mode(self, m):
        self.mode = m
        self.state = "ALIGN" if m == "follow" else "-"
        if m != "follow": drone.auto_inp = None
        if m == "off": self.boxes, self.persons = [], 0

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
            if self.mode == "follow": self.follow(tgt, W, H, now)

    @staticmethod
    def _p(e, kp, mx, dz):
        if abs(e) < dz: return 0.0
        v = max(-mx, min(mx, kp * e))
        return math.copysign(max(abs(v), MIN_CMD), v)

    def follow(self, tgt, W, H, now):
        if drone.cam != 1:                       # follow only works with the front camera
            self.state = "NEED FRONT CAM"; drone.auto_inp = None; return
        if tgt is None:                          # lost -> hover
            self.state = "LOST - HOVER"
            drone.auto_inp, drone.auto_t = (0, 0, 0, 0), now
            return
        x1, y1, x2, y2, _ = tgt
        dx, dy = (x1 + x2) / 2 - W / 2, (y1 + y2) / 2 - H / 2
        R = self.radius * min(W, H)
        dist = math.hypot(dx, dy) if self.alt else abs(dx)
        size = (y2 - y1) / H

        if self.state in ("ALIGN", "LOST - HOVER", "NEED FRONT CAM", "-"):
            self.state = "ALIGN"
        if self.state == "ALIGN" and dist <= R:
            self.state = "FORWARD"
        elif self.state in ("FORWARD", "ARRIVED") and dist > R * RELEASE:
            self.state = "ALIGN"
        if self.state in ("FORWARD", "ARRIVED"):
            if size >= self.stop: self.state = "ARRIVED"
            elif self.state == "ARRIVED" and size < self.stop * 0.85: self.state = "FORWARD"

        yaw = self._p(dx / (W / 2), KP_YAW, MAX_YAW, 0.4 * R / (W / 2))
        thr = self._p(-dy / (H / 2), KP_THR, MAX_THR, 0.4 * R / (H / 2)) if self.alt else 0.0
        pitch = FWD if self.state == "FORWARD" else 0.0
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
    """Draw detections + follow circle on a copy of the frame."""
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
        col = {"FORWARD": (0, 220, 0), "ARRIVED": (255, 160, 0)}.get(det.state, (0, 200, 255))
        cv2.circle(f, (W // 2, H // 2), int(det.radius * min(W, H)), col, 2)
        cv2.drawMarker(f, (W // 2, H // 2), col, cv2.MARKER_CROSS, 14, 1)
    label = "loading model..." if det.loading else (f"FOLLOW: {det.state}" if det.mode == "follow" else "DETECT")
    cv2.putText(f, f"{label}  persons:{det.persons}  {det.fps:.0f} fps", (8, H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
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
        if self.path == "/":
            self.send_body(PAGE.encode(), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            self.send_body(json.dumps(dict(tx=drone.sent, rx=drone.rx, type=drone.dtype, cam=drone.cam,
                gear=drone.gear, headless=drone.headless, axes=drone.axes,
                locked=time.time() < drone.lock_until,
                mode=det.mode, state=det.state, persons=det.persons, fps=round(det.fps, 1),
                loading=det.loading)).encode(), "application/json")
        elif self.path.startswith("/snap/"):
            n = int(self.path.split("/")[2].split("?")[0])
            self.send_body(jpeg(last.get(n), f"camera {n}: no frame yet"), "image/jpeg")
        elif self.path.startswith("/video"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.send_header("Cache-Control", "no-store"); self.end_headers()
            try:
                while True:
                    b = jpeg(overlay(stream.frame))
                    self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(b) + b + b"\r\n")
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass
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
let cam=1,curMode='off',v={L:[0,0],R:[0,0]};
const post=(u,b)=>fetch(u,{method:'POST',body:JSON.stringify(b),keepalive:true}).catch(()=>{});
const cmd=(c,x)=>post('/api/cmd',{cmd:c,v:x});
const gear=n=>cmd('gear',n);
function toggleMode(m){ if(m=='follow'&&curMode!='follow'&&cam!=1)setCam(1); cmd('mode',curMode==m?'off':m) }
function cfg(){
 const r=+document.getElementById('rad').value,s=+document.getElementById('stp').value,a=document.getElementById('alt').checked;
 document.getElementById('radv').textContent=r+'%';document.getElementById('stpv').textContent=s+'%';
 post('/api/cmd',{cmd:'cfg',radius:r,stop:s,alt:a})}
function setCam(n){cam=n;cmd('cam',n);render()}
function render(){
 for(const n of [1,2]){document.getElementById('t'+n).classList.toggle('active',n==cam);
  const im=document.getElementById('i'+n);
  im.src = n==cam ? '/video?'+Date.now() : '/snap/'+n+'?'+Date.now();
  document.getElementById('g'+n).textContent='CAM '+n+(n==1?' front':' bottom')+(n==cam?' · LIVE':' · last frame')}}
setInterval(()=>{const o=cam==1?2:1;document.getElementById('i'+o).src='/snap/'+o+'?'+Date.now()},2500);
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
 curMode=s.mode;
 let ai=s.mode=='off'?'':` · AI ${s.mode}${s.loading?' (loading model…)':''} ${s.mode=='follow'?s.state+' ':''}· ${s.persons} person · ${s.fps} fps`;
 document.getElementById('st').textContent=(s.rx?'● link OK':'○ no telemetry')+` · ${s.type==2?'GL':'Legacy'} · tx ${s.tx} rx ${s.rx} · R${s.axes[0]} P${s.axes[1]} T${s.axes[2]} Y${s.axes[3]}`+ai+(s.locked?' · EMERGENCY LANDING':'');
 [1,2,3].forEach(n=>document.getElementById('b'+n).classList.toggle('sel',s.gear==n));
 document.getElementById('bd').classList.toggle('sel',s.mode=='detect');
 document.getElementById('bf').classList.toggle('sel',s.mode=='follow');
 document.getElementById('hl').classList.toggle('sel',s.headless)}catch(e){document.getElementById('st').textContent='server offline'}},500);
render();cfg();
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
