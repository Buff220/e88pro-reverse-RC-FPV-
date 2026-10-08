"""
E88 Pro / RC UFO web controller  (protocol: github.com/CraxCurl/RC-Swamp working.md)

    pip install opencv-python numpy
    1) join the drone's Wi-Fi   2) python e88pro_web.py   3) open http://localhost:8080
Options: --type 2|10 (force protocol)  --port 8080  --host 127.0.0.1
TEST WITH PROPELLERS REMOVED FIRST.
"""
import argparse, json, os, socket, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay"
import cv2
import numpy as np

DRONE_IP, CTRL_PORT = "192.168.1.1", 7099
RTSP_URL = f"rtsp://{DRONE_IP}:7070/webcam"
C = 128
GEARS = {1: 40, 2: 60, 3: 127}
STALE = 0.4   # no stick message for this long -> sticks go neutral (dead-man)


class Drone:
    def __init__(self, dtype=None):
        self.auto, self.dtype = dtype is None, dtype or 10
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.addr = (DRONE_IP, CTRL_PORT)
        self.inp = (0.0, 0.0, 0.0, 0.0)          # roll, pitch, throttle, yaw in -1..1
        self.inp_t = 0.0
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
        self.lock_until = time.time() + 4.0
        self.pulse("land", 4.0)

    def compute_axes(self):
        now = time.time()
        if now < self.lock_until or now - self.inp_t > STALE:
            return (C, C, C, C)
        r, p, t, y = self.inp
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


drone = None
last = {1: None, 2: None}
stream = None


def switch_camera(cam):
    global stream
    drone.cam = cam
    drone.raw(bytes([6, cam]))
    stream.close(); time.sleep(0.3)
    stream = Stream(cam, last)


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
                locked=time.time() < drone.lock_until)).encode(), "application/json")
        elif self.path.startswith("/snap/"):
            n = int(self.path.split("/")[2].split("?")[0])
            self.send_body(jpeg(last.get(n), f"camera {n}: no frame yet"), "image/jpeg")
        elif self.path.startswith("/video"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.send_header("Cache-Control", "no-store"); self.end_headers()
            try:
                while True:
                    b = jpeg(stream.frame)
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
            elif c == "land": drone.pulse("land", 1.0)
            elif c == "emland": drone.emergency_land(); print("EMERGENCY LANDING")
            elif c == "estop": drone.pulse("estop", 1.0); print("MOTOR KILL")
            elif c == "gyro": drone.pulse("gyro", 2.0)
            elif c == "headless": drone.headless = not drone.headless
            elif c == "gear": drone.gear = int(data.get("v", 1))
            elif c == "cam": switch_camera(int(data.get("v", 1)))
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
.sel{background:#1f6feb}
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
  <button id="b1" onclick="gear(1)">Speed 30%</button><button id="b2" onclick="gear(2)">Speed 60%</button>
  <button id="b3" onclick="gear(3)">Speed 100%</button><button onclick="setCam(cam==1?2:1)">⇄ Switch camera</button>
  <button onclick="cmd('gyro')">Gyro calibrate</button><button id="hl" onclick="cmd('headless')">Headless</button>
  <button class="r" style="grid-column:span 2" onclick="cmd('estop')">KILL MOTORS (drone will fall)</button>
 </div>
 <div class="joy" id="jR"><small style="top:6px;left:72px">FORWARD</small><small style="bottom:6px;left:78px">BACK</small><small style="top:90px;left:8px">LEFT</small><small style="top:90px;right:4px">RIGHT</small><div class="knob"></div></div>
</div>
<script>
let cam=1,v={L:[0,0],R:[0,0]};
const post=(u,b)=>fetch(u,{method:'POST',body:JSON.stringify(b),keepalive:true}).catch(()=>{});
const cmd=(c,x)=>post('/api/cmd',{cmd:c,v:x});
const gear=n=>cmd('gear',n);
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
setInterval(()=>{ // 25 Hz stick stream; server zeroes sticks if this stops
 let [yl,tl]=[v.L[0],v.L[1]],[rr,pr]=[v.R[0],v.R[1]];
 yl+=kk('KeyD','KeyA');tl+=kk('KeyW','KeyS');rr+=kk('ArrowRight','ArrowLeft');pr+=kk('ArrowUp','ArrowDown');
 post('/api/ctl',{yaw:yl,thr:tl,roll:rr,pitch:pr})},40);
setInterval(async()=>{try{const s=await (await fetch('/api/status')).json();
 document.getElementById('st').textContent=(s.rx?'● link OK':'○ no telemetry')+` · ${s.type==2?'GL':'Legacy'} · tx ${s.tx} rx ${s.rx} · R${s.axes[0]} P${s.axes[1]} T${s.axes[2]} Y${s.axes[3]}`+(s.locked?' · EMERGENCY LANDING':'');
 [1,2,3].forEach(n=>document.getElementById('b'+n).classList.toggle('sel',s.gear==n));
 document.getElementById('hl').classList.toggle('sel',s.headless)}catch(e){document.getElementById('st').textContent='server offline'}},500);
render();
</script></body></html>"""


def main():
    global drone, stream
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", type=int, choices=(2, 10))
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="127.0.0.1")
    a = ap.parse_args()
    drone = Drone(a.type); drone.start()
    stream = Stream(1, last)
    srv = ThreadingHTTPServer((a.host, a.port), H)
    srv.daemon_threads = True
    print(f"Open http://localhost:{a.port}  (Ctrl+C to quit)")
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        drone.emergency_land(); time.sleep(1.2)
        drone.raw(b"\x08\x01"); drone.run = False; stream.close()


if __name__ == "__main__":
    main()
