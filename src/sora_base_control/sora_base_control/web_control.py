#!/usr/bin/env python3
"""Phone-friendly web control for the Innovation Cup demo.

Dependency-free HTTP server (Python stdlib) serving a Start/Stop page that
relays controls to the robot over plain topics, and reads the robot's JSON
status back:

    browser --HTTP--> web_control --/demo/command--------> demo_wander  (start/stop)
                                   --/demo/mode-----------> demo_wander  (curious/cute)
                                   --/demo/showmode-------> demo_wander  (auto-switch on/off)
                                   --/demo/showmode_interval-> demo_wander (switch period)
                                   --/demo/manual_cmd-----> demo_wander  (manual drive Twist)
                                   --/demo/mood-----------> motor_driver (chirp mood)
                                   --/demo/chirp_enable---> motor_driver (chirps on/off)
    browser <-HTTP-- web_control <--/demo/status---------- demo_wander  (state/mode/countdown)

Three tabs: Control (start/stop + mode/mood/chirps/showmode + timer),
Manual (drive it by hand), Network (showmode period + switch back to home WiFi).
Self-contained (no external CSS/JS/fonts) so it works on an offline AP.
"""

import json
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String


MODES = ("curious", "cute")
MOODS = ("curious", "excited", "happy", "sad", "sleepy")

# Manual drive speeds (base frame).
MANUAL_LIN = 0.25     # m/s forward/back
MANUAL_STRAFE = 0.22  # m/s sideways
MANUAL_TURN = 1.0     # rad/s twist
MANUAL_DIRS = {
    "forward":      (MANUAL_LIN, 0.0, 0.0),
    "back":         (-MANUAL_LIN, 0.0, 0.0),
    "strafe_left":  (0.0, MANUAL_STRAFE, 0.0),
    "strafe_right": (0.0, -MANUAL_STRAFE, 0.0),
    "twist_left":   (0.0, 0.0, MANUAL_TURN),
    "twist_right":  (0.0, 0.0, -MANUAL_TURN),
    "stop":         (0.0, 0.0, 0.0),
}

SWITCH_LAN_CMD = [
    "sudo", "systemd-run", "--unit=sora-to-lan", "--collect",
    "bash", "-c",
    "nmcli connection modify Hotspot connection.autoconnect no; "
    "nmcli connection up WAROLIDI",
]


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>SORA Demo Control</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  body {
    margin: 0; min-height: 100vh;
    font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
    background: #0f1115; color: #e8eaed;
    display: flex; flex-direction: column; align-items: center;
    padding: 18px 16px 40px; gap: 16px;
  }
  h1 { font-size: 1.25rem; margin: 2px 0 0; letter-spacing: .5px; }
  .tabs { display: flex; gap: 8px; width: 100%; max-width: 460px; }
  .tab { flex: 1; text-align: center; padding: 10px 0; border-radius: 12px;
    background: #1c2029; border: 1px solid #2a2f3a; color: #cbd1da; font-weight: 600; cursor: pointer; }
  .tab.active { background: #2b3647; color: #fff; border-color: #3b4a63; }
  .panel { width: 100%; max-width: 460px; display: none; flex-direction: column; gap: 16px; }
  .panel.active { display: flex; }
  .status { align-self: center; display: flex; align-items: center; gap: 10px;
    font-size: 1.1rem; font-weight: 700; padding: 9px 18px; border-radius: 999px;
    background: #1c2029; border: 1px solid #2a2f3a; }
  .dot { width: 14px; height: 14px; border-radius: 50%; background: #6b7280; }
  .dot.roaming { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
  .dot.stopped { background: #ef4444; }
  .dot.manual { background: #f59e0b; box-shadow: 0 0 10px #f59e0b; }
  .dot.offline { background: #6b7280; }
  .timer { align-self: center; text-align: center; font-size: .95rem; color: #cbd1da; min-height: 1.2em; }
  .timer b { color: #93c5fd; font-variant-numeric: tabular-nums; }
  button.big { width: 100%; border: none; border-radius: 18px; color: #fff;
    font-size: 1.45rem; font-weight: 700; padding: 26px 0; cursor: pointer;
    transition: transform .05s ease, filter .15s ease; }
  button.big:active { transform: scale(.98); }
  .start { background: #16a34a; } .stop { background: #dc2626; } .lan { background: #b45309; }
  .card { background: #1c2029; border: 1px solid #2a2f3a; border-radius: 14px; padding: 14px 16px;
    display: flex; flex-direction: column; gap: 14px; }
  .row { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
  .row label { font-weight: 600; }
  select, input[type=number] { background: #0f1115; color: #e8eaed; border: 1px solid #3a4150;
    border-radius: 10px; padding: 10px 12px; font-size: 1rem; }
  select { min-width: 150px; }
  input[type=number] { width: 90px; }
  .switch { position: relative; width: 58px; height: 32px; }
  .switch input { opacity: 0; width: 0; height: 0; }
  .slider { position: absolute; inset: 0; background: #3a4150; border-radius: 999px; transition: .2s; }
  .slider:before { content: ""; position: absolute; height: 26px; width: 26px; left: 3px; top: 3px;
    background: #fff; border-radius: 50%; transition: .2s; }
  .switch input:checked + .slider { background: #16a34a; }
  .switch input:checked + .slider:before { transform: translateX(26px); }
  .hint { color: #9aa0aa; font-size: .85rem; text-align: center; }
  .warn { color: #fca5a5; font-size: .9rem; line-height: 1.4; }
  #lanMsg { color: #fcd34d; font-size: .9rem; min-height: 1.2em; text-align: center; }
  /* manual pad */
  .pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; touch-action: none; }
  .pad button { border: none; border-radius: 14px; background: #2b3647; color: #fff;
    font-size: 1.5rem; font-weight: 700; padding: 26px 0; user-select: none; touch-action: none; }
  .pad button:active { background: #3b4a63; }
  .pad .mstop { background: #dc2626; }
  .pad .empty { visibility: hidden; }
  .applied { color: #86efac; font-size: .85rem; text-align: center; min-height: 1.1em; }
</style>
</head>
<body>
  <h1>SORA Demo Control</h1>

  <div class="tabs">
    <div class="tab active" id="t-control" onclick="showTab('control')">Control</div>
    <div class="tab" id="t-manual" onclick="showTab('manual')">Manual</div>
    <div class="tab" id="t-network" onclick="showTab('network')">Network</div>
  </div>

  <!-- CONTROL -->
  <div class="panel active" id="panel-control">
    <div class="status"><span id="dot" class="dot offline"></span><span id="state">connecting…</span></div>
    <div class="timer" id="timer"></div>
    <button class="big start" onclick="send('start')">START ROAMING</button>
    <button class="big stop"  onclick="send('stop')">STOP</button>
    <div class="card">
      <div class="row">
        <label for="mode">Mode</label>
        <select id="mode" onchange="setMode(this.value)">
          <option value="curious">Curious (wander)</option>
          <option value="cute">Cute (stay &amp; play)</option>
        </select>
      </div>
      <div class="row">
        <label for="mood">Mood</label>
        <select id="mood" onchange="setMood(this.value)">
          <option value="curious">Curious</option><option value="excited">Excited</option>
          <option value="happy">Happy</option><option value="sad">Sad</option><option value="sleepy">Sleepy</option>
        </select>
      </div>
      <div class="row"><label for="chirp">Chirps</label>
        <label class="switch"><input type="checkbox" id="chirp" onchange="setChirp(this.checked)"><span class="slider"></span></label></div>
      <div class="row"><label for="showmode">Showmode (auto-switch)</label>
        <label class="switch"><input type="checkbox" id="showmode" onchange="setShowmode(this.checked)"><span class="slider"></span></label></div>
    </div>
    <p class="hint">Showmode: after START it switches between Curious and Cute on a timer (set the period in the Network tab).</p>
  </div>

  <!-- MANUAL -->
  <div class="panel" id="panel-manual">
    <div class="status"><span id="dot2" class="dot offline"></span><span id="state2">…</span></div>
    <div class="pad">
      <button class="empty"></button>
      <button data-dir="forward">▲</button>
      <button class="empty"></button>
      <button data-dir="twist_left">⟲</button>
      <button class="mstop" onclick="manualStop()">■</button>
      <button data-dir="twist_right">⟳</button>
      <button data-dir="strafe_left">◀</button>
      <button data-dir="back">▼</button>
      <button data-dir="strafe_right">▶</button>
    </div>
    <p class="hint">Press and hold to drive. ▲▼ forward/back · ◀▶ strafe · ⟲⟳ twist. Releasing stops. (No obstacle avoidance in manual.)</p>
  </div>

  <!-- NETWORK -->
  <div class="panel" id="panel-network">
    <div class="card">
      <div class="row">
        <label for="interval">Showmode switch every</label>
        <span><input type="number" id="interval" min="0.1" step="0.5" value="5"> min</span>
      </div>
      <button class="big" style="background:#2b3647" onclick="applyInterval()">Apply period</button>
      <div class="applied" id="intvMsg"></div>
    </div>
    <div class="card">
      <p class="warn">Switch the robot off its own <b>SORA-DEMO</b> network back onto home WiFi to update its software. Your phone <b>will lose this page</b>. Reboot the robot to return to demo mode.</p>
      <button class="big lan" onclick="switchLan()">Switch to home WiFi (for updates)</button>
      <div id="lanMsg"></div>
    </div>
  </div>

<script>
  function el(id){ return document.getElementById(id); }
  function showTab(n){
    ['control','manual','network'].forEach(function(x){
      el('panel-'+x).classList.toggle('active', x===n);
      el('t-'+x).classList.toggle('active', x===n);
    });
  }
  function setSel(id,v){ var e=el(id); if(e && v && e.value!==v) e.value=v; }
  function badge(d, online){
    var cls='offline', txt='offline — reconnecting…';
    if(online){
      if(d.state==='ROAMING'){cls='roaming'; txt='ROAMING';}
      else if(d.state==='STOPPED'){cls='stopped'; txt='STOPPED';}
      else if(d.state==='MANUAL'){cls='manual'; txt='MANUAL';}
      else {cls='offline'; txt=d.state||'unknown';}
    }
    el('dot').className='dot '+cls; el('state').textContent=txt;
    el('dot2').className='dot '+cls; el('state2').textContent=txt;
  }
  function fmt(s){ s=Math.max(0,Math.round(s)); var m=Math.floor(s/60); var ss=(s%60); return m+':'+(ss<10?'0':'')+ss; }
  function render(d, online){
    badge(d, online);
    if(!online){ el('timer').textContent=''; return; }
    if(d.mode) setSel('mode', d.mode);
    if(d.mood) setSel('mood', d.mood);
    if(typeof d.chirp==='boolean') el('chirp').checked=d.chirp;
    if(typeof d.showmode==='boolean') el('showmode').checked=d.showmode;
    // timer line
    var t=el('timer');
    if(d.showmode && d.state==='ROAMING' && typeof d.remaining==='number' && d.remaining>=0){
      var next=(d.mode==='curious')?'Cute':'Curious';
      t.innerHTML='Mode: '+(d.mode||'?')+' · switches to '+next+' in <b>'+fmt(d.remaining)+'</b>';
    } else if(d.showmode){
      t.textContent='Showmode on — timer starts on START';
    } else {
      t.textContent='Mode: '+(d.mode||'?');
    }
  }
  function post(p){ return fetch(p,{method:'POST'}); }
  function send(c){ post('/api/'+c).then(function(){setTimeout(poll,120);setTimeout(poll,500);}).catch(function(){render({},false);}); }
  function setMode(v){ post('/api/mode/'+v); }
  function setMood(v){ post('/api/mood/'+v); }
  function setChirp(on){ post('/api/chirp/'+(on?'on':'off')); }
  function setShowmode(on){ post('/api/showmode/'+(on?'on':'off')).then(function(){setTimeout(poll,150);}); }
  function applyInterval(){
    var mins=parseFloat(el('interval').value); if(isNaN(mins)||mins<=0){ return; }
    post('/api/interval/'+Math.round(mins*60)).then(function(){ el('intvMsg').textContent='Set to '+mins+' min'; setTimeout(poll,150); });
  }
  function switchLan(){
    if(!confirm('Switch the robot to home WiFi? This page will disconnect.')) return;
    el('lanMsg').textContent='Switching… this page will go offline shortly.';
    post('/api/switch_lan').catch(function(){});
  }
  // Manual: press-and-hold
  function manualSend(dir){ post('/api/manual/'+dir); }
  function manualStop(){ manualSend('stop'); }
  var held=null, heldTimer=null;
  function startHold(dir){
    held=dir; manualSend(dir);
    clearInterval(heldTimer); heldTimer=setInterval(function(){ if(held) manualSend(held); }, 150);
  }
  function endHold(){ if(held){ held=null; clearInterval(heldTimer); heldTimer=null; manualSend('stop'); } }
  document.querySelectorAll('.pad button[data-dir]').forEach(function(b){
    var dir=b.getAttribute('data-dir');
    b.addEventListener('pointerdown', function(e){ e.preventDefault(); startHold(dir); });
    b.addEventListener('pointerup', function(e){ e.preventDefault(); endHold(); });
    b.addEventListener('pointerleave', function(){ endHold(); });
    b.addEventListener('pointercancel', function(){ endHold(); });
  });
  function poll(){
    fetch('/api/status',{cache:'no-store'}).then(function(r){return r.json();})
      .then(function(d){ render(d,true); }).catch(function(){ render({},false); });
  }
  setInterval(poll, 1000); poll();
</script>
</body>
</html>
"""


def _make_handler(node: "WebControl"):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _send(self, code, body, content_type):
            data = body.encode("utf-8") if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(data)
            except Exception:
                pass

        def _status(self, code=200):
            self._send(code, json.dumps(node.status()), "application/json")

        def _route(self):
            path = self.path.split("?", 1)[0].rstrip("/")
            if path in ("", "/index.html"):
                self._send(200, PAGE, "text/html; charset=utf-8")
                return
            parts = [p for p in path.split("/") if p]
            if parts[:1] != ["api"]:
                self._send(404, "not found", "text/plain")
                return
            api = parts[1:]
            if api == ["status"]:
                self._status()
            elif api == ["start"]:
                node.send_command("start"); self._status()
            elif api == ["stop"]:
                node.send_command("stop"); self._status()
            elif len(api) == 2 and api[0] == "mode":
                node.set_mode(api[1]); self._status()
            elif len(api) == 2 and api[0] == "mood":
                node.set_mood(api[1]); self._status()
            elif len(api) == 2 and api[0] == "chirp":
                node.set_chirp(api[1] == "on"); self._status()
            elif len(api) == 2 and api[0] == "showmode":
                node.set_showmode(api[1] == "on"); self._status()
            elif len(api) == 2 and api[0] == "interval":
                node.set_interval(api[1]); self._status()
            elif len(api) == 2 and api[0] == "manual":
                node.send_manual(api[1]); self._status()
            elif api == ["switch_lan"]:
                ok, msg = node.switch_to_lan()
                self._send(200 if ok else 500, json.dumps({"ok": ok, "msg": msg}),
                           "application/json")
            else:
                self._send(404, "not found", "text/plain")

        def do_GET(self):
            self._route()

        def do_POST(self):
            self._route()

        def log_message(self, fmt, *args):
            node.get_logger().debug("HTTP " + (fmt % args))

    return Handler


class WebControl(Node):
    def __init__(self) -> None:
        super().__init__("web_control")

        self.declare_parameter("port", 8080)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("default_mood", "curious")

        port = int(self.get_parameter("port").value)
        bind = self.get_parameter("bind_address").value

        # Settings owned by the UI (robot reports state/mode/remaining back).
        self._robot = {"state": "UNKNOWN", "mode": "curious",
                       "showmode": False, "interval": 300.0, "remaining": -1.0}
        self._mood = self.get_parameter("default_mood").value
        self._chirp = False
        self._lock = threading.Lock()

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._cmd_pub = self.create_publisher(String, "/demo/command", 10)
        self._mode_pub = self.create_publisher(String, "/demo/mode", latched)
        self._mood_pub = self.create_publisher(String, "/demo/mood", latched)
        self._chirp_pub = self.create_publisher(Bool, "/demo/chirp_enable", latched)
        self._showmode_pub = self.create_publisher(Bool, "/demo/showmode", latched)
        self._interval_pub = self.create_publisher(Float64, "/demo/showmode_interval", latched)
        self._manual_pub = self.create_publisher(Twist, "/demo/manual_cmd", 10)
        self.create_subscription(String, "/demo/status", self._on_status, latched)

        # Publish initial settings so the other nodes start in a known state.
        self._publish_str(self._mood_pub, self._mood)
        self._publish_bool(self._chirp_pub, self._chirp)
        self._publish_bool(self._showmode_pub, False)

        self._httpd = ThreadingHTTPServer((bind, port), _make_handler(self))
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            "web_control serving on http://%s:%d/  (phone on the robot's WiFi)" % (bind, port))

    # --- helpers ------------------------------------------------------
    @staticmethod
    def _publish_str(pub, value):
        m = String(); m.data = value; pub.publish(m)

    @staticmethod
    def _publish_bool(pub, value):
        b = Bool(); b.data = bool(value); pub.publish(b)

    def _on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._robot.update(d)

    # --- actions ------------------------------------------------------
    def status(self) -> dict:
        with self._lock:
            r = dict(self._robot)
            r["mood"] = self._mood
            r["chirp"] = self._chirp
            return r

    def send_command(self, cmd: str) -> None:
        self._publish_str(self._cmd_pub, cmd)
        self.get_logger().info("web command: %s" % cmd)

    def set_mode(self, mode: str) -> None:
        if mode in MODES:
            self._publish_str(self._mode_pub, mode)

    def set_mood(self, mood: str) -> None:
        if mood in MOODS:
            with self._lock:
                self._mood = mood
            self._publish_str(self._mood_pub, mood)

    def set_chirp(self, on: bool) -> None:
        with self._lock:
            self._chirp = bool(on)
        self._publish_bool(self._chirp_pub, on)

    def set_showmode(self, on: bool) -> None:
        self._publish_bool(self._showmode_pub, on)

    def set_interval(self, seconds_str: str) -> None:
        try:
            seconds = max(5.0, float(seconds_str))
        except ValueError:
            return
        m = Float64(); m.data = seconds; self._interval_pub.publish(m)
        self.get_logger().info("web showmode interval: %.0fs" % seconds)

    def send_manual(self, direction: str) -> None:
        vx, vy, wz = MANUAL_DIRS.get(direction, (0.0, 0.0, 0.0))
        t = Twist()
        t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
        self._manual_pub.publish(t)

    def switch_to_lan(self):
        self.get_logger().warn("web requested switch to home WiFi (LAN)")
        try:
            subprocess.Popen(SWITCH_LAN_CMD)
            return True, "Switching to home WiFi; this page will disconnect."
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("switch_to_lan failed: %s" % e)
            return False, "Failed to launch switch: %s" % e

    def destroy_node(self) -> None:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
