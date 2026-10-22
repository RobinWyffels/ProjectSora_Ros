#!/usr/bin/env python3
"""Phone-friendly web control for the Innovation Cup demo.

Runs a small dependency-free HTTP server (Python stdlib) that serves a
Start/Stop page and relays controls to the robot over plain topics:

    browser --HTTP--> web_control --/demo/command----> demo_wander  (start/stop)
                                   --/demo/mode-------> demo_wander  (curious/cute)
                                   --/demo/mood-------> motor_driver (chirp mood)
                                   --/demo/chirp_enable-> motor_driver (chirps on/off)
    browser <-HTTP-- web_control <--/demo/state------- demo_wander  (STOPPED/ROAMING)

Point a phone connected to the robot's WiFi at ``http://<robot-ip>:<port>/``.
The page is entirely self-contained (no external CSS/JS/fonts) so it works on
an access point with no internet.

A second "Network" tab exposes a button that switches the robot off its own
access point and back onto the home WiFi (so its code can be updated).
"""

import json
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String


MODES = ("curious", "cute")
MOODS = ("curious", "excited", "happy", "sad", "sleepy")

# Command run by the "switch to home WiFi" button (detached so it survives the
# network flip that drops this very connection).
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
    padding: 20px 16px 40px; gap: 18px;
  }
  h1 { font-size: 1.3rem; margin: 4px 0 0; letter-spacing: .5px; }
  .tabs { display: flex; gap: 8px; width: 100%; max-width: 440px; }
  .tab {
    flex: 1; text-align: center; padding: 10px 0; border-radius: 12px;
    background: #1c2029; border: 1px solid #2a2f3a; color: #cbd1da;
    font-weight: 600; cursor: pointer;
  }
  .tab.active { background: #2b3647; color: #fff; border-color: #3b4a63; }
  .panel { width: 100%; max-width: 440px; display: none; flex-direction: column; gap: 18px; }
  .panel.active { display: flex; }
  .status {
    align-self: center; display: flex; align-items: center; gap: 10px;
    font-size: 1.1rem; font-weight: 700;
    padding: 10px 18px; border-radius: 999px; background: #1c2029;
    border: 1px solid #2a2f3a;
  }
  .dot { width: 14px; height: 14px; border-radius: 50%; background: #6b7280; }
  .dot.roaming { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
  .dot.stopped { background: #ef4444; }
  .dot.offline { background: #6b7280; }
  button.big {
    width: 100%; border: none; border-radius: 18px; color: #fff;
    font-size: 1.5rem; font-weight: 700; padding: 30px 0; cursor: pointer;
    transition: transform .05s ease, filter .15s ease;
  }
  button.big:active { transform: scale(.98); }
  .start { background: #16a34a; }
  .stop  { background: #dc2626; }
  .card { background: #1c2029; border: 1px solid #2a2f3a; border-radius: 14px; padding: 14px 16px; display: flex; flex-direction: column; gap: 14px; }
  .row { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
  .row label { font-weight: 600; }
  select {
    background: #0f1115; color: #e8eaed; border: 1px solid #3a4150;
    border-radius: 10px; padding: 10px 12px; font-size: 1rem; min-width: 150px;
  }
  /* toggle switch */
  .switch { position: relative; width: 58px; height: 32px; }
  .switch input { opacity: 0; width: 0; height: 0; }
  .slider { position: absolute; inset: 0; background: #3a4150; border-radius: 999px; transition: .2s; }
  .slider:before { content: ""; position: absolute; height: 26px; width: 26px; left: 3px; top: 3px; background: #fff; border-radius: 50%; transition: .2s; }
  .switch input:checked + .slider { background: #16a34a; }
  .switch input:checked + .slider:before { transform: translateX(26px); }
  .hint { color: #9aa0aa; font-size: .85rem; text-align: center; }
  .warn { color: #fca5a5; font-size: .9rem; line-height: 1.4; }
  button.lan { background: #b45309; }
  #lanMsg { color: #fcd34d; font-size: .9rem; min-height: 1.2em; text-align: center; }
</style>
</head>
<body>
  <h1>SORA Demo Control</h1>

  <div class="tabs">
    <div class="tab active" id="tabControlBtn" onclick="showTab('control')">Control</div>
    <div class="tab" id="tabNetworkBtn" onclick="showTab('network')">Network</div>
  </div>

  <!-- CONTROL -->
  <div class="panel active" id="panel-control">
    <div class="status"><span id="dot" class="dot offline"></span><span id="state">connecting…</span></div>
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
          <option value="curious">Curious</option>
          <option value="excited">Excited</option>
          <option value="happy">Happy</option>
          <option value="sad">Sad</option>
          <option value="sleepy">Sleepy</option>
        </select>
      </div>
      <div class="row">
        <label for="chirp">Chirps</label>
        <label class="switch"><input type="checkbox" id="chirp" onchange="setChirp(this.checked)"><span class="slider"></span></label>
      </div>
    </div>
    <p class="hint">Mode sets how it moves. Mood sets the chirp sounds it makes while chirps are on.</p>
  </div>

  <!-- NETWORK -->
  <div class="panel" id="panel-network">
    <div class="card">
      <p class="warn">This switches the robot off its own <b>SORA-DEMO</b> network and back onto the home WiFi so its software can be updated.<br><br>Your phone <b>will lose this page</b> when you press it. Reconnect your phone to the normal WiFi afterwards.</p>
      <button class="big lan" onclick="switchLan()">Switch to home WiFi (for updates)</button>
      <div id="lanMsg"></div>
    </div>
    <p class="hint">To return to demo mode, reboot the robot (it starts its own network automatically).</p>
  </div>

<script>
  var dot = document.getElementById('dot');
  var stateEl = document.getElementById('state');

  function showTab(name) {
    document.getElementById('panel-control').classList.toggle('active', name === 'control');
    document.getElementById('panel-network').classList.toggle('active', name === 'network');
    document.getElementById('tabControlBtn').classList.toggle('active', name === 'control');
    document.getElementById('tabNetworkBtn').classList.toggle('active', name === 'network');
  }
  function render(d, online) {
    if (!online) { dot.className = 'dot offline'; stateEl.textContent = 'offline — reconnecting…'; return; }
    if (d.state === 'ROAMING') { dot.className = 'dot roaming'; stateEl.textContent = 'ROAMING'; }
    else if (d.state === 'STOPPED') { dot.className = 'dot stopped'; stateEl.textContent = 'STOPPED'; }
    else { dot.className = 'dot offline'; stateEl.textContent = d.state || 'unknown'; }
    if (d.mode) setSel('mode', d.mode);
    if (d.mood) setSel('mood', d.mood);
    var c = document.getElementById('chirp'); if (typeof d.chirp === 'boolean') c.checked = d.chirp;
  }
  function setSel(id, v) { var e = document.getElementById(id); if (e && e.value !== v) e.value = v; }
  function post(path) { return fetch(path, { method: 'POST' }); }
  function send(cmd) { post('/api/' + cmd).then(function(){ setTimeout(poll,120); setTimeout(poll,500); }).catch(function(){ render({}, false); }); }
  function setMode(v) { post('/api/mode/' + v); }
  function setMood(v) { post('/api/mood/' + v); }
  function setChirp(on) { post('/api/chirp/' + (on ? 'on' : 'off')); }
  function switchLan() {
    if (!confirm('Switch the robot to home WiFi? This page will disconnect.')) return;
    document.getElementById('lanMsg').textContent = 'Switching… this page will go offline shortly.';
    post('/api/switch_lan').catch(function(){});
  }
  function poll() {
    fetch('/api/status', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (d) { render(d, true); })
      .catch(function () { render({}, false); });
  }
  setInterval(poll, 1000);
  poll();
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

        def _send_status(self, code=200):
            self._send(code, json.dumps(node.status()), "application/json")

        def _route(self, method):
            path = self.path.split("?", 1)[0].rstrip("/")
            parts = [p for p in path.split("/") if p]

            if method == "GET" and path in ("", "/index.html"):
                self._send(200, PAGE, "text/html; charset=utf-8")
                return
            if parts[:1] != ["api"]:
                self._send(404, "not found", "text/plain")
                return

            api = parts[1:]
            if api == ["status"]:
                self._send_status()
            elif api == ["start"]:
                node.send_command("start"); self._send_status()
            elif api == ["stop"]:
                node.send_command("stop"); self._send_status()
            elif len(api) == 2 and api[0] == "mode":
                node.set_mode(api[1]); self._send_status()
            elif len(api) == 2 and api[0] == "mood":
                node.set_mood(api[1]); self._send_status()
            elif len(api) == 2 and api[0] == "chirp":
                node.set_chirp(api[1] == "on"); self._send_status()
            elif api == ["switch_lan"]:
                ok, msg = node.switch_to_lan()
                self._send(200 if ok else 500, json.dumps({"ok": ok, "msg": msg}),
                           "application/json")
            else:
                self._send(404, "not found", "text/plain")

        def do_GET(self):
            self._route("GET")

        def do_POST(self):
            self._route("POST")

        def log_message(self, fmt, *args):
            node.get_logger().debug("HTTP " + (fmt % args))

    return Handler


class WebControl(Node):
    def __init__(self) -> None:
        super().__init__("web_control")

        self.declare_parameter("port", 8080)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("command_topic", "/demo/command")
        self.declare_parameter("state_topic", "/demo/state")
        self.declare_parameter("mode_topic", "/demo/mode")
        self.declare_parameter("mood_topic", "/demo/mood")
        self.declare_parameter("chirp_topic", "/demo/chirp_enable")
        self.declare_parameter("default_mode", "curious")
        self.declare_parameter("default_mood", "curious")

        port = int(self.get_parameter("port").value)
        bind = self.get_parameter("bind_address").value

        self._state = "UNKNOWN"
        self._mode = self.get_parameter("default_mode").value
        self._mood = self.get_parameter("default_mood").value
        self._chirp = False
        self._lock = threading.Lock()

        self._cmd_pub = self.create_publisher(
            String, self.get_parameter("command_topic").value, 10)

        # Latched so demo_wander / motor_driver pick up the current settings
        # even if they (re)start after web_control.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._mode_pub = self.create_publisher(
            String, self.get_parameter("mode_topic").value, latched)
        self._mood_pub = self.create_publisher(
            String, self.get_parameter("mood_topic").value, latched)
        self._chirp_pub = self.create_publisher(
            Bool, self.get_parameter("chirp_topic").value, latched)
        self.create_subscription(
            String, self.get_parameter("state_topic").value, self._on_state, latched)

        # Publish initial settings so the other nodes start in a known state.
        self._publish_mode()
        self._publish_mood()
        self._publish_chirp()

        self._httpd = ThreadingHTTPServer((bind, port), _make_handler(self))
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(
            "web_control serving on http://%s:%d/  (open it from a phone on the robot's WiFi)"
            % (bind, port)
        )

    # --- ROS plumbing -------------------------------------------------
    def _on_state(self, msg: String) -> None:
        with self._lock:
            self._state = msg.data

    def _publish_mode(self):
        m = String(); m.data = self._mode; self._mode_pub.publish(m)

    def _publish_mood(self):
        m = String(); m.data = self._mood; self._mood_pub.publish(m)

    def _publish_chirp(self):
        b = Bool(); b.data = self._chirp; self._chirp_pub.publish(b)

    # --- Actions invoked by the HTTP handler --------------------------
    def status(self) -> dict:
        with self._lock:
            return {"state": self._state, "mode": self._mode,
                    "mood": self._mood, "chirp": self._chirp}

    def send_command(self, cmd: str) -> None:
        m = String(); m.data = cmd; self._cmd_pub.publish(m)
        self.get_logger().info("web command: %s" % cmd)

    def set_mode(self, mode: str) -> None:
        if mode in MODES:
            with self._lock:
                self._mode = mode
            self._publish_mode()
            self.get_logger().info("web mode: %s" % mode)

    def set_mood(self, mood: str) -> None:
        if mood in MOODS:
            with self._lock:
                self._mood = mood
            self._publish_mood()
            self.get_logger().info("web mood: %s" % mood)

    def set_chirp(self, on: bool) -> None:
        with self._lock:
            self._chirp = bool(on)
        self._publish_chirp()
        self.get_logger().info("web chirps: %s" % ("on" if on else "off"))

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
