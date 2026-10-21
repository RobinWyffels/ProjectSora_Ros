#!/usr/bin/env python3
"""Phone-friendly web control for the Innovation Cup demo.

Runs a small dependency-free HTTP server (Python stdlib) that serves a
Start/Stop page and relays the button presses to the ``demo_wander`` node:

    browser  --HTTP-->  web_control  --/demo/command-->  demo_wander
    browser  <-HTTP--   web_control  <--/demo/state----  demo_wander

Point a phone connected to the robot's WiFi at ``http://<robot-ip>:<port>/``.
The page is entirely self-contained (no external CSS/JS/fonts) so it works on
an access point with no internet.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


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
    padding: 24px 16px; gap: 24px;
  }
  h1 { font-size: 1.35rem; margin: 8px 0 0; letter-spacing: .5px; }
  .status {
    display: flex; align-items: center; gap: 10px;
    font-size: 1.1rem; font-weight: 600;
    padding: 10px 18px; border-radius: 999px; background: #1c2029;
    border: 1px solid #2a2f3a;
  }
  .dot { width: 14px; height: 14px; border-radius: 50%; background: #6b7280; }
  .dot.roaming { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
  .dot.stopped { background: #ef4444; }
  .dot.offline { background: #6b7280; }
  .buttons { width: 100%; max-width: 420px; display: flex; flex-direction: column; gap: 18px; margin-top: 8px; }
  button {
    width: 100%; border: none; border-radius: 18px; color: #fff;
    font-size: 1.6rem; font-weight: 700; padding: 34px 0; cursor: pointer;
    transition: transform .05s ease, filter .15s ease;
  }
  button:active { transform: scale(.98); }
  button:disabled { filter: grayscale(.6) brightness(.6); cursor: default; }
  .start { background: #16a34a; }
  .stop  { background: #dc2626; }
  .hint { color: #9aa0aa; font-size: .85rem; text-align: center; max-width: 420px; }
</style>
</head>
<body>
  <h1>SORA Demo Control</h1>
  <div class="status"><span id="dot" class="dot offline"></span><span id="state">connecting…</span></div>
  <div class="buttons">
    <button class="start" id="startBtn" onclick="send('start')">START ROAMING</button>
    <button class="stop"  id="stopBtn"  onclick="send('stop')">STOP</button>
  </div>
  <p class="hint">START makes the robot wander and avoid obstacles. STOP halts it in place. Keep clear while it is roaming.</p>
<script>
  var dot = document.getElementById('dot');
  var stateEl = document.getElementById('state');
  function render(state, online) {
    if (!online) { dot.className = 'dot offline'; stateEl.textContent = 'offline — reconnecting…'; return; }
    if (state === 'ROAMING') { dot.className = 'dot roaming'; stateEl.textContent = 'ROAMING'; }
    else if (state === 'STOPPED') { dot.className = 'dot stopped'; stateEl.textContent = 'STOPPED'; }
    else { dot.className = 'dot offline'; stateEl.textContent = state || 'unknown'; }
  }
  function send(cmd) {
    fetch('/api/' + cmd, { method: 'POST' })
      .then(function () { setTimeout(poll, 120); setTimeout(poll, 500); })
      .catch(function () { render(null, false); });
  }
  function poll() {
    fetch('/api/status', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (d) { render(d.state, true); })
      .catch(function () { render(null, false); });
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
            self.wfile.write(data)

        def _send_state(self, code=200):
            self._send(code, json.dumps({"state": node.get_state()}), "application/json")

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif self.path.startswith("/api/status"):
                self._send_state()
            elif self.path == "/api/start":
                node.send_command("start")
                self._send_state()
            elif self.path == "/api/stop":
                node.send_command("stop")
                self._send_state()
            else:
                self._send(404, "not found", "text/plain")

        def do_POST(self):
            if self.path == "/api/start":
                node.send_command("start")
                self._send_state()
            elif self.path == "/api/stop":
                node.send_command("stop")
                self._send_state()
            else:
                self._send(404, "not found", "text/plain")

        def log_message(self, fmt, *args):
            # Route HTTP access logs to ROS debug instead of stderr spam.
            node.get_logger().debug("HTTP " + (fmt % args))

    return Handler


class WebControl(Node):
    def __init__(self) -> None:
        super().__init__("web_control")

        self.declare_parameter("port", 8080)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("command_topic", "/demo/command")
        self.declare_parameter("state_topic", "/demo/state")

        port = int(self.get_parameter("port").value)
        bind = self.get_parameter("bind_address").value
        command_topic = self.get_parameter("command_topic").value
        state_topic = self.get_parameter("state_topic").value

        self._state = "UNKNOWN"
        self._lock = threading.Lock()

        self._cmd_pub = self.create_publisher(String, command_topic, 10)

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, state_topic, self._on_state, state_qos)

        self._httpd = ThreadingHTTPServer((bind, port), _make_handler(self))
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(
            "web_control serving on http://%s:%d/  (open it from a phone on the robot's WiFi)"
            % (bind, port)
        )

    def _on_state(self, msg: String) -> None:
        with self._lock:
            self._state = msg.data

    def get_state(self) -> str:
        with self._lock:
            return self._state

    def send_command(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self.get_logger().info("web command: %s" % cmd)

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
