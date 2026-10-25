#!/usr/bin/env python3
"""
Web interface for choosing which coloured box the Panda should pick.

Open http://localhost:8765 (VS Code forwards the port from the container).
The page shows a live video of the robot from the Gazebo scene camera, the
annotated overhead camera image, the robot status and one button per colour.
Clicking a button publishes the colour on /target_color.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Franka Color Sorting Robot</title>
<style>
  :root { --bg:#f4f5f7; --card:#fff; --text:#1d2330; --muted:#667085; --border:#dde1e7; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#14171c; --card:#1d2128; --text:#e8eaee; --muted:#98a1b0; --border:#2e343e; }
  }
  * { box-sizing: border-box; }
  body { margin:0; font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
         background:var(--bg); color:var(--text); }
  main { max-width: 1280px; margin: 0 auto; padding: 24px 16px 40px; }
  .layout { display:grid; grid-template-columns: minmax(0, 2fr) minmax(0, 1fr); gap:16px; align-items:start; }
  @media (max-width: 900px) { .layout { grid-template-columns: minmax(0, 1fr); } }
  .video { position:relative; background:#000; border-radius:8px; }
  [hidden] { display:none !important; }
  .live { position:absolute; top:10px; left:10px; background:#d92d20; color:#fff; font-size:.75rem; font-weight:700;
          letter-spacing:.05em; padding:3px 8px; border-radius:6px; }
  .live.off { background:#667085; }
  .video img.stale { opacity:.35; }
  .placeholder { position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
                 color:#e8eaee; font-weight:600; text-align:center; padding:16px; pointer-events:none; }
  h1 { font-size: 1.4rem; margin: 0 0 4px; }
  p.lead { margin: 0 0 20px; color: var(--muted); }
  .card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:16px; margin-bottom:16px; }
  .buttons { display:grid; grid-template-columns: repeat(3, 1fr); gap:12px; }
  button { font: inherit; font-weight:600; font-size:1.05rem; color:#fff; border:0; border-radius:10px;
           padding:18px 8px; cursor:pointer; transition: transform .05s, opacity .2s; }
  button:active { transform: scale(.97); }
  button:disabled { opacity:.35; cursor:not-allowed; }
  button.red { background:#d92d20; } button.green { background:#16a34a; } button.blue { background:#2563eb; }
  button.shuffle { width:100%; margin-top:12px; padding:12px 8px; font-size:.95rem; background:#475467; }
  .status { display:flex; align-items:center; gap:10px; }
  .dot { width:12px; height:12px; border-radius:50%; background:#98a1b0; flex:none; }
  .dot.idle, .dot.done { background:#16a34a; } .dot.busy, .dot.starting { background:#f59e0b; }
  .dot.error { background:#d92d20; }
  .message { font-weight:500; }
  .meta { color:var(--muted); font-size:.9rem; margin-top:6px; }
  img { width:100%; border-radius:8px; display:block; background:#000; aspect-ratio: 4 / 3; object-fit: contain; }
  .label { font-size:.8rem; text-transform:uppercase; letter-spacing:.05em; color:var(--muted); margin:0 0 8px; }
</style>
</head>
<body>
<main>
  <h1>Franka Color Sorting Robot</h1>
  <p class="lead">Choose a box colour. The robot plans around the obstacles, picks the box and drops it into the crate.</p>

  <div class="layout">
    <div class="card">
      <p class="label">Gazebo simulation (live)</p>
      <div class="video">
        <img id="scene" src="/scene.mjpg" alt="Live view of the Panda robot in Gazebo">
        <div class="placeholder" id="scene-placeholder" hidden></div>
        <span class="live off" id="live">CONNECTING</span>
      </div>
    </div>

    <div>
      <div class="card">
        <p class="label">Pick a colour</p>
        <div class="buttons">
          <button class="red" data-color="red">Red</button>
          <button class="green" data-color="green">Green</button>
          <button class="blue" data-color="blue">Blue</button>
        </div>
        <button class="shuffle" id="shuffle">New random layout</button>
        <div class="meta">Moves the boxes and obstacles to new random positions (boxes in the crate go back on the table).</div>
      </div>

      <div class="card">
        <p class="label">Robot status</p>
        <div class="status"><span class="dot" id="dot"></span><span class="message" id="message">Connecting…</span></div>
        <div class="meta" id="meta"></div>
      </div>

      <div class="card">
        <p class="label">Overhead camera (detections)</p>
        <div class="video">
          <img id="camera" alt="">
          <div class="placeholder" id="camera-placeholder">Waiting for the overhead camera…</div>
        </div>
        <div class="meta" id="detections"></div>
      </div>
    </div>
  </div>
</main>
<script>
  const buttons = [...document.querySelectorAll('button[data-color]')];
  const shuffle = document.getElementById('shuffle');
  buttons.forEach(b => b.addEventListener('click', async () => {
    buttons.forEach(x => x.disabled = true); shuffle.disabled = true;
    await fetch('/api/pick', { method: 'POST', headers: {'Content-Type': 'application/json'},
                               body: JSON.stringify({ color: b.dataset.color }) });
    refresh();
  }));
  shuffle.addEventListener('click', async () => {
    buttons.forEach(x => x.disabled = true); shuffle.disabled = true;
    await fetch('/api/new_layout', { method: 'POST' });
    refresh();
  });

  async function refresh() {
    try {
      const s = await (await fetch('/api/status')).json();
      const st = s.status || {};
      const state = st.state || 'starting';
      document.getElementById('dot').className = 'dot ' + state;
      document.getElementById('message').textContent = st.message || 'Waiting for the pick-and-place node…';
      const picked = st.picked || [];
      document.getElementById('meta').textContent = picked.length ? 'In the crate: ' + picked.join(', ') : '';
      const ready = state === 'idle' || state === 'done' || state === 'error';
      buttons.forEach(b => b.disabled = !ready || picked.includes(b.dataset.color));
      shuffle.disabled = !ready;
      const d = Object.entries(s.detections || {}).map(([c, p]) => `${c}: (${p[0].toFixed(2)}, ${p[1].toFixed(2)})`);
      document.getElementById('detections').textContent = d.length ? 'Detected (world x, y): ' + d.join(' · ') : 'No boxes detected yet';
      setLive(s.scene_live, 'Waiting for Gazebo to render…');
    } catch (e) {
      document.getElementById('dot').className = 'dot';
      document.getElementById('message').textContent = 'Simulation not running. Start it with: ros2 launch panda_bringup simulated_robot.launch.py';
      document.getElementById('meta').textContent = '';
      document.getElementById('detections').textContent = '';
      buttons.forEach(b => b.disabled = true);
      shuffle.disabled = true;
      setLive(false, 'Simulation not running');
      cameraOk(false);
    }
  }

  // Overhead camera: a still image refreshed every 700 ms; a placeholder replaces
  // the browser's broken-image icon while there is no frame.
  const camera = document.getElementById('camera'), cameraPlaceholder = document.getElementById('camera-placeholder');
  function cameraOk(ok) { camera.style.visibility = ok ? 'visible' : 'hidden'; cameraPlaceholder.hidden = ok; }
  camera.addEventListener('load', () => cameraOk(true));
  camera.addEventListener('error', () => cameraOk(false));
  cameraOk(false);
  function refreshCamera() { camera.src = '/camera.jpg?t=' + Date.now(); }

  // The scene video is an MJPEG stream; reconnect if it drops (e.g. Gazebo restarted).
  const scene = document.getElementById('scene'), live = document.getElementById('live');
  const scenePlaceholder = document.getElementById('scene-placeholder');
  let wasLive = false;
  scene.addEventListener('error', () => {
    setTimeout(() => { scene.src = '/scene.mjpg?t=' + Date.now(); }, 2000);
  });
  function setLive(on, reason) {
    live.textContent = on ? 'LIVE' : 'OFFLINE';
    live.className = on ? 'live' : 'live off';
    scene.classList.toggle('stale', !on);  // dim the last frame instead of showing it as live
    scenePlaceholder.hidden = on;
    scenePlaceholder.textContent = reason;
    // Reopen the stream when the simulation comes back
    if (on && !wasLive) scene.src = '/scene.mjpg?t=' + Date.now();
    wasLive = on;
  }

  setInterval(refresh, 1000); setInterval(refreshCamera, 700);
  refresh(); refreshCamera();
</script>
</body>
</html>
"""

COLOR_NAMES = {"R": "red", "G": "green", "B": "blue"}


class ColorSelector(Node):
    def __init__(self):
        super().__init__("color_selector")
        self.declare_parameter("port", 8765)
        self.port = int(self.get_parameter("port").value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.jpeg = None
        self.status = {}
        self.detections = {}
        # Latest scene camera frame; MJPEG clients wait on the condition for new frames
        self.scene_cond = threading.Condition()
        self.scene_jpeg = None
        self.scene_stamp = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(String, "/target_color", 10)
        self.layout_pub = self.create_publisher(String, "/pick_place/new_layout", 10)
        self.create_subscription(String, "/pick_place/status", self.status_callback, latched)
        self.create_subscription(String, "/color_coordinates", self.coords_callback, 10)
        self.create_subscription(Image, "/color_detector/debug_image", self.image_callback, 1)
        self.create_subscription(Image, "/scene_camera/image", self.scene_callback, 1)

    def status_callback(self, msg):
        with self.lock:
            self.status = json.loads(msg.data)

    def coords_callback(self, msg):
        fields = msg.data.split(",")
        if len(fields) >= 3 and fields[0] in COLOR_NAMES:
            with self.lock:
                self.detections[COLOR_NAMES[fields[0]]] = (float(fields[1]), float(fields[2]), time.monotonic())

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.jpeg = buffer.tobytes()

    def scene_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.scene_cond:
                self.scene_jpeg = buffer.tobytes()
                self.scene_stamp = time.monotonic()
                self.scene_cond.notify_all()

    def pick(self, color):
        self.get_logger().info(f"Colour selected in the web UI: {color}")
        self.target_pub.publish(String(data=color))

    def new_layout(self):
        self.get_logger().info("New random layout requested in the web UI")
        self.layout_pub.publish(String())

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def send(self, code, body, content_type):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/":
                    self.send(200, PAGE.encode(), "text/html; charset=utf-8")
                elif path == "/api/status":
                    with node.lock:
                        detections = {c: d[:2] for c, d in node.detections.items()
                                      if time.monotonic() - d[2] < 3.0}
                        scene_live = time.monotonic() - node.scene_stamp < 3.0
                        body = json.dumps({"status": node.status, "detections": detections,
                                           "scene_live": scene_live})
                    self.send(200, body.encode(), "application/json")
                elif path == "/scene.mjpg":
                    self.stream_scene()
                elif path == "/camera.jpg":
                    with node.lock:
                        jpeg = node.jpeg
                    if jpeg is None:
                        self.send(503, b"camera not available yet", "text/plain")
                    else:
                        self.send(200, jpeg, "image/jpeg")
                else:
                    self.send(404, b"not found", "text/plain")

            def stream_scene(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = None
                try:
                    while True:
                        with node.scene_cond:
                            node.scene_cond.wait_for(lambda: node.scene_jpeg is not last, timeout=5.0)
                            jpeg = node.scene_jpeg
                        if jpeg is None or jpeg is last:
                            continue
                        last = jpeg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                         + jpeg + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                if self.path == "/api/new_layout":
                    node.new_layout()
                    self.send(200, b"{}", "application/json")
                    return
                if self.path != "/api/pick":
                    self.send(404, b"not found", "text/plain")
                    return
                length = int(self.headers.get("Content-Length", 0))
                try:
                    color = json.loads(self.rfile.read(length))["color"]
                except (ValueError, KeyError):
                    self.send(400, b"expected {\"color\": ...}", "text/plain")
                    return
                if color not in COLOR_NAMES.values():
                    self.send(400, b"unknown colour", "text/plain")
                    return
                node.pick(color)
                self.send(200, b"{}", "application/json")

        return Handler

    def serve(self):
        server = ThreadingHTTPServer(("0.0.0.0", self.port), self.make_handler())
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"Colour selector UI running at http://localhost:{self.port}")
        return server


def main():
    rclpy.init()
    node = ColorSelector()
    server = node.serve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
