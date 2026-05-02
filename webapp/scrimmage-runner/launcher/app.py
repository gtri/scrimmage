"""Flask launcher sidecar for SCRIMMAGE.

Runs as PID 1, exposes HTTP on :5050 to start/stop scrimmage processes
and list available missions. Templates the chosen mission XML to enable
gRPC streaming on :50051 before launching.
"""
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from xml.etree import ElementTree as ET

from flask import Flask, jsonify, request

# Patterns to suppress in scrimmage's stdout/stderr — these fire dozens of times per
# second from a plugin internal-state mismatch we don't care about for the demo.
NOISE_PATTERNS = (
    "VariableIO::output index",
)

# Rolling buffer of scrimmage's most recent (filtered) output lines. The post-mission
# report (SimpleCollisionMetrics, SimpleCaptureMetrics, etc.) lands at the end of this
# buffer when scrimmage exits, and is exposed via GET /missions/report.
_OUTPUT_BUFFER_LINES = 500
_output_buffer: deque[str] = deque(maxlen=_OUTPUT_BUFFER_LINES)
_buffer_lock = threading.Lock()


def _forward_filtered(stream):
    """Forward a child process's output line-by-line, dropping known noise lines."""
    for line in iter(stream.readline, ''):
        if any(p in line for p in NOISE_PATTERNS):
            continue
        with _buffer_lock:
            _output_buffer.append(line)
        sys.stdout.write(line)
        sys.stdout.flush()

MISSIONS_DIR = Path(os.environ.get("MISSIONS_DIR", "/root/scrimmage/scrimmage/missions"))
ACTIVE_MISSION_PATH = Path("/tmp/active_mission.xml")

app = Flask(__name__)

_state = {"proc": None, "mission": None, "started_at": None, "origin": None}


def _running() -> bool:
    p = _state["proc"]
    return p is not None and p.poll() is None


def _stop():
    p = _state["proc"]
    if p and p.poll() is None:
        # If the process is currently SIGSTOPped (paused), it can't receive SIGTERM.
        # Resume it first so termination actually takes effect.
        if _state.get("paused"):
            try:
                p.send_signal(signal.SIGCONT)
            except Exception:
                pass
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    _state.update({"proc": None, "mission": None, "started_at": None, "origin": None, "paused": False})


def _template_mission(src: Path, time_warp=None) -> dict:
    """Copy mission XML to /tmp with network_gui flipped on. Return parsed origin.

    Optional time_warp overrides the mission's built-in time_warp attribute (e.g.,
    capture-the-flag defaults to 20; pass 5 to slow it down ~4x).
    """
    text = src.read_text()
    # Flip network_gui="false" to "true" (regex tolerates whitespace around =)
    text = re.sub(r'network_gui\s*=\s*"false"', 'network_gui="true"', text)
    # Force enable_gui off so we don't try to open VTK in a headless container
    text = re.sub(r'enable_gui\s*=\s*"\$\{enable_gui=true\}"',
                  'enable_gui="false"', text)
    text = re.sub(r'enable_gui\s*=\s*"true"', 'enable_gui="false"', text)
    # Route the scrimmage gRPC stream to the API container's HTTP/2 cleartext port.
    # The API listens on :8080 (HTTP/1 for REST + SignalR) and :50051 (HTTP/2 cleartext for gRPC).
    text = re.sub(r'<stream_ip>[^<]*</stream_ip>', '<stream_ip>api</stream_ip>', text)
    text = re.sub(r'<stream_port>[^<]*</stream_port>', '<stream_port>50051</stream_port>', text)
    # Override time_warp if requested (operator-controlled simulation speed)
    if time_warp is not None:
        text = re.sub(r'time_warp\s*=\s*"[^"]*"', f'time_warp="{time_warp}"', text)
    ACTIVE_MISSION_PATH.write_text(text)

    # Parse origin from the templated file
    root = ET.fromstring(text)
    def _find(tag):
        node = root.find(tag)
        return float(node.text) if node is not None and node.text else None
    return {
        "lat": _find("latitude_origin"),
        "lon": _find("longitude_origin"),
        "alt": _find("altitude_origin"),
    }


@app.get("/missions")
def list_missions():
    if not MISSIONS_DIR.exists():
        return jsonify({"error": f"missions dir not found: {MISSIONS_DIR}"}), 500
    files = sorted(p.name for p in MISSIONS_DIR.glob("*.xml"))
    return jsonify(files)


@app.post("/missions/start")
def start_mission():
    body = request.get_json(silent=True) or {}
    name = body.get("name")
    if not name:
        return jsonify({"error": "missing 'name'"}), 400
    # Operator-controlled simulation speed; defaults to whatever the mission XML has
    time_warp = body.get("timeWarp")
    src = MISSIONS_DIR / name
    if not src.exists():
        return jsonify({"error": f"mission not found: {name}"}), 404

    # Stop anything currently running
    _stop()

    # Fresh mission → fresh output buffer (any prior report is now stale)
    with _buffer_lock:
        _output_buffer.clear()

    # Template + parse origin
    try:
        origin = _template_mission(src, time_warp=time_warp)
    except Exception as e:
        return jsonify({"error": f"failed to template mission: {e}"}), 500
    if not all(origin.values()):
        return jsonify({"error": "mission has no geographic origin (lat/lon/alt) — incompatible with Cesium viewer"}), 400

    # Launch scrimmage. Capture stdout/stderr through a filter thread so we can
    # suppress known plugin-noise lines while still surfacing real errors.
    proc = subprocess.Popen(
        ["scrimmage", str(ACTIVE_MISSION_PATH)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        text=True,
    )
    threading.Thread(target=_forward_filtered, args=(proc.stdout,), daemon=True).start()
    _state.update({
        "proc": proc,
        "mission": name,
        "started_at": time.time(),
        "origin": origin,
        "time_warp": time_warp,
    })

    return jsonify({
        "status": "started",
        "pid": proc.pid,
        "mission": name,
        "origin": origin,
        "timeWarp": time_warp,
    })


@app.post("/missions/stop")
def stop_mission():
    _stop()
    # Give the forwarder thread a moment to drain any final lines (the report)
    # that scrimmage emitted on its way out before we respond.
    time.sleep(0.3)
    return jsonify({"status": "stopped"})


@app.get("/missions/report")
def get_report():
    """Return the most recent ~500 lines of scrimmage output.

    The post-mission report (Metrics summaries) is at the tail; any lines before
    it are recent run-time chatter that didn't fit in the rolling buffer.
    """
    with _buffer_lock:
        lines = list(_output_buffer)
    return jsonify({"lines": lines})


@app.post("/missions/pause")
def pause_mission():
    """SIGSTOP the scrimmage process — freezes it OS-level. State preserved exactly."""
    p = _state["proc"]
    if not p or p.poll() is not None:
        return jsonify({"error": "no mission running"}), 400
    if _state.get("paused"):
        return jsonify({"status": "paused"})  # idempotent
    try:
        p.send_signal(signal.SIGSTOP)
    except Exception as e:
        return jsonify({"error": f"failed to pause: {e}"}), 500
    _state["paused"] = True
    return jsonify({"status": "paused"})


@app.post("/missions/resume")
def resume_mission():
    """SIGCONT the scrimmage process — resumes it from where it was frozen."""
    p = _state["proc"]
    if not p or p.poll() is not None:
        return jsonify({"error": "no mission running"}), 400
    if not _state.get("paused"):
        return jsonify({"status": "running"})  # idempotent
    try:
        p.send_signal(signal.SIGCONT)
    except Exception as e:
        return jsonify({"error": f"failed to resume: {e}"}), 500
    _state["paused"] = False
    return jsonify({"status": "running"})


@app.get("/status")
def status():
    if not _running():
        return jsonify({"status": "idle"})
    return jsonify({
        "status": "paused" if _state.get("paused") else "running",
        "mission": _state["mission"],
        "uptime_s": time.time() - _state["started_at"],
        "origin": _state["origin"],
        "paused": bool(_state.get("paused")),
    })


def _shutdown(*_):
    _stop()
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    app.run(host="0.0.0.0", port=5050)
