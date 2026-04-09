"""
Run-time View Flask Server - Swarm Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>

Provides a REST API and WebSocket stream for live drone telemetry.
"""

import os
import json
from flask import Flask, jsonify, render_template, request
from flask_sock import Sock

# Use absolute imports from project root
from simulation.live_telemetry import MAVLinkLiveSource, TelemetryQueue

app = Flask(__name__, static_folder='web', template_folder='web')
sock = Sock(app)

# Global telemetry source and queue (singleton for the server)
telemetry_queue = TelemetryQueue(maxlen=4096)
live_source = None

# Track active WebSocket clients
clients = []

@app.route('/')
def index():
    """Render the mission launcher."""
    return render_template('index.html')

@app.route('/live')
def live_view():
    """Render the 3D live view."""
    return render_template('live.html')

@app.route('/api/missions')
def get_missions():
    """Return the catalogue of available missions."""
    missions_path = os.path.join(os.path.dirname(__file__), 'missions.json')
    if os.path.exists(missions_path):
        with open(missions_path, 'r') as f:
            return jsonify(json.load(f))
    return jsonify([])

@app.route('/api/status')
def get_status():
    """Return the current telemetry status."""
    latest = telemetry_queue.latest()
    return jsonify({
        "connected": live_source is not None and live_source._running,
        "sample_count": len(telemetry_queue),
        "latest_sample": latest.to_dict() if latest else None
    })

@sock.route('/ws/telemetry')
def telemetry_stream(ws):
    """Stream latest telemetry samples via WebSocket."""
    clients.append(ws)
    try:
        # Send current snapshot first
        snapshot = telemetry_queue.snapshot(n=100)
        ws.send(json.dumps({"type": "snapshot", "data": [s.to_dict() for s in snapshot]}))

        # Then wait for new samples
        last_t = 0
        while True:
            latest = telemetry_queue.latest()
            if latest and latest.t_wall > last_t:
                ws.send(json.dumps({"type": "sample", "data": latest.to_dict()}))
                last_t = latest.t_wall
            # Small sleep to prevent busy loop, though ws.receive() or similar is better,
            # for a stream we just poll the queue.
            import time
            time.sleep(0.02) # 50Hz cap
    finally:
        clients.remove(ws)

def start_telemetry(listen_port=14550):
    """Initialize and start the MAVLink telemetry source."""
    global live_source
    if live_source is None:
        live_source = MAVLinkLiveSource(listen_port=listen_port, queue=telemetry_queue)
        live_source.start()

def run_server(host='127.0.0.1', port=5000, mav_port=14550):
    """Run the Flask development server."""
    start_telemetry(mav_port)
    app.run(host=host, port=port, threaded=True)

if __name__ == '__main__':
    run_server()
