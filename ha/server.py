import json, time
from flask import Flask, Response, request, jsonify, send_from_directory
import os

def create_app(pipe):
    app = Flask(__name__, static_folder=os.path.join(os.path.dirname(__file__), "static"))

    @app.get("/")
    def index():
        return send_from_directory(app.static_folder, "index.html")

    @app.get("/api/state")
    def state():
        with pipe.lock: return jsonify(pipe.state)

    @app.get("/api/stream")
    def stream():
        def gen():
            while True:
                with pipe.lock: s = json.dumps(pipe.state)
                yield f"data: {s}\n\n"; time.sleep(0.1)
        return Response(gen(), mimetype="text/event-stream", headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @app.post("/api/control")
    def control():
        pipe.control(request.get_json(force=True) or {}); return jsonify(ok=True)

    return app
