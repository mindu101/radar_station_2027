#!/usr/bin/env python3
"""Local-only dashboard and launcher for the existing C++ radar pipeline."""

import argparse
import json
import mimetypes
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit


ROOT = Path(__file__).resolve().parent.parent
WEB = ROOT / "dashboard" / "web"
RUNTIME = ROOT / ".cache" / "dashboard"
SNAPSHOT = RUNTIME / "snapshot"
UPLOADS = RUNTIME / "uploads"
RADAR = ROOT / "build" / "bin" / "radar"


class RadarProcess:
    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.state = "idle"
        self.exit_code = None
        self.source = None
        self.started_at = None
        self.log_path = RUNTIME / "radar.log"
        self.command = None
        self.rebuilt_engines = False

    def spawn(self, command, append=False):
        with self.log_path.open("a" if append else "w") as output:
            self.process = subprocess.Popen(command, cwd=ROOT, stdout=output,
                                            stderr=subprocess.STDOUT, start_new_session=True)

    def refresh(self):
        if self.process and self.process.poll() is not None:
            self.exit_code = self.process.returncode
            if self.state == "running" and not self.rebuilt_engines and self.log_path.exists() and \
                    "engine plan file is not compatible" in self.log_path.read_text(errors="replace"):
                self.rebuilt_engines = True
                backup = RUNTIME / "engine-backup"
                backup.mkdir(parents=True, exist_ok=True)
                for engine in (ROOT / "models" / "car.engine", ROOT / "models" / "armor.engine"):
                    if engine.is_file():
                        shutil.copy2(engine, backup / engine.name)
                self.spawn([str(RADAR), "-m=0"], append=True)
                self.state = "preparing"
                self.exit_code = None
                return
            if self.state == "preparing" and self.exit_code == 0:
                self.spawn(self.command, append=True)
                self.state = "running"
                self.exit_code = None
                return
            self.state = "finished" if self.exit_code == 0 else "failed"
            self.process = None

    @staticmethod
    def calibrated():
        config = ROOT / "config.yml"
        if not config.exists():
            return False
        content = config.read_text(errors="replace")
        return bool(re.search(r"^rvec\s*:", content, re.M) and
                    re.search(r"^tvec\s*:", content, re.M))

    def status(self):
        with self.lock:
            self.refresh()
            log = ""
            if self.state == "failed" and self.log_path.exists():
                log = "\n".join(self.log_path.read_text(errors="replace").splitlines()[-12:])
            return {"state": self.state, "exit_code": self.exit_code,
                    "source": self.source, "started_at": self.started_at,
                    "calibrated": self.calibrated(), "log": log,
                    "binary_ready": RADAR.is_file()}

    def start(self, request):
        with self.lock:
            self.refresh()
            if self.process:
                raise ValueError("雷达进程已经在运行")
            if not RADAR.is_file():
                raise ValueError("未找到 build/bin/radar，请先构建项目")
            source = request.get("source")
            if source not in ("recording", "opencv", "aravis"):
                raise ValueError("请选择相机或录像来源")
            raw_value = str(request.get("value", "")).strip()
            if source == "recording":
                video = Path(raw_value).expanduser().resolve()
                if not video.is_file():
                    raise ValueError("录像文件不存在")
                value = str(video)
                video_type = "0"
            else:
                if not raw_value.isdecimal():
                    raise ValueError("相机编号必须是非负整数")
                value = raw_value
                video_type = "1" if source == "opencv" else "2"
            SNAPSHOT.mkdir(parents=True, exist_ok=True)
            (SNAPSHOT / "state.json").unlink(missing_ok=True)
            for old_frame in SNAPSHOT.glob("frame-*.jpg"):
                old_frame.unlink()
            command = [str(RADAR), "-m=" + ("7" if request.get("calibrate") else "3"),
                       "-V=" + video_type, "--dashboard-dir=" + str(SNAPSHOT),
                       "-I=" + str(ROOT / "field.jpg")]
            if source == "recording":
                command.append("--realtime")
                if request.get("loop"):
                    command[1] = "-m=" + ("71" if request.get("calibrate") else "67")
            command.append(value)
            RUNTIME.mkdir(parents=True, exist_ok=True)
            self.command = command
            self.rebuilt_engines = False
            self.spawn(command)
            self.state = "running"
            self.exit_code = None
            self.source = source
            self.started_at = int(time.time() * 1000)
            return self.status_unlocked()

    def status_unlocked(self):
        return {"state": self.state, "source": self.source,
                "calibrated": self.calibrated(), "binary_ready": True}

    def stop(self):
        with self.lock:
            self.refresh()
            if self.process:
                process = self.process
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                self.process = None
                self.exit_code = process.returncode
            self.state = "stopped"
            return self.status_unlocked()


PROCESS = RadarProcess()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format_string, *args):
        if not self.path.startswith(("/api/state", "/api/status", "/api/frame")):
            super().log_message(format_string, *args)

    def respond(self, status, body, content_type="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/status":
            return self.respond(200, PROCESS.status())
        if path == "/api/state":
            file = SNAPSHOT / "state.json"
            return self.respond(200, file.read_bytes() if file.is_file() else {}, "application/json")
        if path == "/api/frame":
            sequence = parse_qs(urlsplit(self.path).query).get("sequence", [""])[0]
            if not sequence.isdecimal():
                return self.respond(400, {"error": "Missing frame sequence"})
            file = SNAPSHOT / f"frame-{sequence}.jpg"
            if not file.is_file():
                return self.respond(404, {"error": "Frame unavailable"})
            return self.respond(200, file.read_bytes(), "image/jpeg")
        allowed = {"/": WEB / "index.html", "/style.css": WEB / "style.css",
                   "/app.js": WEB / "app.js", "/field.jpg": ROOT / "field.jpg"}
        file = allowed.get(path)
        if not file or not file.is_file():
            return self.respond(404, {"error": "Not found"})
        mime = mimetypes.guess_type(file.name)[0] or "application/octet-stream"
        if mime.startswith("text/") or mime == "application/javascript":
            mime += "; charset=utf-8"
        self.respond(200, file.read_bytes(), mime)

    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if path == "/api/upload":
                if not 0 < size <= 8 * 1024 ** 3:
                    raise ValueError("录像文件大小必须在 0–8 GB 之间")
                original = Path(unquote(self.headers.get("X-Filename", "recording.mp4"))).name
                name = re.sub(r"[^\w.\-]", "_", original)
                UPLOADS.mkdir(parents=True, exist_ok=True)
                destination = UPLOADS / (str(int(time.time() * 1000)) + "_" + name)
                with destination.open("wb") as output:
                    remaining = size
                    while remaining:
                        chunk = self.rfile.read(min(1024 * 1024, remaining))
                        if not chunk:
                            raise ValueError("录像上传中断")
                        output.write(chunk)
                        remaining -= len(chunk)
                return self.respond(200, {"path": str(destination)})
            if size > 65536:
                raise ValueError("请求过大")
            data = json.loads(self.rfile.read(size) or b"{}")
            if path == "/api/start":
                return self.respond(200, PROCESS.start(data))
            if path == "/api/stop":
                return self.respond(200, PROCESS.stop())
            self.respond(404, {"error": "Not found"})
        except (ValueError, OSError, json.JSONDecodeError) as error:
            self.respond(400, {"error": str(error)})


def main():
    parser = argparse.ArgumentParser(description="Radar station local dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Radar dashboard: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        PROCESS.stop()
        server.server_close()


if __name__ == "__main__":
    main()
