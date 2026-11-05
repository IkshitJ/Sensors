#!/usr/bin/env python3
"""Local web app for the zebra-crossing simulator (Python standard library only).

    python3 server.py            # builds the simulator if needed, serves http://localhost:8080
    python3 server.py --port 9000

The browser talks to ONE running simulator process (a real Arduino sketch compiled
natively, see simulation/src/harness.cpp) through:
    GET  /api/stream         Server-Sent Events: one JSON frame per 40 ms
    POST /api/cmd            text body, e.g. "spawn group 3" or "serial r"
    POST /api/firmware       {"fw": "v2"|"cal", "scenario": "normal", "seed": 1}
    POST /api/bench          {"scenarios": [...], "seeds": 3, "duration": 600}
    GET  /api/bench          progress / last result
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
sys.path.insert(0, HERE)
import bench  # noqa: E402

FIRMWARES = {"v2", "cal"}


def build():
    print("Building simulator (make)...", flush=True)
    r = subprocess.run(["make", "-C", HERE], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("Build failed - is Xcode Command Line Tools (clang++) installed?")


class Simulator:
    """Owns the running firmware process and fans its frames out to SSE clients."""

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.fw = None
        self.scenario = "normal"
        self.clients = set()

    def start(self, fw="v2", scenario="normal", seed=1):
        with self.lock:
            self._stop()
            exe = os.path.join(HERE, "bin", "zebra_" + fw)
            self.proc = subprocess.Popen([exe, "--interactive", "--scenario", scenario, "--seed", str(seed)],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
            self.fw = fw
            self.scenario = scenario
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()

    def _stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin.close()
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()
        self.proc = None

    def _pump(self, proc):
        for line in proc.stdout:
            dead = []
            for q in list(self.clients):
                try:
                    q.put_nowait(line)
                except queue.Full:
                    dead.append(q)  # slow client: drop frames rather than block
            for q in dead:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass

    def send(self, cmd):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                self.proc.stdin.write(cmd.strip() + "\n")
                self.proc.stdin.flush()


SIM = Simulator()
BENCH = {"running": False, "done": 0, "total": 0, "result": None, "error": None}
RESULTS_FILE = os.path.join(HERE, "results.json")
if os.path.exists(RESULTS_FILE):
    try:
        with open(RESULTS_FILE) as f:
            BENCH["result"] = {"table": json.load(f), "fws": ["v2"], "seeds": 3, "duration": 600,
                               "source": "results.json"}
    except Exception:
        pass


def run_bench(params):
    try:
        fws = ["v2"]
        scs = [s for s in params.get("scenarios", bench.SCENARIOS) if s in bench.SCENARIOS]
        seeds = max(1, min(10, int(params.get("seeds", 3))))
        duration = max(30, min(3600, float(params.get("duration", 600))))
        BENCH.update(running=True, done=0, total=len(fws) * len(scs) * seeds, error=None)

        def prog(d, t):
            BENCH["done"] = d

        table = bench.run_suite(fws, scs, seeds, duration, progress=prog)
        BENCH["result"] = {"table": table, "fws": fws, "seeds": seeds, "duration": duration, "source": "live run"}
    except Exception as e:  # report to the UI
        BENCH["error"] = str(e)
    finally:
        BENCH["running"] = False


MIME = {".html": "text/html; charset=utf-8", ".js": "application/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".json": "application/json"}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        return self.rfile.read(n).decode() if n else ""

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/api/stream":
            return self._stream()
        if path == "/api/bench":
            return self._send(200, json.dumps(BENCH))
        if path == "/api/state":
            return self._send(200, json.dumps({"fw": SIM.fw, "scenario": SIM.scenario}))
        if path == "/":
            path = "/index.html"
        f = os.path.normpath(os.path.join(WEB, path.lstrip("/")))
        if not f.startswith(WEB) or not os.path.isfile(f):
            return self._send(404, "not found", "text/plain")
        with open(f, "rb") as fh:
            self._send(200, fh.read(), MIME.get(os.path.splitext(f)[1], "application/octet-stream"))

    def do_POST(self):
        path = self.path.split("?")[0]
        if path == "/api/cmd":
            SIM.send(self._body()[:200])
            return self._send(200, "{}")
        if path == "/api/firmware":
            p = json.loads(self._body() or "{}")
            fw = p.get("fw", "v2")
            if fw not in FIRMWARES:
                return self._send(400, '{"error":"unknown firmware"}')
            SIM.start(fw, p.get("scenario", SIM.scenario or "normal"), int(p.get("seed", 1)))
            return self._send(200, "{}")
        if path == "/api/bench":
            if BENCH["running"]:
                return self._send(409, '{"error":"already running"}')
            p = json.loads(self._body() or "{}")
            threading.Thread(target=run_bench, args=(p,), daemon=True).start()
            return self._send(200, "{}")
        self._send(404, "{}")

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = queue.Queue(maxsize=50)
        SIM.clients.add(q)
        try:
            while True:
                try:
                    line = q.get(timeout=5)
                    self.wfile.write(b"data: " + line.strip().encode() + b"\n\n")
                except queue.Empty:
                    self.wfile.write(b": keep-alive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            SIM.clients.discard(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="127.0.0.1")
    a = ap.parse_args()
    build()
    SIM.start("v2", "normal", 1)
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    print(f"Zebra crossing simulator running at  http://localhost:{a.port}   (Ctrl+C to stop)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        SIM._stop()


if __name__ == "__main__":
    main()
