#!/usr/bin/env python3
"""A web dashboard for one question: is it the servos, or is it the policy?

    sudo python3 duck_dash.py                 # on the board
    ssh -L 8770:127.0.0.1:8770 radxa@<board>  # on the laptop, then open
                                              #   http://127.0.0.1:8770

It subscribes to `robot.state`, keeps a rolling window, and shows for every joint the command,
the measurement, and four numbers that separate the ways a gait fails:

  * mean / max tracking error   how far behind the servo is
  * lag (ms)                    cross-correlation delay between command and measurement
  * oscillation (crossings/s)   how often the joint reverses direction -- twitchiness
  * current (mA)                load: high current with a big error means it is *pushing*

The verdict column is a guess from those four, not an oracle:

    跟得住                   error small, nothing else stands out
    太软/跟不上              error large, little current -- gains, not hardware
    顶着（限位或卡死）        error large, high current -- it is straining at something
    在振荡                    frequent reversals -- too stiff, or the loop is unstable
    稳态偏（零点）            a constant offset with no oscillation

Nothing here writes to the robot. `init`, `enable` and `relax` are the three buttons, and they
send the same JSON-RPC calls `robotctl` does.
"""

import argparse
import collections
import json
import math
import os
import socket
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOCKET = "/run/robotd.sock"
DEG = 180.0 / math.pi
K = 360.0 / 4096.0
VERSION = "2026-10-20-v2"

NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]

# Filled in at startup from the robot's own calibration and from the windows read out of the
# servos. With a window here the page can say "this joint is outside its travel" instead of
# inferring it from a current threshold.
CAL = {"windows": {}, "zeros": [2048] * 15, "dirs": [1.0] * 15}
MODEL = {"bytes": None, "path": None}


def load_model(path: str) -> None:
    """Load robotctl's baked render mesh once for the browser's 3D direction view."""
    candidates = [
        path,
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "duck.bin"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "microduck_ft1910", "robotctl", "assets", "duck.bin"),
    ]
    for candidate in candidates:
        try:
            with open(candidate, "rb") as fh:
                blob = fh.read()
        except OSError:
            continue
        if not blob.startswith(b"DUCK"):
            print(f"note: {candidate} is not a duck.bin")
            continue
        MODEL["bytes"] = blob
        MODEL["path"] = os.path.abspath(candidate)
        print(f"3D model: {MODEL['path']} ({len(blob)} bytes)")
        return
    print("note: no duck.bin found; the 3D direction view will stay empty.")
    print("      copy robotctl/assets/duck.bin beside duck_dash.py.")


def load_calibration(scs_path: str, limits_path: str) -> None:
    try:
        with open(scs_path, encoding="utf-8") as fh:
            document = json.load(fh)
        CAL["zeros"] = [int(j.get("zero_ticks", 2048)) for j in document["joints"]]
        CAL["dirs"] = [float(j.get("direction", 1)) for j in document["joints"]]
    except (OSError, ValueError, KeyError, TypeError) as e:
        print(f"note: cannot read {scs_path} ({e}); assuming zero 2048 and direction +1")
    try:
        with open(limits_path, encoding="utf-8") as fh:
            windows = json.load(fh)
        CAL["windows"] = {int(sid): tuple(value) for sid, value in windows.items()}
        print(f"limits: {len(CAL['windows'])} windows from {limits_path}")
    except (OSError, ValueError, TypeError):
        print(f"note: no window file at {limits_path} — the 限位 column stays blank.")
        print("      make one with robotd stopped:")
        print("        python3 set_limits.py --port /dev/ttyACM0 --save /etc/robot/limits.json")


class Feed(threading.Thread):
    """Keeps the last few seconds of `robot.state`."""

    def __init__(self, hz: int, seconds: float, record=None):
        super().__init__(daemon=True)
        self.hz = hz
        self.frames = collections.deque(maxlen=int(hz * seconds * 1.5))
        self.lock = threading.Lock()
        self.error = None
        self.enabled_at = None
        # Frozen stops new frames going in, so the window keeps the seconds that mattered.
        self.frozen = False
        self.policy = None
        # Everything, not just the window: a file on the board survives the tab being closed,
        # the robot falling over and this process being restarted.
        self.record = record
        self.written = 0

    def _write(self, state):
        if not self.record:
            return
        safety = state.get("safety") or {}
        line = {
            "t": round(state.get("t", 0.0), 3),
            "policy": state.get("policy"),
            "joints": [round(v, 4) for v in state.get("joints") or []],
            "targets": [round(v, 4) for v in state.get("targets") or []],
            "currents_ma": [round(v, 1) for v in state.get("currents_ma") or []],
            "safety": {k: safety.get(k) for k in ("fallen", "limp", "gain")},
            "loop": state.get("loop"),
        }
        self.record.write(json.dumps(line) + "\n")
        self.written += 1
        if self.written % 50 == 0:
            self.record.flush()

    def clear(self):
        with self.lock:
            self.frames.clear()
        self.enabled_at = time.time()

    def run(self):
        while True:
            try:
                connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                connection.settimeout(10.0)
                connection.connect(SOCKET)
                connection.sendall((json.dumps({
                    "jsonrpc": "2.0", "id": 1, "method": "robot.subscribe",
                    "params": {"hz": self.hz},
                }) + "\n").encode())
                self.error = None
                buffer = b""
                while True:
                    chunk = connection.recv(65536)
                    if not chunk:
                        raise OSError("robotd closed the stream")
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            message = json.loads(line)
                        except ValueError:
                            continue
                        state = message.get("params")
                        if isinstance(state, dict) and "joints" in state:
                            self.policy = state.get("policy")
                            self._write(state)
                            if self.frozen:
                                continue
                            with self.lock:
                                self.frames.append(state)
            except OSError as e:
                self.error = str(e)
                time.sleep(1.0)


def rpc(request, timeout=8.0):
    """One request, one answer, on a connection of its own."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(SOCKET)
        connection.sendall((json.dumps(request) + "\n").encode())
        buffer = b""
        while b"\n" not in buffer:
            chunk = connection.recv(65536)
            if not chunk:
                break
            buffer += chunk
        line = buffer.split(b"\n", 1)[0]
        return json.loads(line) if line.strip() else None


# Off by default. A watchdog that fires on its own the first time somebody enables the robot is
# worse than no watchdog: it looks exactly like a broken robot. Arm it with --watch.
WATCH = {"armed": False, "note": "", "trip": 15.0, "action": "relax", "floor": 6.6}


def stop(feed: Feed, why: str):
    """Cut torque on every joint. The one action that actually stops the current draw."""
    feed.frozen = True
    try:
        rpc({"jsonrpc": "2.0", "id": 1, "method": "robot.relax"})
        WATCH["note"] = f"已停止（脱力）— {why}，窗口已冻结"
    except OSError as e:
        WATCH["note"] = f"停止失败：{e}"


def set_driving(feed: Feed, on: bool):
    """Hand the robot to the policy, or take it back and hold the pose where it is."""
    answer = rpc({"jsonrpc": "2.0", "id": 1, "method": "robot.enable",
                  "params": {"on": on, "toggle": False}})
    if on:
        feed.frozen = False
        feed.clear()
    else:
        feed.frozen = True
    return answer


def watchdog(feed: Feed):
    """Stop driving by itself when a joint is far off its command for four ticks running.

    The point is the seconds that matter. A duck that twitches and falls gives you one or two of
    them, and a human hand on a mouse is slower than that -- so the window freezes on the way out
    and the numbers you needed are still there when you look.
    """
    while True:
        time.sleep(0.1)
        if not WATCH["armed"] or feed.frozen or feed.policy in (None, "held"):
            continue
        # Never within three seconds of an enable. Whatever the numbers say in that window, the
        # robot is still standing up, and stopping it there is indistinguishable from a bug.
        if time.time() - (feed.enabled_at or 0.0) < 3.0:
            continue
        with feed.lock:
            recent = list(feed.frames)[-4:]
        if len(recent) < 4:
            continue
        worst = []
        for frame in recent:
            pairs = zip(frame.get("targets") or [], frame.get("joints") or [])
            worst.append(max((abs(t - m) for t, m in pairs), default=0.0) * DEG)
        battery = HEALTH.get("battery") or {}
        volts = battery.get("volts")
        low = volts is not None and volts < WATCH["floor"]
        # Not `fallen`: a robot held in the air, or one whose imu_mount has not been measured, is
        # routinely "fallen" by that flag, and robotd already has its own fall handling.
        if not (low or min(worst) > WATCH["trip"]):
            continue

        if low:
            why = f"电压 {volts:.2f} V 低于 {WATCH['floor']:.1f} V"
        else:
            why = f"误差 {min(worst):.0f}° 超过 {WATCH['trip']:.0f}°"
        if WATCH["action"] == "hold":
            feed.frozen = True
            try:
                rpc({"jsonrpc": "2.0", "id": 1, "method": "robot.enable",
                     "params": {"on": False, "toggle": False}})
                WATCH["note"] = f"自动暂停（{why}），关节还带电保持姿势"
            except OSError as e:
                WATCH["note"] = f"自动暂停失败：{e}"
        else:
            stop(feed, why)


def oscillate(values, span):
    if len(values) < 10 or span <= 0:
        return 0.0
    deltas = [values[i + 1] - values[i] for i in range(len(values) - 1)]
    flips = sum(1 for i in range(1, len(deltas))
                if deltas[i] * deltas[i - 1] < 0 and abs(deltas[i]) > 0.01)
    return flips / span / 2.0


def best_lag(command, measure, span, max_lag=12):
    """(lag_ms, correlation) at the shift that lines measurement up with command best."""
    n = len(command)
    if n < max_lag + 25 or span <= 0:
        return None, 0.0
    cm = sum(command) / n
    mm = sum(measure) / n
    c = [x - cm for x in command]
    m = [x - mm for x in measure]
    cden = math.sqrt(sum(x * x for x in c))
    if cden <= 0:
        return None, 0.0
    best, best_k = -2.0, 0
    for k in range(max_lag + 1):
        mden = math.sqrt(sum(m[i] * m[i] for i in range(k, n)))
        if mden <= 0:
            continue
        r = sum(c[i] * m[i + k] for i in range(n - k)) / (cden * mden)
        if r > best:
            best, best_k = r, k
    step = span / (n - 1)
    return best_k * step * 1000.0, best


def verdict(error_mean, error_max, bias, cmd_span, osc, milliamps, out_pct=0.0, over_deg=0.0):
    if error_max < 2.0:
        return "跟得住"
    if out_pct > 10.0:
        return f"撞窗口（超出 {over_deg:.0f}°，{out_pct:.0f}% 时间）"
    if osc >= 5.0:
        return "在振荡"
    if milliamps is not None and milliamps > 700 and error_mean > 3.0:
        return "顶着（限位或卡死）"
    # A command that never moved, with a constant gap, is a zero that is off — not a servo that
    # cannot keep up. Telling those apart is the whole reason cmd_span is here.
    if cmd_span < 1.0 and abs(bias) > 2.0:
        return "稳态偏（零点）"
    if milliamps is not None and milliamps < 200 and error_mean > 3.0:
        return "太软/跟不上"
    return "一般"


def summarise(feed: Feed):
    with feed.lock:
        frames = list(feed.frames)
    empty = {"ok": False, "error": feed.error, "frames": len(frames),
             "frozen": feed.frozen, "policy": feed.policy,
             "watching": WATCH["armed"], "trip": WATCH["trip"], "note": WATCH["note"]}
    # Six frames is 120 ms: enough to show last-commanded vs last-measured, which is often all a
    # robot that twitches and stops gives you. The window used to demand 25 and blanked the whole
    # table on exactly the runs worth looking at.
    if len(frames) < 6:
        return empty

    ts = [frame.get("t", 0.0) for frame in frames]
    span = ts[-1] - ts[0]
    joints = [[frame.get("joints", [])[i] * DEG for frame in frames]
              for i in range(len(NAMES))]
    targets = [[frame.get("targets", [])[i] * DEG for frame in frames]
               for i in range(len(NAMES))]
    currents = [[frame.get("currents_ma", [])[i] for frame in frames]
                for i in range(len(NAMES))]

    rows = []
    for i, name in enumerate(NAMES):
        errors = [targets[i][j] - joints[i][j] for j in range(len(frames))]
        error_mean = sum(abs(e) for e in errors) / len(errors)
        error_max = max(abs(e) for e in errors)
        bias = sum(errors) / len(errors)
        cmd_span = max(targets[i]) - min(targets[i])
        lag, correlation = best_lag(targets[i], joints[i], span)
        osc = oscillate(joints[i], span)
        # Against the servo's own travel window, in the servo's own counts.
        low, high = CAL["windows"].get(IDS[i], (None, None))
        out_pct = over_deg = 0.0
        if low is not None:
            # targets[i] is already in degrees; /K turns the angle into counts.
            counts = [CAL["zeros"][i] + (targets[i][j] / K) * CAL["dirs"][i]
                      for j in range(len(frames))]
            outside = [c for c in counts if c < low - 1 or c > high + 1]
            out_pct = 100.0 * len(outside) / len(frames)
            over_deg = max([0.0] + [max(low - c, c - high) for c in counts]) * K
        reported = [c for c in currents[i] if isinstance(c, (int, float))]
        milliamps = sum(reported) / len(reported) if reported else None
        rows.append({
            "joint": name, "id": IDS[i],
            "cmd": targets[i][-1], "meas": joints[i][-1], "error": errors[-1],
            "mean": error_mean, "max": error_max, "bias": bias,
            "span": cmd_span, "lag": lag, "correlation": correlation, "osc": osc,
            "ma": milliamps,
            "out_pct": out_pct if low is not None else None,
            "over_deg": over_deg if low is not None else None,
            "window": [low, high] if low is not None else None,
            "verdict": verdict(error_mean, error_max, bias, cmd_span, osc, milliamps,
                               out_pct, over_deg),
        })

    last = frames[-1]
    safety = last.get("safety") or {}
    loop = last.get("loop") or {}
    gravity = safety.get("gravity") or [0.0, 0.0, 0.0]
    return {
        "ok": True,
        "frames": len(frames),
        "span": span,
        "short": len(frames) < 25,
        "policy": last.get("policy"),
        "frozen": feed.frozen,
        "watching": WATCH["armed"],
        "trip": WATCH["trip"],
        "note": WATCH["note"],
        "fallen": safety.get("fallen"),
        "limp": safety.get("limp"),
        "gain": safety.get("gain"),
        "gz": gravity[2] if len(gravity) > 2 else 0.0,
        "hz": loop.get("hz"),
        "missed": loop.get("missed"),
        "rows": rows,
        "health": HEALTH,
        "min_v": HEALTH.get("min_v"),
        "record": getattr(feed.record, "name", None),
        "recorded": feed.written,
        "joints": last.get("joints") or [],
        "targets": last.get("targets") or [],
        "gravity": gravity,
    }


def series(feed: Feed, name: str, points: int = 240):
    """Command and measurement for one joint over the window, decimated for a canvas."""
    with feed.lock:
        frames = list(feed.frames)
    if name not in NAMES or len(frames) < 2:
        return None
    index = NAMES.index(name)
    step = max(1, len(frames) // points)
    picked = frames[::step]
    t0 = picked[0].get("t", 0.0)
    result = {
        "joint": name,
        "t": [round(frame.get("t", 0.0) - t0, 3) for frame in picked],
        "cmd": [round((frame.get("targets") or [0.0] * 15)[index] * DEG, 2) for frame in picked],
        "meas": [round((frame.get("joints") or [0.0] * 15)[index] * DEG, 2) for frame in picked],
    }
    low, high = CAL["windows"].get(IDS[index], (None, None))
    if low is None:
        result["low_deg"] = result["high_deg"] = None
    else:
        zero, direction = CAL["zeros"][index], CAL["dirs"][index]
        a = (low - zero) * K * direction
        b = (high - zero) * K * direction
        result["low_deg"], result["high_deg"] = round(min(a, b), 2), round(max(a, b), 2)
    return result


def all_series(feed: Feed, points: int = 120):
    """Every joint's command and measurement in one response, for the fifteen-panel view."""
    with feed.lock:
        frames = list(feed.frames)
    if len(frames) < 2:
        return None
    step = max(1, len(frames) // points)
    picked = frames[::step]
    t0 = picked[0].get("t", 0.0)
    result = {"t": [round(frame.get("t", 0.0) - t0, 2) for frame in picked], "joints": []}
    for index, name in enumerate(NAMES):
        item = {
            "joint": name,
            "cmd": [round((frame.get("targets") or [0.0] * 15)[index] * DEG, 1) for frame in picked],
            "meas": [round((frame.get("joints") or [0.0] * 15)[index] * DEG, 1) for frame in picked],
        }
        low, high = CAL["windows"].get(IDS[index], (None, None))
        if low is None:
            item["low_deg"] = item["high_deg"] = None
        else:
            zero, direction = CAL["zeros"][index], CAL["dirs"][index]
            a = (low - zero) * K * direction
            b = (high - zero) * K * direction
            item["low_deg"], item["high_deg"] = round(min(a, b), 1), round(max(a, b), 1)
        result["joints"].append(item)
    return result


HEALTH = {"battery": None, "motors": None}


def poll_health():
    while True:
        try:
            answer = rpc({"jsonrpc": "2.0", "id": 1, "method": "robot.health"}, timeout=4.0)
            result = (answer or {}).get("result") or {}
            HEALTH["battery"] = result.get("battery")
            HEALTH["motors"] = result.get("motors")
            volts = (result.get("battery") or {}).get("volts")
            if isinstance(volts, (int, float)):
                lowest = HEALTH.get("min_v")
                HEALTH["min_v"] = volts if lowest is None else min(lowest, volts)
        except (OSError, ValueError):
            pass
        time.sleep(2.0)


PAGE = """<!doctype html>
<meta charset="utf-8">
<title>duck servo check</title>
<style>
  :root { color-scheme: dark; }
  body { font: 13px/1.4 ui-monospace, SFMono-Regular, Menlo, monospace;
         background: #11151c; color: #d7dde6; margin: 0; padding: 16px 20px; }
  h1 { font-size: 15px; font-weight: 600; margin: 0 0 10px; letter-spacing: .04em; }
  .bar { display: flex; gap: 18px; flex-wrap: wrap; margin-bottom: 12px; color: #9fb0c4; }
  .bar b { color: #e6edf6; font-weight: 600; }
  button { font: inherit; padding: 6px 14px; border-radius: 6px; border: 1px solid #33415a;
           background: #1b2330; color: #d7dde6; cursor: pointer; }
  button:hover { background: #243046; }
  button.hot { border-color: #6b3a3a; background: #2a1a1e; }
  button.stop { border-color: #a33; background: #3a1418; color: #ffd9d9; font-weight: 600; }
  button.stop:hover { background: #51191f; }
  button.go { border-color: #2f6b45; background: #16281d; color: #c9f5d8; }
  button.go:active { background: #1d3a27; }
  #msg { height: 18px; margin: 8px 0 12px; color: #8fd68f; }
  #msg.bad { color: #ff8a8a; }
  table { border-collapse: collapse; width: 100%; }
  th, td { padding: 3px 8px; text-align: right; white-space: nowrap; }
  th { color: #7f8ea3; font-weight: 500; border-bottom: 1px solid #26303f; }
  td:first-child, th:first-child { text-align: left; }
  tr.bad td { color: #ffb4a2; }
  tr.warn td { color: #ffd479; }
  .v { text-align: left; color: #9fb0c4; }
  canvas { vertical-align: middle; }
  #dirbox { display:grid; grid-template-columns:minmax(0, 1.5fr) minmax(300px, .75fr);
            align-items:start; gap:18px; margin:16px 0; padding:12px;
            border:1px solid #26303f; border-radius:8px; background:#0d1117; }
  #dirbox h2 { font-size:13px; margin:0 0 7px; color:#e6edf6; }
  #dirbox p { margin:5px 0; color:#8f9eb2; }
  #dirbox .big { font-size:18px; color:#c9f5d8; font-weight:600; }
  #jointpick { display:flex; flex-wrap:wrap; gap:5px; margin:8px 0; }
  #jointpick button { padding:4px 7px; font-size:11px; }
  #jointpick button.on { border-color:#d58a35; background:#33220f; color:#ffc36b; }
  #views { display:grid; grid-template-columns:1fr 1fr; align-items:start; gap:8px; }
  #views canvas { display:block; width:100%; height:auto; aspect-ratio:560 / 640;
                  background:#11151c; border-radius:6px; border:1px solid #1f2836;
                  touch-action:none; }
  #home3d { display:block; width:100%; height:auto; aspect-ratio:440 / 500;
            background:#11151c; border-radius:6px; border:1px solid #1f2836;
            margin:8px auto 0; }
  #dirread { font-size:12px; color:#9fb0c4; line-height:1.8; }
  #dirread b { color:#e6edf6; }
  .legend { color:#8f9eb2; font-size:11px; }
  @media (max-width: 1000px) {
    #dirbox { grid-template-columns:1fr; }
    #home3d { max-width:440px; }
  }
  @media (max-width: 620px) {
    body { padding:10px; }
    #views { grid-template-columns:1fr; }
  }
</style>
<h1>duck servo check <span style="color:#5d6b7d;font-weight:400">__VERSION__</span></h1>
<div class="bar" id="bar">connecting…</div>
<div>
  <button data-a="init">init（到 home）</button>
  <button data-a="enable" class="hot">enable（跑策略）</button>
  <button data-a="toggle" id="pause" class="hot">暂停（空格）</button>
  <button data-a="stop" class="stop">停止 / 脱力（Esc）</button>
  <button data-a="watchdog" id="watch">自动暂停：开</button>
  <button data-a="relax">relax（放手）</button>
  <button id="walk" class="go">按住前进</button>
  <input id="vx" type="number" value="0.08" step="0.02" min="-0.3" max="0.3"
         style="width:64px;background:#1b2330;color:#d7dde6;border:1px solid #33415a;border-radius:5px;padding:5px">
  <span style="color:#7f8ea3">m/s</span>
  <a href="/api/dump" download="duck-state.jsonl" style="margin-left:14px;color:#8fb4d8">下载冻结数据</a>
  <a href="/api/file" download style="margin-left:14px;color:#8fb4d8">下载全部记录</a>
</div>
<div id="msg"></div>
<section id="dirbox">
  <div>
    <h2>3D joint direction check</h2>
    <p>Support the duck, then press <b>relax</b>. Choose one joint and move that joint by hand.
       Left is the live measured pose; right is the same pose with that joint at <b>+15°</b>.
       The orange arrow is the model's positive direction.</p>
    <div id="jointpick"></div>
    <div id="views">
      <canvas id="live3d" width="560" height="640"></canvas>
      <canvas id="preview3d" width="560" height="640"></canvas>
    </div>
    <div class="legend">Drag either view to orbit. Root is locked upright here; use
       <b>robotctl monitor</b> when you want the IMU to tilt the whole body.</div>
  </div>
  <div>
    <h2>home reference / home 参考</h2>
    <canvas id="home3d" width="440" height="500"></canvas>
    <h2 id="dirname" style="margin-top:12px">select a joint</h2>
    <div id="dirread">Waiting for robot state.</div>
    <p><button id="rebase">retake baseline / 重新取基准</button></p>
    <p class="big" id="dirhint">+ direction unknown</p>
  </div>
</section>
<table>
  <thead><tr>
    <th>joint</th><th>id</th><th>cmd°</th><th>meas°</th><th>err°</th>
    <th>mean|err|</th><th>max</th><th>lag ms</th><th>r</th><th>osc/s</th>
    <th>mA</th><th>限位</th><th>err bar</th><th>判定</th>
  </tr></thead>
  <tbody id="rows"></tbody>
</table>
<div style="margin-top:16px;color:#7f8ea3">
  蓝 = 指令，绿 = 实测，红虚线 = 舵机自己的行程窗口；每格纵轴各自缩放，左上/左下是这一格的最大/最小值
</div>
<div id="grid" style="margin-top:8px;display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px"></div>
<script>
const $ = (id) => document.getElementById(id);
let busy = false;
let tickCount = 0;

const NAMES = ["left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle",
               "neck_pitch","head_pitch","head_yaw","head_roll","mouth",
               "right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle"];
// The mouth is excluded: it has its own smaller action and never takes part in a gait, so it only
// takes a cell away from the fourteen that do.
const SHOWN = NAMES.map((name, index) => index).filter((index) => NAMES[index] !== 'mouth');
const PANELS = SHOWN.map((index) => {
  const name = NAMES[index];
  const box = document.createElement('div');
  box.style.cssText = 'background:#0d1117;border:1px solid #26303f;border-radius:6px;padding:5px 8px 2px';
  const head = document.createElement('div');
  head.style.cssText = 'font-size:11px;color:#7f8ea3;height:15px';
  head.textContent = name;
  const canvas = document.createElement('canvas');
  canvas.width = 640; canvas.height = 168;
  canvas.style.cssText = 'width:100%;height:92px;display:block';
  box.append(head, canvas);
  $('grid').append(box);
  return {head, canvas, ctx: canvas.getContext('2d'), name, index};
});
const PANEL_OF = {};
PANELS.forEach((panel) => { PANEL_OF[panel.index] = panel; });

// robotctl's baked sim mesh. The same geometry the trained policy sees, parsed here so the
// browser can answer a question the numbers cannot: which way should this joint move?
let DUCK = null;
let liveJoints = null;
let liveGravity = [0, 0, 0];
let selectedJoint = 1;
let viewYaw = -1.45, viewPitch = 0.22;
let dirBaseline = 0;
const HOME_DEG = [0,-5,-26.24,-0.28,25.96,20,20,0,0,0,0,5,26.24,0.28,-25.96];
const HOME_RAD = HOME_DEG.map((value)=>value*Math.PI/180);
// Joint-direction checking cares about the measured joint angles, not the trunk attitude.
// Locking the root upright prevents an IMU mount/sign mismatch from turning the whole model over.
const MODEL_UP = [0,0,-1];

function parseDuck(buffer) {
  const d = new DataView(buffer);
  let at = 0;
  const take = (n) => {
    if (at + n > d.byteLength) throw new Error('duck.bin truncated');
    const out = new Uint8Array(buffer, at, n); at += n; return out;
  };
  const u16 = () => { const v = d.getUint16(at, true); at += 2; return v; };
  const i16 = () => { const v = d.getInt16(at, true); at += 2; return v; };
  const u32 = () => { const v = d.getUint32(at, true); at += 4; return v; };
  const f32 = () => { const v = d.getFloat32(at, true); at += 4; return v; };
  const vec = () => [f32(), f32(), f32()];
  const quat = () => [f32(), f32(), f32(), f32()];
  const magic = Array.from(take(4)).map((x) => String.fromCharCode(x)).join('');
  if (magic !== 'DUCK' || u32() !== 1) throw new Error('duck.bin format mismatch');
  const nm = u16(), nb = u16(), np = u16(), meshes = [], bodies = [], parts = [];
  for (let i = 0; i < nm; i++) {
    const nv = u16(), nt = u16(), verts = [];
    for (let v = 0; v < nv; v++) verts.push(vec());
    const tris = [];
    for (let t = 0; t < nt; t++) tris.push([u16(), u16(), u16()]);
    meshes.push({verts, tris});
  }
  for (let i = 0; i < nb; i++)
    bodies.push({parent: i16(), joint: i16(), pos: vec(), quat: quat(), axis: vec()});
  for (let i = 0; i < np; i++) {
    const body = u16(), mesh = u16(), rgb = Array.from(take(4));
    parts.push({body, mesh, rgb, pos: vec(), quat: quat()});
  }
  return {meshes, bodies, parts};
}

const ident = () => ({r: [[1,0,0],[0,1,0],[0,0,1]], t: [0,0,0]});
function qpose(q, t) {
  const n = Math.hypot(...q) || 1, w=q[0]/n, x=q[1]/n, y=q[2]/n, z=q[3]/n;
  return {r: [
    [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
    [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
    [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]], t};
}
function apose(axis, angle) {
  const n = Math.hypot(...axis) || 1, x=axis[0]/n, y=axis[1]/n, z=axis[2]/n;
  const c=Math.cos(angle), s=Math.sin(angle), i=1-c;
  return {r: [
    [c+x*x*i, x*y*i-z*s, x*z*i+y*s],
    [y*x*i+z*s, c+y*y*i, y*z*i-x*s],
    [z*x*i-y*s, z*y*i+x*s, c+z*z*i]], t:[0,0,0]};
}
function mvec(m, v) {
  return [m[0][0]*v[0]+m[0][1]*v[1]+m[0][2]*v[2],
          m[1][0]*v[0]+m[1][1]*v[1]+m[1][2]*v[2],
          m[2][0]*v[0]+m[2][1]*v[1]+m[2][2]*v[2]];
}
function mmul(a, b) {
  const r = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i=0;i<3;i++) for (let j=0;j<3;j++)
    r[i][j] = a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j];
  return r;
}
function then(a, b) {
  const t = mvec(a.r, b.t);
  return {r: mmul(a.r,b.r), t:[t[0]+a.t[0],t[1]+a.t[1],t[2]+a.t[2]]};
}
function apply(p,v) {
  const q=mvec(p.r,v); return [q[0]+p.t[0],q[1]+p.t[1],q[2]+p.t[2]];
}
function cross(a,b) { return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]; }
function normv(a) { const n=Math.hypot(...a)||1; return [a[0]/n,a[1]/n,a[2]/n]; }
function attitude(g) {
  const v=normv(g), down=[0,0,-1], axis=cross(v,down), s=Math.hypot(...axis), c=v[2];
  if (s < 1e-6) return c > 0 ? ident() : apose([1,0,0],Math.PI);
  return apose([axis[0]/s,axis[1]/s,axis[2]/s],Math.atan2(s,c));
}
function fk(angles, gravity) {
  const poses=[];
  for (const body of DUCK.bodies) {
    let pose = body.parent < 0 ? attitude(gravity) : poses[body.parent];
    pose = then(pose, qpose(body.quat, body.pos));
    if (body.joint >= 0) pose = then(pose, apose(body.axis, angles[body.joint] || 0));
    poses.push(pose);
  }
  return poses;
}
function descendant(bodyIndex, poses, origin) {
  let best=null, bestLength=-1;
  for (let i=0;i<DUCK.bodies.length;i++) {
    let p=DUCK.bodies[i].parent;
    while (p >= 0) {
      if (p === bodyIndex) {
        const q=poses[i].t, length=Math.hypot(q[0]-origin[0],q[1]-origin[1],q[2]-origin[2]);
        if (length > bestLength) { best=poses[i]; bestLength=length; }
        break;
      }
      p = DUCK.bodies[p].parent;
    }
  }
  return best;
}
function arrowFor(jointIndex, poses) {
  if (jointIndex < 0) return null;
  const bi = DUCK.bodies.findIndex((b) => b.joint === jointIndex);
  if (bi < 0) return null;
  const origin = poses[bi].t;
  const axis = normv(mvec(poses[bi].r, DUCK.bodies[bi].axis));
  const child = descendant(bi, poses, origin);
  if (!child) return {points:[origin,[origin[0]+axis[0]*0.026,
                                      origin[1]+axis[1]*0.026,
                                      origin[2]+axis[2]*0.026]], kind:'axis'};
  const radius = [child.t[0]-origin[0], child.t[1]-origin[1], child.t[2]-origin[2]];
  const tangent = cross(axis, radius), length = Math.hypot(...tangent);
  if (length < 1e-5) return {points:[origin,[origin[0]+axis[0]*0.026,
                                         origin[1]+axis[1]*0.026,
                                         origin[2]+axis[2]*0.026]], kind:'axis'};
  return {points:[origin, [origin[0]+tangent[0]/length*0.026,
                           origin[1]+tangent[1]/length*0.026,
                           origin[2]+tangent[2]/length*0.026]], kind:'move'};
}
function project(vertices, w, h, yaw = viewYaw, pitch = viewPitch) {
  let floor=Infinity;
  for (const v of vertices) floor=Math.min(floor,v[2]);
  const look=[0,0,0.13], ce=Math.cos(pitch), se=Math.sin(pitch);
  const ca=Math.cos(yaw), sa=Math.sin(yaw);
  const fwd=[-ce*ca,-ce*sa,-se], right=normv(cross(fwd,[0,0,1]));
  const up=cross(right,fwd), window=0.36, scale=Math.min(w,h)/window;
  return (v) => {
    const p=[v[0],v[1],v[2]-floor];
    return [w/2 + (p[0]*right[0]+p[1]*right[1]+p[2]*right[2])*scale,
            h/2 + ((p[0]*up[0]+p[1]*up[1]+p[2]*up[2]) -
                   (look[0]*up[0]+look[1]*up[1]+look[2]*up[2]))*scale,
            p[0]*fwd[0]+p[1]*fwd[1]+p[2]*fwd[2]];
  };
}
function drawDuck(canvas, angles, gravity, selected, ghost, label, cameraYaw, cameraPitch) {
  if (!DUCK) return;
  const ctx=canvas.getContext('2d'), w=canvas.width, h=canvas.height;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle='#11151c'; ctx.fillRect(0,0,w,h);
  const poses=fk(angles,gravity), vertices=[], triangles=[];
  const selectedBody=selected >= 0 ? DUCK.bodies.findIndex((body)=>body.joint===selected) : -1;
  const light=normv([0.35,0.45,-0.82]);
  for (const part of DUCK.parts) {
    const pose=then(poses[part.body],qpose(part.quat,part.pos));
    const mesh=DUCK.meshes[part.mesh];
    const start=vertices.length;
    for (const v of mesh.verts) vertices.push(apply(pose,v));
    for (const tri of mesh.tris) {
      const a=vertices[start+tri[0]], b=vertices[start+tri[1]], c=vertices[start+tri[2]];
      const n=normv(cross([b[0]-a[0],b[1]-a[1],b[2]-a[2]],
                          [c[0]-a[0],c[1]-a[1],c[2]-a[2]]));
      const shade=0.32+0.68*Math.abs(n[0]*light[0]+n[1]*light[1]+n[2]*light[2]);
      const rgb=ghost ? [244,144,48] : (part.body===selectedBody ? [62,190,220] : part.rgb);
      triangles.push({a,b,c,depth:(a[0]+b[0]+c[0]+a[1]+b[1]+c[1]+a[2]+b[2]+c[2])/3,
        colour:`rgb(${rgb[0]*shade|0},${rgb[1]*shade|0},${rgb[2]*shade|0})`});
    }
  }
  const arrow=arrowFor(selected,poses);
  if (arrow) vertices.push(...arrow.points);
  const P=project(vertices,w,h,cameraYaw,cameraPitch);
  triangles.sort((a,b)=>b.depth-a.depth);
  for (const t of triangles) {
    const a=P(t.a), b=P(t.b), c=P(t.c);
    ctx.fillStyle=t.colour; ctx.beginPath(); ctx.moveTo(a[0],a[1]);
    ctx.lineTo(b[0],b[1]); ctx.lineTo(c[0],c[1]); ctx.closePath(); ctx.fill();
  }
  if (arrow) {
    const a=P(arrow.points[0]), b=P(arrow.points[1]);
    const dx=b[0]-a[0], dy=b[1]-a[1], len=Math.hypot(dx,dy)||1;
    const ux=dx/len, uy=dy/len;
    ctx.strokeStyle='#ffad3d'; ctx.fillStyle='#ffad3d'; ctx.lineWidth=3;
    ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(b[0],b[1]);
    ctx.lineTo(b[0]-ux*13-uy*6,b[1]-uy*13+ux*6);
    ctx.lineTo(b[0]-ux*13+uy*6,b[1]-uy*13-ux*6); ctx.closePath(); ctx.fill();
    ctx.fillStyle='#ffc36b'; ctx.font='14px ui-monospace';
    ctx.fillText(arrow.kind==='axis'?'twist axis':'+ direction',b[0]+7,b[1]-7);
  }
  ctx.fillStyle=ghost?'#ffb15c':'#8fd68f'; ctx.font='13px ui-monospace';
  ctx.fillText(label || (ghost?'expected: +15 deg':'live measured'),14,22);
}
function render3d() {
  if (!DUCK || !liveJoints || liveJoints.length !== 15) return;
  const preview=liveJoints.slice(); preview[selectedJoint] += 15*Math.PI/180;
  drawDuck($('live3d'),liveJoints,MODEL_UP,selectedJoint,false);
  drawDuck($('preview3d'),preview,MODEL_UP,selectedJoint,true);
  drawDuck($('home3d'),HOME_RAD,MODEL_UP,-1,false,'home reference (upright)',-0.7,0.32);
  const deg=liveJoints[selectedJoint]*180/Math.PI, delta=deg-dirBaseline;
  $('dirread').innerHTML =
    'selected <b>'+NAMES[selectedJoint]+'</b> · id <b>'+[20,21,22,23,24,30,31,32,33,34,10,11,12,13,14][selectedJoint]+'</b><br>'+
    'measured <b>'+deg.toFixed(1)+'°</b> · since baseline <b>'+(delta>=0?'+':'')+delta.toFixed(1)+'°</b>';
  $('dirhint').textContent = Math.abs(delta)<0.5 ? '+ direction: move it by hand' :
    (delta>0 ? 'you moved toward the orange arrow (+ model direction)' :
               'you moved opposite the orange arrow (- model direction)');
}

const JOINT_PICK=$('jointpick');
$('rebase').onclick=()=>{
  dirBaseline=liveJoints?liveJoints[selectedJoint]*180/Math.PI:0; render3d();
};
NAMES.forEach((name,index)=>{
  const b=document.createElement('button'); b.textContent=name.replace('_hip',' hip').replace('_',' ');
  b.onclick=()=>{
    selectedJoint=index; dirBaseline=liveJoints?liveJoints[index]*180/Math.PI:0;
    for (const x of JOINT_PICK.children) x.classList.toggle('on',Number(x.dataset.i)===index);
    $('dirname').textContent=name; render3d();
  };
  b.dataset.i=String(index); if(index===selectedJoint)b.classList.add('on'); JOINT_PICK.append(b);
});
$('dirname').textContent=NAMES[selectedJoint];

for (const canvas of [$('live3d'),$('preview3d')]) {
  let drag=null;
  canvas.addEventListener('pointerdown',(e)=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener('pointermove',(e)=>{
    if(!drag)return; viewYaw += (e.clientX-drag[0])*0.012;
    viewPitch = Math.max(-0.1,Math.min(1.15,viewPitch+(e.clientY-drag[1])*0.008));
    drag=[e.clientX,e.clientY]; render3d();
  });
  for(const n of ['pointerup','pointercancel']) canvas.addEventListener(n,()=>{drag=null;});
}

fetch('/api/model').then((r)=>r.ok?r.arrayBuffer():Promise.reject(new Error('duck.bin missing')))
  .then((buf)=>{
    DUCK=parseDuck(buf);
    drawDuck($('home3d'),HOME_RAD,MODEL_UP,-1,false,'home reference (upright)',-0.7,0.32);
    render3d();
  })
  .catch((e)=>{$('dirhint').textContent='3D unavailable: '+e.message;});

function drawPanel(panel, item) {
  const ctx = panel.ctx, w = panel.canvas.width, h = panel.canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!item || !item.cmd || item.cmd.length < 2) return;
  let lo = Infinity, hi = -Infinity;
  for (const v of item.cmd.concat(item.meas)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (item.low_deg !== null && item.low_deg !== undefined) {
    lo = Math.min(lo, item.low_deg);
    hi = Math.max(hi, item.high_deg);
  }
  const pad = Math.max(1.5, (hi - lo) * 0.08);
  lo -= pad; hi += pad;
  const X = (i) => 4 + (w - 8) * i / (item.cmd.length - 1);
  const Y = (v) => 8 + (h - 16) * (1 - (v - lo) / (hi - lo));
  const line = (values, colour, width) => {
    ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.beginPath();
    values.forEach((v, i) => { if (i) ctx.lineTo(X(i), Y(v)); else ctx.moveTo(X(i), Y(v)); });
    ctx.stroke();
  };
  if (item.low_deg !== null && item.low_deg !== undefined) {
    ctx.strokeStyle = '#8c3a3a'; ctx.lineWidth = 1; ctx.setLineDash([5, 4]);
    for (const edge of [item.low_deg, item.high_deg]) {
      ctx.beginPath(); ctx.moveTo(4, Y(edge)); ctx.lineTo(w - 4, Y(edge)); ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  line(item.cmd, '#5b9cff', 1.4);
  line(item.meas, '#59c27c', 1.8);
  ctx.fillStyle = '#4c5a6b'; ctx.font = '11px ui-monospace';
  ctx.fillText(hi.toFixed(0), 6, 18);
  ctx.fillText(lo.toFixed(0), 6, h - 8);
}

function drawSeries(data) {
  const canvas = $('plot');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!data || !data.t || data.t.length < 2) return;
  let lo = Infinity, hi = -Infinity;
  for (const v of data.cmd.concat(data.meas)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (data.low_deg !== null && data.low_deg !== undefined) {
    lo = Math.min(lo, data.low_deg);
    hi = Math.max(hi, data.high_deg);
  }
  const pad = Math.max(1, (hi - lo) * 0.12);
  lo -= pad; hi += pad;
  const X = (i) => 46 + (w - 58) * i / (data.t.length - 1);
  const Y = (v) => 12 + (h - 34) * (1 - (v - lo) / (hi - lo));
  ctx.strokeStyle = '#1f2836'; ctx.lineWidth = 1;
  for (let k = 0; k <= 4; k++) {
    const y = 12 + (h - 34) * k / 4;
    ctx.beginPath(); ctx.moveTo(46, y); ctx.lineTo(w - 12, y); ctx.stroke();
  }
  ctx.fillStyle = '#5d6b7d'; ctx.font = '11px ui-monospace';
  ctx.fillText(hi.toFixed(1), 4, 16);
  ctx.fillText(lo.toFixed(1), 4, h - 20);
  ctx.fillText('t=' + data.t[data.t.length - 1].toFixed(1) + 's', w - 74, h - 6);
  const line = (values, colour) => {
    ctx.strokeStyle = colour; ctx.lineWidth = 1.6; ctx.beginPath();
    values.forEach((v, i) => { if (i) ctx.lineTo(X(i), Y(v)); else ctx.moveTo(X(i), Y(v)); });
    ctx.stroke();
  };
  line(data.cmd, '#6ea8fe');
  line(data.meas, '#8fd68f');
  if (data.low_deg !== null && data.low_deg !== undefined) {
    ctx.strokeStyle = '#a04040'; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    for (const edge of [data.low_deg, data.high_deg]) {
      ctx.beginPath(); ctx.moveTo(46, Y(edge)); ctx.lineTo(w - 12, Y(edge)); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.fillStyle = '#a04040'; ctx.fillText('window', w - 58, Y(data.high_deg) - 4);
  }
}

for (const button of document.querySelectorAll('button[data-a]')) {
  button.onclick = async () => {
    if (busy) return;
    busy = true;
    $('msg').className = '';
    $('msg').textContent = button.dataset.a + ' …';
    try {
      const response = await fetch('/api/' + button.dataset.a, {method: 'POST'});
      const body = await response.json();
      const text = JSON.stringify(body.result ?? body.error ?? body);
      $('msg').textContent = button.dataset.a + ' → ' + text;
      $('msg').className = body.ok === false ? 'bad' : '';
    } catch (e) {
      $('msg').textContent = String(e);
      $('msg').className = 'bad';
    }
    busy = false;
  };
}

function fmt(value, digits = 1, signed = false) {
  if (value === null || value === undefined || Number.isNaN(value)) return '–';
  const text = Number(value).toFixed(digits);
  return signed && value >= 0 ? '+' + text : text;
}

document.addEventListener('keydown', (event) => {
  if (event.code === 'Space') { event.preventDefault(); $('pause').click(); }
  if (event.code === 'Escape') { event.preventDefault(); document.querySelector('button[data-a="stop"]').click(); }
});

// Hold to walk. The daemon's walk channel is a deadman: it zeroes the velocity 500 ms after the
// last instruction, so this repeats while the button is down and sends an explicit zero on release.
let walking = null;
function walkSend(vx) {
  fetch('/api/move?vx=' + encodeURIComponent(vx) + '&vyaw=0', {method: 'POST'}).catch(() => {});
}
function stopWalking() {
  if (!walking) return;
  clearInterval(walking);
  walking = null;
  walkSend(0);
}
const walkButton = $('walk');
walkButton.addEventListener('pointerdown', (event) => {
  event.preventDefault();
  if (walking) return;
  const vx = Number($('vx').value) || 0;
  walkSend(vx);
  walking = setInterval(() => walkSend(vx), 120);
});
for (const name of ['pointerup', 'pointerleave', 'pointercancel']) {
  walkButton.addEventListener(name, stopWalking);
}
window.addEventListener('blur', stopWalking);

function bar(value, limit) {
  const width = Math.min(60, Math.abs(value) / limit * 60);
  const colour = Math.abs(value) > limit ? '#ff7b72' : (Math.abs(value) > limit / 3 ? '#ffd479' : '#4f9d69');
  return '<svg width="62" height="10"><rect width="62" height="10" rx="2" fill="#1b2330"/>' +
         '<rect width="' + width + '" height="10" rx="2" fill="' + colour + '"/></svg>';
}

async function tick() {
  let data;
  try {
    data = await (await fetch('/api/data')).json();
  } catch (e) {
    $('bar').textContent = 'no connection to the dashboard server';
    return;
  }
  if (!data.ok) {
    $('bar').textContent = 'waiting for robotd … ' + (data.error || (data.frames + ' frames'));
    $('rows').innerHTML = '';
    return;
  }
  const battery = data.health?.battery;
  const motors = data.health?.motors;
  const state = data.frozen
    ? '<b style="color:#ffd479">已冻结</b> ' + (data.note || '窗口停在这一刻')
    : (data.limp
        ? '<b style="color:#ffd479">robotd 已放软（limp）</b>'
        : '<b style="color:#8fd68f">运行中</b>');
  $('pause').textContent = data.frozen ? '继续（空格）' : '暂停（空格）';
  $('watch').textContent = '自动暂停：' + (data.watching ? '开' : '关');
  $('bar').innerHTML =
    state + ' · ' +
    'policy <b>' + (data.policy ?? '–') + '</b>' +
    ' · fall <b>' + (data.fallen ?? '–') + '</b>' +
    ' · limp <b>' + (data.limp ?? '–') + '</b>' +
    ' · gain <b>' + (data.gain ?? '–') + '</b>' +
    ' · loop <b>' + fmt(data.hz, 1) + ' Hz</b> / missed <b>' + (data.missed ?? '–') + '</b>' +
    ' · gz <b>' + fmt(data.gz, 2, true) + '</b>' +
    ' · pack <b>' + (battery ? fmt(battery.volts, 2) + ' V ' + fmt(battery.percent, 0) + '%' : '–') + '</b>' +
    (data.min_v ? ' (min ' + fmt(data.min_v, 2) + ' V)' : '') +
    ' · ' + (motors ? fmt(motors.max_c, 0) + ' °C' : '–');
  $('bar').innerHTML += ' · rec <b>' +
    (data.record ? String(data.record).split('/').pop() : '–') + '</b> ' +
    (data.recorded ?? 0) + ' 帧';

  if (data.short) {
    $('bar').innerHTML += ' · <span style="color:#ffd479">只有 ' + data.frames +
      ' 帧（' + fmt(data.span, 2) + ' 秒），lag/osc 仅供参考</span>';
  }

  $('rows').innerHTML = data.rows.map((row) => {
    const level = row.max > 6 ? 'bad' : (row.max > 3 ? 'warn' : '');
    return '<tr class="' + level + '">' +
      '<td>' + row.joint + '</td><td>' + row.id + '</td>' +
      '<td>' + fmt(row.cmd, 1, true) + '</td>' +
      '<td>' + fmt(row.meas, 1, true) + '</td>' +
      '<td>' + fmt(row.error, 1, true) + '</td>' +
      '<td>' + fmt(row.mean, 1) + '</td>' +
      '<td>' + fmt(row.max, 1) + '</td>' +
      '<td>' + fmt(row.lag, 0) + '</td>' +
      '<td>' + fmt(row.correlation, 2) + '</td>' +
      '<td>' + fmt(row.osc, 1) + '</td>' +
      '<td>' + fmt(row.ma, 0) + '</td>' +
      '<td>' + (row.out_pct === null ? '–' :
                 fmt(row.out_pct, 0) + '% / ' + fmt(row.over_deg, 0) + '°') + '</td>' +
      '<td>' + bar(row.mean, 6) + '</td>' +
      '<td class="v">' + row.verdict + '</td>' +
    '</tr>';
  }).join('');

  if (Array.isArray(data.joints) && data.joints.length === 15) {
    if (!liveJoints) dirBaseline = data.joints[selectedJoint] * 180 / Math.PI;
    liveJoints = data.joints;
    liveGravity = data.gravity || [0, 0, 0];
    if (tickCount % 2 === 0) render3d();
  }

  tickCount += 1;
  if (tickCount % 2 === 0) {
    try {
      const all = await (await fetch('/api/series/all')).json();
      if (all && all.joints) {
        all.joints.forEach((item, i) => {
          const panel = PANEL_OF[i];
          if (!panel) return;
          drawPanel(panel, item);
          const worst = Math.max(...item.cmd.map((v, k) => Math.abs(v - item.meas[k])));
          panel.head.textContent = item.joint + '   max|e| ' + worst.toFixed(0) + '°';
          panel.head.style.color = worst > 15 ? '#ff9d8a' : (worst > 6 ? '#ffd479' : '#7f8ea3');
        });
      }
    } catch (e) { /* the table is the important half */ }
  }
}

tick();
setInterval(tick, 250);
</script>
"""


class Handler(BaseHTTPRequestHandler):
    feed = None

    def log_message(self, *_args):
        pass

    def send(self, code, body, kind="application/json"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send(200, PAGE.replace("__VERSION__", VERSION), "text/html; charset=utf-8")
        elif self.path == "/api/data":
            self.send(200, json.dumps(summarise(self.feed)))
        elif self.path.startswith("/api/series"):
            query = dict(urllib.parse.parse_qsl(urllib.parse.urlparse(self.path).query))
            if self.path.split("?", 1)[0] == "/api/series/all":
                self.send(200, json.dumps(all_series(self.feed)))
                return
            self.send(200, json.dumps(series(self.feed, query.get("joint", NAMES[0]))))
        elif self.path == "/api/file":
            path = getattr(self.feed.record, "name", None)
            if not path or not os.path.exists(path):
                self.send(404, json.dumps({"error": "nothing recorded yet"}))
                return
            with open(path, "rb") as fh:
                self.send(200, fh.read(), "application/x-ndjson")
        elif self.path == "/api/dump":
            with self.feed.lock:
                frames = list(self.feed.frames)
            self.send(200, "\n".join(json.dumps(frame) for frame in frames),
                      "application/x-ndjson")
        elif self.path == "/api/model":
            if MODEL["bytes"] is None:
                self.send(404, json.dumps({"error": "duck.bin is missing"}))
            else:
                self.send(200, MODEL["bytes"], "application/octet-stream")
        else:
            self.send(404, json.dumps({"error": "no such path"}))

    def do_POST(self):
        action = self.path.split("?", 1)[0].rsplit("/", 1)[-1]
        if action == "init":
            call = {"jsonrpc": "2.0", "id": 1, "method": "robot.init"}
        elif action == "enable":
            WATCH["note"] = ""
            try:
                answer = set_driving(self.feed, True) or {}
                self.send(200, json.dumps({"ok": "error" not in answer, **answer}))
            except OSError as e:
                self.send(200, json.dumps({"ok": False, "error": str(e)}))
            return
        elif action == "toggle":
            running = self.feed.policy not in (None, "held")
            WATCH["note"] = ""
            try:
                answer = set_driving(self.feed, not running) or {}
                self.send(200, json.dumps({"ok": "error" not in answer, **answer}))
            except OSError as e:
                self.send(200, json.dumps({"ok": False, "error": str(e)}))
            return
        elif action == "watchdog":
            WATCH["armed"] = not WATCH["armed"]
            self.send(200, json.dumps({"ok": True, "armed": WATCH["armed"]}))
            return
        elif action == "stop":
            stop(self.feed, "手动")
            self.send(200, json.dumps({"ok": True, "note": WATCH["note"]}))
            return
        elif action == "move":
            # The walk channel is a deadman: the daemon zeroes the velocity 500 ms after the last
            # one, so the page repeats this while the button is held and sends a zero on release.
            query = dict(urllib.parse.parse_qsl(urllib.parse.urlparse(self.path).query))
            try:
                vx = float(query.get("vx", 0.0))
                vyaw = float(query.get("vyaw", 0.0))
            except ValueError:
                self.send(200, json.dumps({"ok": False, "error": "bad vx/vyaw"}))
                return
            call = {"jsonrpc": "2.0", "id": 1, "method": "robot.move",
                    "params": {"vx": vx, "vy": 0.0, "vyaw": vyaw}}
        elif action == "relax":
            call = {"jsonrpc": "2.0", "id": 1, "method": "robot.relax"}
        else:
            self.send(404, json.dumps({"ok": False, "error": "no such action"}))
            return
        try:
            answer = rpc(call) or {}
            ok = "error" not in answer
            self.send(200, json.dumps({"ok": ok, **answer}))
        except OSError as e:
            self.send(200, json.dumps({"ok": False, "error": str(e)}))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--hz", type=int, default=50)
    ap.add_argument("--seconds", type=float, default=8.0, help="window the numbers cover")
    ap.add_argument("--trip", type=float, default=15.0,
                    help="auto-pause when a joint is this many degrees off for four ticks")
    ap.add_argument("--on-trip", choices=("relax", "hold"), default="relax",
                    help="what the watchdog does: relax cuts torque, hold keeps the pose")
    ap.add_argument("--floor", type=float, default=6.6,
                    help="relax when the pack reads below this many volts")
    ap.add_argument("--watch", action="store_true",
                    help="arm the automatic stop at startup (off by default)")
    ap.add_argument("--record", default=None,
                    help="write every frame here (default: duck-run-<timestamp>.jsonl)")
    ap.add_argument("--scs", default="/etc/robot/scs.json",
                    help="the calibration the daemon is using")
    ap.add_argument("--limits", default="/etc/robot/limits.json",
                    help="windows read out of the servos with set_limits.py --save")
    ap.add_argument("--model", default="duck.bin",
                    help="robotctl/assets/duck.bin for the 3D direction view")
    args = ap.parse_args()

    if not os.path.exists(SOCKET):
        print(f"{SOCKET} is missing — is robotd running?  systemctl status robotd")
        return 1

    load_calibration(args.scs, args.limits)
    load_model(args.model)
    path = args.record or f"duck-run-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    record = open(path, "a", encoding="utf-8")
    feed = Feed(args.hz, args.seconds, record)
    feed.start()
    WATCH["trip"] = args.trip
    WATCH["action"] = args.on_trip
    WATCH["floor"] = args.floor
    WATCH["armed"] = args.watch
    threading.Thread(target=watchdog, args=(feed,), daemon=True).start()
    threading.Thread(target=poll_health, daemon=True).start()
    Handler.feed = feed

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"duck_dash {VERSION}")
    print(f"recording every frame to {os.path.abspath(path)}")
    print(f"http://127.0.0.1:{args.port}   (from the laptop: "
          f"ssh -L {args.port}:127.0.0.1:{args.port} radxa@<board>)")
    print("Ctrl-C to stop. Nothing is written to the robot by this program.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        record.flush()
        record.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
