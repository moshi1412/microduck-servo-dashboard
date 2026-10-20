#!/usr/bin/env python3
"""Plot commanded vs measured for all fifteen joints from a `robot.state` recording.

    python plot_state.py duck-run-20261018-153012.jsonl -o curves.png
    python plot_state.py duck-state.jsonl --joints left_hip_roll,head_yaw

Reads the JSONL that `duck_dash.py --record` writes (and the `/api/dump` file, which is the same
shape). The blue line is what the policy asked for — **before** the servo window clamps it — so a
green line that goes flat while blue keeps swinging is the window edge, not a servo that gave up.
"""

import argparse
import json
import math
import sys

NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
DEG = 180.0 / math.pi


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("-o", "--out", default="curves.png")
    ap.add_argument("--joints", help="comma separated subset, default all fifteen")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("needs matplotlib:  pip install matplotlib")
        return 1

    frames = []
    with open(args.path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    if len(frames) < 2:
        print(f"{args.path}: needs at least two frames")
        return 1

    wanted = ([NAMES.index(n) for n in args.joints.split(",")] if args.joints
              else list(range(len(NAMES))))
    t = [frame.get("t", 0.0) - frames[0].get("t", 0.0) for frame in frames]

    columns = 3
    rows = (len(wanted) + columns - 1) // columns
    figure, axes = plt.subplots(rows, columns, figsize=(5 * columns, 3.2 * rows), sharex=True,
                                squeeze=False)
    for slot, index in enumerate(wanted):
        axis = axes[slot // columns][slot % columns]
        commanded = [frame["targets"][index] * DEG for frame in frames]
        measured = [frame["joints"][index] * DEG for frame in frames]
        error = [abs(a - b) for a, b in zip(commanded, measured)]
        axis.plot(t, commanded, color="#1f6feb", linewidth=1.5, label="commanded")
        axis.plot(t, measured, color="#2ea043", linewidth=1.5, label="measured")
        axis.fill_between(t, commanded, measured, color="#8b949e", alpha=0.18)
        axis.set_title(f"{NAMES[index]}   mean|e| {sum(error) / len(error):.1f}°"
                       f"  max {max(error):.0f}°", fontsize=10)
        axis.grid(alpha=0.25, linewidth=0.6)
        axis.tick_params(labelsize=8)
        if slot % columns == 0:
            axis.set_ylabel("degrees", fontsize=8)
    for slot in range(len(wanted), rows * columns):
        axes[slot // columns][slot % columns].axis("off")
    axes[0][0].legend(fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel("seconds", fontsize=8)

    last = frames[-1]
    safety = last.get("safety") or {}
    figure.suptitle(f"{args.path} — {len(frames)} frames, {t[-1]:.1f} s, "
                    f"policy={last.get('policy')} fallen={safety.get('fallen')} "
                    f"limp={safety.get('limp')} gain={safety.get('gain')}", fontsize=10)
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    figure.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
