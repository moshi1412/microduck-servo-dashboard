#!/usr/bin/env python3
"""Write each HD-1910M's own travel window (registers 9..12) — the hard limit.

    sudo systemctl stop robotd
    python3 set_limits.py --port /dev/ttyACM0                     # read only: show what is there
    python3 set_limits.py --port /dev/ttyACM0 --apply             # write, then read back

The window lives in EEPROM on the servo, so the servo itself refuses a target outside it and
survives the daemon being wrong, replaced or absent. Registers 9..12 are EEPROM, so each write
is preceded by clearing the lock (register 55 = 0) and followed by restoring it — the same
sequence the vendor's tool and this project's `persist_gains` use.

Counts are the vendor's sign-magnitude 16-bit form: bit 15 is the sign, bits 14..0 the
magnitude. One count is 360/4096 = 0.0879°, so the windows below are the ones in the table in
the conversation, written through unchanged.

**Writing EEPROM is the one operation on this bus that can leave a servo unusable** (an
interrupted write), so this prints every servo's current values before it changes anything,
and reads them all back afterwards.
"""

import argparse
import json
import sys
import time

import serial

INST_READ = 0x02
INST_WRITE = 0x03

REG_MIN_ANGLE_LIMIT_L = 9
REG_MAX_ANGLE_LIMIT_L = 11
REG_LOCK = 55
REG_PRESENT_POSITION_L = 56

# id: (minimum, maximum), in the servo's own counts.
#
# Measured on this robot on 2026-10-17, after the midpoint CAL that made every servo read 2048 at
# its mechanical zero (`record_limits.py`: each joint pushed by hand to both mechanical ends). The
# 40-count margin is already applied. Every window here contains 2048 and that joint's home
# target — the two things robotd refuses to start without.
#
# id 12 and id 22 (the two hip pitches) were measured one-sided: the legs hang, so pushing to one
# stop and letting go before pressing Enter records a position that never crossed the zero. Left
# came back +233..+1741 counts from the zero, right −1845..+28, i.e. each joint kept only one half.
# The legs are mechanically mirrored, so the two halves are combined into the same span and split
# back: 1400..3789 on the left and its mirror 307..2696 on the right. Both are cross-checked
# against the pre-CAL measurements (which put the left's low end near 1319 and the right's high end
# near 2807), so neither end is a guess outside evidence.
LIMITS = {
    10: (1777, 3109),    # right hip yaw
    11: (1901, 2362),    # right hip roll
    12: (307, 2696),     # right hip pitch  — reconstructed, see above
    13: (1002, 2325),    # right knee
    14: (1324, 2915),    # right ankle
    20: (1052, 2291),    # left hip yaw
    21: (1851, 2332),    # left hip roll
    22: (1400, 3789),    # left hip pitch   — reconstructed, see above
    23: (1346, 3033),    # left knee
    24: (1204, 2902),    # left ankle
    30: (1975, 3429),    # neck pitch
    31: (1799, 2807),    # head pitch
    32: (1854, 2916),    # head yaw
    33: (1701, 2104),    # head roll
    34: (1944, 2328),    # mouth — closed is its own stop, so the margin only bites at one end
}


def frame(servo_id, inst, addr, data):
    length = len(data) + 3
    total = servo_id + length + inst + addr + sum(data)
    return bytes([0xFF, 0xFF, servo_id, length, inst, addr, *data, (~total) & 0xFF])


def counts_to_raw(counts):
    magnitude = min(abs(counts), 0x7FFF)
    return (magnitude | 0x8000) if counts < 0 else magnitude


def raw_to_counts(raw):
    magnitude = raw & 0x7FFF
    return -magnitude if raw & 0x8000 else magnitude


def write_reg(port, servo_id, addr, data, settle=0.05):
    """One acknowledged write. The settle is generous on purpose: an EEPROM write leaves the
    servo busy for tens of milliseconds, and a command sent into that window is lost."""
    port.reset_input_buffer()
    port.write(frame(servo_id, INST_WRITE, addr, data))
    port.flush()
    time.sleep(settle)
    port.reset_input_buffer()


def read_reg(port, servo_id, addr, count):
    """Read `count` bytes, resynchronising on the answer rather than trusting the first byte:
    the adapter echoes, and a shared bus turns the line around mid-stream."""
    port.reset_input_buffer()
    port.write(frame(servo_id, INST_READ, addr, [count]))
    port.flush()
    want = count + 6
    buf = b""
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        buf += chunk
        for start in range(max(0, len(buf) - want) + 1):
            packet = buf[start:start + want]
            if len(packet) < want or packet[:2] != b"\xff\xff":
                continue
            if packet[2] != servo_id or packet[3] != count + 2:
                continue
            if (~sum(packet[2:-1])) & 0xFF != packet[-1]:
                continue
            return packet[5:-1]
    return None


def read_window(port, servo_id):
    block = read_reg(port, servo_id, REG_MIN_ANGLE_LIMIT_L, 4)
    if block is None or len(block) != 4:
        return None
    low = raw_to_counts(block[0] | (block[1] << 8))
    high = raw_to_counts(block[2] | (block[3] << 8))
    return low, high


def read_position(port, servo_id):
    block = read_reg(port, servo_id, REG_PRESENT_POSITION_L, 2)
    if block is None or len(block) != 2:
        return None
    return raw_to_counts(block[0] | (block[1] << 8))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--apply", action="store_true", help="actually write (default is read-only)")
    ap.add_argument("--set", metavar="ID:LOW:HIGH", action="append", default=[],
                    help="write exactly this window for one servo, e.g. --set 22:1749:3789")
    ap.add_argument("--only", type=int, action="append", metavar="ID",
                    help="limit the run to these servo ids")
    ap.add_argument("--all", action="store_true",
                    help="allow writing every window in the built-in table")
    ap.add_argument("--save", metavar="JSON",
                    help="write the windows read from the servos here (read-only)")
    args = ap.parse_args()

    targets = dict(LIMITS)
    named = []
    for item in args.set:
        try:
            servo_id, low, high = (int(part) for part in item.split(":"))
        except ValueError:
            print(f"--set wants ID:LOW:HIGH, got {item!r}")
            return 1
        if servo_id not in LIMITS:
            print(f"{servo_id} is not one of the fifteen servo ids")
            return 1
        targets[servo_id] = (low, high)
        named.append(servo_id)
    if args.only:
        missing = [i for i in args.only if i not in targets]
        if missing:
            print("not one of the fifteen servo ids: " + ", ".join(str(i) for i in missing))
            return 1
        targets = {i: targets[i] for i in args.only}
    elif named:
        # `--set` names a window to write, so it means only that: writing the rest of the table
        # too would silently replace thirteen windows nobody asked about.
        targets = {i: targets[i] for i in named}
    # What the daemon will think of these windows, before anything is written. Its own check
    # refuses a joint whose window is not inside one turn, and one whose calibrated zero is
    # outside the window — both are worth knowing about *now* rather than at the next boot.
    for servo_id, (low, high) in targets.items():
        if low >= high:
            print(f"id {servo_id}: window {low}..{high} is inverted")
            return 1
        if not low <= 2048 <= high:
            print(f"id {servo_id}: {low}..{high} does not contain 2048, which is the zero_ticks")
            print("      every servo is set to after the midpoint CAL — the daemon will refuse")
            print("      to open the bus for this joint.")
        if high > 4095:
            print(f"note: id {servo_id}'s maximum {high} is past one turn (4095). The servo")
            print("      takes it, but the daemon's own sanity check refuses such a window and")
            print("      will not open the bus until that check is relaxed.")

    port = serial.Serial(args.port, args.baud, timeout=0.02)
    have_windows = {}
    try:
        print("current windows (read-only):")
        for servo_id in sorted(targets):
            have = read_window(port, servo_id)
            if have is not None:
                have_windows[servo_id] = list(have)
            here = read_position(port, servo_id)
            want = targets[servo_id]
            shown = "no answer" if have is None else f"{have[0]:>6} .. {have[1]:>6}"
            if have is None or here is None:
                print(f"  id {servo_id:>3}  now {shown}   pos {'-':>6}   want {want[0]:>6} .. {want[1]:>6}")
            else:
                below, above = here - have[0], have[1] - here
                print(f"  id {servo_id:>3}  now {shown}   pos {here:>6}"
                      f"   margin {below:>+6}/{above:<+6} counts"
                      f" ({below * 0.0879:>+6.1f}° / {above * 0.0879:<+6.1f}°)")
        if args.save and len(have_windows) == len(targets):
            with open(args.save, "w", encoding="utf-8") as fh:
                json.dump({str(k): v for k, v in sorted(have_windows.items())}, fh, indent=2)
                fh.write("\n")
            print(f"\nwindows saved to {args.save}")
        elif args.save:
            print(f"\n--save: only {len(have_windows)} of {len(targets)} answered, not writing")

        if not args.apply:
            print("\nread-only. Re-run with --apply to write these windows.")
            return 0

        print("\nwriting (EEPROM: unlock, write, lock, read back)")
        failures = []
        for servo_id, (low, high) in sorted(targets.items()):
            if read_reg(port, servo_id, REG_LOCK, 1) is None:
                print(f"  id {servo_id:>3}: no answer, skipped")
                failures.append(servo_id)
                continue
            write_reg(port, servo_id, REG_LOCK, [0])
            write_reg(port, servo_id, REG_MIN_ANGLE_LIMIT_L,
                      [counts_to_raw(low) & 0xFF, counts_to_raw(low) >> 8])
            write_reg(port, servo_id, REG_MAX_ANGLE_LIMIT_L,
                      [counts_to_raw(high) & 0xFF, counts_to_raw(high) >> 8])
            write_reg(port, servo_id, REG_LOCK, [1])
            have = read_window(port, servo_id)
            ok = have == (low, high)
            if not ok:
                failures.append(servo_id)
            print(f"  id {servo_id:>3}: {low}..{high}  {'ok' if ok else f'READ BACK {have}'}")

        print()
        if failures:
            print("these did not verify: " + ", ".join(str(i) for i in failures))
            return 1
        print(f"all {len(targets)} windows written and verified")
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())
