"""Diagnose leader arm vibration while holding a pose (leader arm only, no RB-Y1).

Reproduces the "button released" state of example 35 / script 14: operating mode 5
(current-based position), goal current from 0.5 Nm, goal position = current position.
Gains (addresses 80-84) are never written. Every condition records Present Input Voltage,
because a 0.65 A 12 V adapter may sag under ~2 A of shoulder/elbow current.

Conditions, each HOLD_S seconds, for pose A (arms hanging down) and pose B (READY-like):
    C0  all torque off, arm on a support          -> baseline (encoder noise, outside vibration)
    C3  only wrists (IDs 4-6, 11-13) hold          -> wrist alone with a light supply load
    C1  all 14 joints hold, read all 14 (~200 Hz)  -> same state as example 35 / 14 released
    C2  right after C1, still holding, read only IDs 4 and 0 fast (~1 kHz) -> vibration frequency
C1 vs C3 separates "supply sags under load" from "the wrist itself".

Read block per joint (one GroupSyncRead, address 124, 23 bytes; ROBOTIS control tables
xm430_w210.model / xm540_w150.model): Present PWM 124 (2), Present Current 126 (2),
Present Velocity 128 (4), Present Position 132 (4), Present Input Voltage 144 (2), Present Temperature 146 (1).
Input voltage unit 0.1 V (ROBOTIS e-Manual, not in a local file; checked at start against Min Voltage Limit).

Usage:
    python scripts/16_leader_hold_diag.py --label adapter065        # measure (writes data/16_hold_diag_<label>_<time>.csv)
    python scripts/16_leader_hold_diag.py --compare A.csv B.csv     # compare two runs, no hardware
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

import config

ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT / "data"

PORT = config.LEADER_PORT
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
JOINT_IDS = list(range(14))
WRIST_IDS = [4, 5, 6, 11, 12, 13]
FAST_IDS = [4, 0]          # right wrist yaw1, right shoulder pitch
HOLD_S = 5.0               # per condition (adapter is overloaded while holding; keep it short)
HOLD_TORQUE_NM = 0.5       # example 35 ma_torque_limit / script 14

# Addresses (ROBOTIS control tables in dynamixel_easy_sdk, same for both models)
ADDR_MIN_VOLTAGE_LIMIT = 34    # 2 bytes, 0.1 V
ADDR_HW_ERROR = 70             # 1 byte
ADDR_OPERATING_MODE = 11       # 1 byte
ADDR_TORQUE_ENABLE = 64        # 1 byte
ADDR_GOAL_CURRENT = 102        # 2 bytes
ADDR_GOAL_POSITION = 116       # 4 bytes
READ_START, READ_LEN = 124, 23 # 124 PWM .. 146 temperature
CURRENT_BASED_POSITION_MODE = 5
CURRENT_UNIT_MA = 2.69         # rby1-sdk dynamixel_bus.cpp:501 (raw = Nm / Kt * 1000 / 2.69)
VOLT_UNIT = 0.1
TORQUE_CONSTANT_BY_MODEL = {1130: 1.6591, 1030: 1.3043}   # see docs/decisions.md (14)
MAX_TEMP_C = 60

CSV_HEADER = ["label", "pose", "cond", "t_s", "id", "pos_raw", "vel_raw", "cur_raw", "pwm_raw", "vin_raw", "temp_c"]


def s16(v):
    return v - 65536 if v >= 32768 else v


def s32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


class Leader:
    def __init__(self, port_name):
        import dynamixel_sdk.port_handler as dxl_port_handler
        from dynamixel_sdk import PacketHandler, PortHandler
        lt = read_latency_timer(port_name)
        if lt == 1:
            dxl_port_handler.LATENCY_TIMER = 1
        print(f"FTDI latency_timer {lt} ms" + ("" if lt == 1 else "  (WARNING: 1 ms expected, reads will be slow)"))
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        self.models = {}
        self.min_v = {}

    def open(self):
        from dynamixel_sdk import COMM_SUCCESS
        if not self.port.openPort() or not self.port.setBaudRate(BAUDRATE):
            raise SystemExit("Cannot open the leader arm port.")
        for i in JOINT_IDS:
            model, result, _ = self.ph.ping(self.port, i)
            if result != COMM_SUCCESS:
                raise SystemExit(f"ID {i} did not answer the ping.")
            if model not in TORQUE_CONSTANT_BY_MODEL:
                raise SystemExit(f"ID {i}: unknown model {model}, not writing.")
            self.models[i] = model
            v, r, _ = self.ph.read2ByteTxRx(self.port, i, ADDR_MIN_VOLTAGE_LIMIT)
            self.min_v[i] = v * VOLT_UNIT if r == COMM_SUCCESS else None

    def status(self):
        """Hardware error, temperature and input voltage of every joint (slow individual reads)."""
        out = {}
        for i in JOINT_IDS:
            e, _, _ = self.ph.read1ByteTxRx(self.port, i, ADDR_HW_ERROR)
            t, _, _ = self.ph.read1ByteTxRx(self.port, i, 146)
            v, _, _ = self.ph.read2ByteTxRx(self.port, i, 144)
            out[i] = (e, t, v * VOLT_UNIT)
        return out

    def make_reader(self, ids):
        from dynamixel_sdk import GroupSyncRead
        reader = GroupSyncRead(self.port, self.ph, READ_START, READ_LEN)
        for i in ids:
            reader.addParam(i)
        return reader

    def read(self, reader, ids):
        """One sync read -> {id: (pos, vel, cur, pwm, vin, temp)} or None on failure."""
        from dynamixel_sdk import COMM_SUCCESS
        if reader.txRxPacket() != COMM_SUCCESS:
            return None
        out = {}
        for i in ids:
            if not reader.isAvailable(i, READ_START, READ_LEN):
                return None
            out[i] = (s32(reader.getData(i, 132, 4)), s32(reader.getData(i, 128, 4)),
                      s16(reader.getData(i, 126, 2)), s16(reader.getData(i, 124, 2)),
                      reader.getData(i, 144, 2), reader.getData(i, 146, 1))
        return out

    def _write(self, address, size, id_values, signed=False):
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite
        w = GroupSyncWrite(self.port, self.ph, address, size)
        for i, v in id_values.items():
            w.addParam(i, list(int(v).to_bytes(size, "little", signed=signed)))
        r = w.txPacket()
        if r != COMM_SUCCESS:
            raise RuntimeError(f"sync write to {address} failed: {self.ph.getTxRxResult(r)}")

    def torque_off(self, ids=JOINT_IDS):
        self._write(ADDR_TORQUE_ENABLE, 1, {i: 0 for i in ids})

    def hold_here(self, ids):
        """Diagnostic write: hold the current pose of `ids` like example 35 released (mode 5, 0.5 Nm)."""
        if not ids:
            return
        self.torque_off(ids)
        modes = {}
        for i in ids:
            m, _, _ = self.ph.read1ByteTxRx(self.port, i, ADDR_OPERATING_MODE)
            if m != CURRENT_BASED_POSITION_MODE:
                modes[i] = CURRENT_BASED_POSITION_MODE
        if modes:   # operating mode is in EEPROM; write only when needed
            self._write(ADDR_OPERATING_MODE, 1, modes)
        self._write(ADDR_GOAL_CURRENT, 2, {i: int(HOLD_TORQUE_NM / TORQUE_CONSTANT_BY_MODEL[self.models[i]]
                                                  * 1000.0 / CURRENT_UNIT_MA) for i in ids}, signed=True)
        reader = self.make_reader(ids)
        now = self.read(reader, ids)
        if now is None:
            raise RuntimeError("could not read positions before holding")
        self._write(ADDR_GOAL_POSITION, 4, {i: now[i][0] for i in ids}, signed=True)   # goal position first
        self._write(ADDR_TORQUE_ENABLE, 1, {i: 1 for i in ids})                       # then torque on

    def close(self):
        self.port.closePort()


def read_latency_timer(port_name):
    dev = os.path.basename(os.path.realpath(port_name))
    try:
        return int(Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer").read_text().strip())
    except OSError:
        return None


def record(leader, ids, duration, label, pose, cond, rows):
    """Read `ids` as fast as possible for `duration` s. Stops early on low voltage or high temperature."""
    reader = leader.make_reader(ids)
    t0 = time.perf_counter()
    n = fails = 0
    stop_reason = None
    while time.perf_counter() - t0 < duration:
        data = leader.read(reader, ids)
        t = time.perf_counter() - t0
        if data is None:
            fails += 1
            continue
        n += 1
        for i in ids:
            pos, vel, cur, pwm, vin, temp = data[i]
            rows.append([label, pose, cond, f"{t:.5f}", i, pos, vel, cur, pwm, vin, temp])
            if temp > MAX_TEMP_C:
                stop_reason = f"ID {i} temperature {temp} C"
            elif leader.min_v.get(i) and vin * VOLT_UNIT < leader.min_v[i]:
                stop_reason = f"ID {i} input voltage {vin * VOLT_UNIT:.1f} V < min limit {leader.min_v[i]:.1f} V"
        if stop_reason:
            break
    elapsed = time.perf_counter() - t0
    print(f"    {cond}: {n} reads in {elapsed:.2f} s ({n / elapsed:.0f} Hz), failed {fails}"
          + (f", STOPPED: {stop_reason}" if stop_reason else ""))
    return stop_reason


def summarize(rows):
    """Per (pose, cond, id): position spread/reversals, current, voltage, and dominant frequency."""
    from collections import defaultdict
    groups = defaultdict(list)
    for r in rows:
        groups[(r[1], r[2], int(r[4]))].append(r)
    out = {}
    for key, g in sorted(groups.items()):
        t = np.array([float(r[3]) for r in g]); pos = np.array([int(r[5]) for r in g], float)
        cur = np.array([int(r[7]) for r in g], float) * CURRENT_UNIT_MA
        vin = np.array([int(r[9]) for r in g], float) * VOLT_UNIT
        d = np.diff(pos); nz = d[d != 0]
        rev = 100.0 * np.mean(np.sign(nz[1:]) != np.sign(nz[:-1])) if len(nz) > 2 else 0.0
        freq = np.nan
        if len(t) > 64 and np.ptp(pos) > 0:   # no frequency for a signal that never moves
            fs = (len(t) - 1) / (t[-1] - t[0])
            x = pos - np.polyval(np.polyfit(t, pos, 1), t)          # remove slow drift
            spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
            f = np.fft.rfftfreq(len(x), 1.0 / fs)
            k = np.argmax(spec[f > 2]) if np.any(f > 2) else None   # ignore < 2 Hz
            freq = f[f > 2][k] if k is not None else np.nan
        out[key] = dict(hz=(len(t) - 1) / max(t[-1] - t[0], 1e-9), p2p=np.ptp(pos), std=pos.std(), rev=rev,
                        cur_mean=np.abs(cur).mean(), cur_p2p=np.ptp(cur), vmin=vin.min(), vmean=vin.mean(), freq=freq)
    return out


def print_summary(summary, title):
    print(f"\n{title}")
    print(f"{'pose':<4} {'cond':<4} {'ID':>3} {'Hz':>5} {'pos p2p':>7} {'std':>5} {'rev%':>5} "
          f"{'|I| mA':>7} {'I p2p':>6} {'Vmin':>5} {'Vmean':>5} {'f Hz':>6}")
    for (pose, cond, i), s in summary.items():
        print(f"{pose:<4} {cond:<4} {i:>3} {s['hz']:5.0f} {s['p2p']:7.0f} {s['std']:5.1f} {s['rev']:5.0f} "
              f"{s['cur_mean']:7.0f} {s['cur_p2p']:6.0f} {s['vmin']:5.1f} {s['vmean']:5.1f} {s['freq']:6.1f}")
    print("pos in encoder ticks (1 tick = 0.088 deg); rev% = direction changes among moving steps; "
          "f = strongest position frequency above 2 Hz")


def load_rows(path):
    with open(path) as f:
        return [r for r in csv.reader(f)][1:]


def compare(a, b):
    sa, sb = summarize(load_rows(a)), summarize(load_rows(b))
    la, lb = load_rows(a)[0][0], load_rows(b)[0][0]
    print(f"A = {a} (label {la})\nB = {b} (label {lb})")
    print(f"{'pose':<4} {'cond':<4} {'ID':>3} | {'p2p A':>6} {'p2p B':>6} | {'rev% A':>6} {'rev% B':>6} | "
          f"{'Vmin A':>6} {'Vmin B':>6} | {'f A':>6} {'f B':>6}")
    for key in sorted(set(sa) | set(sb)):
        x, y = sa.get(key), sb.get(key)
        g = lambda s, k, fmt: (fmt % s[k]) if s else "-"
        print(f"{key[0]:<4} {key[1]:<4} {key[2]:>3} | {g(x,'p2p','%6.0f'):>6} {g(y,'p2p','%6.0f'):>6} | "
              f"{g(x,'rev','%6.0f'):>6} {g(y,'rev','%6.0f'):>6} | {g(x,'vmin','%6.1f'):>6} {g(y,'vmin','%6.1f'):>6} | "
              f"{g(x,'freq','%6.1f'):>6} {g(y,'freq','%6.1f'):>6}")


def measure(label, port_name):
    leader = Leader(port_name)
    leader.open()
    rows = []
    try:
        mv = sorted(set(round(v, 1) for v in leader.min_v.values() if v is not None))
        print(f"Min Voltage Limit (address 34) read as {mv} V  (expected about 9.5 V if the 0.1 V unit is right)")
        st = leader.status()
        print("Start status  ID:(hw_error, temp C, input V) ", {i: (e, t, round(v, 1)) for i, (e, t, v) in st.items()})
        leader.torque_off()
        poses = [("A", "arms hanging down"), ("B", "READY-like: elbows bent about 90-120 deg, forearms forward")]
        for pose, desc in poses:
            input(f"\nPose {pose}: put both leader arms {desc}, on the support for C0/C3. Press Enter...")
            print(f"  Pose {pose}")
            leader.torque_off()
            if record(leader, JOINT_IDS, HOLD_S, label, pose, "C0", rows):
                break
            leader.hold_here(WRIST_IDS)                           # C3: wrists only
            stop = record(leader, JOINT_IDS, HOLD_S, label, pose, "C3", rows)
            leader.torque_off()
            if stop:
                break
            input("  Next: C1/C2, all 14 joints hold for 10 s. Remove the support, hands ready below the arms. Enter...")
            leader.hold_here(JOINT_IDS)                           # C1 then C2 without releasing
            stop = record(leader, JOINT_IDS, HOLD_S, label, pose, "C1", rows)
            if not stop:
                stop = record(leader, FAST_IDS, HOLD_S, label, pose, "C2", rows)
            leader.torque_off()
            if stop:
                break
        st = leader.status()
        print("\nEnd status    ID:(hw_error, temp C, input V) ", {i: (e, t, round(v, 1)) for i, (e, t, v) in st.items()})
    finally:
        try:
            leader.torque_off()
            print("All leader joints torque off.")
        except Exception as e:
            print(f"WARNING: torque off failed: {e}")
        leader.close()

    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"16_hold_diag_{label}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)
    print_summary(summarize(rows), f"Summary ({label})")
    print(f"\nSaved: {path.relative_to(ROOT)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", help="power setup name, e.g. adapter065 or rby1power")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--compare", nargs=2, metavar=("A_CSV", "B_CSV"), help="compare two runs (no hardware)")
    args = parser.parse_args()
    if args.compare:
        compare(*args.compare)
    elif args.label:
        measure(args.label, args.port)
    else:
        parser.error("give --label to measure or --compare A.csv B.csv")


if __name__ == "__main__":
    main()
