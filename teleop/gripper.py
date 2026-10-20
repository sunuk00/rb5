"""Gripper side of the teleop: one Dynamixel MX-106T (3D-printed parallel gripper) on its own U2D2.

- Motor: the MX-106(2.0) control (read, write, state, cleanup), moved from scripts/28 (grip-3a).
- start_motor / check_motor: the startup checks and writes of scripts/28 (operating mode 5).
- GripCommand: open fraction (trigger / 1000) -> goal position, clipped to the measured range, with the
  catch (no jump at the start). Pure logic.
- GripperWorker: a thread that owns the motor bus: writes the latest goal and reads the state at its own
  50 Hz, so the caller (scripts/28, the teleop loop since grip-3b) never waits for the gripper.

The motor runs in current-based position control (operating mode 5): it goes to the goal position, but
its current (the gripping force) is capped by Goal Current. In mode 5 the motor's own position limits
(48, 52) are NOT used (e-Manual), so the measured open / closed positions are the only position limits.
Addresses and units: ROBOTIS e-Manual, MX-106T/R(2.0) control table.
"""
import os
import threading
import time

GRIPPER_ID = 2                 # scripts/27
GRIPPER_BAUD = 1000000
MODEL_MX106_2 = 321
CURRENT_UNIT_MA = 3.36
TICK_DEG = 360.0 / 4096
MODE_CURRENT_POSITION = 5
MODE_AT_END = 3                # position control, how the motor was before grip-2 (user: back to 3)
MAX_MOVE_ON_MODE_CHANGE = 20   # ticks: the position must not change when the mode changes
# Added to dynamixel_sdk's reply timeout (about 4.3 ms with latency_timer 1) on the gripper port only. Python
# switches threads every 5 ms (GIL): when the 32-byte state reply arrives in two USB pieces and the teleop
# loop is planning in between, the reply was already there but timed out (Real 20261017_143135: 3 in a row,
# the gripper thread stopped). Read-only test with the planning load: 2859 / 3001 failed, +6 or +12 ms: 0.
REPLY_TIMEOUT_EXTRA_MS = 10.0
# Grasp squeeze (GripperWorker): in mode 5 the current is the position error x the position gain, capped by Goal
# Current; a thin object stops the fingers only a few ticks short of the goal, so the cap was never reached
# (Real 20261019_095845: 17 ticks short, 363 mA with a 672 mA cap). When the gripper stops short of a closing
# goal, the goal moves squeeze_ticks past the stop point, so the grip force is the Goal Current cap.
# Detection uses the motor's own profile (Position Trajectory, 140): an object holds the position back while
# the profile has already reached the goal. Comparing with the goal alone fired at the start of every move
# (the motor starts about 40 ms later and accelerates) and while following a slow trigger (replay of 095845).
GRASP_SHORT_TICKS = 8          # position this far behind the profile (settled error without an object: 0-4)
GRASP_PROFILE_DONE_TICKS = 2   # the profile is within this of the goal written (its move is finished)
GRASP_STALL_RPM = 1.0          # |velocity| below this counts as stopped
GRASP_STALL_CYCLES = 3         # all of the above for this many cycles in a row (60 ms at 50 Hz)

# MX-106(2.0) control table (address, size)
TORQUE_ENABLE = (64, 1)
OPERATING_MODE = (11, 1)
MODEL_NUMBER = (0, 2)
HARDWARE_ERROR = (70, 1)
GOAL_CURRENT = (102, 2)
PROFILE_ACC = (108, 4)
PROFILE_VEL = (112, 4)
VELOCITY_LIMIT = (44, 4)       # profile velocity must not exceed it (default 210)
GOAL_POSITION = (116, 4)
PRESENT_POSITION = (132, 4)
STATE_BLOCK = (126, 21)        # 126 current (2), 128 velocity (4), 132 position (4), 136 / 140 trajectory, 144 voltage (2), 146 temperature (1)


def signed(value, size):
    bits = 8 * size
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


def add_reply_margin(port, extra_ms):
    """Make this port's packet timeout extra_ms longer (setPacketTimeout is called before every reply)."""
    base = port.setPacketTimeout

    def set_packet_timeout(packet_length):
        base(packet_length)
        port.packet_timeout += extra_ms

    port.setPacketTimeout = set_packet_timeout


class Motor:
    """One MX-106(2.0) on its own U2D2."""

    def __init__(self, port_name, dxl_id=GRIPPER_ID, baud=GRIPPER_BAUD):
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler   # imported here so tests need no hardware
        self.ok = COMM_SUCCESS
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(2.0)
        self.id = dxl_id
        self.baud = baud
        add_reply_margin(self.port, REPLY_TIMEOUT_EXTRA_MS)

    def open(self):
        """Returns a list of problems (empty = OK)."""
        try:
            opened = self.port.openPort()
        except Exception as e:   # pyserial raises when the device cannot be opened
            return [f"failed to open {self.port.getPortName()}: {e}"]
        if not opened:
            return [f"failed to open {self.port.getPortName()}"]
        if not self.port.setBaudRate(self.baud):
            return [f"failed to set baud rate {self.baud}"]
        return []

    def read(self, item):
        addr, size = item
        fn = {1: self.ph.read1ByteTxRx, 2: self.ph.read2ByteTxRx, 4: self.ph.read4ByteTxRx}[size]
        value, result, error = fn(self.port, self.id, addr)
        return signed(value, size) if result == self.ok else None

    def write(self, item, value):
        """Write and check the reply. Returns an error text or None."""
        addr, size = item
        fn = {1: self.ph.write1ByteTxRx, 2: self.ph.write2ByteTxRx, 4: self.ph.write4ByteTxRx}[size]
        result, error = fn(self.port, self.id, addr, value & ((1 << (8 * size)) - 1))
        if result != self.ok:
            return f"write {value} to address {addr}: {self.ph.getTxRxResult(result)}"
        if error & 0x7F:
            return f"write {value} to address {addr}: {self.ph.getRxPacketError(error)}"
        return None

    def state(self):
        """(current mA, velocity rpm, position ticks, voltage V, temperature C, position trajectory ticks) or None."""
        data, result, _ = self.ph.readTxRx(self.port, self.id, *STATE_BLOCK)
        if result != self.ok or len(data) < STATE_BLOCK[1]:
            return None
        word = lambda o, n: signed(int.from_bytes(bytes(data[o:o + n]), "little"), n)
        return (word(0, 2) * CURRENT_UNIT_MA, word(2, 4) * 0.229, word(6, 4), word(18, 2) / 10.0, data[20], word(14, 4))

    def reset_port(self):
        """After an interrupted transaction (Ctrl+C in the middle of a read or write) dynamixel_sdk keeps
        port.is_using = True and refuses every later packet ("port is in use"): clear it and the input."""
        self.port.is_using = False
        self.port.clearPort()

    def end_state(self):
        """Torque off and operating mode MODE_AT_END, each tried 3 times, then read back.
        Returns (torque, mode) read back (None if unreadable)."""
        self.reset_port()
        for item, value in ((TORQUE_ENABLE, 0), (OPERATING_MODE, MODE_AT_END)):
            for _ in range(3):
                try:
                    if self.write(item, value) is None:
                        break
                except Exception:
                    pass
                self.reset_port()
                time.sleep(0.05)
        try:
            return self.read(TORQUE_ENABLE), self.read(OPERATING_MODE)
        except Exception:
            return None, None

    def close(self):
        self.port.closePort()


def check_motor(motor, min_voltage):
    """Read-only start checks: model, hardware error, input voltage. Returns a list of problems."""
    problems = []
    model = motor.read(MODEL_NUMBER)
    if model != MODEL_MX106_2:
        problems.append(f"gripper ID {motor.id}: model {model} (need {MODEL_MX106_2} = MX-106(2.0)); run script 27")
        return problems
    err = motor.read(HARDWARE_ERROR)
    s = motor.state()
    if err != 0:
        problems.append(f"hardware error status {err} (bit 0 input voltage, 2 overheating, 3 encoder, 4 shock, 5 overload)")
    if s is None:
        problems.append("cannot read the gripper state")
    elif s[3] < min_voltage:
        problems.append(f"input voltage {s[3]:.1f} V < {min_voltage} V (12 V recommended)")
    return problems


def start_motor(motor, position_before, goal_current, profile_velocity, profile_acceleration):
    """The confirmed writes: torque off, operating mode 5 (checked, and the position must not have moved
    from position_before), profile velocity, profile acceleration, goal current, goal position = present
    position (no jump), torque on. Returns (held position, None) or (None, error text)."""
    for item, value in ((TORQUE_ENABLE, 0), (OPERATING_MODE, MODE_CURRENT_POSITION)):
        e = motor.write(item, value)
        if e:
            return None, e
    if motor.read(OPERATING_MODE) != MODE_CURRENT_POSITION:
        return None, f"operating mode did not change to {MODE_CURRENT_POSITION}"
    pos = motor.read(PRESENT_POSITION)
    if pos is None or abs(pos - position_before) > MAX_MOVE_ON_MODE_CHANGE:
        return None, (f"position changed from {position_before} to {pos} with the mode change: the open / closed "
                      "values no longer match")
    for item, value in ((PROFILE_VEL, profile_velocity), (PROFILE_ACC, profile_acceleration),
                        (GOAL_CURRENT, goal_current), (GOAL_POSITION, pos), (TORQUE_ENABLE, 1)):
        e = motor.write(item, value)
        if e:
            return None, e
    return pos, None


class GripCommand:
    """Open fraction (0 = closed .. 1 = fully open; the trigger / 1000) -> goal position:
        goal = closed + frac * (open - closed), clipped to the range.
    Catch: the goal stays at the held position until the fraction points within catch_frac of the range of
    it, or passes over it between two updates (a fast trigger skips the window: grip-2, first run); no jump
    at the start, like the teleop clutch. None (no trigger value) keeps the goal."""

    def __init__(self, closed, opened, held, catch_frac=0.05):
        self.closed, self.opened = closed, opened
        self.lo, self.hi = min(closed, opened), max(closed, opened)
        self.goal = held
        self.catch_frac = catch_frac
        self.following = False
        self.prev_side = None   # sign of (fraction goal - held position) at the last update

    def frac_of(self, position):
        """Open fraction that points at a position (the trigger value needed for the catch / 1000)."""
        return (position - self.closed) / (self.opened - self.closed)

    def update(self, frac):
        """Returns (goal, event or None)."""
        if frac is None:
            return self.goal, None
        frac = min(max(frac, 0.0), 1.0)
        goal = min(max(int(round(self.closed + frac * (self.opened - self.closed))), self.lo), self.hi)
        event = None
        if not self.following:
            side = (goal > self.goal) - (goal < self.goal)
            crossed = self.prev_side is not None and side != self.prev_side
            self.prev_side = side
            if abs(goal - self.goal) <= self.catch_frac * (self.hi - self.lo) or crossed:
                self.following = True
                event = f"gripper caught at open fraction {frac:.2f}: following"
            else:
                return self.goal, None
        self.goal = goal
        return goal, event


class GripperWorker(threading.Thread):
    """Owns the motor bus after start_motor. Each cycle (period_s): take the latest open fraction from
    set(), GripCommand -> goal, write it when it changed by deadband ticks or more since the last write,
    read the state and the hardware error, keep one row (ROW_HEADER) and the latest one (latest()).

    Stops by itself (stop_reason set, the motor left as it is: holding its last goal) on a hardware error,
    a temperature above max_temperature, fail_stop failed state reads in a row or any error (e.g. the
    U2D2 disconnected). Input voltage below min_voltage stops it too if low_voltage_stops, otherwise
    gives a warning event once per drop (teleop: turning the torque off would drop the object).
    Grasp (squeeze_ticks > 0): stopped short of a closing goal (an object) -> the goal written is the stop
    point + squeeze_ticks toward closed, until the trigger goal opens back to the object or the gripper
    gets there (the object is gone). Without an object the fingers reach the goal and nothing changes.
    With grasp_current (and goal_current, the value start_motor wrote), Goal Current is grasp_current while
    grasping and goal_current otherwise, written before the goal of that cycle: a stronger hold without the
    higher cap during moves (the supply dipped to 9.5 V when the moves started at a higher cap).
    finish(): stop the thread, then torque off and operating mode 3 (Motor.end_state)."""

    ROW_HEADER = ["t_s", "open_frac", "frac_age_ms", "following", "goal", "goal_written", "position", "current_mA",
                  "velocity_rpm", "voltage_V", "temperature_C", "hardware_error", "work_ms", "grasp", "position_trajectory",
                  "goal_current"]

    def __init__(self, motor, command, period_s=0.02, deadband=2, min_voltage=10.0, max_temperature=70,
                 fail_stop=3, low_voltage_stops=True, squeeze_ticks=0, goal_current=None, grasp_current=None):
        super().__init__(daemon=True)
        self.motor, self.command = motor, command
        self.period_s, self.deadband, self.fail_stop = period_s, deadband, fail_stop
        self.min_voltage, self.max_temperature = min_voltage, max_temperature
        self.low_voltage_stops = low_voltage_stops
        self.squeeze_ticks = squeeze_ticks
        self.goal_current, self.grasp_current = goal_current, grasp_current
        self.cap = goal_current           # Goal Current last written (start_motor wrote goal_current)
        self.pos = self.vel = self.traj = None   # from the last state read
        self.stalled, self.grasp_at = 0, None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frac, self.frac_t = None, None
        self.written = command.goal       # goal last written (start_motor wrote the held position)
        self.rows = []                    # read by the caller after finish()
        self.last = None                  # latest row (dict)
        self.pending = []                 # events for the caller
        self.stop_reason = None
        self.t0 = time.monotonic()

    def set(self, frac):
        """Latest open fraction from the caller (None: no trigger value this cycle). Does not wait."""
        with self.lock:
            self.frac, self.frac_t = frac, time.monotonic()

    def latest(self):
        """The latest row as a dict (None before the first cycle)."""
        with self.lock:
            return None if self.last is None else dict(self.last)

    def events(self):
        """Events since the last call (caught, low voltage, stopped)."""
        with self.lock:
            out, self.pending = self.pending, []
        return out

    def _event(self, text):
        with self.lock:
            self.pending.append(text)

    def _grasp_goal(self, goal):
        """The goal to write: the trigger goal, or while grasping the stop point + squeeze_ticks toward closed."""
        c = self.command
        if not self.squeeze_ticks or not c.following or self.pos is None:
            self.stalled, self.grasp_at = 0, None
            return goal
        d = 1 if c.closed > c.opened else -1   # direction of closing in ticks
        if self.grasp_at is not None:
            opened_back = (goal - self.grasp_at) * d <= 0                              # trigger at or past the object
            gone = (self.pos - self.grasp_at) * d >= self.squeeze_ticks - GRASP_SHORT_TICKS   # closed on: no object
            if not (opened_back or gone):
                return self.grasp_at + d * self.squeeze_ticks
            self.grasp_at, self.stalled = None, 0
            self._event("gripper: grasp released" if opened_back else "gripper: grasp lost (closed past the object)")
        stopped_short = (self.traj is not None and abs(self.traj - self.written) <= GRASP_PROFILE_DONE_TICKS
                         and (self.traj - self.pos) * d >= GRASP_SHORT_TICKS and (goal - self.pos) * d >= GRASP_SHORT_TICKS
                         and abs(self.vel) < GRASP_STALL_RPM)
        self.stalled = self.stalled + 1 if stopped_short else 0
        if self.stalled >= GRASP_STALL_CYCLES:
            self.grasp_at = self.pos
            cap = self.grasp_current if self.grasp_current is not None else self.goal_current
            self._event(f"gripper: grasp at {self.pos} ({abs(goal - self.pos)} ticks short), squeezing"
                        + ("" if cap is None else f" at {cap * CURRENT_UNIT_MA:.0f} mA"))
            return self.grasp_at + d * self.squeeze_ticks
        return goal

    def _set_cap(self):
        """Goal Current: grasp_current while grasping, goal_current otherwise (written only on a change)."""
        if self.grasp_current is None or self.goal_current is None:
            return
        want = self.grasp_current if self.grasp_at is not None else self.goal_current
        if want != self.cap:
            e = self.motor.write(GOAL_CURRENT, want)
            if e:
                self._event(f"gripper: {e}")
            else:
                self.cap = want

    def run(self):
        fails, low = 0, False
        next_t = time.monotonic()
        try:
            while not self.stop_event.is_set():
                t_start = time.monotonic()
                with self.lock:
                    frac, frac_t = self.frac, self.frac_t
                goal, event = self.command.update(frac)
                if event:
                    self._event(event)
                goal = self._grasp_goal(goal)
                self._set_cap()
                write = abs(goal - self.written) >= self.deadband
                if write:
                    e = self.motor.write(GOAL_POSITION, goal)
                    if e:
                        self._event(f"gripper: {e}")
                    else:
                        self.written = goal
                s = self.motor.state()
                err = self.motor.read(HARDWARE_ERROR)
                fails = 0 if s is not None else fails + 1
                if s is not None:
                    self.pos, self.vel, self.traj = s[2], s[1], s[5]
                row = dict(zip(self.ROW_HEADER, [
                    t_start - self.t0, frac, None if frac_t is None else (t_start - frac_t) * 1000.0,
                    int(self.command.following), goal, int(write)]
                    + (list(s[2:3]) + [s[0], s[1], s[3], s[4]] if s else [None] * 5)
                    + [err, (time.monotonic() - t_start) * 1000.0, int(self.grasp_at is not None), s[5] if s else None,
                       self.cap]))
                with self.lock:
                    self.rows.append(row)
                    self.last = row
                reason = None
                if err:
                    reason = f"hardware error status {err}"
                elif s and s[4] > self.max_temperature:
                    reason = f"temperature {s[4]} C > {self.max_temperature} C"
                elif fails >= self.fail_stop:
                    reason = f"{self.fail_stop} failed gripper reads in a row"
                elif s and s[3] < self.min_voltage:
                    if self.low_voltage_stops:
                        reason = f"input voltage {s[3]:.1f} V < {self.min_voltage} V"
                    elif not low:
                        self._event(f"gripper: input voltage {s[3]:.1f} V < {self.min_voltage} V (12 V recommended)")
                    low = True
                elif s:
                    low = False
                if reason:
                    self.stop_reason = reason
                    self._event(f"gripper stopped: {reason}")
                    return
                next_t += self.period_s
                time.sleep(max(0.0, next_t - time.monotonic()))
        except Exception as e:   # e.g. termios.error when the U2D2 disconnects
            self.stop_reason = f"error: {e!r}"
            self._event(f"gripper stopped: {self.stop_reason}")

    def csv_rows(self):
        """rows as CSV values in the order of ROW_HEADER (blank for None)."""
        fmt = lambda v: "" if v is None else f"{v:.4f}" if isinstance(v, float) else v   # noqa: E731
        return [[fmt(r[k]) for k in self.ROW_HEADER] for r in self.rows]

    def finish(self):
        """Stop the thread (if started), then torque off and mode 3. Returns (torque, mode) read back."""
        self.stop_event.set()
        if self.is_alive():
            self.join(1.0)
        return self.motor.end_state()


# ---------- start sequence shared by the teleop and the policy runner (rec-6c) ----------

GRIPPER_DEADBAND = 2   # ticks: the gripper goal is written when it changes by this much (scripts/28)


def open_gripper(g, required_latency_ms):
    """Open the gripper U2D2 and run the read-only checks. Returns (motor, problems); motor is None on problems."""
    from .leader import read_latency_timer
    from .settings import GRIPPER_MIN_RANGE
    if not os.path.exists(g.port):
        return None, [f"port {g.port} does not exist (U2D2 USB cable? gripper.port in the settings)"]
    latency = read_latency_timer(g.port)
    if latency != required_latency_ms:
        return None, [f"gripper FTDI latency_timer {latency} ms (need {required_latency_ms}; udev rule, docs/teleop_guide.md)"]
    motor = Motor(g.port)
    problems = motor.open()
    if not problems:
        problems = check_motor(motor, g.min_voltage)
    if not problems:
        vlimit = motor.read(VELOCITY_LIMIT)
        if vlimit is None or g.profile_velocity > vlimit:
            problems.append(f"gripper.profile_velocity {g.profile_velocity} is above the motor's velocity limit {vlimit}")
        pos = motor.read(PRESENT_POSITION)
        lo, hi = min(g.closed_ticks, g.open_ticks), max(g.closed_ticks, g.open_ticks)
        if pos is None or not lo - GRIPPER_MIN_RANGE <= pos <= hi + GRIPPER_MIN_RANGE:
            problems.append(f"the gripper is at {pos}, outside closed_ticks / open_ticks {lo}..{hi}: wrong motor "
                            "or the settings no longer match the gripper (scripts/28 measures them)")
    if problems:
        motor.close()
        return None, problems
    return motor, []


def start_lines(g, pos):
    """What the start writes do, for the screen before the typed yes."""
    return [f"  torque off, operating mode {MODE_AT_END} -> 5 (current-based position; {MODE_AT_END} again at the end)",
            f"  profile velocity {g.profile_velocity} ({g.profile_velocity * 0.229:.1f} rpm), profile acceleration "
            f"{g.profile_acceleration}, goal current {g.goal_current} ({g.goal_current * CURRENT_UNIT_MA:.0f} mA, force cap; "
            f"{g.grasp_current} = {g.grasp_current * CURRENT_UNIT_MA:.0f} mA while grasping an object)",
            f"  goal position {pos} (present position, no jump), torque on",
            "Keep fingers out of the gripper. At the end the gripper torque goes off: put any held object down "
            "before stopping."]


def start_worker(motor, g, pos, period_s, catch_frac):
    """The start writes (start_motor) and a started GripperWorker. Returns (worker, command, held, problems);
    on problems the torque was turned off (end_state) and worker is None. catch_frac: the teleop's (a trigger
    must come near the gripper first); the policy runner passes 1.0 (it follows at once)."""
    held, e = start_motor(motor, pos, g.goal_current, g.profile_velocity, g.profile_acceleration)
    if e:
        t_back, m_back = motor.end_state()
        return None, None, None, [e, f"read back: torque {t_back}, operating mode {m_back}"]
    command = GripCommand(g.closed_ticks, g.open_ticks, held, catch_frac)
    worker = GripperWorker(motor, command, period_s, GRIPPER_DEADBAND, g.min_voltage, g.max_temperature,
                           g.fail_stop, low_voltage_stops=False,   # low voltage is a warning (user, grip-3)
                           squeeze_ticks=g.squeeze_ticks, goal_current=g.goal_current, grasp_current=g.grasp_current)
    worker.start()
    return worker, command, held, []
