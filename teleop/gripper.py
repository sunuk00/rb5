"""Gripper side of the teleop: one Dynamixel MX-106T (3D-printed parallel gripper) on its own U2D2.

- Motor: the MX-106(2.0) control (read, write, state, cleanup), moved from scripts/28 (grip-3a).
- start_motor / check_motor: the startup checks and writes of scripts/28 (operating mode 5).
- GripCommand: open fraction (trigger / 1000) -> goal position, clipped to the measured range, with the
  catch (no jump at the start). Pure logic.
- GripperWorker: a thread that owns the motor bus: writes the latest goal and reads the state at its own
  50 Hz, so the caller (scripts/28, later the teleop loop) never waits for the gripper.
- GripSignal: the old right-trigger signal the teleop still logs; replaced by the above in grip-3b.

The motor runs in current-based position control (operating mode 5): it goes to the goal position, but
its current (the gripping force) is capped by Goal Current. In mode 5 the motor's own position limits
(48, 52) are NOT used (e-Manual), so the measured open / closed positions are the only position limits.
Addresses and units: ROBOTIS e-Manual, MX-106T/R(2.0) control table.
"""
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


class Motor:
    """One MX-106(2.0) on its own U2D2."""

    def __init__(self, port_name, dxl_id=GRIPPER_ID, baud=GRIPPER_BAUD):
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler   # imported here so tests need no hardware
        self.ok = COMM_SUCCESS
        self.port = PortHandler(port_name)
        self.ph = PacketHandler(2.0)
        self.id = dxl_id
        self.baud = baud

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
        """(current mA, velocity rpm, position ticks, voltage V, temperature C) or None."""
        data, result, _ = self.ph.readTxRx(self.port, self.id, *STATE_BLOCK)
        if result != self.ok or len(data) < STATE_BLOCK[1]:
            return None
        word = lambda o, n: signed(int.from_bytes(bytes(data[o:o + n]), "little"), n)
        return (word(0, 2) * CURRENT_UNIT_MA, word(2, 4) * 0.229, word(6, 4), word(18, 2) / 10.0, data[20])

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
    finish(): stop the thread, then torque off and operating mode 3 (Motor.end_state)."""

    ROW_HEADER = ["t_s", "open_frac", "frac_age_ms", "following", "goal", "goal_written", "position", "current_mA",
                  "velocity_rpm", "voltage_V", "temperature_C", "hardware_error", "work_ms"]

    def __init__(self, motor, command, period_s=0.02, deadband=2, min_voltage=10.0, max_temperature=70,
                 fail_stop=3, low_voltage_stops=True):
        super().__init__(daemon=True)
        self.motor, self.command = motor, command
        self.period_s, self.deadband, self.fail_stop = period_s, deadband, fail_stop
        self.min_voltage, self.max_temperature = min_voltage, max_temperature
        self.low_voltage_stops = low_voltage_stops
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
                row = dict(zip(self.ROW_HEADER, [
                    t_start - self.t0, frac, None if frac_t is None else (t_start - frac_t) * 1000.0,
                    int(self.command.following), goal, int(write)]
                    + (list(s[2:3]) + [s[0], s[1], s[3], s[4]] if s else [None] * 5)
                    + [err, (time.monotonic() - t_start) * 1000.0]))
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

    def finish(self):
        """Stop the thread (if started), then torque off and mode 3. Returns (torque, mode) read back."""
        self.stop_event.set()
        if self.is_alive():
            self.join(1.0)
        return self.motor.end_state()


class GripSignal:
    """Gripper closing from the tool trigger. The trigger reads 0 when pulled closed and trigger_full
    (1000) when fully open / released (user, 2026-10-12), so grip_close = 1 - trigger / trigger_full:
    pulled = closed, released = open. State closed above close_above, open below open_below, kept in
    between. A failed read keeps the previous value (no drop), unlike the clutch. Independent of the
    clutch."""

    def __init__(self, g):
        self.g = g
        self.trigger = None        # last good raw trigger value
        self.close = None          # 0 = open .. 1 = closed
        self.closed = None         # True closed / False open (hysteresis)
        self.fail_count = 0

    def update(self, raw):
        """Feed one tool read (raw value or None). Returns an event string or None."""
        if raw is None:
            self.fail_count += 1
            if self.fail_count == self.g.fail_warn:
                return f"gripper: {self.g.fail_warn} failed tool reads in a row, keeping the last value"
            return None
        self.fail_count = 0
        self.trigger = (raw >> 16) & 0xFFFF
        self.close = 1.0 - min(max(self.trigger / self.g.trigger_full, 0.0), 1.0)
        previous = self.closed
        if self.closed is None:
            self.closed = self.close >= 0.5
        elif self.close > self.g.close_above:
            self.closed = True
        elif self.close < self.g.open_below:
            self.closed = False
        if self.closed != previous:
            return f"gripper {'CLOSED' if self.closed else 'OPEN'} (trigger {self.trigger})"
        return None
