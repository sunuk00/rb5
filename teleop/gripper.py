"""Gripper side of the teleop: the leader trigger -> a gripper closing signal.

Moved from teleop/leader.py (2026-10-16) so the gripper has its own module before the gripper (3D-printed parallel
gripper, one Dynamixel MX-106T on its own U2D2) is connected; its commands will go here. For now the signal is only logged (CSV columns
trigger_raw, grip_close, grip_state_closed).

Open (docs/decisions.md PENDING): the right trigger read 0 for whole Real runs, so the direction below
(pulled = closed) is not confirmed on the hardware yet.
"""


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
