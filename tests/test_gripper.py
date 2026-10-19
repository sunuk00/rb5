"""Tests for teleop.gripper (grip-3a): GripCommand, GripperWorker, start_motor and Motor.end_state with a fake
motor / fake dynamixel_sdk port (no hardware). Run: python -m unittest discover tests"""
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from teleop import gripper as g  # noqa: E402

CLOSED, OPEN = 3968, 2196   # docs/hardware.md (closed has the larger value)


class FakeMotor:
    """Moves toward the goal by `step` ticks per state read (velocity = the move, 0 when stopped). Hooks:
    fail_state (count of failed state reads), voltage, error, temperature, raise_on_state, block_at (an
    object: closing, i.e. increasing, stops there), delay (state reads after a goal change before the
    position starts to follow; the profile, i.e. the position trajectory, starts at once)."""

    def __init__(self, position=CLOSED, step=100):
        self.id = 2
        self.regs = {g.OPERATING_MODE: 3, g.TORQUE_ENABLE: 0, g.HARDWARE_ERROR: 0, g.PRESENT_POSITION: position}
        self.goal, self.step, self.traj, self.delay, self.wait = position, step, position, 0, 0
        self.writes = []
        self.fail_state, self.voltage, self.temperature, self.raise_on_state = 0, 11.0, 30, None
        self.block_at = None
        self.ended = False

    def read(self, item):
        if item == g.PRESENT_POSITION:
            return self.regs[item]
        return self.regs.get(item, 0)

    def write(self, item, value):
        self.writes.append((item, value))
        self.regs[item] = value
        if item == g.GOAL_POSITION:
            self.goal, self.wait = value, self.delay
        return None

    def state(self):
        if self.raise_on_state:
            raise self.raise_on_state
        if self.fail_state:
            self.fail_state -= 1
            return None
        self.traj += max(-self.step, min(self.step, self.goal - self.traj))
        p0 = self.regs[g.PRESENT_POSITION]
        if self.wait:
            self.wait -= 1
            p = p0
        else:
            p = self.traj
        if self.block_at is not None and p > self.block_at:
            p = max(p0, self.block_at)
        self.regs[g.PRESENT_POSITION] = p
        return (100.0, float(p - p0), p, self.voltage, self.temperature, self.traj)

    def end_state(self):
        self.ended = True
        self.regs[g.TORQUE_ENABLE], self.regs[g.OPERATING_MODE] = 0, g.MODE_AT_END
        return 0, g.MODE_AT_END


def frac(position):
    return (position - CLOSED) / (OPEN - CLOSED)


class TestGripCommand(unittest.TestCase):
    def test_proportional_and_clipped_once_following(self):
        c = g.GripCommand(CLOSED, OPEN, held=CLOSED)
        self.assertEqual(c.update(0.0)[0], CLOSED)               # trigger 0 = closed: caught at once
        self.assertTrue(c.following)
        self.assertEqual(c.update(0.5)[0], round(CLOSED + 0.5 * (OPEN - CLOSED)))
        self.assertEqual(c.update(1.0)[0], OPEN)                 # 1000 = fully open
        self.assertEqual(c.update(1.3)[0], OPEN)                 # clipped to the range
        self.assertEqual(c.update(-0.2)[0], CLOSED)
        self.assertEqual(c.update(None)[0], CLOSED)              # no trigger value: keep the goal

    def test_catch_window(self):
        held = 3000
        c = g.GripCommand(CLOSED, OPEN, held)
        self.assertEqual(c.update(1.0), (held, None))            # far away: holds
        self.assertEqual(c.update(0.8), (held, None))
        goal, event = c.update(frac(held) + 0.03)                # within 5 % of the range
        self.assertIn("caught", event)
        self.assertNotEqual(goal, held)
        self.assertAlmostEqual(c.frac_of(held), frac(held))

    def test_catch_when_a_fast_trigger_passes_over(self):
        held = 3000
        c = g.GripCommand(CLOSED, OPEN, held)
        self.assertEqual(c.update(frac(held) - 0.3)[0], held)    # one side
        goal, event = c.update(frac(held) + 0.3)                 # skips the window to the other side
        self.assertIn("caught", event)
        self.assertEqual(goal, round(CLOSED + (frac(held) + 0.3) * (OPEN - CLOSED)))

    def test_none_does_not_break_the_catch(self):
        c = g.GripCommand(CLOSED, OPEN, 3000)
        c.update(frac(3000) - 0.3)
        c.update(None)
        self.assertIn("caught", c.update(frac(3000) + 0.3)[1])


def run_worker(motor, fracs, held=CLOSED, **kw):
    """Run the worker thread with a 1 ms period, feeding one fraction per 3 ms; returns the worker after finish()."""
    w = g.GripperWorker(motor, g.GripCommand(CLOSED, OPEN, held), period_s=0.001, **kw)
    w.start()
    for f in fracs:
        w.set(f)
        time.sleep(0.003)
        if not w.is_alive():
            break
    w.end = w.finish()
    return w


class TestGripperWorker(unittest.TestCase):
    def test_follows_and_writes_only_on_change(self):
        m = FakeMotor()
        w = run_worker(m, [0.0] * 3 + [1.0] * 40)
        goals = [v for item, v in m.writes if item == g.GOAL_POSITION]
        self.assertEqual(goals, [OPEN])                          # 0.0 = the held position: nothing to write
        self.assertEqual(m.regs[g.PRESENT_POSITION], OPEN)
        self.assertIsNone(w.stop_reason)
        self.assertTrue(m.ended)
        self.assertEqual(w.end, (0, g.MODE_AT_END))
        self.assertEqual(sum(r["goal_written"] for r in w.rows), 1)
        self.assertEqual(list(w.rows[-1]), g.GripperWorker.ROW_HEADER)
        self.assertEqual(w.latest()["position"], OPEN)

    def test_deadband(self):
        m = FakeMotor()
        tick = 1.0 / (CLOSED - OPEN)
        run_worker(m, [0.0] * 3 + [1 * tick] * 5 + [3 * tick] * 5)
        self.assertEqual([v for item, v in m.writes if item == g.GOAL_POSITION], [CLOSED - 3])   # 1 tick: not written

    def test_stops_on_failed_reads(self):
        m = FakeMotor()
        m.fail_state = 3
        w = run_worker(m, [0.0] * 20)
        self.assertEqual(w.stop_reason, "3 failed gripper reads in a row")
        self.assertIn("gripper stopped: 3 failed gripper reads in a row", w.events())
        self.assertTrue(m.ended)

    def test_stops_on_hardware_error_and_temperature(self):
        m = FakeMotor()
        m.regs[g.HARDWARE_ERROR] = 32
        self.assertEqual(run_worker(m, [0.0] * 10).stop_reason, "hardware error status 32")
        m = FakeMotor()
        m.temperature = 71
        self.assertEqual(run_worker(m, [0.0] * 10).stop_reason, "temperature 71 C > 70 C")

    def test_low_voltage_stops_or_warns(self):
        m = FakeMotor()
        m.voltage = 9.8
        self.assertEqual(run_worker(m, [0.0] * 10).stop_reason, "input voltage 9.8 V < 10.0 V")
        m = FakeMotor()
        m.voltage = 9.8
        w = run_worker(m, [0.0] * 10, low_voltage_stops=False)   # teleop: warn once, keep running
        self.assertIsNone(w.stop_reason)
        self.assertEqual([e for e in w.events() if "voltage" in e], ["gripper: input voltage 9.8 V < 10.0 V (12 V recommended)"])

    def test_error_stops_and_cleanup_still_runs(self):
        m = FakeMotor()
        m.raise_on_state = OSError(5, "Input/output error")
        w = run_worker(m, [0.0] * 10)
        self.assertTrue(w.stop_reason.startswith("error: OSError"))
        self.assertTrue(m.ended)

    def test_finish_without_start(self):
        m = FakeMotor()
        w = g.GripperWorker(m, g.GripCommand(CLOSED, OPEN, CLOSED))
        self.assertEqual(w.finish(), (0, g.MODE_AT_END))
        self.assertEqual(w.rows, [])


class TestGrasp(unittest.TestCase):
    def goals(self, m):
        return [v for item, v in m.writes if item == g.GOAL_POSITION]

    def test_object_stops_the_fingers_then_squeeze(self):
        m = FakeMotor(position=OPEN)
        m.block_at = 3000                                          # an object
        w = run_worker(m, [1.0] * 3 + [0.0] * 60, held=OPEN, squeeze_ticks=150)
        self.assertEqual(self.goals(m), [CLOSED, 3150])            # trigger goal, then stop point + 150
        self.assertEqual(w.rows[-1]["grasp"], 1)
        self.assertEqual(w.rows[-1]["goal"], 3150)
        self.assertIn("gripper: grasp at 3000 (968 ticks short), squeezing", w.events())

    def test_empty_close_no_squeeze(self):
        m = FakeMotor(position=OPEN)
        w = run_worker(m, [1.0] * 3 + [0.0] * 60, held=OPEN, squeeze_ticks=150)
        self.assertEqual(self.goals(m), [CLOSED])                  # reaches the goal: nothing past it
        self.assertEqual(m.regs[g.PRESENT_POSITION], CLOSED)
        self.assertFalse(any(r["grasp"] for r in w.rows))

    def test_opening_the_trigger_releases(self):
        m = FakeMotor(position=OPEN)
        m.block_at = 3000
        f_open = (2800 - CLOSED) / (OPEN - CLOSED)                 # trigger goal 2800: back past the object
        w = run_worker(m, [1.0] * 3 + [0.0] * 60 + [f_open] * 40, held=OPEN, squeeze_ticks=150)
        self.assertEqual(self.goals(m), [CLOSED, 3150, 2800])
        self.assertEqual(m.regs[g.PRESENT_POSITION], 2800)
        self.assertIn("gripper: grasp released", w.events())

    def test_object_gone_closes_to_the_trigger_goal(self):
        m = FakeMotor(position=OPEN)
        m.block_at = 3000
        w = g.GripperWorker(m, g.GripCommand(CLOSED, OPEN, OPEN), period_s=0.001, squeeze_ticks=150)
        w.start()
        for f in [1.0] * 3 + [0.0] * 60:
            w.set(f)
            time.sleep(0.003)
        m.block_at = None                                          # the object slips out
        for _ in range(60):
            w.set(0.0)
            time.sleep(0.003)
        w.finish()
        self.assertEqual(self.goals(m), [CLOSED, 3150, CLOSED])
        self.assertEqual(m.regs[g.PRESENT_POSITION], CLOSED)
        self.assertIn("gripper: grasp lost (closed past the object)", w.events())

    def test_grasp_current_only_while_grasping(self):
        m = FakeMotor(position=OPEN)
        m.block_at = 3000
        f_open = (2800 - CLOSED) / (OPEN - CLOSED)
        w = run_worker(m, [1.0] * 3 + [0.0] * 60 + [f_open] * 40, held=OPEN, squeeze_ticks=60,
                       goal_current=150, grasp_current=250)
        order = [(item, v) for item, v in m.writes if item in (g.GOAL_CURRENT, g.GOAL_POSITION)]
        self.assertEqual(order, [(g.GOAL_POSITION, CLOSED), (g.GOAL_CURRENT, 250), (g.GOAL_POSITION, 3060),
                                 (g.GOAL_CURRENT, 150), (g.GOAL_POSITION, 2800)])   # the cap before the goal
        self.assertEqual(max(r["goal_current"] for r in w.rows if r["grasp"]), 250)
        self.assertEqual(w.rows[-1]["goal_current"], 150)

    def test_no_grasp_while_the_motor_is_starting(self):
        m = FakeMotor(position=OPEN)
        m.delay = 5                                                # 5 reads before it moves; the profile already runs
        w = run_worker(m, [1.0] * 3 + [0.0] * 60, held=OPEN, squeeze_ticks=150)
        self.assertEqual(self.goals(m), [CLOSED])
        self.assertFalse(any(r["grasp"] for r in w.rows))

    def test_no_grasp_without_a_profile_value(self):
        class NoTrajectory(FakeMotor):                            # if 140 did not follow in mode 5: never squeeze
            def state(self):
                s = super().state()
                return None if s is None else s[:5] + (0,)
        m = NoTrajectory(position=OPEN)
        m.block_at = 3000
        w = run_worker(m, [1.0] * 3 + [0.0] * 60, held=OPEN, squeeze_ticks=150)
        self.assertEqual(self.goals(m), [CLOSED])

    def test_off_with_zero(self):
        m = FakeMotor(position=OPEN)
        m.block_at = 3000
        w = run_worker(m, [1.0] * 3 + [0.0] * 60, held=OPEN)       # squeeze_ticks 0 (default)
        self.assertEqual(self.goals(m), [CLOSED])
        self.assertFalse(any(r["grasp"] for r in w.rows))


class TestStartMotor(unittest.TestCase):
    def test_writes_in_order(self):
        m = FakeMotor(position=3000)
        held, e = g.start_motor(m, 3000, 150, 200, 100)
        self.assertEqual((held, e), (3000, None))
        self.assertEqual(m.writes, [(g.TORQUE_ENABLE, 0), (g.OPERATING_MODE, 5), (g.PROFILE_VEL, 200),
                                    (g.PROFILE_ACC, 100), (g.GOAL_CURRENT, 150), (g.GOAL_POSITION, 3000),
                                    (g.TORQUE_ENABLE, 1)])

    def test_refuses_when_the_position_moved(self):
        m = FakeMotor(position=3100)
        held, e = g.start_motor(m, 3000, 150, 200, 100)
        self.assertIsNone(held)
        self.assertIn("position changed", e)
        self.assertNotIn((g.TORQUE_ENABLE, 1), m.writes)


class FakePort:
    def __init__(self):
        self.is_using = True   # left by a transaction interrupted by Ctrl+C
        self.cleared = 0

    def clearPort(self):
        self.cleared += 1


class FakePacketHandler:
    """Refuses every packet while the port is marked in use, like dynamixel_sdk (COMM_PORT_BUSY)."""
    BUSY = -1000

    def __init__(self):
        self.regs = {64: 1, 11: 5}

    def write1ByteTxRx(self, port, dxl_id, addr, value):
        if port.is_using:
            return self.BUSY, 0
        self.regs[addr] = value
        return 0, 0

    def read1ByteTxRx(self, port, dxl_id, addr):
        if port.is_using:
            return 0, self.BUSY, 0
        return self.regs[addr], 0, 0

    def getTxRxResult(self, result):
        return "port is in use"

    write2ByteTxRx = write4ByteTxRx = write1ByteTxRx
    read2ByteTxRx = read4ByteTxRx = read1ByteTxRx


class TestReplyMargin(unittest.TestCase):
    def test_gripper_port_timeout_is_longer(self):
        class Port:
            def setPacketTimeout(self, n):
                self.packet_timeout = 0.01 * n + 4.0
        port = Port()
        g.add_reply_margin(port, g.REPLY_TIMEOUT_EXTRA_MS)
        port.setPacketTimeout(32)
        self.assertAlmostEqual(port.packet_timeout, 0.32 + 4.0 + g.REPLY_TIMEOUT_EXTRA_MS)
        port.setPacketTimeout(11)                                   # every call, not added up
        self.assertAlmostEqual(port.packet_timeout, 0.11 + 4.0 + g.REPLY_TIMEOUT_EXTRA_MS)


class TestEndState(unittest.TestCase):
    def test_clears_the_busy_flag_then_torque_off_and_mode_3(self):
        m = g.Motor.__new__(g.Motor)   # without dynamixel_sdk / a port
        m.ok, m.id, m.port, m.ph = 0, 2, FakePort(), FakePacketHandler()
        self.assertEqual(m.end_state(), (0, g.MODE_AT_END))
        self.assertGreaterEqual(m.port.cleared, 1)


if __name__ == "__main__":
    unittest.main()
