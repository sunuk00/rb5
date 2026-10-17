"""RB5 kinematics (DH model + TCP offset), rotation helpers and the cylinder range.

Moved from scripts/21_leader_rb5_position.py without changes in behavior.
DH: RB5-850 standard DH (rb_cobot_docs technical_docs/dh_parameters), mm / deg.
Rotations: ZY'X'' Euler, R = Rz(rz) Ry(ry) Rx(rx) (UI Script 1.10, analytic IK page).
"""
import numpy as np

D1, D2, D3, D4, D5, D6, A1, A2 = 169.2, 148.4, 148.4, 110.7, 110.7, 96.7, 425.0, 392.0


def rot_zyx(rx, ry, rz):
    """Rotation matrix of a ZY'X'' Euler pose (deg): R = Rz(rz) Ry(ry) Rx(rx)."""
    a, b, c = np.radians([rx, ry, rz])
    rx_m = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    ry_m = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    rz_m = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def zyx_from_rot(r):
    """ZY'X'' Euler angles (rx, ry, rz) [deg] of a rotation matrix."""
    return np.degrees([np.arctan2(r[2, 1], r[2, 2]), np.arcsin(np.clip(-r[2, 0], -1.0, 1.0)),
                       np.arctan2(r[1, 0], r[0, 0])])


def rot_z(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def axis_rotation(axis, angle):
    """Rodrigues formula: rotation by angle [rad] about a (normalized) axis."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def clip_norm(v, limit):
    n = np.linalg.norm(v)
    return v if n <= limit else v * (limit / n)


def wrap_deg(a):
    """Angle wrapped to [-180, 180) deg."""
    return (a + 180.0) % 360.0 - 180.0


class RB5Model:
    """RB5-850 standard DH model plus the TCP offset (flange frame, mm). Validated against 17 / 18."""

    def __init__(self, tcp_offset_mm):
        self.offset = np.asarray(tcp_offset_mm, dtype=float)

    @staticmethod
    def _dh(theta, d, a, alpha):
        th, al = np.radians(theta), np.radians(alpha)
        ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                         [0, sa, ca, d], [0, 0, 0, 1]])

    def flange(self, q):
        t1, t2, t3, t4, t5, t6 = q
        t = np.eye(4)
        for row in ((t1, D1, 0, -90), (t2 - 90, -D2, 0, 0), (0, 0, A1, 0), (t3, D3, 0, 0), (0, 0, A2, 0),
                    (t4 + 90, -D4, 0, 0), (0, 0, 0, 90), (t5, D5, 0, -90), (t6, -D6, 0, 90)):
            t = t @ self._dh(*row)
        return t

    def shoulder_margin(self, q):
        """Horizontal radius of the wrist point P05 minus d4 [mm] (P05 = flange + d6 * flange Y, Rainbow IK)."""
        t = self.flange(q)
        p05 = t[:3, 3] + D6 * t[:3, 1]
        return np.hypot(p05[0], p05[1]) - D4

    def tcp(self, q):
        t = self.flange(q)
        return t[:3, 3] + t[:3, :3] @ self.offset, t[:3, :3]

    def jacobian(self, q, h_deg=1e-4):
        """6x6: rows = TCP velocity (mm) and angular velocity, per rad of each joint (finite differences)."""
        p0, r0 = self.tcp(q)
        jac = np.zeros((6, 6))
        for i in range(6):
            dq = np.array(q, dtype=float)
            dq[i] += h_deg
            p, r = self.tcp(dq)
            dr = r @ r0.T
            jac[:3, i] = (p - p0) / np.radians(h_deg)
            jac[3:, i] = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2 / np.radians(h_deg)
        return jac

    def predict(self, q, dp_mm, jac=None, dyaw_deg=0.0):
        """Joint motion [deg] for a TCP position change dp and a rotation dyaw about base z (TCP fixed),
        from the current q. Pass jac = jacobian(q) to reuse it (one Jacobian per cycle)."""
        return np.degrees(np.linalg.solve(self.jacobian(q) if jac is None else jac,
                                          np.r_[dp_mm, 0, 0, np.radians(dyaw_deg)]))


def tcp_offset_from(tcp, tfc):
    """TCP offset in the flange frame from get_tcp_info / get_tfc_info poses (17): R_tfc^T (tcp - tfc).
    Returns (offset_mm, orientation_offset_deg)."""
    tcp, tfc = np.asarray(tcp, dtype=float), np.asarray(tfc, dtype=float)
    return rot_zyx(*tfc[3:]).T @ (tcp[:3] - tfc[:3]), float(np.abs(wrap_deg(tcp[3:] - tfc[3:])).max())


class CylinderRange:
    """Cylinder piece about the base z axis in absolute base-frame values (independent of the start
    pose): r_min..r_max, z_min..z_max, azimuth az_min..az_max [deg]. Optional r_max_by_z: [z, r_max]
    points where the outer radius falls with height (linear between points, the first value below the
    first point, the last above the last), so the top outer corner the arm cannot reach is cut off.
    Optional r_min_by_z: [z, r_min] points the same way for the inner radius (never below r_min): low down
    the folded arm brings J3 near its limit, so the inner radius rises toward the floor (2026-10-17).
    Clamped in z first, then r (at the clamped z), and azimuth."""

    def __init__(self, r_min, r_max, z_min, z_max, az_min, az_max, r_max_by_z=None, r_min_by_z=None):
        self.r_min, self.r_max = r_min, r_max
        self.z_lo, self.z_hi = z_min, z_max
        self.az0, self.az_half = 0.5 * (az_min + az_max), 0.5 * (az_max - az_min)   # center and half width
        if r_max_by_z:
            table = np.asarray(r_max_by_z, dtype=float)
            self.table_z, self.table_r = table[:, 0], table[:, 1]
        else:
            self.table_z = self.table_r = None
        if r_min_by_z:
            table = np.asarray(r_min_by_z, dtype=float)
            self.inner_z, self.inner_r = table[:, 0], table[:, 1]
        else:
            self.inner_z = self.inner_r = None

    def r_min_at(self, z):
        """Inner radius at height z (absolute)."""
        if self.inner_z is None:
            return self.r_min
        return max(self.r_min, float(np.interp(z, self.inner_z, self.inner_r)))

    def r_max_at(self, z):
        """Outer radius at height z (absolute)."""
        if self.table_z is None:
            return self.r_max
        return min(self.r_max, float(np.interp(z, self.table_z, self.table_r)))

    @classmethod
    def from_settings(cls, rng):
        return cls(rng.r_min_mm, rng.r_max_mm, rng.z_min_mm, rng.z_max_mm, rng.azimuth_min_deg,
                   rng.azimuth_max_deg, rng.r_max_by_z, rng.r_min_by_z)

    def clamp(self, p):
        """Return the point clamped into the range (r, z and azimuth separately)."""
        r = np.hypot(p[0], p[1])
        daz = (np.degrees(np.arctan2(p[1], p[0])) - self.az0 + 180.0) % 360.0 - 180.0
        z_c = np.clip(p[2], self.z_lo, self.z_hi)
        r_c = np.clip(r, self.r_min_at(z_c), self.r_max_at(z_c))
        az_c = np.radians(self.az0 + np.clip(daz, -self.az_half, self.az_half))
        out = np.array([r_c * np.cos(az_c), r_c * np.sin(az_c), z_c])
        return out if np.linalg.norm(out - p) > 1e-9 else np.array(p, dtype=float)

    def contains(self, p):
        return np.linalg.norm(self.clamp(p) - p) < 1e-6
