# controller/pid.py
from dataclasses import dataclass

@dataclass
class PIDGains:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0

class PID:
    """
    Plain PID with clamped output and simple anti-windup.
    e(t) = L_sp - L_meas  [mm]
    u    = desired linear "effort" (think mm/s), then mapped to DAC bits.
    """
    def __init__(self, gains: PIDGains, umin: float = -1e6, umax: float = 1e6):
        self.g = gains
        self.umin = umin
        self.umax = umax
        self._i = 0.0
        self._prev_e = None

    def reset(self):
        self._i = 0.0
        self._prev_e = None

    def step(self, e: float, dt: float, ff: float = 0.0) -> float:
        """Return u = Kp*e + Ki*∫e + Kd*de/dt + ff."""
        de_dt = 0.0 if (self._prev_e is None or dt <= 0) else (e - self._prev_e) / dt
        self._i += e * dt

        u = self.g.kp * e + self.g.ki * self._i + self.g.kd * de_dt + ff

        # clamp + soft anti-windup
        if u > self.umax:
            u = self.umax
            if e > 0:  # only unwind in same sign as saturation
                self._i -= 0.5 * e * dt
        elif u < self.umin:
            u = self.umin
            if e < 0:
                self._i -= 0.5 * e * dt

        self._prev_e = e
        return u
