# controller/trajectory.py
import math

class RotaryToLinearTrajectory:
    """
    Provides L(t) and dL/dt(t) for feed-forward.

    Units: A, B in mm. θ0, Δθ in degrees. t in seconds. Returns mm and mm/s.
    """
    def __init__(self, A_mm: float, B_mm: float, theta0_deg: float,
                 delta_deg: float, duration_s: float):
        self.A = float(A_mm)
        self.B = float(B_mm)
        self.theta0 = math.radians(theta0_deg)
        self.delta = math.radians(delta_deg)
        self.T = float(duration_s)
        self.omega = self.delta / self.T if self.T > 0 else 0.0
        # precompute constants
        self._C = self.A * self.A + self.B * self.B
        self._AB = self.A * self.B

    def theta(self, t: float) -> float:
        # clamp to [0, T] to avoid overshoot in precomputed path
        tau = 0.0 if t <= 0.0 else (self.T if t >= self.T else t)
        return self.theta0 + self.omega * tau

    def length(self, t: float) -> float:
        th = self.theta(t)
        return math.sqrt(self._C + 2.0 * self._AB * math.sin(th))

    def dL_dt(self, t: float) -> float:
        th = self.theta(t)
        L = math.sqrt(self._C + 2.0 * self._AB * math.sin(th))
        if L <= 1e-9:
            return 0.0
        dL_dth = (self._AB * math.cos(th)) / L
        return dL_dth * self.omega
