import math, time
from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console
from controller.conversions import voltage_to_length  # uses your existing calibration

# --- helpers: counts->mm, percent->bits (10V cap on 11V DAC) ---
COUNTS_MAX = 65535.0
VREF_ADC   = 10.0  # ADC 0..10V
def counts_to_mm(counts: float) -> float:
    volts = float(counts) * (VREF_ADC / COUNTS_MAX)
    return float(voltage_to_length(volts))

def percent_to_bits(pct: float, bits_max: int = 8191, v_target: float = 10.0, dac_fs: float = 11.0) -> int:
    p = 0.0 if pct is None else max(0.0, min(100.0, float(pct)))
    scale = v_target / dac_fs  # 10/11 cap
    return int(round(p * 0.01 * bits_max * scale))

BITS_MAX_10V = percent_to_bits(100.0)  # ~7447
MIN_BITS     = 400                     # overcome stiction
DEFAULT_DBMM = 0.2

# --- trajectory: L(θ)=sqrt(A^2+B^2+2AB sinθ), θ(t)=θ0+ωt, ω=Δθ/T ---
class RotaryToLinearTrajectory:
    def __init__(self, A_mm: float, B_mm: float, theta0_deg: float, delta_deg: float, duration_s: float):
        self.A = float(A_mm); self.B = float(B_mm)
        self.theta0 = math.radians(theta0_deg)
        self.delta  = math.radians(delta_deg)
        self.T      = float(duration_s)
        self.omega  = self.delta / self.T if self.T > 0 else 0.0
        self._C  = self.A*self.A + self.B*self.B
        self._AB = self.A*self.B

    def _theta(self, t: float) -> float:
        tau = 0.0 if t <= 0 else (self.T if t >= self.T else t)
        return self.theta0 + self.omega * tau

    def length(self, t: float) -> float:
        th = self._theta(t)
        return math.sqrt(self._C + 2.0 * self._AB * math.sin(th))

    def dL_dt(self, t: float) -> float:
        th = self._theta(t)
        L  = math.sqrt(self._C + 2.0 * self._AB * math.sin(th))
        if L <= 1e-9: return 0.0
        dL_dth = (self._AB * math.cos(th)) / L
        return dL_dth * self.omega

# --- simple PID ---
class PID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.0, umin=-1e6, umax=1e6):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.umin, self.umax = umin, umax
        self._i = 0.0; self._prev_e = None
    def reset(self): self._i = 0.0; self._prev_e = None
    def step(self, e: float, dt: float) -> float:
        de = 0.0 if (self._prev_e is None or dt <= 0) else (e - self._prev_e)/dt
        self._i += e*dt
        u = self.kp*e + self.ki*self._i + self.kd*de
        if u > self.umax:
            u = self.umax
            if e > 0: self._i -= 0.5*e*dt
        elif u < self.umin:
            u = self.umin
            if e < 0: self._i -= 0.5*e*dt
        self._prev_e = e
        return u

class autoGrouserWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, parent=None, io_worker=None, A_mm=955.9, B_mm=247.5, snapshot_key="grouser_pos"):
        super().__init__(parent)
        self.setWindowTitle("auto – Grouser")

        self.io = io_worker
        self.A  = float(A_mm); self.B = float(B_mm)
        self.snapshot_key = snapshot_key

        # --- plots: L_meas and L_sp ---
        self.plot1 = LivePlotWidget(self, self._plot_meas, 50, 12.0)
        self.plot1.ax.set_ylabel("L_meas [mm]")
        self.plot2 = LivePlotWidget(self, self._plot_sp, 50, 12.0)
        self.plot2.ax.set_ylabel("L_sp [mm]")

        # --- controls (your original buttons) ---
        self.btn_Start  = QtWidgets.QPushButton("Start")
        self.btn_Zero   = QtWidgets.QPushButton("Zero")
        self.btn_tare   = QtWidgets.QPushButton("tare θ0 from current L")
        self.btn_return = QtWidgets.QPushButton("Return")
        self.btn_log    = QtWidgets.QPushButton("Log")
        self.btn_stop   = QtWidgets.QPushButton("Stop")

        # --- NEW: form for trajectory + PID + scaling ---
        self.spin_deg    = QtWidgets.QDoubleSpinBox(); self.spin_deg.setRange(-3600, 3600); self.spin_deg.setValue(90.0)
        self.spin_time   = QtWidgets.QDoubleSpinBox(); self.spin_time.setRange(0.1, 300.0); self.spin_time.setValue(10.0)
        self.spin_theta0 = QtWidgets.QDoubleSpinBox(); self.spin_theta0.setRange(-3600, 3600); self.spin_theta0.setValue(0.0)
        self.kp = QtWidgets.QDoubleSpinBox(); self.kp.setRange(0, 1000); self.kp.setDecimals(3); self.kp.setValue(2.0)
        self.ki = QtWidgets.QDoubleSpinBox(); self.ki.setRange(0, 1000); self.ki.setDecimals(3); self.ki.setValue(0.0)
        self.kd = QtWidgets.QDoubleSpinBox(); self.kd.setRange(0, 1000); self.kd.setDecimals(3); self.kd.setValue(0.0)
        self.kff = QtWidgets.QDoubleSpinBox(); self.kff.setRange(0, 1000); self.kff.setDecimals(3); self.kff.setValue(1.0)
        self.scale_bits = QtWidgets.QDoubleSpinBox(); self.scale_bits.setRange(0, 10000); self.scale_bits.setValue(80.0)
        self.deadband   = QtWidgets.QDoubleSpinBox(); self.deadband.setRange(0.0, 10.0); self.deadband.setDecimals(2); self.deadband.setValue(DEFAULT_DBMM)

        form = QtWidgets.QFormLayout()
        form.addRow("Δθ (deg)",      self.spin_deg)
        form.addRow("T (s)",         self.spin_time)
        form.addRow("θ0 (deg)",      self.spin_theta0)
        form.addRow("Kp", self.kp);  form.addRow("Ki", self.ki); form.addRow("Kd", self.kd)
        form.addRow("Kff (→ bits)",  self.kff)
        form.addRow("Bits per mm/s", self.scale_bits)
        form.addRow("Deadband (mm)", self.deadband)

        # stack the form above your buttons
        controls = QtWidgets.QVBoxLayout()
        controls.addLayout(form)
        controls.addSpacing(8)
        controls.addWidget(self.btn_Start)
        controls.addWidget(self.btn_Zero)
        controls.addWidget(self.btn_tare)
        controls.addWidget(self.btn_return)
        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_log)
        controls_widget = QWidget(); controls_widget.setLayout(controls)

        # --- top: two plots + controls ---
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot1, stretch=1)
        hbox.addWidget(self.plot2, stretch=1)
        hbox.addWidget(controls_widget)
        top_widget = QWidget(); top_widget.setLayout(hbox)

        # --- console ---
        self.console = QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumBlockCount(1000)
        font = self.console.font(); font.setFamily("Consolas"); self.console.setFont(font)
        connect_console(self.console, channel="grouser")
        print_console("auto Grouser ready.", channel="grouser")

        # --- main layout ---
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)
        container = QWidget(); container.setLayout(vbox)
        self.setCentralWidget(container)

        # wiring
        self.btn_return.clicked.connect(self.close)
        self.btn_tare.clicked.connect(self._tare_theta0_from_current)
        self.btn_Start.clicked.connect(self._start_control)
        self.btn_Zero.clicked.connect(self._zero_actuator)  # <-- zero button
        self.btn_stop.clicked.connect(self._stop_control)

        # control state
        self._traj = None
        self._pid  = PID(self.kp.value(), self.ki.value(), self.kd.value())
        self._t0 = None
        self._running = False
        self._last_dir = None
        self._last_bits = 0
        self._last_Lsp  = 0.0
        self._last_Lmeas = 0.0

        # zeroing state (no-PID): go to 2 mm at 80% with safeguards
        self.zero_target_mm   = 2.0
        self.zero_speed_pct   = 80.0
        self.zero_deadband_mm = 0.2
        self.zero_timeout_s   = 15.0
        self._zeroing      = False
        self._zero_dir     = None
        self._zero_started = 0.0
        self._zero_timer   = QtCore.QTimer(self)
        self._zero_timer.timeout.connect(self._zero_tick)

        # 100 Hz control loop
        self._timer = QtCore.QTimer(self); self._timer.timeout.connect(self._tick); self._timer.start(10)

    # ---------- plotting providers ----------
    def _plot_meas(self, _t):
        return float(self._last_Lmeas)
    def _plot_sp(self, _t):
        return float(self._last_Lsp)

    # ---------- control lifecycle ----------
    def _start_control(self):
        if self.io is None:
            print_console("[AUTO] No io_worker; cannot run.", channel="grouser"); return
        # don't fight zeroing
        if self._zeroing:
            self._finish_zeroing(say="Stopped zeroing to start AUTO.")
        self._traj = RotaryToLinearTrajectory(
            A_mm=self.A, B_mm=self.B,
            theta0_deg=self.spin_theta0.value(),
            delta_deg=self.spin_deg.value(),
            duration_s=self.spin_time.value()
        )
        self._pid = PID(self.kp.value(), self.ki.value(), self.kd.value())
        self._pid.reset()
        self._t0 = time.perf_counter()
        self._running = True
        print_console("[AUTO] start", channel="grouser")

    def _stop_control(self):
        self._running = False
        if self._zeroing:
            self._finish_zeroing(say="Zeroing aborted by Stop.")
        self._send("grouser:SPEED:0")
        self._send("ALL:DIR:OFF")
        print_console("[AUTO] stop", channel="grouser")

    # ---------- ZERO: go to 2 mm at 80% (no PID) ----------
    def _zero_actuator(self):
        if self.io is None:
            print_console("[ZERO] No io_worker.", channel="grouser"); return
        frame = self.io.snapshot() or {}
        counts = frame.get(self.snapshot_key, None)
        if counts is None:
            print_console("[ZERO] No measurement yet.", channel="grouser"); return

        # Pause auto controller if active
        if self._running:
            self._running = False
            print_console("[ZERO] Paused AUTO while zeroing.", channel="grouser")

        Lmeas = counts_to_mm(counts)
        e = Lmeas - self.zero_target_mm
        if abs(e) <= self.zero_deadband_mm:
            print_console(f"[ZERO] Already within {self.zero_deadband_mm} mm (L={Lmeas:.2f}).", channel="grouser")
            return

        bits = max(MIN_BITS, percent_to_bits(self.zero_speed_pct))
        desired_dir = 'BWD' if e > 0 else 'FWD'  # too long => retract; too short => extend

        self._send("grouser:SPEED:0")
        self._send(f"DIR:{desired_dir}:grouser")
        self._send(f"grouser:SPEED:{bits}")

        self._zeroing = True
        self._zero_dir = desired_dir
        self._zero_started = time.perf_counter()
        self._zero_timer.start(20)  # 50 Hz
        print_console(f"[ZERO] Moving {desired_dir} @ {self.zero_speed_pct:.0f}% toward {self.zero_target_mm} mm (L={Lmeas:.2f}).",
                      channel="grouser")

    def _zero_tick(self):
        if not self._zeroing or self.io is None:
            return
        frame = self.io.snapshot() or {}
        counts = frame.get(self.snapshot_key, None)
        if counts is None:
            return
        Lmeas = counts_to_mm(counts)
        e = Lmeas - self.zero_target_mm

        # stop if within deadband
        if abs(e) <= self.zero_deadband_mm:
            self._finish_zeroing(say=f"Reached target: L={Lmeas:.2f} mm.")
            return

        # stop if crossed the target (direction would flip)
        desired_now = 'BWD' if e > 0 else 'FWD'
        if desired_now != self._zero_dir:
            self._finish_zeroing(say=f"Crossed target; stopping at L={Lmeas:.2f} mm.")
            return

        # timeout safety
        if time.perf_counter() - self._zero_started > self.zero_timeout_s:
            self._finish_zeroing(say="Timeout; stopped.")
            return

    def _finish_zeroing(self, say=""):
        self._zero_timer.stop()
        self._send("grouser:SPEED:0")
        self._send("ALL:DIR:OFF")
        self._zeroing = False
        self._zero_dir = None
        if say:
            print_console(f"[ZERO] {say}", channel="grouser")

    # ---------- control tick @ 100 Hz ----------
    def _tick(self):
        # don't run AUTO loop while zeroing
        if self._zeroing:
            return
        if not self._running or self._traj is None or self.io is None:
            return

        now = time.perf_counter()
        t   = now - (self._t0 or now)

        # setpoint + feedforward
        Lsp     = self._traj.length(t)
        Lsp_dot = self._traj.dL_dt(t)
        self._last_Lsp = Lsp

        # measurement
        frame = self.io.snapshot() or {}
        counts = frame.get(self.snapshot_key, None)
        if counts is None:
            return
        Lmeas = counts_to_mm(counts)
        self._last_Lmeas = Lmeas

        # PID (error in mm)
        e  = Lsp - Lmeas
        u  = self._pid.step(e, dt=0.01)  # treat u as desired mm/s

        # map to bits (PID + feedforward)
        bits_from_pid = abs(u) * float(self.scale_bits.value())
        bits_from_ff  = abs(Lsp_dot) * float(self.kff.value()) * float(self.scale_bits.value())
        bits = int(min(BITS_MAX_10V, max(0.0, bits_from_pid + bits_from_ff)))

        # deadband & minimum
        if abs(e) < float(self.deadband.value()):
            bits = 0
        elif bits > 0:
            bits = max(MIN_BITS, bits)

        desired_dir = 'FWD' if (e >= 0.0) else 'BWD'  # flip if your wiring is opposite

        # be kind to relays: zero before direction change
        if self._last_dir and desired_dir != self._last_dir and self._last_bits > 0:
            self._send("grouser:SPEED:0")
            self._last_bits = 0
            return  # change dir next tick

        if desired_dir != self._last_dir:
            self._send(f"DIR:{desired_dir}:grouser")
            self._last_dir = desired_dir

        if bits != self._last_bits:
            if bits == 0:
                self._send("grouser:SPEED:0")
            else:
                self._send(f"grouser:SPEED:{bits}")
            self._last_bits = bits

        # auto-stop at end if within deadband
        if t >= self._traj.T and abs(e) < float(self.deadband.value()):
            self._stop_control()

    # ---------- tare θ0 from current measured length ----------
    def _tare_theta0_from_current(self):
        frame = self.io.snapshot() if self.io else None
        if not frame:
            print_console("[tare] No data", channel="grouser"); return
        L = counts_to_mm(frame.get(self.snapshot_key, 0.0))
        # invert L^2 = A^2 + B^2 + 2AB sinθ  => sinθ = (L^2 - A^2 - B^2)/(2AB)
        num = L*L - (self.A*self.A + self.B*self.B)
        den = 2.0*self.A*self.B if self.A*self.B != 0 else 1.0
        s   = max(-1.0, min(1.0, num/den))
        theta0 = math.degrees(math.asin(s))
        self.spin_theta0.setValue(theta0)
        print_console(f"[tare] θ0 set to {theta0:.2f} deg (from L={L:.2f} mm)", channel="grouser")

    # ---------- utilities ----------
    def _send(self, cmd: str):
        try:
            self.io.enqueue_command(cmd, require_ack=False)
            print_console(f">>> {cmd}", channel="grouser")
        except Exception as e:
            print_console(f"[ERROR] enqueue_command failed: {e}", channel="grouser")

    def closeEvent(self, e):
        try:
            self.plot1.stop(); self.plot2.stop()
        except Exception:
            pass
        self._stop_control()
        if self._zeroing:
            self._finish_zeroing()
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
