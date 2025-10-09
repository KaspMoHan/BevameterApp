import math, time
from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console
from controller.conversions import bits_to_length, bits_to_torque, percent_to_bits


# --- helpers/limits ---
def counts_to_mm(counts: float) -> float:
    """ADC counts -> extension [mm] via your calibration pipeline."""
    try:
        return float(bits_to_length(float(counts)))
    except Exception:
        return 0.0

BITS_MAX_10V = percent_to_bits(100.0)   # (~7447 when 10/11 cap)
MIN_BITS     = 400                      # overcome stiction
DEFAULT_DBMM = 0.2                      # position deadband [mm]


class RotaryToLinearTrajectory:
    """
    Crank/rod geometry:
      x = fixed P-to-O distance [mm]
      r = crank radius [mm]
      A(t) = theta0 + omega * t  (clamped to [0, T])
    Returns eye-to-eye length and its time derivative.
    NOTE: In the UI we keep names A_mm, B_mm for backward-compat; here A_mm:=x, B_mm:=r.
    """
    def __init__(self, A_mm: float, B_mm: float, theta0_deg: float, delta_deg: float, duration_s: float):
        self.x = float(A_mm)            # <- fixed distance x
        self.r = float(B_mm)            # <- crank radius r
        self.theta0 = math.radians(theta0_deg)
        self.delta  = math.radians(delta_deg)
        self.T      = float(duration_s)
        self.omega  = self.delta / self.T if self.T > 0 else 0.0
        self._x2 = self.x * self.x
        self._r2 = self.r * self.r

    def _theta(self, t: float) -> float:
        tau = 0.0 if t <= 0 else (self.T if t >= self.T else t)
        return self.theta0 + self.omega * tau

    def length(self, t: float) -> float:
        """Eye-to-eye length L_e2e(t) [mm]."""
        th = self._theta(t)
        # L = sqrt(x^2 + r^2 - 2 x r cos(th))
        return math.sqrt(self._x2 + self._r2 - 2.0 * self.x * self.r * math.cos(th))

    def dL_dt(self, t: float, dtheta_dt: float = None) -> float:
        """Time derivative dL/dt [mm/s]. dtheta_dt defaults to constant self.omega."""
        th = self._theta(t)
        L  = math.sqrt(self._x2 + self._r2 - 2.0 * self.x * self.r * math.cos(th))
        if L <= 1e-9:
            return 0.0
        omega = self.omega if (dtheta_dt is None) else float(dtheta_dt)
        # dL/dA = (x r sin A)/L, so dL/dt = (x r sin A / L) * dA/dt
        return (self.x * self.r * math.sin(th) / L) * omega



# --- simple PID (u is mm/s equiv) ---
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


class autoRubberWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, parent=None, io_worker=None,
                 A_mm=960, B_mm=247.5,
                 plot1_key="rubber_torque",   # left plot key (torque)
                 plot2_key="rubber_pos"       # right plot key (position)
                 ):
        super().__init__(parent)
        self.setWindowTitle("auto – rubber")

        self.io = io_worker
        self.A, self.B = float(A_mm), float(B_mm)
        self.plot1_key = plot1_key
        self.plot2_key = plot2_key  # <-- used as the position measurement key

        # hidden E2E offset so E2E_meas = L0_e2e + extension
        self._L0_e2e = 0.0

        # last values for plotting (extension)
        self._last = {self.plot1_key: 0.0, self.plot2_key: 0.0}
        self._last_Lsp = 0.0         # extension setpoint for overlay
        self._last_Lmeas = 0.0       # extension measured

        # --- LEFT: torque plot (MA) ---
        self.plot1 = LivePlotWidget(
            self, self._data_fn_for(self.plot1_key),
            update_hz=50, window_seconds=12.0,
            ymin=0, ymax=500, show_ma=True, ma_window=20,
            label="Torque"
        )
        self.plot1.ax.set_ylabel("Torque [Nm]")

        # --- RIGHT: length plot (MA + velocity + SP overlay) ---
        self.plot2 = LivePlotWidget(
            self, self._data_fn_for(self.plot2_key),  # measured extension
            update_hz=50, window_seconds=12.0,
            ymin=0, ymax=1000,
            show_ma=True, ma_window=10,
            show_velocity=True, vel_window=7,
            show_vel_ma=True, vel_ma_window=20,
            y2min=-50, y2max=50, y2label="Velocity [mm/s]",
            label="L_meas", show_legend=True
        )
        self.plot2.ax.set_ylabel("Length [mm]")
        if hasattr(self.plot2, "add_overlay"):
            self.plot2.add_overlay("L_sp", self._plot_sp)

        # --- controls ---
        self.btn_Start  = QtWidgets.QPushButton("Start")
        self.btn_Zero   = QtWidgets.QPushButton("Zero")
        self.btn_tare   = QtWidgets.QPushButton("tare θ0 from current L")
        self.btn_return = QtWidgets.QPushButton("Return")
        self.btn_log    = QtWidgets.QPushButton("Log")
        self.btn_stop   = QtWidgets.QPushButton("Stop")

        # trajectory/PID/scaling
        self.spin_deg    = QtWidgets.QDoubleSpinBox(); self.spin_deg.setRange(-3600, 3600); self.spin_deg.setValue(90.0)
        self.spin_time   = QtWidgets.QDoubleSpinBox(); self.spin_time.setRange(0.1, 300.0); self.spin_time.setValue(10.0)
        self.spin_theta0 = QtWidgets.QDoubleSpinBox(); self.spin_theta0.setRange(-3600, 3600); self.spin_theta0.setValue(0.0)
        self.kp = QtWidgets.QDoubleSpinBox(); self.kp.setRange(0, 1000); self.kp.setDecimals(3); self.kp.setValue(2.0)
        self.ki = QtWidgets.QDoubleSpinBox(); self.ki.setRange(0, 1000); self.ki.setDecimals(3); self.ki.setValue(0.0)
        self.kd = QtWidgets.QDoubleSpinBox(); self.kd.setRange(0, 1000); self.kd.setDecimals(3); self.kd.setValue(0.0)
        self.kff = QtWidgets.QDoubleSpinBox(); self.kff.setRange(0, 1000); self.kff.setDecimals(3); self.kff.setValue(1.0)
        self.scale_bits = QtWidgets.QDoubleSpinBox(); self.scale_bits.setRange(0, 10000); self.scale_bits.setValue(180.0)
        self.deadband   = QtWidgets.QDoubleSpinBox(); self.deadband.setRange(0.0, 10.0); self.deadband.setDecimals(2); self.deadband.setValue(DEFAULT_DBMM)

        form = QtWidgets.QFormLayout()
        form.addRow("Δθ (deg)",      self.spin_deg)
        form.addRow("T (s)",         self.spin_time)
        form.addRow("θ0 (deg)",      self.spin_theta0)
        form.addRow("Kp", self.kp);  form.addRow("Ki", self.ki); form.addRow("Kd", self.kd)
        form.addRow("Kff (→ bits)",  self.kff)
        form.addRow("Bits per mm/s", self.scale_bits)
        form.addRow("Deadband (mm)", self.deadband)

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

        # --- layout ---
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot1, stretch=1)
        hbox.addWidget(self.plot2, stretch=1)
        hbox.addWidget(controls_widget)
        top_widget = QWidget(); top_widget.setLayout(hbox)

        self.console = QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumBlockCount(1000)
        font = self.console.font(); font.setFamily("Consolas"); self.console.setFont(font)
        connect_console(self.console, channel="rubber")
        print_console("auto rubber ready.", channel="rubber")

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)
        container = QWidget(); container.setLayout(vbox)
        self.setCentralWidget(container)

        # wiring
        self.btn_return.clicked.connect(self.close)
        self.btn_tare.clicked.connect(self._tare_theta0_from_current)
        self.btn_Start.clicked.connect(self._start_control)
        self.btn_Zero.clicked.connect(self._zero_actuator)
        self.btn_stop.clicked.connect(self._stop_control)

        # control state
        self._traj = None
        self._pid  = PID(self.kp.value(), self.ki.value(), self.kd.value())
        self._t0 = None
        self._running = False
        self._last_dir = None
        self._last_bits = 0

        self.settle_band_mm = 0.5  # stay within ±0.5 mm to start settling
        self.settle_time_s = 0.3  # must remain in band for 0.3 s
        self._settle_start = None  # internal timer for settling

        # zeroing state
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

    # ---------- data fns (like Manual window) ----------
    def _data_fn_for(self, key: str):
        kl = key.lower()
        if "torque" in kl:
            conv = bits_to_torque        # ADC bits → torque [Nm]
        elif "pos" in kl or "length" in kl:
            conv = bits_to_length        # ADC bits → extension [mm]
        else:
            conv = float

        def _fn(_t):
            if self.io is None:
                return float(self._last.get(key, 0.0))
            frame = self.io.snapshot() or {}
            v = frame.get(key, None)
            if v is None:
                return float(self._last.get(key, 0.0))
            try:
                fv = float(conv(float(v)))
                self._last[key] = fv
                if key == self.plot2_key:   # keep extension cache aligned
                    self._last_Lmeas = fv
                return fv
            except Exception:
                return float(self._last.get(key, 0.0))
        return _fn

    # overlay for right plot (extension setpoint)
    def _plot_sp(self, _t):
        return float(self._last_Lsp)

    # ---------- lifecycle ----------
    def _start_control(self):
        if self.io is None:
            print_console("[AUTO] No io_worker; cannot run.", channel="rubber"); return
        if self._zeroing:
            self._finish_zeroing(say="Stopped zeroing to start AUTO.")

        # Build trajectory with current UI values
        self._traj = RotaryToLinearTrajectory(
            A_mm=self.A, B_mm=self.B,
            theta0_deg=self.spin_theta0.value(),
            delta_deg=self.spin_deg.value(),
            duration_s=self.spin_time.value()
        )

        # Measure current EXTENSION and compute hidden E2E offset so
        # E2E_meas(0) == E2E_sp(0)  → no initial jump.
        frame = self.io.snapshot() or {}
        counts = frame.get(self.plot2_key, None)
        Lext0 = counts_to_mm(counts) if counts is not None else 0.0
        Lsp0_e2e = self._traj.length(0.0)
        self._L0_e2e = Lsp0_e2e - Lext0
        print_console(f"[AUTO] Calibrated L0={self._L0_e2e:.1f} mm (Lsp0_e2e={Lsp0_e2e:.1f}, Lext0={Lext0:.1f})",
                      channel="rubber")

        # Init PID
        self._pid = PID(self.kp.value(), self.ki.value(), self.kd.value())
        self._pid.reset()
        self._t0 = time.perf_counter()
        self._running = True
        print_console("[AUTO] start", channel="rubber")

    def _stop_control(self):
        self._running = False
        if self._zeroing:
            self._finish_zeroing(say="Zeroing aborted by Stop.")
        self._send("rubber:SPEED:0")
        self._send("ALL:DIR:OFF")
        print_console("[AUTO] stop", channel="rubber")

    # ---------- ZERO: go to 2 mm at 80% (no PID) ----------
    def _zero_actuator(self):
        if self.io is None:
            print_console("[ZERO] No io_worker.", channel="rubber"); return
        frame = self.io.snapshot() or {}
        counts = frame.get(self.plot2_key, None)  # position key (extension)
        if counts is None:
            print_console("[ZERO] No measurement yet.", channel="rubber"); return

        if self._running:
            self._running = False
            print_console("[ZERO] Paused AUTO while zeroing.", channel="rubber")

        Lmeas = counts_to_mm(counts)
        e = Lmeas - self.zero_target_mm
        if abs(e) <= self.zero_deadband_mm:
            print_console(f"[ZERO] Already within {self.zero_deadband_mm} mm (L={Lmeas:.2f}).", channel="rubber")
            return

        bits = max(MIN_BITS, percent_to_bits(self.zero_speed_pct))
        desired_dir = 'BWD' if e > 0 else 'FWD'  # too long ⇒ retract

        self._send("rubber:SPEED:0")
        self._send(f"DIR:{desired_dir}:rubber")
        self._send(f"rubber:SPEED:{bits}")

        self._zeroing = True
        self._zero_dir = desired_dir
        self._zero_started = time.perf_counter()
        self._zero_timer.start(20)  # 50 Hz
        print_console(f"[ZERO] Moving {desired_dir} @ {self.zero_speed_pct:.0f}% toward {self.zero_target_mm} mm (L={Lmeas:.2f}).",
                      channel="rubber")

    def _zero_tick(self):
        if not self._zeroing or self.io is None:
            return
        frame = self.io.snapshot() or {}
        counts = frame.get(self.plot2_key, None)
        if counts is None:
            return
        Lmeas = counts_to_mm(counts)
        e = Lmeas - self.zero_target_mm

        if abs(e) <= self.zero_deadband_mm:
            self._finish_zeroing(say=f"Reached target: L={Lmeas:.2f} mm."); return

        desired_now = 'BWD' if e > 0 else 'FWD'
        if desired_now != self._zero_dir:
            self._finish_zeroing(say=f"Crossed target; stopping at L={Lmeas:.2f} mm."); return

        if time.perf_counter() - self._zero_started > self.zero_timeout_s:
            self._finish_zeroing(say="Timeout; stopped."); return

    def _finish_zeroing(self, say=""):
        self._zero_timer.stop()
        self._send("rubber:SPEED:0")
        self._send("ALL:DIR:OFF")
        self._zeroing = False
        self._zero_dir = None
        if say:
            print_console(f"[ZERO] {say}", channel="rubber")

    # ---------- control tick @ 100 Hz ----------
    def _tick(self):
        if self._zeroing:
            return
        if not self._running or self._traj is None or self.io is None:
            return

        now = time.perf_counter()
        t = now - (self._t0 or now)

        # Eye-to-eye setpoint & rate (signed!)
        Lsp_e2e = self._traj.length(t)
        Lsp_dot = self._traj.dL_dt(t)  # can be +/- depending on sin(theta)

        # Measurement: extension -> eye-to-eye by adding L0
        frame = self.io.snapshot() or {}
        counts = frame.get(self.plot2_key, None)
        if counts is None:
            return
        Lext_meas = counts_to_mm(counts)
        Le2e_meas = self._L0_e2e + Lext_meas

        # PID on E2E length (error in mm)
        e = Lsp_e2e - Le2e_meas
        v_pid = self._pid.step(e, dt=0.01)  # desired mm/s from PID
        v_ff = float(self.kff.value()) * float(Lsp_dot)  # signed feedforward mm/s
        v_cmd = v_ff + v_pid  # total desired mm/s (signed)

        # Map |v_cmd| -> DAC bits
        scale = float(self.scale_bits.value())  # bits per (mm/s)
        bits = int(min(BITS_MAX_10V, max(0.0, abs(v_cmd) * scale)))

        # deadband: when we're close, allow zero command
        if abs(e) < float(self.deadband.value()):
            bits = 0
        elif bits > 0:
            bits = max(MIN_BITS, bits)

        # choose direction from the SIGN of v_cmd (not from e)
        desired_dir = 'FWD' if (v_cmd >= 0.0) else 'BWD'

        # gentle reversals: zero before changing direction if currently driving
        if self._last_dir and desired_dir != self._last_dir and self._last_bits > 0:
            self._send("rubber:SPEED:0")
            self._last_bits = 0
            return  # flip direction on next tick

        if desired_dir != self._last_dir:
            self._send(f"DIR:{desired_dir}:rubber")
            self._last_dir = desired_dir

        if bits != self._last_bits:
            if bits == 0:
                self._send("rubber:SPEED:0")
            else:
                self._send(f"rubber:SPEED:{bits}")
            self._last_bits = bits

        # Update what we plot (extension only)
        self._last_Lmeas = Lext_meas
        self._last_Lsp = max(0.0, Lsp_e2e - self._L0_e2e)

        # --- settle & hold after the trajectory time ---
        if t >= self._traj.T:
            if abs(e) <= self.settle_band_mm:
                if self._settle_start is None:
                    self._settle_start = now
                elif (now - self._settle_start) >= self.settle_time_s:
                    self._stop_control()
                    return
            else:
                # left the band -> reset the settle timer
                self._settle_start = None

    # ---------- tare θ0 from current length ----------
    def _tare_theta0_from_current(self):
        """Set θ0 so current E2E equals the geometry at θ0."""
        frame = self.io.snapshot() if self.io else None
        if not frame:
            print_console("[tare] No data", channel="rubber"); return
        Lext = counts_to_mm(frame.get(self.plot2_key, 0.0))
        Le2e = self._L0_e2e + Lext  # convert extension → E2E for correct inversion
        num = Le2e*Le2e - (self.A*self.A + self.B*self.B)
        den = 2.0*self.A*self.B if self.A*self.B != 0 else 1.0
        s   = max(-1.0, min(1.0, num/den))
        theta0 = math.degrees(math.asin(s))
        self.spin_theta0.setValue(theta0)
        print_console(f"[tare] θ0 set to {theta0:.2f} deg (from Le2e={Le2e:.2f} mm, L0={self._L0_e2e:.2f})",
                      channel="rubber")

    # ---------- utils ----------
    def _send(self, cmd: str):
        try:
            self.io.enqueue_command(cmd, require_ack=False)
            print_console(f">>> {cmd}", channel="rubber")
        except Exception as e:
            print_console(f"[ERROR] enqueue_command failed: {e}", channel="rubber")

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
