import math, time
from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console
from controller.conversions import adc_bits_to_length, bits_to_torque, percent_to_bits

# NEW: reusable logger controls
from widgets.logger_controls import LoggerControls


# --- helpers/limits ---
def counts_to_mm(counts: float) -> float:
    try:
        return float(adc_bits_to_length(float(counts)))
    except Exception:
        return 0.0

BITS_MAX_10V = percent_to_bits(100.0)   # (~7447 with 10/11 cap)
MIN_BITS     = 400                      # overcome stiction
DEFAULT_DBMM = 0.2                      # position deadband [mm]


class RotaryToLinearTrajectory:
    def __init__(self, A_mm: float, B_mm: float, theta0_deg: float, delta_deg: float, duration_s: float):
        self.x = float(A_mm)
        self.r = float(B_mm)
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
        th = self._theta(t)
        return math.sqrt(self._x2 + self._r2 - 2.0 * self.x * self.r * math.cos(th))

    def dL_dt(self, t: float, dtheta_dt: float = None) -> float:
        th = self._theta(t)
        L  = math.sqrt(self._x2 + self._r2 - 2.0 * self.x * self.r * math.cos(th))
        if L <= 1e-9:
            return 0.0
        omega = self.omega if (dtheta_dt is None) else float(dtheta_dt)
        return (self.x * self.r * math.sin(th) / L) * omega


class PID:
    def __init__(self, kp=3.0, ki=0.0, kd=0.0, umin=-1e6, umax=1e6):
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

    def __init__(self, parent=None, io_worker=None,
                 A_mm=960, B_mm=248.0,
                 plot1_key="grouser_torque",
                 plot2_key="grouser_pos",
                 L0_e2e_mm=820.0,
                 auto_theta_from_L0=True):
        super().__init__(parent)
        self.setWindowTitle("auto – Grouser")

        self.io = io_worker
        self.A, self.B = float(A_mm), float(B_mm)
        self.plot1_key = plot1_key
        self.plot2_key = plot2_key

        self.L0_e2e_user = float(L0_e2e_mm)
        self.auto_theta_from_L0 = bool(auto_theta_from_L0)
        self._L0_e2e = self.L0_e2e_user

        self._last = {self.plot1_key: 0.0, self.plot2_key: 0.0}
        self._last_Lsp = 0.0
        self._last_Lmeas = 0.0

        # --- plots ---
        self.plot1 = LivePlotWidget(
            self, self._data_fn_for(self.plot1_key),
            update_hz=50, window_seconds=12.0,
            ymin=0, ymax=50, show_ma=True, ma_window=20, label="Torque"
        )
        self.plot1.ax.set_ylabel("Torque [Nm]")

        self.plot2 = LivePlotWidget(
            self, self._data_fn_for(self.plot2_key),
            update_hz=50, window_seconds=12.0,
            ymin=0, ymax=550,
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
        self.btn_stop   = QtWidgets.QPushButton("Stop")

        # NEW: Start/Stop logging controls (CSV)
        self.logger_controls = LoggerControls(
            parent=self,
            io_worker=self.io,
            console_channel="grouser",
            default_folder="logs",
            default_test_type="auto-grouser"
        )

        self.spin_deg    = QtWidgets.QDoubleSpinBox(); self.spin_deg.setRange(-3600, 3600); self.spin_deg.setValue(90.0)
        self.spin_time   = QtWidgets.QDoubleSpinBox(); self.spin_time.setRange(0.1, 300.0); self.spin_time.setValue(30.0)
        self.spin_theta0 = QtWidgets.QDoubleSpinBox(); self.spin_theta0.setRange(-3600, 3600); self.spin_theta0.setValue(0.0)
        self.kp = QtWidgets.QDoubleSpinBox(); self.kp.setRange(0, 1000); self.kp.setDecimals(3); self.kp.setValue(3.0)
        self.ki = QtWidgets.QDoubleSpinBox(); self.ki.setRange(0, 1000); self.ki.setDecimals(3); self.ki.setValue(0.0)
        self.kd = QtWidgets.QDoubleSpinBox(); self.kd.setRange(0, 1000); self.kd.setDecimals(3); self.kd.setValue(0.0)
        self.kff = QtWidgets.QDoubleSpinBox(); self.kff.setRange(0, 1000); self.kff.setDecimals(3); self.kff.setValue(1.0)
        self.scale_bits = QtWidgets.QDoubleSpinBox(); self.scale_bits.setRange(0, 10000); self.scale_bits.setValue(200.0)
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
        controls.addWidget(self.logger_controls)  # NEW
        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls_widget = QWidget(); controls_widget.setLayout(controls)

        # --- layout ---
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot1, stretch=1)
        hbox.addWidget(self.plot2, stretch=1)
        hbox.addWidget(controls_widget)
        top_widget = QWidget(); top_widget.setLayout(hbox)

        self.console = QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumBlockCount(1000)
        font = self.console.font(); font.setFamily("Consolas"); self.console.setFont(font)
        connect_console(self.console, channel="grouser")
        print_console("auto Grouser ready.", channel="grouser")

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

        self.settle_band_mm = 0.5
        self.settle_time_s  = 0.3
        self._settle_start  = None

        # zeroing
        self.zero_target_mm   = 0.0
        self.zero_speed_pct   = 80.0
        self.zero_deadband_mm = 0.2
        self.zero_timeout_s   = 30.0
        self._zeroing      = False
        self._zero_dir     = None
        self._zero_started = 0.0
        self._zero_timer   = QtCore.QTimer(self)
        self._zero_timer.timeout.connect(self._zero_tick)

        # 100 Hz control loop
        self._timer = QtCore.QTimer(self); self._timer.timeout.connect(self._tick); self._timer.start(10)

        # NEW: acquire shared sensor stream @ 50 Hz while this window is open
        self._stream_tag = f"grouser:{id(self)}"
        if self.io is not None and hasattr(self.io, "acquire_stream"):
            self.io.acquire_stream(self._stream_tag, hz=50.0)

    # ---------- data fns ----------
    def _data_fn_for(self, key: str):
        kl = key.lower()
        if "torque" in kl:
            conv = bits_to_torque
        elif "pos" in kl or "length" in kl:
            conv = adc_bits_to_length
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
                if key == self.plot2_key:
                    self._last_Lmeas = fv
                return fv
            except Exception:
                return float(self._last.get(key, 0.0))
        return _fn

    def _plot_sp(self, _t):
        return float(self._last_Lsp)

    # ---------- lifecycle ----------
    def _start_control(self):
        if self.io is None:
            print_console("[AUTO] No io_worker; cannot run.", channel="grouser"); return
        if self._zeroing:
            self._finish_zeroing(say="Stopped zeroing to start AUTO.")

        if self.auto_theta_from_L0:
            try:
                c0 = (self.A*self.A + self.B*self.B - self.L0_e2e_user*self.L0_e2e_user) / (2.0*self.A*self.B)
                c0 = max(-1.0, min(1.0, c0))
                self.spin_theta0.setValue(math.degrees(math.acos(c0)))
            except Exception:
                pass

        self._traj = RotaryToLinearTrajectory(
            A_mm=self.A, B_mm=self.B,
            theta0_deg=self.spin_theta0.value(),
            delta_deg=self.spin_deg.value(),
            duration_s=self.spin_time.value()
        )

        self._L0_e2e = float(self.L0_e2e_user)
        print_console(f"[AUTO] Using L0 = {self._L0_e2e:.1f} mm (user-measured).", channel="grouser")

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

    # ---------- ZERO (no PID) ----------
    def _zero_actuator(self):
        if self.io is None:
            print_console("[ZERO] No io_worker.", channel="grouser"); return
        frame = self.io.snapshot() or {}
        counts = frame.get(self.plot2_key, None)
        if counts is None:
            print_console("[ZERO] No measurement yet.", channel="grouser"); return

        if self._running:
            self._running = False
            print_console("[ZERO] Paused AUTO while zeroing.", channel="grouser")

        Lmeas = counts_to_mm(counts)
        e = Lmeas - self.zero_target_mm
        if abs(e) <= self.zero_deadband_mm:
            print_console(f"[ZERO] Already within {self.zero_deadband_mm} mm (L={Lmeas:.2f}).", channel="grouser")
            return

        bits = max(MIN_BITS, percent_to_bits(self.zero_speed_pct))
        desired_dir = 'BWD' if e > 0 else 'FWD'

        self._send("grouser:SPEED:0")
        self._send(f"DIR:{desired_dir}:grouser")
        self._send(f"grouser:SPEED:{bits}")

        self._zeroing = True
        self._zero_dir = desired_dir
        self._zero_started = time.perf_counter()
        self._zero_timer.start(20)
        print_console(f"[ZERO] Moving {desired_dir} @ {self.zero_speed_pct:.0f}% toward {self.zero_target_mm} mm (L={Lmeas:.2f}).",
                      channel="grouser")

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
        self._send("grouser:SPEED:0")
        self._send("ALL:DIR:OFF")
        self._zeroing = False
        self._zero_dir = None
        if say:
            print_console(f"[ZERO] {say}", channel="grouser")

    # ---------- control tick @ 100 Hz ----------
    def _tick(self):
        if self._zeroing or not self._running or self._traj is None or self.io is None:
            return

        now = time.perf_counter()
        t = now - (self._t0 or now)

        Lsp_e2e = self._traj.length(t)
        Lsp_dot = self._traj.dL_dt(t)

        frame = self.io.snapshot() or {}
        counts = frame.get(self.plot2_key, None)
        if counts is None:
            return
        Lext_meas = counts_to_mm(counts)

        Lsp_ext = max(0.0, Lsp_e2e - self._L0_e2e)
        e = Lsp_ext - Lext_meas

        v_pid = self._pid.step(e, dt=0.01)
        v_ff  = float(self.kff.value()) * float(Lsp_dot)
        v_cmd = v_ff + v_pid

        scale = float(self.scale_bits.value())
        bits = int(min(BITS_MAX_10V, max(0.0, abs(v_cmd) * scale)))

        if abs(e) < float(self.deadband.value()):
            bits = 0
        elif bits > 0:
            bits = max(MIN_BITS, bits)

        desired_dir = 'FWD' if (v_cmd >= 0.0) else 'BWD'

        if self._last_dir and desired_dir != self._last_dir and self._last_bits > 0:
            self._send("grouser:SPEED:0")
            self._last_bits = 0
            return

        if desired_dir != self._last_dir:
            self._send(f"DIR:{desired_dir}:grouser")
            self._last_dir = desired_dir

        if bits != self._last_bits:
            if bits == 0:
                self._send("grouser:SPEED:0")
            else:
                self._send(f"grouser:SPEED:{bits}")
            self._last_bits = bits

        self._last_Lmeas = Lext_meas
        self._last_Lsp   = Lsp_ext

        if t >= self._traj.T:
            if abs(e) <= self.settle_band_mm:
                if getattr(self, "_settle_start", None) is None:
                    self._settle_start = now
                elif (now - self._settle_start) >= self.settle_time_s:
                    self._stop_control()
                    return
            else:
                self._settle_start = None

    def _tare_theta0_from_current(self):
        frame = self.io.snapshot() if self.io else None
        if not frame:
            print_console("[tare] No data", channel="grouser"); return
        Lext = counts_to_mm(frame.get(self.plot2_key, 0.0))
        Le2e = self._L0_e2e + Lext
        num = (self.A*self.A + self.B*self.B) - (Le2e*Le2e)
        den = 2.0*self.A*self.B if self.A*self.B != 0 else 1.0
        c   = max(-1.0, min(1.0, num/den))
        theta0 = math.degrees(math.acos(c))
        self.spin_theta0.setValue(theta0)
        print_console(f"[tare] θ0 set to {theta0:.2f} deg (from Le2e={Le2e:.2f} mm, L0={self._L0_e2e:.2f})",
                      channel="grouser")

    # ---------- utils ----------
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
        # NEW: release shared sensor stream and stop logger if it’s still running
        try:
            if self.io is not None and hasattr(self.io, "release_stream"):
                self.io.release_stream(self._stream_tag)
        except Exception:
            pass
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
