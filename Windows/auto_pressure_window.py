from __future__ import annotations

from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console
from controller.conversions import adc_bits_to_voltage, voltage_to_dac_bits

# Optional (if you want same logger UX as other windows)
try:
    from widgets.logger_controls import LoggerControls
except Exception:
    LoggerControls = None


class AutoPressureWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    # ---------------- Load cell calibration ----------------
    # Your fit: V_in = 0.0039 * kg + 4.9556
    CAL_M_V_PER_KG: float = 0.0039
    CAL_B_V: float = 4.9556
    G_N_PER_KG: float = 9.80665

    # Expected use range (plot)
    FORCE_MIN_N: float = -1_000.0   # ~ -1 kN tension
    FORCE_MAX_N: float = 10_000.0   # ~ +10 kN compression

    # HARD SAFETY LIMIT (user can never command above this)
    HARD_MAX_FORCE_N: float = 8_000.0

    def __init__(self, parent=None, io_worker=None,
                 force_key="pressure_force", pos_key="pressure_pos"):
        super().__init__(parent)
        self.setWindowTitle("auto – Pressure (F1/F2/F3 cycles)")

        self.io = io_worker
        self.force_key = force_key
        self.pos_key = pos_key

        # Acquire shared SENSOR:READ_ALL stream at 50 Hz while this window is open
        self._stream_tag = f"pressure_auto:{id(self)}"
        if self.io and hasattr(self.io, "acquire_stream"):
            self.io.acquire_stream(self._stream_tag, hz=50.0)

        # ---------------- State machine ----------------
        # Sequence: CAPTURE_P1 -> DOWN_TO_F1 -> WAIT -> UP_TO_P1 -> WAIT -> ... -> DONE
        self._state = "IDLE"
        self._active_dir = None
        self._last_speed_bits = 0

        self._P1_abs_mm = None   # absolute position reference (not tared)
        self._step_index = 0     # 0..2 for F1/F2/F3
        self._wait_until_ms = 0

        # For plots (store DISPLAYED values = tared)
        self._last_values = {self.force_key: 0.0, self.pos_key: 0.0}

        # GUI-only tare offsets (do NOT affect logging/OPTA failsafe)
        self._tare_force_N = 0.0
        self._tare_pos_mm = 0.0

        # ---------------- Plots ----------------
        self.plot_force = LivePlotWidget(
            self, self._data_fn_for(self.force_key, kind="force"),
            update_hz=50, window_seconds=12.0,
            ymin=self.FORCE_MIN_N, ymax=self.FORCE_MAX_N,
            show_ma=True, ma_window=20,
            label="Force"
        )
        self.plot_force.ax.set_ylabel("Force [N] (tared)")
        self.plot_force.ax.grid(True, which="both", linestyle="--", alpha=0.4)

        self.plot_pos = LivePlotWidget(
            self, self._data_fn_for(self.pos_key, kind="pos"),
            update_hz=50, window_seconds=12.0,
            ymin=0, ymax=1000,
            show_ma=True, ma_window=10,
            show_velocity=True, vel_window=7,
            show_vel_ma=True, vel_ma_window=20,
            y2min=-50, y2max=50, y2label="Velocity [mm/s]",
            label="Position", show_legend=True
        )
        self.plot_pos.ax.set_ylabel("Position [mm] (tared)")
        self.plot_pos.ax.grid(True, which="both", linestyle="--", alpha=0.4)
        try:
            if getattr(self.plot_pos, "ax2", None) is not None:
                self.plot_pos.ax2.grid(True, which="both", linestyle="--", alpha=0.25)
        except Exception:
            pass

        # ---------------- Controls ----------------
        # Speed (%)
        self.spin_speed = QtWidgets.QDoubleSpinBox()
        self.spin_speed.setRange(0, 100)
        self.spin_speed.setDecimals(0)
        self.spin_speed.setSuffix(" %")
        self.spin_speed.setValue(40.0)

        # Wait time (s)
        self.spin_wait = QtWidgets.QDoubleSpinBox()
        self.spin_wait.setRange(0.0, 30.0)
        self.spin_wait.setDecimals(1)
        self.spin_wait.setSingleStep(0.5)
        self.spin_wait.setSuffix(" s")
        self.spin_wait.setValue(3.0)

        # Targets (tared force targets)
        self.spin_F1 = QtWidgets.QDoubleSpinBox()
        self.spin_F2 = QtWidgets.QDoubleSpinBox()
        self.spin_F3 = QtWidgets.QDoubleSpinBox()
        for sp, default in ((self.spin_F1, 1000.0), (self.spin_F2, 3000.0), (self.spin_F3, 5000.0)):
            sp.setRange(self.FORCE_MIN_N, self.HARD_MAX_FORCE_N)
            sp.setDecimals(0)
            sp.setSuffix(" N")
            sp.setValue(default)
            try:
                sp.setKeyboardTracking(False)
            except Exception:
                pass
            sp.editingFinished.connect(self._enforce_force_limits)

        # Deadbands
        self.spin_db_force = QtWidgets.QDoubleSpinBox()
        self.spin_db_force.setRange(0, 5000)
        self.spin_db_force.setDecimals(0)
        self.spin_db_force.setSuffix(" N")
        self.spin_db_force.setValue(50)

        self.spin_db_pos = QtWidgets.QDoubleSpinBox()
        self.spin_db_pos.setRange(0, 50)
        self.spin_db_pos.setDecimals(2)
        self.spin_db_pos.setSuffix(" mm")
        self.spin_db_pos.setValue(0.5)

        # Buttons
        self.btn_start = QtWidgets.QPushButton("Start Auto (P1 → F1/F2/F3 cycles)")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_return = QtWidgets.QPushButton("Return")

        # Tare buttons (GUI only)
        self.btn_tare_force = QtWidgets.QPushButton("Tare Force")
        self.btn_tare_pos = QtWidgets.QPushButton("Tare Position")
        self.btn_clear_tare = QtWidgets.QPushButton("Clear Tares")

        # Optional logger controls
        self.logger_controls = None
        if LoggerControls is not None:
            self.logger_controls = LoggerControls(
                parent=self,
                io_worker=self.io,
                console_channel="pressure",
                default_folder="logs",
                default_test_type="auto-pressure"
            )

        # Layout
        form = QtWidgets.QFormLayout()
        form.addRow("Speed", self.spin_speed)
        form.addRow("Wait between steps", self.spin_wait)
        form.addRow("F1 target (tared)", self.spin_F1)
        form.addRow("F2 target (tared)", self.spin_F2)
        form.addRow("F3 target (tared)", self.spin_F3)
        form.addRow("Force deadband", self.spin_db_force)
        form.addRow("Pos deadband (return)", self.spin_db_pos)

        controls = QtWidgets.QVBoxLayout()
        controls.addLayout(form)
        controls.addSpacing(8)
        controls.addWidget(self.btn_start)

        if self.logger_controls is not None:
            controls.addWidget(self.logger_controls)

        tare_row = QtWidgets.QHBoxLayout()
        tare_row.addWidget(self.btn_tare_force)
        tare_row.addWidget(self.btn_tare_pos)
        controls.addSpacing(6)
        controls.addLayout(tare_row)
        controls.addWidget(self.btn_clear_tare)

        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_return)

        controls_widget = QWidget()
        controls_widget.setLayout(controls)

        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot_force, stretch=1)
        hbox.addWidget(self.plot_pos, stretch=1)
        hbox.addWidget(controls_widget)
        top_widget = QWidget()
        top_widget.setLayout(hbox)

        # Console
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(1000)
        font = self.console.font()
        font.setFamily("Consolas")
        self.console.setFont(font)
        connect_console(self.console, channel="pressure")
        print_console("auto pressure ready.", channel="pressure")

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)
        container = QWidget()
        container.setLayout(vbox)
        self.setCentralWidget(container)

        # Wiring
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_return.clicked.connect(self.on_return)

        self.btn_tare_force.clicked.connect(self._tare_force_now)
        self.btn_tare_pos.clicked.connect(self._tare_pos_now)
        self.btn_clear_tare.clicked.connect(self._clear_tares)

        # Control loop (50 Hz supervision)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(20)

    # ---------------- Converters ----------------
    @staticmethod
    def _clamp_0_10(v: float) -> float:
        if v < 0.0:
            return 0.0
        if v > 10.0:
            return 10.0
        return v

    @classmethod
    def _bits_to_force_newton(cls, bits: int) -> float:
        v = cls._clamp_0_10(float(adc_bits_to_voltage(int(bits))))
        m = float(cls.CAL_M_V_PER_KG)
        b = float(cls.CAL_B_V)
        if abs(m) < 1e-12:
            return 0.0
        kg = (v - b) / m
        return kg * float(cls.G_N_PER_KG)

    @classmethod
    def _bits_to_pos_mm(cls, bits: int) -> float:
        v = float(adc_bits_to_voltage(int(bits)))
        return (v - 0.38) * 336

    @staticmethod
    def _speed_percent_to_dac_bits(pct: float) -> int:
        """
        0%→0 V (stop), 20%→0.5 V, 100%→5.0 V (linear between 20–100%).
        """
        p = 0.0 if pct is None else float(pct)
        if p <= 0.0:
            return 0
        if p < 20.0:
            p = 20.0
        if p > 100.0:
            p = 100.0
        v = 0.5 + (p - 20.0) * (5.0 - 0.5) / (100.0 - 20.0)
        return int(voltage_to_dac_bits(v))

    # ---------------- Plot data providers ----------------
    def _data_fn_for(self, key: str, kind: str):
        conv_abs = self._bits_to_force_newton if kind == "force" else self._bits_to_pos_mm

        def _fn(_t):
            if self.io is None:
                return float(self._last_values.get(key, 0.0))

            frame = self.io.snapshot() or {}
            val = frame.get(key, None)
            if val is None:
                return float(self._last_values.get(key, 0.0))

            try:
                abs_val = float(conv_abs(int(float(val))))
                if kind == "force":
                    disp = abs_val - float(self._tare_force_N)
                else:
                    disp = abs_val - float(self._tare_pos_mm)
                self._last_values[key] = disp
                return disp
            except Exception:
                return float(self._last_values.get(key, 0.0))

        return _fn

    # ---------------- GUI tare ----------------
    def _get_abs_force_pos(self):
        if self.io is None:
            return None, None
        frame = self.io.snapshot() or {}
        f_bits = frame.get(self.force_key, None)
        p_bits = frame.get(self.pos_key, None)
        if f_bits is None or p_bits is None:
            return None, None
        try:
            f_abs = self._bits_to_force_newton(int(float(f_bits)))
            p_abs = self._bits_to_pos_mm(int(float(p_bits)))
            return float(f_abs), float(p_abs)
        except Exception:
            return None, None

    def _tare_force_now(self):
        f_abs, _p_abs = self._get_abs_force_pos()
        if f_abs is None:
            print_console("[TARE] No force data available.", channel="pressure")
            return
        self._tare_force_N = float(f_abs)
        print_console(f"[TARE] Force tared at {self._tare_force_N:.1f} N.", channel="pressure")

    def _tare_pos_now(self):
        _f_abs, p_abs = self._get_abs_force_pos()
        if p_abs is None:
            print_console("[TARE] No position data available.", channel="pressure")
            return
        self._tare_pos_mm = float(p_abs)
        print_console(f"[TARE] Position tared at {self._tare_pos_mm:.2f} mm.", channel="pressure")

    def _clear_tares(self):
        self._tare_force_N = 0.0
        self._tare_pos_mm = 0.0
        print_console("[TARE] Cleared force + position tares.", channel="pressure")

    # ---------------- Safety caps ----------------
    def _enforce_force_limits(self):
        for sp in (self.spin_F1, self.spin_F2, self.spin_F3):
            try:
                v = float(sp.value())
            except Exception:
                continue
            if v > float(self.HARD_MAX_FORCE_N):
                sp.setValue(float(self.HARD_MAX_FORCE_N))
        # no print spam; keep it quiet

    def _targets_list(self) -> list[float]:
        self._enforce_force_limits()
        return [float(self.spin_F1.value()), float(self.spin_F2.value()), float(self.spin_F3.value())]

    # ---------------- Commands ----------------
    def _send(self, cmd: str, require_ack: bool = True):
        if self.io is None:
            return
        try:
            self.io.enqueue_command(cmd, require_ack=require_ack)
            print_console(f">>> {cmd}", channel="pressure")
        except Exception as e:
            print_console(f"[ERROR] enqueue_command failed: {e}", channel="pressure")

    def _apply_speed(self):
        bits = self._speed_percent_to_dac_bits(float(self.spin_speed.value()))
        if bits <= 0:
            if self._last_speed_bits != 0:
                self._send("pressure:SPEED:0")
                self._last_speed_bits = 0
        else:
            if bits != self._last_speed_bits:
                self._send(f"pressure:SPEED:{bits}")
                self._last_speed_bits = bits

    def _set_dir(self, dir_str: str):
        if dir_str not in ("FWD", "BWD"):
            return
        # gentle reversal: set speed 0 before switching if currently driving
        if self._active_dir and self._active_dir != dir_str and self._last_speed_bits > 0:
            self._send("pressure:SPEED:0")
            self._last_speed_bits = 0
        self._send(f"DIR:{dir_str}:pressure")
        self._active_dir = dir_str
        self._apply_speed()

    def _stop_motion(self):
        self._send("pressure:SPEED:0")
        self._send("ALL:DIR:OFF")
        self._active_dir = None
        self._last_speed_bits = 0

    # ---------------- UI actions ----------------
    def on_start(self):
        if self.io is None:
            print_console("[AUTO] No io_worker connected.", channel="pressure")
            return
        if self._state != "IDLE":
            print_console("[AUTO] Already running.", channel="pressure")
            return

        # Capture P1 from absolute position (not tared)
        f_abs, p_abs = self._get_abs_force_pos()
        if p_abs is None:
            print_console("[AUTO] No position reading yet; cannot start.", channel="pressure")
            return

        self._P1_abs_mm = float(p_abs)
        self._step_index = 0
        self._state = "DOWN_TO_TARGET"
        self._wait_until_ms = 0

        targets = self._targets_list()
        print_console(
            f"[AUTO] Start: P1_abs={self._P1_abs_mm:.2f} mm. Targets: {targets[0]:.0f}, {targets[1]:.0f}, {targets[2]:.0f} N (tared).",
            channel="pressure"
        )

        # Start moving down towards first target
        self._set_dir("FWD")

    def on_stop(self):
        self._stop_motion()
        self._state = "IDLE"
        self._P1_abs_mm = None
        self._step_index = 0
        self._wait_until_ms = 0
        print_console("[STOP] Auto stopped.", channel="pressure")

    def on_return(self):
        self.on_stop()
        self.close()

    # ---------------- Control loop ----------------
    def _tick(self):
        if self._state == "IDLE" or self.io is None:
            return

        frame = self.io.snapshot() or {}
        f_bits = frame.get(self.force_key, None)
        p_bits = frame.get(self.pos_key, None)
        if f_bits is None or p_bits is None:
            return

        # absolute sensor values
        force_abs = self._bits_to_force_newton(int(float(f_bits)))
        pos_abs = self._bits_to_pos_mm(int(float(p_bits)))

        # tared values for comparisons + display
        force_t = float(force_abs) - float(self._tare_force_N)
        pos_t = float(pos_abs) - float(self._tare_pos_mm)

        # wait handling
        now_ms = int(QtCore.QDateTime.currentMSecsSinceEpoch())
        if self._state == "WAIT":
            if now_ms >= int(self._wait_until_ms):
                # transition to next motion step
                if self._next_after_wait == "UP_TO_P1":
                    self._state = "UP_TO_P1"
                    self._set_dir("BWD")
                    print_console("[AUTO] Return to P1 started.", channel="pressure")
                elif self._next_after_wait == "DOWN_TO_TARGET":
                    self._state = "DOWN_TO_TARGET"
                    self._set_dir("FWD")
                    tgt = self._targets_list()[self._step_index]
                    print_console(f"[AUTO] Step {self._step_index+1}: Down to target {tgt:.0f} N started.", channel="pressure")
                elif self._next_after_wait == "DONE":
                    self.on_stop()
                    print_console("[AUTO] Sequence complete.", channel="pressure")
                return
            else:
                return

        # Safety: if P1 was lost somehow
        if self._P1_abs_mm is None:
            self.on_stop()
            print_console("[AUTO] Missing P1 reference; stopped.", channel="pressure")
            return

        wait_s = float(self.spin_wait.value())
        dbF = float(self.spin_db_force.value())
        dbP = float(self.spin_db_pos.value())
        targets = self._targets_list()

        # ---------------- DOWN phase ----------------
        if self._state == "DOWN_TO_TARGET":
            tgt = float(targets[self._step_index])

            # Hit target (with optional deadband)
            if force_t >= (tgt - dbF):
                self._stop_motion()
                print_console(
                    f"[AUTO] Hit F{self._step_index+1} target: {force_t:.0f} N (tared). Waiting {wait_s:.1f}s.",
                    channel="pressure"
                )
                self._state = "WAIT"
                self._next_after_wait = "UP_TO_P1"
                self._wait_until_ms = now_ms + int(wait_s * 1000.0)
                return

            # keep commanding speed (in case user changed speed mid-run)
            self._apply_speed()

        # ---------------- UP phase ----------------
        elif self._state == "UP_TO_P1":
            # return to absolute P1 (not tared)
            if abs(float(pos_abs) - float(self._P1_abs_mm)) <= dbP:
                self._stop_motion()
                print_console(
                    f"[AUTO] Returned to P1: pos_abs={pos_abs:.2f} mm (P1={self._P1_abs_mm:.2f}). Waiting {wait_s:.1f}s.",
                    channel="pressure"
                )

                # decide next step
                self._step_index += 1
                if self._step_index >= 3:
                    self._state = "WAIT"
                    self._next_after_wait = "DONE"
                    self._wait_until_ms = now_ms + int(wait_s * 1000.0)
                else:
                    self._state = "WAIT"
                    self._next_after_wait = "DOWN_TO_TARGET"
                    self._wait_until_ms = now_ms + int(wait_s * 1000.0)
                return

            self._apply_speed()

    # ---------------- lifecycle ----------------
    def closeEvent(self, e):
        try:
            self.plot_force.stop()
            self.plot_pos.stop()
        except Exception:
            pass

        try:
            self.on_stop()
        except Exception:
            pass

        try:
            if self.io and hasattr(self.io, "release_stream"):
                self.io.release_stream(self._stream_tag)
        except Exception:
            pass

        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
