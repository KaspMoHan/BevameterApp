from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console
from controller.conversions import (
    adc_bits_to_voltage,
    voltage_to_dac_bits,
)

from widgets.logger_controls import LoggerControls


class ManualPressureWindow(QtWidgets.QMainWindow):
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
        self.setWindowTitle("Manual – Pressure (Const-speed Extend→Target→Retract)")

        self.io = io_worker
        self.force_key = force_key
        self.pos_key = pos_key

        # NEW: acquire shared SENSOR:READ_ALL stream at 50 Hz
        self._stream_tag = f"pressure:{id(self)}"
        if self.io:
            self.io.acquire_stream(self._stream_tag, hz=50.0)

        # State machine
        self._state = "IDLE"          # IDLE / EXTENDING / RETRACTING
        self._active_dir = None       # None / "FWD" / "BWD"
        self._last_bits = 0

        # Jog state (manual control at 20% when in Distance mode)
        self._jog_active = False
        self._jog_dir = None

        # last values for plots (store DISPLAYED values = tared)
        self._last_values = {self.force_key: 0.0, self.pos_key: 0.0}

        # ---------------- Tare offsets (GUI ONLY) ----------------
        # These offsets are applied to the plotted/displayed values and comparisons.
        # Raw bits/volts still stream + log unchanged, and OPTA failsafe still works.
        self._tare_force_N = 0.0
        self._tare_pos_mm = 0.0

        # ------------- Plots (force left, position right) -------------
        self.plot_force = LivePlotWidget(
            self, self._data_fn_for(self.force_key, kind="force"),
            update_hz=50, window_seconds=10.0,
            ymin=self.FORCE_MIN_N, ymax=self.FORCE_MAX_N,
            show_ma=True, ma_window=20,
            label="Force"
        )
        self.plot_force.ax.set_ylabel("Force [N] (tared)")
        self.plot_force.ax.grid(True, which="both", linestyle="--", alpha=0.4)

        self.plot_pos = LivePlotWidget(
            self, self._data_fn_for(self.pos_key, kind="pos"),
            update_hz=50, window_seconds=10.0,
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

        # ------------- Controls -------------
        self.mode_label = QtWidgets.QLabel("Control mode")
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Force", "Distance"])

        # Targets
        self.spin_target_force = QtWidgets.QDoubleSpinBox()
        self.spin_target_force.setRange(self.FORCE_MIN_N, self.HARD_MAX_FORCE_N)  # N (relative, tared)
        self.spin_target_force.setDecimals(0)
        self.spin_target_force.setSuffix(" N")
        self.spin_target_force.setValue(1000)

        try:
            self.spin_target_force.setKeyboardTracking(False)
        except Exception:
            pass
        self.spin_target_force.editingFinished.connect(self._enforce_force_limit)

        self.spin_target_pos = QtWidgets.QDoubleSpinBox()
        self.spin_target_pos.setRange(0, 1000)     # mm (relative, tared)
        self.spin_target_pos.setDecimals(1)
        self.spin_target_pos.setSuffix(" mm")
        self.spin_target_pos.setValue(50.0)

        # Deadbands
        self.spin_db_force = QtWidgets.QDoubleSpinBox()
        self.spin_db_force.setRange(0, 5000)
        self.spin_db_force.setDecimals(0)
        self.spin_db_force.setSuffix(" N")
        self.spin_db_force.setValue(50)

        self.spin_db_pos = QtWidgets.QDoubleSpinBox()
        self.spin_db_pos.setRange(0, 20)
        self.spin_db_pos.setDecimals(1)
        self.spin_db_pos.setSuffix(" mm")
        self.spin_db_pos.setValue(1.0)

        # Speed
        self.lbl_speed = QtWidgets.QLabel("Speed")
        self.spin_speed = QtWidgets.QDoubleSpinBox()
        self.spin_speed.setRange(0, 100)
        self.spin_speed.setDecimals(0)
        self.spin_speed.setSuffix(" %")
        self.spin_speed.setValue(40.0)

        # Run / Stop / Return
        self.btn_run    = QtWidgets.QPushButton("Run (Extend → Target → Retract)")
        self.btn_stop   = QtWidgets.QPushButton("Stop")
        self.btn_return = QtWidgets.QPushButton("Return")

        # Manual jog buttons (20% speed, Distance mode only)
        self.btn_jog_fwd = QtWidgets.QPushButton("Jog FWD (20%)")
        self.btn_jog_bwd = QtWidgets.QPushButton("Jog BWD (20%)")

        # --------- NEW: Tare buttons (GUI only) ---------
        self.btn_tare_force = QtWidgets.QPushButton("Tare Force")
        self.btn_tare_pos   = QtWidgets.QPushButton("Tare Position")
        self.btn_clear_tare = QtWidgets.QPushButton("Clear Tares")

        # Logging controls
        self.logger_controls = LoggerControls(
            parent=self,
            io_worker=self.io,
            console_channel="pressure",
            default_folder="logs",
            default_test_type="manual-pressure"
        )

        # Layout controls
        form = QtWidgets.QFormLayout()
        form.addRow(self.mode_label, self.mode_combo)
        form.addRow("Target force (tared)", self.spin_target_force)
        form.addRow("Target distance (tared)", self.spin_target_pos)
        form.addRow("Force deadband", self.spin_db_force)
        form.addRow("Pos deadband", self.spin_db_pos)
        form.addRow(self.lbl_speed, self.spin_speed)

        controls = QtWidgets.QVBoxLayout()
        controls.addLayout(form)
        controls.addSpacing(8)
        controls.addWidget(self.btn_run)

        # Logging widget
        controls.addWidget(self.logger_controls)

        # NEW: tare row
        tare_row = QtWidgets.QHBoxLayout()
        tare_row.addWidget(self.btn_tare_force)
        tare_row.addWidget(self.btn_tare_pos)
        controls.addSpacing(6)
        controls.addLayout(tare_row)
        controls.addWidget(self.btn_clear_tare)

        # jog controls
        controls.addSpacing(8)
        controls.addWidget(self.btn_jog_fwd)
        controls.addWidget(self.btn_jog_bwd)

        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_return)

        controls_widget = QWidget()
        controls_widget.setLayout(controls)

        # Top row
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot_force, stretch=1)
        hbox.addWidget(self.plot_pos, stretch=1)
        hbox.addWidget(controls_widget)
        top_widget = QWidget(); top_widget.setLayout(hbox)

        # Console
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(1000)
        font = self.console.font(); font.setFamily("Consolas"); self.console.setFont(font)
        connect_console(self.console, channel="pressure")
        print_console("Manual pressure ready (const-speed, target-based).", channel="pressure")

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)
        container = QWidget(); container.setLayout(vbox)
        self.setCentralWidget(container)

        # Wiring
        self.btn_return.clicked.connect(self.on_return)
        self.btn_run.clicked.connect(self.on_run_clicked)
        self.btn_stop.clicked.connect(self.on_stop)
        self.spin_speed.valueChanged.connect(self._on_speed_changed)

        self.btn_jog_fwd.pressed.connect(lambda: self._on_jog_pressed("FWD"))
        self.btn_jog_fwd.released.connect(self._on_jog_released)
        self.btn_jog_bwd.pressed.connect(lambda: self._on_jog_pressed("BWD"))
        self.btn_jog_bwd.released.connect(self._on_jog_released)

        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self._on_mode_changed(self.mode_combo.currentText())

        # NEW: tare wiring
        self.btn_tare_force.clicked.connect(self._tare_force_now)
        self.btn_tare_pos.clicked.connect(self._tare_pos_now)
        self.btn_clear_tare.clicked.connect(self._clear_tares)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(20)

    # ---------------- HARD LIMIT ENFORCEMENT ----------------
    def _enforce_force_limit(self):
        try:
            v = float(self.spin_target_force.value())
        except Exception:
            return
        if v > float(self.HARD_MAX_FORCE_N):
            self.spin_target_force.setValue(float(self.HARD_MAX_FORCE_N))
            print_console(f"[SAFETY] Target force capped at {self.HARD_MAX_FORCE_N:.0f} N.", channel="pressure")

    def _get_safe_force_target(self) -> float:
        tgt = float(self.spin_target_force.value())
        if tgt > float(self.HARD_MAX_FORCE_N):
            tgt = float(self.HARD_MAX_FORCE_N)
            try:
                self.spin_target_force.setValue(tgt)
            except Exception:
                pass
        return tgt

    # ---------------- GUI TARE ----------------
    def _get_raw_force_pos_from_snapshot(self):
        """Returns (force_N_abs, pos_mm_abs) computed from the latest snapshot (absolute values)."""
        if self.io is None:
            return None, None
        frame = self.io.snapshot() or {}
        f_bits = frame.get(self.force_key, None)
        p_bits = frame.get(self.pos_key, None)
        if f_bits is None or p_bits is None:
            return None, None
        try:
            force_abs = self._bits_to_force_newton(int(float(f_bits)))
            pos_abs = self._bits_to_pos_mm(int(float(p_bits)))
            return force_abs, pos_abs
        except Exception:
            return None, None

    def _tare_force_now(self):
        force_abs, _pos_abs = self._get_raw_force_pos_from_snapshot()
        if force_abs is None:
            print_console("[TARE] No force data available to tare.", channel="pressure")
            return
        # Make displayed force become ~0 by storing offset
        self._tare_force_N = float(force_abs)
        print_console(f"[TARE] Force tared at {self._tare_force_N:.1f} N.", channel="pressure")

    def _tare_pos_now(self):
        _force_abs, pos_abs = self._get_raw_force_pos_from_snapshot()
        if pos_abs is None:
            print_console("[TARE] No position data available to tare.", channel="pressure")
            return
        self._tare_pos_mm = float(pos_abs)
        print_console(f"[TARE] Position tared at {self._tare_pos_mm:.2f} mm.", channel="pressure")

    def _clear_tares(self):
        self._tare_force_N = 0.0
        self._tare_pos_mm = 0.0
        print_console("[TARE] Cleared force + position tares.", channel="pressure")

    # ---------------- Converters ----------------
    @staticmethod
    def _clamp_0_10(v: float) -> float:
        if v < 0.0: return 0.0
        if v > 10.0: return 10.0
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
        p = 0.0 if pct is None else float(pct)
        if p <= 0.0:
            return 0
        if p < 20.0: p = 20.0
        if p > 100.0: p = 100.0
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
                # Apply GUI-only tare
                if kind == "force":
                    disp = abs_val - float(self._tare_force_N)
                else:
                    disp = abs_val - float(self._tare_pos_mm)
                self._last_values[key] = disp
                return disp
            except Exception:
                return float(self._last_values.get(key, 0.0))
        return _fn

    # ---------------- Mode handling ----------------
    def _on_mode_changed(self, text: str):
        is_force = (text == "Force")
        self.spin_target_force.setEnabled(is_force)
        self.spin_db_force.setEnabled(is_force)
        self.spin_target_pos.setEnabled(not is_force)
        self.spin_db_pos.setEnabled(not is_force)
        self.btn_jog_fwd.setEnabled(not is_force)
        self.btn_jog_bwd.setEnabled(not is_force)

    # ---------------- Run/Stop logic ----------------
    def on_run_clicked(self):
        if self._state != "IDLE":
            print_console("[RUN] Already running.", channel="pressure")
            return
        if self.io is None:
            print_console("[RUN] No io_worker connected.", channel="pressure")
            return

        self._enforce_force_limit()

        self._apply_speed_from_spin()
        self._select_dir("FWD")
        self._state = "EXTENDING"
        print_console("[RUN] EXTENDING started.", channel="pressure")

    def on_stop(self):
        self._send_cmd("pressure:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self._active_dir = None
        self._last_bits = 0
        self._state = "IDLE"
        self._jog_active = False
        self._jog_dir = None
        print_console("[STOP] Motion stopped.", channel="pressure")

    def on_return(self):
        self.on_stop()
        self.close()

    # ---------------- Timer tick ----------------
    def _tick(self):
        if self._state == "IDLE" or self.io is None:
            return

        frame = self.io.snapshot() or {}
        f_bits = frame.get(self.force_key, None)
        p_bits = frame.get(self.pos_key, None)
        if f_bits is None or p_bits is None:
            return

        # Absolute computed values
        force_abs = self._bits_to_force_newton(int(float(f_bits)))
        pos_abs = self._bits_to_pos_mm(int(float(p_bits)))

        # GUI tared values used for display + logic
        force_N = float(force_abs) - float(self._tare_force_N)
        pos_mm = float(pos_abs) - float(self._tare_pos_mm)

        mode = self.mode_combo.currentText()
        f_db = float(self.spin_db_force.value())
        p_db = float(self.spin_db_pos.value())

        if self._state == "EXTENDING":
            if mode == "Force":
                tgtF = self._get_safe_force_target()  # target is interpreted as "tared"
                if force_N >= tgtF:
                    self._select_dir("BWD")
                    self._state = "RETRACTING"
                    print_console(
                        f"[RUN] Force target reached ({force_N:.0f} N ≥ {tgtF:.0f} N). RETRACTING.",
                        channel="pressure"
                    )
            else:
                tgtP = float(self.spin_target_pos.value())  # tared distance target
                if pos_mm >= tgtP:
                    self._select_dir("BWD")
                    self._state = "RETRACTING"
                    print_console(
                        f"[RUN] Distance target reached ({pos_mm:.1f} mm ≥ {tgtP:.1f} mm). RETRACTING.",
                        channel="pressure"
                    )

        elif self._state == "RETRACTING":
            # Stop when we're near "zero" in tared coordinates
            if mode == "Force":
                if abs(force_N) <= f_db:
                    self.on_stop()
                    print_console(f"[RUN] Retract complete (|F| ≤ {f_db:.0f} N).", channel="pressure")
            else:
                if pos_mm <= p_db:
                    self.on_stop()
                    print_console(f"[RUN] Retract complete (pos ≤ {p_db:.1f} mm).", channel="pressure")

    # ---------------- Helpers ----------------
    def _apply_speed_from_spin(self):
        bits = self._speed_percent_to_dac_bits(float(self.spin_speed.value()))
        if bits <= 0:
            if self._last_bits != 0:
                self._send_cmd("pressure:SPEED:0")
                self._last_bits = 0
        else:
            if bits != self._last_bits:
                self._send_cmd(f"pressure:SPEED:{bits}")
                self._last_bits = bits

    def _on_speed_changed(self, _value):
        if self._jog_active:
            print_console("[INFO] Speed changed while jogging – ignored.", channel="pressure")
            return
        self._apply_speed_from_spin()

    def _select_dir(self, dir_str: str, apply_speed: bool = True):
        if dir_str not in ("FWD", "BWD"):
            return
        if self._active_dir and self._active_dir != dir_str and self._last_bits > 0:
            self._send_cmd("pressure:SPEED:0")
            self._last_bits = 0
        self._send_cmd(f"DIR:{dir_str}:pressure")
        self._active_dir = dir_str
        if apply_speed:
            self._apply_speed_from_spin()

    def _on_jog_pressed(self, dir_str: str):
        if self.mode_combo.currentText() != "Distance":
            print_console("[JOG] Manual jog only available in Distance mode.", channel="pressure")
            return
        if self.io is None:
            print_console("[JOG] No io_worker connected.", channel="pressure")
            return
        if self._state != "IDLE":
            print_console("[JOG] Cannot jog while auto run is active.", channel="pressure")
            return

        self._jog_active = True
        self._jog_dir = dir_str
        self._select_dir(dir_str, apply_speed=False)

        bits = self._speed_percent_to_dac_bits(20.0)
        if bits > 0:
            self._send_cmd(f"pressure:SPEED:{bits}")
            self._last_bits = bits
            print_console(f"[JOG] {dir_str} at 20% speed.", channel="pressure")

    def _on_jog_released(self):
        if not self._jog_active:
            return
        self._send_cmd("pressure:SPEED:0")
        self._last_bits = 0
        self._send_cmd("ALL:DIR:OFF")
        self._active_dir = None
        self._jog_active = False
        self._jog_dir = None
        print_console("[JOG] Released – speed set to 0% and relays opened.", channel="pressure")

    def _send_cmd(self, cmd: str):
        if self.io is not None:
            try:
                self.io.enqueue_command(cmd, require_ack=True)
                print_console(f">>> {cmd}", channel="pressure")
            except Exception as e:
                print_console(f"[ERROR] enqueue_command failed: {e}", channel="pressure")
        else:
            print_console("[WARN] No io_worker connected.", channel="pressure")

    # ---------------- lifecycle ----------------
    def closeEvent(self, e):
        try:
            self.plot_force.stop()
            self.plot_pos.stop()
        except Exception:
            pass
        self.on_stop()
        if self.io:
            self.io.release_stream(self._stream_tag)
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
