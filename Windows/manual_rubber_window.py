from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

# ✅ These map to the correct hardware paths in your updated conversions.py
from controller.conversions import percent_to_bits, adc_bits_to_length, bits_to_torque
from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console


class ManualRubberWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, parent=None, io_worker=None, plot1_key="rubber_torque", plot2_key="rubber_pos"):
        super().__init__(parent)
        self.setWindowTitle("Manual – rubber")

        self.io = io_worker
        self.plot1_key = plot1_key   # expects ADC bits (torque)
        self.plot2_key = plot2_key   # expects ADC bits (position)

        # active jog state (actuator)
        self._active_dir = None  # None / "FWD" / "BWD"

        # last values to avoid plot jumps
        self._last_values = {self.plot1_key: 0.0, self.plot2_key: 0.0}

        # --- plots ---
        self.plot1 = LivePlotWidget(
            self, self._data_fn_for(self.plot1_key),
            50, 10.0,
            ymin=0, ymax=80,
            show_ma=True, ma_window=20
        )
        self.plot1.ax.set_ylabel("Torque [Nm]")

        self.plot2 = LivePlotWidget(
            self, self._data_fn_for(self.plot2_key),
            50, 10.0,
            ymin=0, ymax=550,
            show_ma=True, ma_window=10,
            show_velocity=True, vel_window=7,
            y2min=-50, y2max=50, y2label="Velocity [mm/s]",
            show_vel_ma=True, vel_ma_window=50
        )
        self.plot2.ax.set_ylabel("Length [mm]")

        # --- controls ---
        self.lbl_speed = QtWidgets.QLabel("Speed")
        self.spin_speed = QtWidgets.QDoubleSpinBox()
        self.spin_speed.setRange(0, 100)
        self.spin_speed.setDecimals(0)
        self.spin_speed.setSuffix(" %")
        self.spin_speed.setValue(0.0)  # default 0%

        self.btn_forward = QtWidgets.QPushButton("Forward")
        self.btn_back    = QtWidgets.QPushButton("Backwards")

        # (no longer toggle)
        self.btn_forward.setCheckable(False)
        self.btn_back.setCheckable(False)

        self.btn_winch_up = QtWidgets.QPushButton("Winch Up")
        self.btn_winch_down = QtWidgets.QPushButton("Winch Down")
        self.btn_stop    = QtWidgets.QPushButton("Stop")
        self.btn_return  = QtWidgets.QPushButton("Return")
        self.btn_log     = QtWidgets.QPushButton("Log")

        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(self.lbl_speed)
        speed_row.addWidget(self.spin_speed)
        speed_row.addStretch()

        controls = QtWidgets.QVBoxLayout()
        controls.addLayout(speed_row)
        controls.addWidget(self.btn_forward)
        controls.addWidget(self.btn_back)
        controls.addWidget(self.btn_winch_up)
        controls.addWidget(self.btn_winch_down)
        controls.addWidget(self.btn_return)
        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_log)

        controls_widget = QWidget(); controls_widget.setLayout(controls)

        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot1, stretch=1)
        hbox.addWidget(self.plot2, stretch=1)
        hbox.addWidget(controls_widget)
        top_widget = QWidget(); top_widget.setLayout(hbox)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(1000)
        font = self.console.font(); font.setFamily("Consolas"); self.console.setFont(font)
        connect_console(self.console, channel="rubber")
        print_console("Manual rubber ready (JOG mode).", channel="rubber")

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)
        container = QWidget(); container.setLayout(vbox)
        self.setCentralWidget(container)

        # ensure we get focus-out events reliably
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # wiring
        self.btn_return.clicked.connect(self.on_return)
        self.btn_back.clicked.connect(self.BackBtn)  # keep your log line

        # ✅ ACTUATOR JOG (press-and-hold)
        self.btn_forward.pressed.connect(lambda: self._actuator_press("FWD"))
        self.btn_forward.released.connect(self._actuator_release)

        self.btn_back.pressed.connect(lambda: self._actuator_press("BWD"))
        self.btn_back.released.connect(self._actuator_release)

        # speed updates live while jogging
        self.spin_speed.valueChanged.connect(self._update_speed_if_active)

        # ✅ WINCH JOG: only active while pressed
        self.btn_winch_up.pressed.connect(self._winch_up_press)
        self.btn_winch_up.released.connect(self._winch_release)
        self.btn_winch_down.pressed.connect(self._winch_down_press)
        self.btn_winch_down.released.connect(self._winch_release)

        self.btn_stop.clicked.connect(self.on_stop)

    # ---------- Plot data providers ----------
    def _data_fn_for(self, key: str):
        key_l = key.lower()
        if "torque" in key_l:
            converter = bits_to_torque
        elif "pos" in key_l or "length" in key_l:
            converter = adc_bits_to_length
        else:
            converter = float

        def _fn(_t):
            if self.io is None:
                return float(self._last_values.get(key, 0.0))
            frame = self.io.snapshot() or {}
            val = frame.get(key, None)
            if val is None:
                return float(self._last_values.get(key, 0.0))
            try:
                fval = float(converter(float(val)))
                self._last_values[key] = fval
                return fval
            except Exception:
                return float(self._last_values.get(key, 0.0))
        return _fn

    # ---------- ACTUATOR (JOG) ----------
    def _actuator_press(self, direction: str):
        if self.io is None:
            print_console("[JOG] No io_worker connected.", channel="rubber")
            return
        self._active_dir = direction
        self._send_cmd(f"DIR:{direction}:rubber")
        self._apply_speed()

    def _actuator_release(self):
        # stop actuator motion immediately on release
        self._send_cmd("rubber:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self._active_dir = None

    def _update_speed_if_active(self, _value):
        if self._active_dir in ("FWD", "BWD"):
            self._apply_speed()

    def _apply_speed(self):
        pct = float(self.spin_speed.value())
        bits = percent_to_bits(pct)
        if bits <= 0:
            self._send_cmd("rubber:SPEED:0")
        else:
            self._send_cmd(f"rubber:SPEED:{bits}")

    # ---------- helpers ----------
    def _send_cmd(self, cmd: str):
        if self.io is not None:
            try:
                self.io.enqueue_command(cmd, require_ack=True)
                print_console(f">>> {cmd}", channel="rubber")
            except Exception as e:
                print_console(f"[ERROR] enqueue_command failed: {e}", channel="rubber")
        else:
            print_console("[WARN] No io_worker connected.", channel="rubber")

    def BackBtn(self):
        print_console("Back button pressed", channel="rubber")

    # ---------- WINCH (JOG) ----------
    def _winch_up_press(self):
        self._send_cmd("RUBBER:WINCH:UP")

    def _winch_down_press(self):
        self._send_cmd("RUBBER:WINCH:DOWN")

    def _winch_release(self):
        self._send_cmd("RUBBER:WINCH:STOP")

    # Safety: if the window loses focus / mouse leaves while holding, stop everything
    def focusOutEvent(self, e):
        self.on_stop()
        super().focusOutEvent(e)

    def leaveEvent(self, e):
        self.on_stop()
        super().leaveEvent(e)

    def on_stop(self):
        self._send_cmd("rubber:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self._send_cmd("RUBBER:WINCH:STOP")
        self._active_dir = None
        print_console("[STOP] Motion stopped.", channel="rubber")

    def on_return(self):
        self.on_stop()
        self.close()

    # ---------- lifecycle ----------
    def closeEvent(self, e):
        try:
            self.plot1.stop(); self.plot2.stop()
        except Exception:
            pass
        self.on_stop()
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
