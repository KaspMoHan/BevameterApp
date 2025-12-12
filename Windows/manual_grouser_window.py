from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from controller.conversions import percent_to_bits, bits_to_length
from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console


class ManualGrouserWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, parent=None, io_worker=None,
                 plot1_key="grouser_torque", plot2_key="grouser_pos"):
        super().__init__(parent)
        self.setWindowTitle("Manual – Grouser")

        self.io = io_worker
        self.plot1_key = plot1_key
        self.plot2_key = plot2_key

        # toggle state
        self._active_dir = None  # None / "FWD" / "BWD"

        # cache to avoid plot jumps
        self._last_values = {self.plot1_key: 0.0, self.plot2_key: 0.0}

        # --- plots (each LivePlotWidget calls its data_fn @ 50Hz) ---
        self.plot1 = LivePlotWidget(
            self, self._data_fn_for(self.plot1_key),
            50, 10.0,
            ymin=0, ymax=1000,
            show_ma=True, ma_window=10,
        )
        self.plot1.ax.set_ylabel("Torque [Nm]")

        self.plot2 = LivePlotWidget(
            self, self._data_fn_for(self.plot2_key),
            50, 10.0,
            ymin=0, ymax=1000,
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
        self.spin_speed.setSingleStep(5)
        self.spin_speed.setSuffix(" %")
        self.spin_speed.setValue(0)  # default 0%

        self.btn_forward   = QtWidgets.QPushButton("Forward")
        self.btn_back      = QtWidgets.QPushButton("Backwards")
        self.btn_forward.setCheckable(True)
        self.btn_back.setCheckable(True)
        self.dir_group = QtWidgets.QButtonGroup(self)
        self.dir_group.setExclusive(True)
        self.dir_group.addButton(self.btn_forward, 1)
        self.dir_group.addButton(self.btn_back, 2)

        self.btn_winch_up   = QtWidgets.QPushButton("Winch Up")
        self.btn_winch_down = QtWidgets.QPushButton("Winch Down")
        self.btn_log        = QtWidgets.QPushButton("Log")
        self.btn_stop       = QtWidgets.QPushButton("Stop")
        self.btn_return     = QtWidgets.QPushButton("Return")

        # layout
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

        # console
        self.console = QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumBlockCount(1000)
        font = self.console.font(); font.setFamily("Consolas"); self.console.setFont(font)
        connect_console(self.console, channel="grouser")
        print_console("Manual Grouser ready.", channel="grouser")

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)
        container = QWidget(); container.setLayout(vbox)
        self.setCentralWidget(container)

        # wiring
        self.btn_return.clicked.connect(self.on_return)
        self.btn_back.clicked.connect(self.BackBtn)  # log only

        self.btn_forward.toggled.connect(self._on_dir_toggled)
        self.btn_back.toggled.connect(self._on_dir_toggled)
        self.spin_speed.valueChanged.connect(self._on_speed_changed)

        self.btn_winch_up.clicked.connect(self.on_winch_up)
        self.btn_winch_down.clicked.connect(self.on_winch_down)
        self.btn_stop.clicked.connect(self.on_stop)

    # ---------- data providers ----------
    def _data_fn_for(self, key: str):
        def _fn(_t):
            if self.io is None:
                return float(self._last_values.get(key, 0.0))
            frame = self.io.snapshot() or {}
            if not frame:
                return float(self._last_values.get(key, 0.0))
            val = frame.get(key, None)
            try:
                if val is None:
                    return float(self._last_values.get(key, 0.0))
                fval = float(bits_to_length(float(val)))
                self._last_values[key] = fval
                return fval
            except (TypeError, ValueError):
                return float(self._last_values.get(key, 0.0))
        return _fn

    # ---------- motion logic ----------
    def _on_dir_toggled(self, checked: bool):
        sender = self.sender()
        if sender not in (self.btn_forward, self.btn_back):
            return
        dir_str = "FWD" if sender is self.btn_forward else "BWD"

        if checked:
            if self._active_dir and self._active_dir != dir_str:
                self._send_cmd("grouser:SPEED:0")
            self._send_cmd(f"DIR:{dir_str}:grouser")
            self._active_dir = dir_str
            self._apply_speed()
        else:
            if not self.btn_forward.isChecked() and not self.btn_back.isChecked():
                self.on_stop()

    def _on_speed_changed(self, _value):
        if self._active_dir in ("FWD", "BWD"):
            self._apply_speed()

    def _apply_speed(self):
        bits = percent_to_bits(float(self.spin_speed.value()))
        if bits <= 0:
            self._send_cmd("grouser:SPEED:0")
        else:
            self._send_cmd(f"grouser:SPEED:{bits}")

    # ---------- helpers ----------
    def _send_cmd(self, cmd: str):
        if self.io is not None:
            try:
                self.io.enqueue_command(cmd, require_ack=True)
                print_console(f">>> {cmd}", channel="grouser")
            except Exception as e:
                print_console(f"[ERROR] enqueue_command failed: {e}", channel="grouser")
        else:
            print_console("[WARN] No io_worker connected.", channel="grouser")

    def BackBtn(self):
        print_console("Back button pressed", channel="grouser")

    def on_winch_up(self):
        self._send_cmd("GROUSER:WINCH:UP")

    def on_winch_down(self):
        self._send_cmd("GROUSER:WINCH:DOWN")

    def on_stop(self):
        self._send_cmd("grouser:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self._send_cmd("GROUSER:WINCH:STOP")
        for btn in (self.btn_forward, self.btn_back):
            was = btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(was)
        self._active_dir = None
        print_console("[STOP] Motion stopped.", channel="grouser")

    def on_return(self):
        self.on_stop()
        self.close()

    # ---------- lifecycle ----------
    def closeEvent(self, e):
        try:
            self.plot1.stop(); self.plot2.stop()
        except Exception:
            pass
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
