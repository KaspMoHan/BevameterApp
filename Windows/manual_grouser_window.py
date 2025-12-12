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

        # active jog state
        self._active_dir = None  # None / "FWD" / "BWD"

        self._last_values = {self.plot1_key: 0.0, self.plot2_key: 0.0}

        # --- plots ---
        self.plot1 = LivePlotWidget(
            self, self._data_fn_for(self.plot1_key),
            50, 10.0, ymin=0, ymax=1000, show_ma=True, ma_window=10
        )
        self.plot1.ax.set_ylabel("Torque [Nm]")

        self.plot2 = LivePlotWidget(
            self, self._data_fn_for(self.plot2_key),
            50, 10.0, ymin=0, ymax=1000,
            show_ma=True, ma_window=10,
            show_velocity=True, vel_window=7,
            y2min=-50, y2max=50, y2label="Velocity [mm/s]",
            show_vel_ma=True, vel_ma_window=50
        )
        self.plot2.ax.set_ylabel("Length [mm]")

        # --- controls ---
        self.spin_speed = QtWidgets.QDoubleSpinBox()
        self.spin_speed.setRange(0, 100)
        self.spin_speed.setDecimals(0)
        self.spin_speed.setSingleStep(5)
        self.spin_speed.setSuffix(" %")
        self.spin_speed.setValue(0)

        self.btn_forward = QtWidgets.QPushButton("Forward")
        self.btn_back = QtWidgets.QPushButton("Backwards")

        self.btn_winch_up = QtWidgets.QPushButton("Winch Up")
        self.btn_winch_down = QtWidgets.QPushButton("Winch Down")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_return = QtWidgets.QPushButton("Return")
        self.btn_log = QtWidgets.QPushButton("Log")

        # layout
        controls = QtWidgets.QVBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Speed"))
        controls.addWidget(self.spin_speed)
        controls.addWidget(self.btn_forward)
        controls.addWidget(self.btn_back)
        controls.addWidget(self.btn_winch_up)
        controls.addWidget(self.btn_winch_down)
        controls.addWidget(self.btn_return)
        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_log)

        controls_widget = QWidget()
        controls_widget.setLayout(controls)

        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot1, 1)
        hbox.addWidget(self.plot2, 1)
        hbox.addWidget(controls_widget)

        top_widget = QWidget()
        top_widget.setLayout(hbox)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(1000)
        font = self.console.font()
        font.setFamily("Consolas")
        self.console.setFont(font)
        connect_console(self.console, channel="grouser")

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console)
        container = QWidget()
        container.setLayout(vbox)
        self.setCentralWidget(container)

        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # ---------- wiring ----------
        self.spin_speed.valueChanged.connect(self._update_speed_if_active)

        # ACTUATOR JOG
        self.btn_forward.pressed.connect(lambda: self._actuator_press("FWD"))
        self.btn_forward.released.connect(self._actuator_release)

        self.btn_back.pressed.connect(lambda: self._actuator_press("BWD"))
        self.btn_back.released.connect(self._actuator_release)

        # WINCH JOG
        self.btn_winch_up.pressed.connect(lambda: self._send_cmd("GROUSER:WINCH:UP"))
        self.btn_winch_up.released.connect(lambda: self._send_cmd("GROUSER:WINCH:STOP"))

        self.btn_winch_down.pressed.connect(lambda: self._send_cmd("GROUSER:WINCH:DOWN"))
        self.btn_winch_down.released.connect(lambda: self._send_cmd("GROUSER:WINCH:STOP"))

        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_return.clicked.connect(self.on_return)

        print_console("Manual Grouser ready (JOG mode).", channel="grouser")

    # ---------- data ----------
    def _data_fn_for(self, key):
        def _fn(_t):
            if not self.io:
                return self._last_values.get(key, 0.0)
            frame = self.io.snapshot() or {}
            val = frame.get(key)
            try:
                fval = float(bits_to_length(float(val)))
                self._last_values[key] = fval
                return fval
            except Exception:
                return self._last_values.get(key, 0.0)
        return _fn

    # ---------- ACTUATOR JOG ----------
    def _actuator_press(self, direction):
        self._active_dir = direction
        self._send_cmd(f"DIR:{direction}:grouser")
        self._apply_speed()

    def _actuator_release(self):
        self._send_cmd("grouser:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self._active_dir = None

    def _update_speed_if_active(self, _):
        if self._active_dir:
            self._apply_speed()

    def _apply_speed(self):
        bits = percent_to_bits(self.spin_speed.value())
        if bits <= 0:
            self._send_cmd("grouser:SPEED:0")
        else:
            self._send_cmd(f"grouser:SPEED:{bits}")

    # ---------- helpers ----------
    def _send_cmd(self, cmd):
        if not self.io:
            return
        try:
            self.io.enqueue_command(cmd, require_ack=True)
            print_console(f">>> {cmd}", channel="grouser")
        except Exception as e:
            print_console(f"[ERROR] {e}", channel="grouser")

    # ---------- SAFETY ----------
    def focusOutEvent(self, e):
        self.on_stop()
        super().focusOutEvent(e)

    def leaveEvent(self, e):
        self.on_stop()
        super().leaveEvent(e)

    def on_stop(self):
        self._send_cmd("grouser:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self._send_cmd("GROUSER:WINCH:STOP")
        self._active_dir = None
        print_console("[STOP] All motion stopped.", channel="grouser")

    def on_return(self):
        self.on_stop()
        self.close()

    def closeEvent(self, e):
        try:
            self.plot1.stop()
            self.plot2.stop()
        except Exception:
            pass
        self.on_stop()
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
