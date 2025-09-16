from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QPlainTextEdit

from controller.conversions import percent_to_bits, bits_to_length
from widgets.live_plot import LivePlotWidget
from utils.console_print import connect_console, disconnect_console, print_console


class ManualGrouserWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, parent=None, io_worker=None, plot1_key="grouser_pos", plot2_key="rubber_pos"):
        """
        io_worker: an instance of SerialWorker (the QThread that owns serial)
        plot1_key / plot2_key: the dict keys to plot from snapshot(), e.g. "pos", "load", "ax", etc.
        """
        super().__init__(parent)
        self.setWindowTitle("Manual – Grouser")

        self.io = io_worker
        self.plot1_key = plot1_key
        self.plot2_key = plot2_key

        self.speed_percent_value = 80.0

        # remembers last valid values so the plot doesn't jump to 0 if a key is missing briefly
        self._last_values = {self.plot1_key: 0.0, self.plot2_key: 0.0}

        # --- plots (each LivePlotWidget calls its data_fn at 50 Hz) ---
        self.plot1 = LivePlotWidget(self, self._data_fn_for(self.plot1_key), 50, 10.0)
        self.plot2 = LivePlotWidget(self, self._data_fn_for(self.plot2_key), 50, 10.0)

        # --- controls ---
        self.btn_forward = QtWidgets.QPushButton("Forward")
        self.btn_back    = QtWidgets.QPushButton("Backwards")
        self.btn_return  = QtWidgets.QPushButton("Return")
        self.btn_log     = QtWidgets.QPushButton("Log")
        self.btn_stop    = QtWidgets.QPushButton("Stop")

        controls = QtWidgets.QVBoxLayout()
        controls.addWidget(self.btn_forward)
        controls.addWidget(self.btn_back)
        controls.addWidget(self.btn_return)
        controls.addStretch()
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_log)

        controls_widget = QWidget()
        controls_widget.setLayout(controls)

        # --- top row: plots + controls ---
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.plot1, stretch=1)
        hbox.addWidget(self.plot2, stretch=1)
        hbox.addWidget(controls_widget)

        top_widget = QWidget()
        top_widget.setLayout(hbox)

        # --- console output ---
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(1000)
        font = self.console.font()
        font.setFamily("Consolas")
        self.console.setFont(font)

        # subscribe the console to the "grouser" channel
        connect_console(self.console, channel="grouser")
        print_console("Manual Grouser ready.", channel="grouser")

        # --- main layout: top + console ---
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)

        container = QWidget()
        container.setLayout(vbox)
        self.setCentralWidget(container)

        # wiring
        self.btn_return.clicked.connect(self.on_return)
        self.btn_back.clicked.connect(self.BackBtn)

        # optional: wire actuator buttons to io_worker commands (safe no-op if io is None)
        self.btn_forward.clicked.connect(self.on_forward)
        self.btn_back.clicked.connect(self.on_backward)
        self.btn_stop.clicked.connect(self.on_stop)

    # ---------- Plot data providers (called by LivePlotWidget @ 50 Hz) ----------
    # in ManualGrouserWindow
    # in ManualGrouserWindow
    def _data_fn_for(self, key: str):
        def _fn(t):  # <-- accept t; ignore it for snapshot pulls
            if self.io is None:
                return float(self._last_values.get(key, 0.0))
            frame = self.io.snapshot()
            if not frame:
                return float(self._last_values.get(key, 0.0))
            val = frame.get(key, None)
            try:
                if val is None:
                    return float(self._last_values.get(key, 0.0))
                fval = bits_to_length(float(val))
                self._last_values[key] = fval
                return fval
            except (TypeError, ValueError):
                return float(self._last_values.get(key, 0.0))

        return _fn

    # ---------- UI helpers ----------
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


    def on_forward(self):
        bits = percent_to_bits(self.speed_percent_value)
        self._send_cmd("grouser:SPEED:0")  # safety: zero before switching
        self._send_cmd("DIR:FWD:grouser")
        self._send_cmd(f"grouser:SPEED:{bits}")


    def on_backward(self):
        bits = percent_to_bits(self.speed_percent_value)
        self._send_cmd("grouser:SPEED:0")
        self._send_cmd("DIR:BWD:grouser")
        self._send_cmd(f"grouser:SPEED:{bits}")

    def on_stop(self):
        self._send_cmd("grouser:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")

    def on_return(self):
        self._send_cmd("grouser:SPEED:0")
        self._send_cmd("ALL:DIR:OFF")
        self.close()


    # ---------- lifecycle ----------
    def closeEvent(self, e):
        try:
            self.plot1.stop()
            self.plot2.stop()
        except Exception:
            pass
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
