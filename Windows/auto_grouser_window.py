from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget, QPlainTextEdit

from widgets.live_plot import LivePlotWidget
from utils.DataGenerator import generate_next_sine
from utils.console_print import connect_console, disconnect_console, print_console


class autoGrouserWindow(QtWidgets.QMainWindow):
    closed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("auto – Grouser")

        # --- plots ---
        self.plot1 = LivePlotWidget(self, generate_next_sine, 50, 10.0)
        self.plot2 = LivePlotWidget(self, generate_next_sine, 50, 10.0)

        # --- controls ---
        self.btn_Start = QtWidgets.QPushButton("Start")
        self.btn_zero    = QtWidgets.QPushButton("Zero")
        self.btn_return  = QtWidgets.QPushButton("Return")
        self.btn_log     = QtWidgets.QPushButton("Log")

        controls = QtWidgets.QVBoxLayout()
        controls.addWidget(self.btn_Start)
        controls.addWidget(self.btn_zero)
        controls.addWidget(self.btn_return)
        controls.addStretch()
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
        self.console.setMaximumBlockCount(1000)  # keep last 1000 lines
        font = self.console.font()
        font.setFamily("Consolas")               # optional: monospaced
        self.console.setFont(font)

        # subscribe the console to the "grouser" channel
        connect_console(self.console, channel="grouser")

        # example writes from anywhere:
        print_console("auto Grouser ready.", channel="grouser")


        # --- main layout: top + console ---
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(top_widget)
        vbox.addWidget(self.console, stretch=1)

        container = QWidget()
        container.setLayout(vbox)
        self.setCentralWidget(container)

        # wiring
        self.btn_return.clicked.connect(self.close)
        self.btn_zero.clicked.connect(self.zeroBtn)

    def zeroBtn(self):
       print_console("zero button pressed",channel="grouser")


    def closeEvent(self, e):
        try:
            self.plot1.stop()
            self.plot2.stop()
        except Exception:
            pass
        # detach console and stop timers
        disconnect_console(self.console)
        self.closed.emit()
        super().closeEvent(e)
