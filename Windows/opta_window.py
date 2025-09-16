# Windows/opta_window.py
import sys
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QPushButton, QComboBox, QHBoxLayout, QLabel, QMessageBox
)
from Windows.selection_window import SelectionWindow
import serial.tools.list_ports

# ⬇️ add this import
from utils.io_worker import SerialWorker
from PySide6 import QtCore

class OptaWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Connection on startup")

        self.selection_window = None
        self.io_worker = None   # ⬅️ keep a reference

        port_label = QLabel("Select COM port:")
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh")
        self.connect_button = QPushButton("Connect to OPTA")

        self.populate_ports()
        self.refresh_button.clicked.connect(self.populate_ports)
        self.connect_button.clicked.connect(self.button_clicked)

        combo_layout = QHBoxLayout()
        combo_layout.addWidget(port_label)
        combo_layout.addWidget(self.port_combo, stretch=1)
        combo_layout.addWidget(self.refresh_button)

        main_layout = QVBoxLayout()
        main_layout.addLayout(combo_layout)
        main_layout.addWidget(self.connect_button)

        self.setLayout(main_layout)

        # stop IO cleanly when app exits (safety net)
        QtCore.QCoreApplication.instance().aboutToQuit.connect(self._stop_io)

    def populate_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        if not port_list:
            port_list = ["(No ports found)"]
        self.port_combo.addItems(port_list)

    def button_clicked(self):
        selected_port = self.port_combo.currentText()
        if selected_port and selected_port != "(No ports found)":
            # 🔧 start SerialWorker here
            try:
                self.io_worker = SerialWorker(port=selected_port, baud=115200)
                self.io_worker.port_error.connect(self._show_error)  # optional: surface errors
                self.io_worker.start()
            except Exception as e:
                QMessageBox.critical(self, "Connection error", str(e))
                return

            # hand the same io_worker to the SelectionWindow
            self.selection_window = SelectionWindow(io_worker=self.io_worker)
            self.selection_window.show()
            self.close()
        else:
            QMessageBox.warning(self, "No COM Port", "Please select a valid COM port.")
            # For testing with no COM port, you can still open the selection window
            # with io_worker=None to keep UI reachable:
            self.selection_window = SelectionWindow(io_worker=None)
            self.selection_window.show()
            self.close()

    def _show_error(self, msg: str):
        QMessageBox.warning(self, "Serial error", msg)

    def _stop_io(self):
        if self.io_worker:
            try:
                self.io_worker.stop()
                self.io_worker.wait(2000)
            except Exception:
                pass
            self.io_worker = None
