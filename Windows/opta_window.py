# Windows/opta_window.py
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QPushButton, QComboBox, QHBoxLayout, QLabel, QMessageBox
)
from PySide6 import QtCore

from Windows.selection_window import SelectionWindow
from serial.tools import list_ports  # <-- IMPORTANT: avoid "import serial...."

from utils.io_worker import SerialWorker


class OptaWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Connection on startup")

        self.selection_window = None
        self.io_worker = None

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

        QtCore.QCoreApplication.instance().aboutToQuit.connect(self._stop_io)

    def populate_ports(self):
        self.port_combo.clear()
        ports = list_ports.comports()
        port_list = [p.device for p in ports]
        if not port_list:
            port_list = ["(No ports found)"]
        self.port_combo.addItems(port_list)

    def button_clicked(self):
        selected_port = self.port_combo.currentText()
        if selected_port and selected_port != "(No ports found)":
            try:
                self.io_worker = SerialWorker(port=selected_port, baud=115200)
                self.io_worker.port_error.connect(self._show_error)
                self.io_worker.start()
            except Exception as e:
                QMessageBox.critical(self, "Connection error", str(e))
                return

            self.selection_window = SelectionWindow(io_worker=self.io_worker)
            self.selection_window.show()
            self.close()
        else:
            QMessageBox.warning(self, "No COM Port", "Please select a valid COM port.")
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
