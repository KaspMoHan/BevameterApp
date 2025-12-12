# Windows/selection_window.py
from PySide6 import QtCore
from PySide6.QtWidgets import QMainWindow, QPushButton, QComboBox, \
    QVBoxLayout, QWidget, QHBoxLayout

from Windows import manual_rubber_window, manual_grouser_window, \
    manual_pressure_window, auto_rubber_window, auto_grouser_window, auto_pressure_window

class SelectionWindow(QMainWindow):
    runRequested = QtCore.Signal(str, str)

    def __init__(self, io_worker=None):  # ⬅️ accept the worker
        super().__init__()
        self.io = io_worker               # ⬅️ keep a reference

        self.states = None
        self.container = None
        self.horizontal_layout = None
        self.vertical_layout = None

        self.setWindowTitle("Select test type")
        self.combobox_automation_selector = QComboBox()
        self.combobox_actuator_selector = QComboBox()
        self.pushbutton_selector = QPushButton()

        self.init_ui()

    def init_ui(self):
        self.pushbutton_selector.setText("OK")
        self.pushbutton_selector.setEnabled(True)

        self.combobox_automation_selector.addItems(["Manual", "Automatic"])
        self.combobox_actuator_selector.addItems(["Grouser", "Rubber", "Pressure/Sinkage"])

        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.combobox_automation_selector)
        self.vertical_layout.addWidget(self.combobox_actuator_selector)

        self.horizontal_layout = QHBoxLayout()
        self.horizontal_layout.addLayout(self.vertical_layout)
        self.horizontal_layout.addWidget(self.pushbutton_selector)

        self.container = QWidget()
        self.container.setLayout(self.horizontal_layout)
        self.setCentralWidget(self.container)

        self.states = {
            ("Manual", "Rubber"): self.run_manual_rubber,
            ("Manual", "Grouser"): self.run_manual_grouser,
            ("Manual", "Pressure/Sinkage"): self.run_manual_pressure,
            ("Automatic", "Rubber"): self.run_auto_rubber,
            ("Automatic", "Grouser"): self.run_auto_grouser,
            ("Automatic", "Pressure/Sinkage"): self.run_auto_pressure}

        self.pushbutton_selector.clicked.connect(self.on_click)

    def on_click(self):
        automation_type = self.combobox_automation_selector.currentText()
        actuator_selector = self.combobox_actuator_selector.currentText()
        self.runRequested.emit(automation_type, actuator_selector)
        state = self.states.get((automation_type, actuator_selector))
        state()
        self.hide()

    # --- Handlers (pass self.io down) ---
    def run_manual_grouser(self):
        self.child = manual_grouser_window.ManualGrouserWindow(parent=self, io_worker=self.io)
        self.child.closed.connect(self.show)
        self.child.show()

    def run_manual_rubber(self):
        self.child = manual_rubber_window.ManualRubberWindow(parent=self,io_worker=self.io)
        self.child.closed.connect(self.show)
        self.child.show()

    def run_manual_pressure(self):
        self.child = manual_pressure_window.ManualPressureWindow(parent=self,io_worker=self.io)
        self.child.closed.connect(self.show)
        self.child.show()

    def run_auto_grouser(self):
        self.child = auto_grouser_window.autoGrouserWindow(parent=self, io_worker=self.io)
        self.child.closed.connect(self.show)
        self.child.show()

    def run_auto_rubber(self):
        self.child = auto_rubber_window.autoRubberWindow(parent=self,io_worker=self.io)
        self.child.closed.connect(self.show)
        self.child.show()

    def run_auto_pressure(self):
        self.child = auto_pressure_window.autoPressureWindow(parent=self)
        self.child.closed.connect(self.show)
        self.child.show()
