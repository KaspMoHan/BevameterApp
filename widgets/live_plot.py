# widgets/live_plot.py
import time
import numpy as np
from PySide6 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class LivePlotWidget(QtWidgets.QWidget):
    """
    Reusable live-plot widget with Clear button and fixed 50 Hz update.
    Parent supplies a data function: y = data_fn(t_seconds)
    """
    def __init__(self, parent=None, data_fn=None, update_hz=50, window_seconds=10.0):
        super().__init__(parent)

        # ---- public config ----
        self.update_hz = int(update_hz)
        self.window_seconds = float(window_seconds)
        self.data_fn = data_fn

        # ---- matplotlib embed ----
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        (self.line,) = self.ax.plot([], [], lw=1.5)

        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Signal")
        self.ax.set_xlim(0, self.window_seconds)
        self.ax.set_ylim(0, 500)

        # ---- controls ----
        self.btn_clear = QtWidgets.QPushButton("Clear")

        ctl = QtWidgets.QHBoxLayout()
        ctl.addWidget(self.btn_clear)
        ctl.addStretch(1)

        root = QtWidgets.QVBoxLayout(self)
        root.addWidget(self.canvas, stretch=1)
        root.addLayout(ctl)
        self.setLayout(root)

        # ---- data buffers ----
        self.maxlen = int(self.update_hz * self.window_seconds)
        self.t = []
        self.y = []
        self._t0 = time.perf_counter()

        # ---- timer (always running) ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(1000 / self.update_hz))

        # ---- wire buttons ----
        self.btn_clear.clicked.connect(self.clear)

    # ---------- API ----------
    def set_data_fn(self, fn):
        """Set or change the data source: fn(t) -> y."""
        self.data_fn = fn


    def clear(self):
        self.t.clear()
        self.y.clear()
        self._t0 = time.perf_counter()
        self._redraw()

    # ---------- lifecycle / safety ----------
    def stop(self):
        """Stop updates safely (idempotent)."""
        if self.timer.isActive():
            self.timer.stop()

    def closeEvent(self, e):
        # Make sure the timer is off before canvas is torn down
        self.stop()
        super().closeEvent(e)

    # ---------- internals ----------
    def _on_tick(self):
        if self._t0 is None:
            self._t0 = time.perf_counter()
        tnow = time.perf_counter() - self._t0

        yval = self._sample(tnow)

        self.t.append(tnow)
        self.y.append(yval)
        if len(self.t) > self.maxlen:
            self.t = self.t[-self.maxlen:]
            self.y = self.y[-self.maxlen:]

        t0_view = max(0.0, tnow - self.window_seconds)
        self.ax.set_xlim(t0_view, t0_view + self.window_seconds)

        self.line.set_data(np.asarray(self.t), np.asarray(self.y))
        self.canvas.draw_idle()

    def _sample(self, t):
        if callable(self.data_fn):
            return self.data_fn(t)
        return 0.0

    def _redraw(self):
        self.line.set_data([], [])
        self.canvas.draw_idle()
