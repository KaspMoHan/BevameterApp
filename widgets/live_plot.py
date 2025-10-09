# widgets/live_plot.py
import time
import numpy as np
from PySide6 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class LivePlotWidget(QtWidgets.QWidget):
    """
    Live plot with:
      - main series (data_fn)
      - optional moving average of main series
      - optional velocity on right y-axis (from main series)
      - NEW: one or more overlay series via add_overlay(label, fn)
    Each data function is called at 'update_hz' (e.g. 50 Hz).
    """

    def __init__(
        self,
        parent=None,
        data_fn=None,
        update_hz=50,
        window_seconds=10.0,
        ymin=None,
        ymax=None,
        show_ma=False,
        ma_window=10,
        show_velocity=False,
        vel_window=7,              # >=2
        y2min=None,
        y2max=None,
        y2label="",
        show_vel_ma=False,
        vel_ma_window=20,
        label="Meas",
        show_legend=True,
    ):
        super().__init__(parent)

        # ---- public config ----
        self.update_hz = int(update_hz)
        self.window_seconds = float(window_seconds)
        self.data_fn = data_fn
        self.show_ma = bool(show_ma)
        self.ma_window = int(ma_window)
        self.show_velocity = bool(show_velocity)
        self.vel_window = max(2, int(vel_window))
        self.y2label = y2label
        self.show_vel_ma = bool(show_vel_ma)
        self.vel_ma_window = max(2, int(vel_ma_window))
        self.main_label = label or "Meas"
        self.show_legend = bool(show_legend)

        # ---- matplotlib embed ----
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        # main line
        (self.line,) = self.ax.plot([], [], lw=1.6, label=self.main_label)
        self.line_ma = None
        if self.show_ma:
            (self.line_ma,) = self.ax.plot([], [], lw=1.0, linestyle="--", alpha=0.7, label=f"{self.main_label} (MA)")

        # secondary axis (velocity)
        self.ax2 = None
        self.vel_line = None
        self.vel_ma_line = None
        if self.show_velocity:
            self.ax2 = self.ax.twinx()
            (self.vel_line,) = self.ax2.plot([], [], lw=1.2, label="Velocity")
            if self.show_vel_ma:
                (self.vel_ma_line,) = self.ax2.plot([], [], lw=1.0, linestyle="--", alpha=0.7, label="Velocity (MA)")
            if y2min is not None and y2max is not None:
                self.ax2.set_ylim(y2min, y2max)
            if self.y2label:
                self.ax2.set_ylabel(self.y2label)

        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Signal")
        if ymin is not None and ymax is not None:
            self.ax.set_ylim(ymin, ymax)
        self.ax.set_xlim(0, self.window_seconds)

        root = QtWidgets.QVBoxLayout(self)
        root.addWidget(self.canvas, stretch=1)
        # small clear button row (kept minimal)
        self.btn_clear = QtWidgets.QPushButton("Clear")
        ctl = QtWidgets.QHBoxLayout()
        ctl.addWidget(self.btn_clear)
        ctl.addStretch()
        root.addLayout(ctl)
        self.setLayout(root)

        # ---- data buffers ----
        self.maxlen = int(self.update_hz * self.window_seconds)
        self.t = []
        self.y = []
        self.y_vel = []
        self._t0 = time.perf_counter()

        # overlays: list of dicts {label, fn, y, line}
        self._overlays = []

        # ---- timer (always running) ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(1000 / self.update_hz))

        # ---- wire buttons ----
        self.btn_clear.clicked.connect(self.clear)

        self._legend_drawn = False

    # ---------- API ----------
    def set_data_fn(self, fn, label=None):
        """Set/change the main data source: fn(t) -> y."""
        self.data_fn = fn
        if label:
            self.main_label = label
            self.line.set_label(self.main_label)
            if self.line_ma:
                self.line_ma.set_label(f"{self.main_label} (MA)")
            self._legend_drawn = False  # refresh legend next tick

    def add_overlay(self, label, fn):
        """Add an overlay series on the primary y-axis."""
        (line,) = self.ax.plot([], [], lw=1.4, linestyle=":", label=label)
        self._overlays.append({
            "label": label,
            "fn": fn,
            "y": [],
            "line": line,
        })
        self._legend_drawn = False

    def clear(self):
        self.t.clear()
        self.y.clear()
        self.y_vel.clear()
        for ov in self._overlays:
            ov["y"].clear()
        self._t0 = time.perf_counter()
        self._redraw()

    # ---------- lifecycle / safety ----------
    def stop(self):
        if self.timer.isActive():
            self.timer.stop()

    def closeEvent(self, e):
        self.stop()
        super().closeEvent(e)

    # ---------- internals ----------
    def _on_tick(self):
        if self._t0 is None:
            self._t0 = time.perf_counter()
        tnow_abs = time.perf_counter()
        tnow = tnow_abs - self._t0

        yval = self._sample(tnow)
        self.t.append(tnow)
        self.y.append(yval)

        # overlays
        for ov in self._overlays:
            try:
                yov = ov["fn"](tnow)
            except Exception:
                yov = float(ov["y"][-1]) if ov["y"] else 0.0
            ov["y"].append(float(yov))

        # velocity from main series
        if self.show_velocity:
            if len(self.t) > self.vel_window:
                i0 = -self.vel_window - 1
                dy = self.y[-1] - self.y[i0]
                dt = self.t[-1] - self.t[i0]
                v = dy / dt if dt > 0 else 0.0
            else:
                v = 0.0
            self.y_vel.append(v)

        # trim buffers
        def _trim(buf):
            if len(buf) > self.maxlen:
                del buf[:len(buf) - self.maxlen]

        _trim(self.t)
        _trim(self.y)
        if self.show_velocity:
            _trim(self.y_vel)
        for ov in self._overlays:
            _trim(ov["y"])

        # view window
        t0_view = max(0.0, tnow - self.window_seconds)
        self.ax.set_xlim(t0_view, t0_view + self.window_seconds)

        # main line
        self.line.set_data(np.asarray(self.t), np.asarray(self.y))

        # main moving average
        if self.line_ma:
            y_ma = self._moving_average(self.y, self.ma_window)
            self.line_ma.set_data(np.asarray(self.t[-len(y_ma):]), np.asarray(y_ma))

        # overlays
        for ov in self._overlays:
            ov["line"].set_data(np.asarray(self.t), np.asarray(ov["y"]))

        # velocity lines
        if self.ax2 and self.vel_line:
            self.vel_line.set_data(np.asarray(self.t), np.asarray(self.y_vel))
            if self.show_vel_ma and self.vel_ma_line:
                v_ma = self._moving_average(self.y_vel, self.vel_ma_window)
                self.vel_ma_line.set_data(np.asarray(self.t[-len(v_ma):]), np.asarray(v_ma))

        # legend (combine ax + ax2)
        if self.show_legend and not self._legend_drawn:
            h1, l1 = self.ax.get_legend_handles_labels()
            h2, l2 = (self.ax2.get_legend_handles_labels() if self.ax2 else ([], []))
            if h1 or h2:
                self.ax.legend(h1 + h2, l1 + l2, loc="best")
                self._legend_drawn = True

        self.canvas.draw_idle()

    def _sample(self, t):
        if callable(self.data_fn):
            return float(self.data_fn(t))
        return 0.0

    def _moving_average(self, arr, win):
        if win <= 1 or len(arr) < win:
            return arr[-len(arr):]
        # simple sliding window mean
        c = np.cumsum(np.insert(np.asarray(arr, dtype=float), 0, 0.0))
        ma = (c[win:] - c[:-win]) / float(win)
        return ma.tolist()

    def _redraw(self):
        self.line.set_data([], [])
        if self.line_ma:
            self.line_ma.set_data([], [])
        for ov in self._overlays:
            ov["line"].set_data([], [])
        if self.ax2 and self.vel_line:
            self.vel_line.set_data([], [])
        if self.ax2 and self.show_vel_ma and self.vel_ma_line:
            self.vel_ma_line.set_data([], [])
        self.canvas.draw_idle()
