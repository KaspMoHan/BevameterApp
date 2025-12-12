# widgets/logger_controls.py
from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QWidget, QHBoxLayout, QPushButton, QLineEdit, QFileDialog
from pathlib import Path
import time

class LoggerControls(QWidget):
    """
    Reusable Start/Stop logging control that talks to io_worker (SerialWorker).
    - Works with any window that has self.io (the SerialWorker) and a console channel.
    - Lets user pick a folder and base name; defaults to ./logs and 'session'.
    - Calls io.start_logger(<folder>/<base>) and io.stop_logger().
    """
    def __init__(self, parent=None, io_worker=None, console_channel: str = "system",
                 default_folder: str = "logs", default_base: str = "session"):
        super().__init__(parent)
        self.io = io_worker
        self.console_channel = console_channel

        self.folder_edit = QLineEdit(default_folder)
        self.base_edit   = QLineEdit(default_base)
        self.btn_browse  = QPushButton("Folder…")
        self.btn_toggle  = QPushButton("Start logging")

        lay = QHBoxLayout(self)
        lay.addWidget(self.btn_toggle)
        lay.addWidget(self.folder_edit, stretch=1)
        lay.addWidget(self.base_edit)
        lay.addWidget(self.btn_browse)

        self.btn_browse.clicked.connect(self._pick_folder)
        self.btn_toggle.clicked.connect(self._toggle)

        # reflect current state if already running
        self._sync_btn()

    def _pick_folder(self):
        cur = self.folder_edit.text().strip() or "."
        folder = QFileDialog.getExistingDirectory(self, "Select log folder", cur)
        if folder:
            self.folder_edit.setText(folder)

    def _is_logging(self) -> bool:
        # We don't reach into private attrs; ask by behavior:
        # If start_logger is no-op when running, we'll infer by remembering label.
        return self.btn_toggle.text().startswith("Stop")

    def _sync_btn(self, running: bool = None):
        if running is None:
            # optimistic: keep current text unless caller specifies
            running = self._is_logging()
        self.btn_toggle.setText("Stop logging" if running else "Start logging")

    def _emit_console(self, msg: str):
        try:
            from utils.console_print import print_console
            print_console(msg, channel=self.console_channel)
        except Exception:
            pass

    def _toggle(self):
        if self.io is None:
            self._emit_console("[LOGGER] No io_worker connected.")
            return

        if not self._is_logging():
            folder = self.folder_edit.text().strip() or "logs"
            base   = self.base_edit.text().strip() or "session"
            Path(folder).mkdir(parents=True, exist_ok=True)

            # compose base path (no extension; DataLogger appends timestamp+index)
            base_path = str(Path(folder) / base)

            try:
                # include a small suffix to distinguish windows if they share the same base
                suffix = time.strftime("-%H%M%S")
                self.io.start_logger(out_path=base_path + suffix)
                self._emit_console(f"[LOGGER] started → {base_path}*.csv")
                self._sync_btn(True)
            except Exception as e:
                self._emit_console(f"[LOGGER] failed to start: {e}")
        else:
            try:
                self.io.stop_logger()
                self._emit_console("[LOGGER] stopped")
                self._sync_btn(False)
            except Exception as e:
                self._emit_console(f"[LOGGER] failed to stop: {e}")
