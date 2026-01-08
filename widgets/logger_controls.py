# widgets/logger_controls.py
from PySide6 import QtWidgets
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QLineEdit, QFileDialog
)
from pathlib import Path


class LoggerControls(QWidget):
    """
    Reusable Start/Stop logging control that talks to io_worker (SerialWorker).

    Updated behavior:
    - User MUST enter a comment (required) or logging won't start
    - Test type is provided by the window (auto-grouser, manual-pressure, etc.)
    - Logger produces filenames like:
        YYYY-MM-DD-HHMM__<test_type>__<comment>__NNN.csv
      (safe for Windows; ':' not allowed in filenames)
    """
    def __init__(
        self,
        parent=None,
        io_worker=None,
        console_channel: str = "system",
        default_folder: str = "logs",
        default_test_type: str = "session",
    ):
        super().__init__(parent)
        self.io = io_worker
        self.console_channel = console_channel

        self.btn_toggle = QPushButton("Start logging")
        self.folder_edit = QLineEdit(default_folder)
        self.test_edit = QLineEdit(default_test_type)
        self.test_edit.setToolTip("Test type (e.g. auto-grouser, manual-pressure)")
        self.comment_edit = QLineEdit("")
        self.comment_edit.setPlaceholderText("REQUIRED comment (e.g. sand_10pct_run2)")
        self.btn_browse = QPushButton("Folder…")

        lay = QHBoxLayout(self)
        lay.addWidget(self.btn_toggle)
        lay.addWidget(self.folder_edit, stretch=1)
        lay.addWidget(self.test_edit)
        lay.addWidget(self.comment_edit, stretch=2)
        lay.addWidget(self.btn_browse)

        self.btn_browse.clicked.connect(self._pick_folder)
        self.btn_toggle.clicked.connect(self._toggle)

        self._sync_btn()

    def _pick_folder(self):
        cur = self.folder_edit.text().strip() or "."
        folder = QFileDialog.getExistingDirectory(self, "Select log folder", cur)
        if folder:
            self.folder_edit.setText(folder)

    def _is_logging(self) -> bool:
        # UI-based inference (safe enough). If you later add SerialWorker.is_logging(),
        # you can replace this with a real query.
        return self.btn_toggle.text().startswith("Stop")

    def _sync_btn(self, running: bool = None):
        if running is None:
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
            folder = (self.folder_edit.text().strip() or "logs").strip()
            test_type = (self.test_edit.text().strip() or "session").strip()
            comment = (self.comment_edit.text().strip() or "").strip()

            if not comment:
                self._emit_console("[LOGGER] Please enter a comment to start logging.")
                self.comment_edit.setFocus()
                return

            Path(folder).mkdir(parents=True, exist_ok=True)

            try:
                # NEW API: out_dir + test_type + comment (filename built in DataLogger)
                self.io.start_logger(
                    out_dir=folder,
                    test_type=test_type,
                    comment=comment,
                )
                self._emit_console("[LOGGER] started")
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
