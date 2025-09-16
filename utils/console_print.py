# utils/console_print.py
from PySide6 import QtCore

class _ConsoleBus(QtCore.QObject):
    # channel, text
    message = QtCore.Signal(str, str)

bus = _ConsoleBus()

def print_console(text: str, channel: str = "*"):
    """Publish a line to all consoles listening on `channel` (or everyone with '*')."""
    bus.message.emit(channel, str(text))

def connect_console(console_widget, channel: str = "*"):
    """
    Attach a QPlainTextEdit to the bus. It will receive logs for its channel,
    '*' (broadcast), and '' (unspecified).
    """

    def _append(ch, txt):
        if channel == "*" or ch in ("", "*") or ch == channel:
            console_widget.appendPlainText(txt)
            sb = console_widget.verticalScrollBar()
            sb.setValue(sb.maximum())

    # keep ref so we can disconnect later
    console_widget._console_slot = _append
    bus.message.connect(_append)
    return _append

def disconnect_console(console_widget):
    """Detach the console from the bus."""
    slot = getattr(console_widget, "_console_slot", None)
    if slot:
        try:
            bus.message.disconnect(slot)
        except Exception:
            pass
        delattr(console_widget, "_console_slot")
