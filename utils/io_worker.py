# io_worker.py
import time, threading, queue, sys
from typing import Dict, Optional, Any
from PySide6 import QtCore

try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception as e:
    serial = None
    list_ports = None
    print("[WARN] pyserial not available:", e, file=sys.stderr)


# --------------------------- FrameBus (fan-out) ---------------------------

class FrameBus:
    """
    A lightweight publish/subscribe queue.
    SerialWorker publishes each parsed frame once; consumers (e.g., DataLogger)
    can read from the queue without interfering with snapshot() or control.
    """
    def __init__(self, maxsize: int = 10000):
        self._q = queue.Queue(maxsize=maxsize)

    def publish(self, frame: Dict[str, Any]) -> None:
        try:
            self._q.put_nowait(frame)
        except queue.Full:
            # Drop oldest then insert (bounded and non-blocking)
            try:
                _ = self._q.get_nowait()
                self._q.put_nowait(frame)
            except queue.Empty:
                pass

    def get_queue(self) -> queue.Queue:
        return self._q


# --------------------------- Optional DataLogger ---------------------------

class DataLogger(threading.Thread):
    """
    Standalone logger that consumes frames from FrameBus and writes CSV.
    - Runs in its own daemon thread.
    - Auto-discovers columns on the first frame.
    - Rotates files by size.
    """
    def __init__(self, q: queue.Queue, out_path: str,
                 rotate_mb: float = 100.0, flush_every: int = 200):
        import csv, os
        super().__init__(daemon=True)
        self.csv = csv
        self.os = os
        self.q = q
        self._stop = threading.Event()
        self._fh = None
        self._writer = None
        self._fields = None
        self._bytes_written = 0
        self._rows = 0
        self._rotate_bytes = int(rotate_mb * 1024 * 1024)
        self._flush_every = max(1, int(flush_every))
        self._base = out_path
        self._index = 0
        self._open_new_file()

    def _open_new_file(self):
        if self._fh:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
        ts = time.strftime("%Y%m%d-%H%M%S")
        path = f"{self._base}_{ts}_{self._index:03d}.csv"
        self._index += 1
        self._fh = open(path, "wt", newline="")
        self._writer = None
        self._fields = None
        self._bytes_written = 0
        self._rows = 0
        self._path = path

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            while not self._stop.is_set():
                try:
                    frame = self.q.get(timeout=0.25)
                except queue.Empty:
                    continue

                if self._fields is None:
                    self._fields = sorted(frame.keys())
                    self._writer = self.csv.DictWriter(
                        self._fh, fieldnames=self._fields, extrasaction="ignore"
                    )
                    self._writer.writeheader()

                self._writer.writerow(frame)
                self._rows += 1

                # rough byte accounting for rotation
                self._bytes_written += sum(
                    len(str(frame.get(f, ""))) for f in self._fields
                ) + len(self._fields)

                if self._rows % self._flush_every == 0:
                    self._fh.flush()

                if self._bytes_written >= self._rotate_bytes:
                    self._open_new_file()
        finally:
            try:
                if self._fh:
                    self._fh.flush()
                    self._fh.close()
            except Exception:
                pass


# --------------------------- SerialWorker (QThread) ---------------------------

SENS_PREFIX = "SENS:"
ACK_PREFIXES = (
    "grouser:", "rubber:", "pressure:",
    "RUBBER:", "GROUSER:", "PRESSURE:", "ALL:"
)


def _parse_sens_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse lines like:
      SENS:ts_ms=123,grouser_pos=1234,rubber_pos=2345,pressure_pos=...,pressure_force=...
    Returns a dict or None if not a SENS line.
    """
    if not line.startswith(SENS_PREFIX):
        return None
    payload = line[len(SENS_PREFIX):].strip()
    out: Dict[str, Any] = {}
    for kv in payload.split(","):
        if "=" not in kv:
            continue
        k, v = kv.split("=", 1)
        k = k.strip()
        v = v.strip()
        try:
            if "." in v:
                out[k] = float(v)
            else:
                out[k] = int(v)
        except ValueError:
            out[k] = v
    return out


class SerialWorker(QtCore.QThread):
    """
    Owns the serial port in a dedicated QThread.

    Features:
    - High/low priority TX queues (controls vs. sensor polls).
    - Central sensor poller (shared across all windows) with per-consumer rates.
    - Snapshot() of the latest parsed SENS frame (thread-safe).
    - Frame bus for logging (CSV logger runs in its own thread).
    """
    console_text = QtCore.Signal(str)  # optional: UI console mirror
    port_error   = QtCore.Signal(str)  # port / connection errors
    port_ok      = QtCore.Signal(str)  # port opened successfully

    def __init__(self, port: Optional[str] = None, baud: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._ser = None
        self._stop_evt = threading.Event()

        # snapshot
        self._latest_lock = threading.Lock()
        self._latest_frame: Dict[str, Any] = {}

        # TX queues
        self._cmd_q_hi: "queue.Queue[str]" = queue.Queue(maxsize=2000)  # controls (DIR/SPEED/...)
        self._ack_q_hi: "queue.Queue[bool]" = queue.Queue(maxsize=2000)
        self._cmd_q_lo: "queue.Queue[str]" = queue.Queue(maxsize=2000)  # SENSOR:READ_ALL
        self._ack_q_lo: "queue.Queue[bool]" = queue.Queue(maxsize=2000)

        # central stream requesters: tag -> hz
        self._stream_consumers: Dict[str, float] = {}
        self._poll_enabled = False
        self._poll_interval_s = 0.02  # default 50 Hz
        self._next_poll_time = 0.0

        # fan-out bus + optional logger
        self.frame_bus = FrameBus()
        self._logger: Optional[DataLogger] = None

    # ---------- public API ----------

    def snapshot(self) -> Dict[str, Any]:
        """Return a shallow copy of the latest frame (non-blocking)."""
        with self._latest_lock:
            return dict(self._latest_frame)

    def enqueue_command(self, cmd: str, require_ack: bool = False, low_priority: bool = False) -> None:
        """
        Queue a command line. Controls should use low_priority=False (default).
        The poller uses low_priority=True for SENSOR:READ_ALL.
        """
        line = (cmd or "").strip()
        if not line:
            return
        q_cmd = self._cmd_q_lo if low_priority else self._cmd_q_hi
        q_ack = self._ack_q_lo if low_priority else self._ack_q_hi
        try:
            q_cmd.put_nowait(line)
            q_ack.put_nowait(bool(require_ack))
        except queue.Full:
            self.console_text.emit(
                f"[WARN] CMD queue full ({'LO' if low_priority else 'HI'}), dropped: {line}"
            )

    # --- shared sensor stream management ---
    def acquire_stream(self, tag: str, hz: float = 50.0):
        """Register a consumer of the SENSOR:READ_ALL stream."""
        if hz and hz > 0:
            self._stream_consumers[str(tag)] = float(hz)
        else:
            self._stream_consumers[str(tag)] = 50.0
        self._recalc_poll_rate()

    def release_stream(self, tag: str):
        self._stream_consumers.pop(str(tag), None)
        self._recalc_poll_rate()

    def _recalc_poll_rate(self):
        if self._stream_consumers:
            max_hz = max(self._stream_consumers.values()) if self._stream_consumers else 0.0
            max_hz = 1.0 if max_hz < 1.0 else max_hz
            self._poll_interval_s = 1.0 / max_hz
            self._poll_enabled = True
        else:
            self._poll_enabled = False

    # --- logger control ---
    def start_logger(self, out_path: str, rotate_mb: float = 100.0, flush_every: int = 200):
        if self._logger is not None:
            return
        self._logger = DataLogger(
            self.frame_bus.get_queue(),
            out_path=out_path,
            rotate_mb=rotate_mb,
            flush_every=flush_every,
        )
        self._logger.start()
        self.console_text.emit(f"[LOGGER] started → {out_path}_*.csv")

    def stop_logger(self):
        if self._logger is not None:
            self._logger.stop()
            self._logger.join(timeout=2.0)
            self._logger = None
            self.console_text.emit("[LOGGER] stopped")

    def stop(self):
        """Signal the thread to stop; serial closed inside run()."""
        self._stop_evt.set()

    # ---------- QThread entry ----------

    def run(self):
        self._open_serial()
        read_buf = bytearray()
        self._next_poll_time = time.time() + self._poll_interval_s

        while not self._stop_evt.is_set():
            # 1) Periodic low-priority sensor poll
            now = time.time()
            if self._poll_enabled and now >= self._next_poll_time:
                # only enqueue poll if HI queue isn't heavily backed up
                if self._cmd_q_hi.qsize() < 50:  # throttle to keep controls snappy
                    self.enqueue_command(
                        "SENSOR:READ_ALL",
                        require_ack=False,
                        low_priority=True
                    )
                self._next_poll_time = now + self._poll_interval_s

            # 2) Drain TX (prioritize high)
            self._drain_commands()

            # 3) Read lines
            if self._ser:
                try:
                    chunk = self._ser.read(self._ser.in_waiting or 1)
                except Exception as e:
                    msg = f"[SER] read error: {e}"
                    self.console_text.emit(msg)
                    self.port_error.emit(msg)
                    self._reopen_serial()
                    continue

                if chunk:
                    read_buf.extend(chunk)
                    while True:
                        nl = read_buf.find(b"\n")
                        if nl < 0:
                            break
                        raw = read_buf[:nl]
                        if raw.endswith(b"\r"):
                            raw = raw[:-1]
                        del read_buf[:nl+1]
                        line = raw.decode(errors="replace").strip()
                        self._handle_line(line)

            # small pacing
            self.msleep(1)

        # shutdown
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass

    # ---------- internals ----------

    def _handle_line(self, line: str):
        if not line:
            return

        frame = _parse_sens_line(line)
        if frame is not None:
            frame["ts_host"] = time.time()
            with self._latest_lock:
                self._latest_frame.update(frame)
            self.frame_bus.publish(dict(frame))
            return

        if line.startswith(ACK_PREFIXES) or line.startswith(("ERR:", "WARN:")):
            self.console_text.emit(line)
            return

        self.console_text.emit(line)

    def _drain_commands(self):
        if not self._ser:
            return
        # send up to N HI commands, then at most 1 LO per cycle
        sent_lo = False
        for _ in range(64):
            try:
                # prefer HI if available
                if not self._cmd_q_hi.empty():
                    cmd = self._cmd_q_hi.get_nowait()
                    _ = self._ack_q_hi.get_nowait()
                else:
                    if sent_lo or self._cmd_q_lo.empty():
                        break
                    cmd = self._cmd_q_lo.get_nowait()
                    _ = self._ack_q_lo.get_nowait()
                    sent_lo = True
            except queue.Empty:
                break

            try:
                self._ser.write((cmd + "\n").encode())
            except Exception as e:
                msg = f"[SER] write error: {e}"
                self.console_text.emit(msg)
                self.port_error.emit(msg)
                self._reopen_serial()
                break

    def _open_serial(self):
        if serial is None:
            msg = "[SER] pyserial unavailable."
            self.console_text.emit(msg)
            self.port_error.emit(msg)
            return

        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        port = self._port or self._auto_detect_port()
        if not port:
            msg = "[SER] no port available"
            self.console_text.emit(msg)
            self.port_error.emit(msg)
            return

        try:
            self._ser = serial.Serial(port=port, baudrate=self._baud, timeout=0.0)
            msg = f"[SER] open {port} @ {self._baud}"
            self.console_text.emit(msg)
            self.port_ok.emit(port)
        except Exception as e:
            self._ser = None
            msg = f"[SER] open failed {port}: {e}"
            self.console_text.emit(msg)
            self.port_error.emit(msg)

    def _reopen_serial(self):
        time.sleep(0.2)
        self._open_serial()

    def _auto_detect_port(self) -> Optional[str]:
        if list_ports is None:
            return None
        candidates = []
        for p in list_ports.comports():
            desc = f"{p.device} {p.manufacturer or ''} {p.description or ''}".lower()
            if "arduino" in desc or "opta" in desc or "stm" in desc or "usb serial" in desc:
                candidates.append(p.device)
        if candidates:
            return candidates[0]
        ports = [p.device for p in list_ports.comports()]
        return ports[0] if ports else None
