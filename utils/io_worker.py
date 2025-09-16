# io_worker.py
from PySide6.QtCore import QThread, Signal, QReadWriteLock
import serial, time, queue

class SerialWorker(QThread):
    """
    Owns the serial port in a dedicated QThread.
    - Continuous non-blocking read; parses lines into frames
    - (Optional) Polls SENSOR:READ_ALL at poll_hz if expect_push=False
    - Stores latest frame for GUI snapshot via snapshot() (Option B)
    - Outbound command queue for actuator commands (no SEQ/ACK)
    """
    sensor_all = Signal(dict)     # every parsed sensor frame (use or ignore)
    cmd_result = Signal(int, str) # kept for compatibility; emits only if 'ACK:' lines appear
    port_error = Signal(str)      # error messages

    def __init__(self, port="COM3", baud=115200, expect_push=False, poll_hz=50, parent=None):
        """
        expect_push=False → we poll SENSOR:READ_ALL at poll_hz (default 50 Hz).
        expect_push=True  → no polling; device is expected to push SENS:... frames.
        """
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._ser = None
        self._running = True

        self._out_q = queue.Queue(maxsize=500)

        self._latest = None
        self._lock = QReadWriteLock()

        self._awaiting_ack = None  # unused unless your device emits "ACK:<n>"

        # --- polling scheduler ---
        self._expect_push = expect_push
        self._poll_hz = None if expect_push else (poll_hz or 50)
        self._poll_period = None if self._poll_hz is None else 1.0 / float(self._poll_hz)
        self._next_poll = time.perf_counter() + (self._poll_period or 0.0)

    # ---------------- Public API (call from GUI thread) ----------------
    def enqueue_command(self, cmd: str, require_ack: bool = False):
        """
        Enqueue a command string (without newline).
        No SEQ prefix is added (Arduino does not parse SEQ).
        """
        try:
            self._out_q.put_nowait((cmd, require_ack, time.time()))
        except queue.Full:
            self.port_error.emit("Outbound command queue full")

    def snapshot(self):
        """Return the latest parsed frame (or None). Thread-safe."""
        self._lock.lockForRead()
        try:
            if self._latest is None:
                return None
            return dict(self._latest)
        finally:
            self._lock.unlock()

    def stop(self):
        self._running = False

    # ---------------- QThread lifecycle ----------------
    def run(self):
        try:
            # timeout=0 -> non-blocking reads
            self._ser = serial.Serial(self._port, self._baud, timeout=0)
        except Exception as e:
            self.port_error.emit(f"Open failed: {e}")
            return

        rx_buf = bytearray()

        while self._running:
            # 1) READ any available bytes (never pause reading)
            try:
                data = self._ser.read(4096)
                if data:
                    rx_buf.extend(data)
                    while True:
                        nl = rx_buf.find(b'\n')
                        if nl < 0:
                            break
                        raw = rx_buf[:nl].decode('ascii', errors='ignore').strip()
                        del rx_buf[:nl+1]
                        self._handle_line(raw)
            except Exception as e:
                self.port_error.emit(f"Read error: {e}")
                time.sleep(0.005)

            # 2) Schedule periodic poll SENSOR:READ_ALL (no SEQ, no ACK)
            if self._poll_period:
                now = time.perf_counter()
                if now >= self._next_poll:
                    # Don’t congest the queue; skip poll if many pending writes
                    if self._out_q.qsize() < 50:
                        try:
                            self._out_q.put_nowait(("SENSOR:READ_ALL", False, now))
                        except queue.Full:
                            pass
                    # catch up if we slipped
                    while now >= self._next_poll:
                        self._next_poll += self._poll_period

            # 3) SEND pending commands (non-blocking)
            try:
                cmd, require_ack, ts = self._out_q.get_nowait()
                wire = f"{cmd}\n".encode('ascii')  # ← no SEQ prefix
                try:
                    self._ser.write(wire)
                    # If your device ever emits "ACK:<n>", you can set self._awaiting_ack here.
                    # We keep require_ack for API symmetry, but it's not used without SEQ.
                except Exception as e:
                    self.port_error.emit(f"Write error: {e}")
            except queue.Empty:
                pass

            # 4) Tiny breather to avoid pegging a core
            time.sleep(0.001)

        try:
            self._ser.close()
        except Exception:
            pass

    # ---------------- Helpers ----------------
    def _handle_line(self, line: str):
        if not line:
            return

        if line.startswith("SENS:"):
            frame = self._parse_sensor_payload(line[5:])
            frame["t_rx"] = time.time()
            # emit if you want to hook a logger; harmless if no slots connected
            self.sensor_all.emit(frame)
            # update snapshot
            self._lock.lockForWrite()
            try:
                self._latest = frame
            finally:
                self._lock.unlock()

        elif line.startswith("ACK:"):
            # Only relevant if your device prints ACKs (it currently doesn't)
            try:
                seq = int(line.split(":", 1)[1])
            except ValueError:
                seq = -1
            if self._awaiting_ack is None:
                self.cmd_result.emit(seq, "OK")
            else:
                self.cmd_result.emit(seq, "OK" if seq == self._awaiting_ack else "LATE_OK")
                self._awaiting_ack = None

        elif line.startswith("ERR:"):
            self.port_error.emit(line[4:])

        else:
            # Device echoes like "grouser:DIR:FWD" or "rubber:SPEED:1234" → ignore or log if desired
            pass

    def _parse_sensor_payload(self, payload: str) -> dict:
        """key=value CSV → dict of floats/ints/strings."""
        out = {}
        for token in payload.split(","):
            token = token.strip()
            if not token:
                continue
            if "=" in token:
                k, v = token.split("=", 1)
                k = k.strip(); v = v.strip()
                try:
                    if "." in v:
                        out[k] = float(v)
                    else:
                        out[k] = int(v)
                except ValueError:
                    out[k] = v
        return out
