import serial
import time
from .conversions import (
    bits_to_length,
    length_to_bits,
    voltage_to_bits,
    voltage_to_length,
    bits_to_voltage,
    MAX_DAC_BITS,
)

class OPTAController:
    def __init__(self, port: str = "COM3", baudrate: int = 115200, timeout: float = 0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        # flush any startup noise
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def _write(self, cmd: str) -> None:
        self.ser.write((cmd + "\n").encode("ascii"))

    def _readline(self) -> str:
        return self.ser.readline().decode("ascii", errors="ignore").strip()

    def open_relay_fwd(self, relay_str: str) -> None:
        """Activate forward relay."""
        if relay_str == "grouser":
            self._write("DIR:FWD:grouser")
            self._readline()
        elif relay_str == "rubber":
            self._write("DIR:FWD:rubber")
            self._readline()
        else:
            return

    def open_relay_bwd(self, relay_str: str) -> None:
        """Activate backward relay."""
        if relay_str == "grouser":
            self._write("DIR:BWD:grouser")
            self._readline()
        elif relay_str == "rubber":
            self._write("DIR:BWD:rubber")
            self._readline()
        else:
            return


    def disable_relays(self) -> None:
        """Deactivate both relays."""
        self._write("ALL:DIR:OFF")
        self._readline()

    def set_speed_bits(self, bits: int, relay_str: str) -> None:
        """
        Send SPEED:<bits> on every call (no threshold suppression).
        """
        b = max(0, min(bits, MAX_DAC_BITS))
        if relay_str == "grouser":
            self._write(f"grouser:SPEED:{b}")
        elif relay_str == "rubber":
            self._write(f"rubber:SPEED:{b}")
        else:
            return
        try:
            resp = self._readline()
            if not resp.startswith(f"{relay_str}:SPEED:"):
                raise RuntimeError(f"Unexpected response: {resp!r}")
        except Exception as e:
            print(f"Failed to set speed bits: {e}")

    def set_speed_percent(self, percent: float, relay_str: str) -> None:
        """
        Set speed as a percentage of the ADC’s 0–10 V input range,
        by scaling the DAC’s native 0–11 V full-scale accordingly.

        Args:
            percent: 0.0–100.0, where 100% ⇒ 10 V output.
        """
        p = max(0.0, min(percent, 100.0))
        # scale to 0–11 V bits then multiply by 10/11 to cap at 10 V
        bits = int(round((p / 100.0) * MAX_DAC_BITS * (10.0 / 11.0)))
        self.set_speed_bits(bits, relay_str)

    def coast(self, relay_str: str) -> None:
        """
        Kill drive voltage (SPEED:0) but leave relay state untouched.
        """
        if relay_str == "grouser":
            self._write(f"grouser:SPEED:0")
        elif relay_str == "rubber":
            self._write(f"rubber:SPEED:0")
        else:
            return
        try:
            self._readline()
        except Exception:
            pass

    def stop(self, relay_str: str) -> None:
        """
        Full shutdown: coast (zero voltage) then open both relays.
        """
        self.coast(relay_str)
        self.disable_relays()

    def get_position_bits(self, relay_str: str) -> int:
        """
        Issue READ_POS and parse POS:<bits>, skipping any other lines.
        """
        # clear any leftover data
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        if relay_str == "grouser": 
            self._write("grouser:READ_POS")
        elif relay_str == "rubber":
            self._write("rubber:READ_POS")
        else:
            return
        start = time.time()
        # loop until we see POS: or timeout
        while True:
            resp = self._readline()
            if resp.startswith("grouser:") or resp.startswith("rubber:"):
                return int(resp.split(":POS:", 1)[1])
            if time.time() - start > self.ser.timeout:
                raise RuntimeError(f"Timeout waiting for POS, last line was: {resp!r}")

    def get_position_voltage(self, relay_str:str) -> float:
        """
        Read raw bits and convert to voltage.
        """
        bits = self.get_position_bits(relay_str)
        return bits_to_voltage(bits)

    def get_position_mm(self, relay_str:str) -> float:
        """
        Read raw bits and convert to mm.
        """
        bits = self.get_position_bits(relay_str)
        return bits_to_length(bits)
