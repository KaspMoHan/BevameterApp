"""
Conversions between raw DAC/ADC bits, voltages, and physical units.

- ADC (inputs): 16-bit, 0..65535 → 0..10 V (sensors: position, torque)
- DAC (outputs): 13-bit, 0..8191  → 0..11 V full-scale (we cap 100% to 10 V)

Actuator length: assume 0..10 V maps to 0..MAX_LENGTH (mm)
Torque: V → Nm via your calibration.
"""

from typing import Union

Number = Union[int, float]

# -------------------- Hardware constants --------------------
# ADC (sensors)
ADC_BITS_MAX: int = (1 << 16) - 1   # 65535
ADC_VREF_V:  float = 10.0           # sensors output 0..10 V

# DAC (setpoints)
DAC_BITS_MAX: int = (1 << 13) - 1   # 8191
DAC_FS_V_V:  float = 11.0           # hardware full-scale
DAC_TARGET_V: float = 10.0          # we treat 100% as 10 V (cap)

# Mechanics / scaling
MAX_LENGTH_MM: float = 1200.0       # length at 10 V

# Torque calibration
TORQUE_V_PER_NM: float = 0.02       # 0.02 V per Nm (sensor nominal)
TORQUE_CAL_M: float = 0.8515        # your linear fit slope
TORQUE_CAL_B: float = 1.1652        # your linear fit intercept


# -------------------- Utils --------------------
def _clamp(x: Number, lo: Number, hi: Number) -> Number:
    return lo if x < lo else hi if x > hi else x


# -------------------- ADC side (inputs) --------------------
def adc_bits_to_voltage(bits: int) -> float:
    """ADC raw bits → volts (0..10 V)."""
    b = int(_clamp(int(bits), 0, ADC_BITS_MAX))
    return (b / ADC_BITS_MAX) * ADC_VREF_V


def voltage_to_adc_bits(volts: float) -> int:
    """Volts (0..10 V) → ADC raw bits."""
    v = float(_clamp(volts, 0.0, ADC_VREF_V))
    return int(round((v / ADC_VREF_V) * ADC_BITS_MAX))


def adc_bits_to_length(bits: int) -> float:
    """ADC raw bits → actuator length (mm)."""
    v = adc_bits_to_voltage(bits)
    return (v / ADC_VREF_V) * MAX_LENGTH_MM


def length_to_adc_bits(length_mm: float) -> int:
    """Actuator length (mm) → ADC raw bits (equivalent voltage)."""
    L = float(_clamp(length_mm, 0.0, MAX_LENGTH_MM))
    v = (L / MAX_LENGTH_MM) * ADC_VREF_V
    return voltage_to_adc_bits(v)


# -------------------- DAC side (outputs) --------------------
def dac_bits_to_voltage(bits: int) -> float:
    """DAC raw bits → volts (0..11 V hardware full-scale)."""
    b = int(_clamp(int(bits), 0, DAC_BITS_MAX))
    return (b / DAC_BITS_MAX) * DAC_FS_V_V


def voltage_to_dac_bits(volts: float) -> int:
    """Volts (0..11 V FS) → DAC raw bits."""
    v = float(_clamp(volts, 0.0, DAC_FS_V_V))
    return int(round((v / DAC_FS_V_V) * DAC_BITS_MAX))


def percent_to_dac_bits(percent: float,
                        v_target: float = DAC_TARGET_V) -> int:
    """
    Map 0–100 % → DAC bits such that 100% produces v_target volts (default 10 V),
    even though the DAC hardware full-scale is 11 V.
    """
    p = 0.0 if percent is None else float(percent)
    p = _clamp(p, 0.0, 100.0)
    scale = (v_target / DAC_FS_V_V)  # e.g., 10/11
    return int(round((p / 100.0) * DAC_BITS_MAX * scale))


# -------------------- Torque (ADC) --------------------
def bits_to_torque(bits: int) -> float:
    """
    ADC bits → torque [Nm].
    1) bits → volts (0..10 V)
    2) volts / (V/Nm) → raw torque
    3) apply your linear calibration: torque = M*raw + B
    """
    v = adc_bits_to_voltage(bits)
    raw_nm = v / TORQUE_V_PER_NM
    return TORQUE_CAL_M * raw_nm + TORQUE_CAL_B


def torque_to_bits(torque_nm: float) -> int:
    """
    Torque [Nm] → ADC bits (inverse of the above; useful for test/sim).
    """
    raw_nm = (float(torque_nm) - TORQUE_CAL_B) / TORQUE_CAL_M
    v = _clamp(raw_nm * TORQUE_V_PER_NM, 0.0, ADC_VREF_V)
    return voltage_to_adc_bits(v)


# -------------------- Backward-compatible names --------------------
# Your code already calls these; they now route to the *correct* side.

def bits_to_voltage(bits: int) -> float:
    """(Back-compat) Treat input bits as ADC and return sensor voltage."""
    return adc_bits_to_voltage(bits)


def voltage_to_bits(voltage: float) -> int:
    """(Back-compat) Treat input voltage as sensor voltage → ADC bits."""
    return voltage_to_adc_bits(voltage)


def voltage_to_length(voltage: float) -> float:
    """(Back-compat) Sensor volts → mm."""
    v = float(_clamp(voltage, 0.0, ADC_VREF_V))
    return (v / ADC_VREF_V) * MAX_LENGTH_MM


def length_to_voltage(length: float) -> float:
    """(Back-compat) mm → sensor volts."""
    L = float(_clamp(length, 0.0, MAX_LENGTH_MM))
    return (L / MAX_LENGTH_MM) * ADC_VREF_V


def bits_to_length(bits: int) -> float:
    """(Back-compat) Interpret bits as ADC → length (mm)."""
    return adc_bits_to_length(bits)


def length_to_bits(length: float) -> int:
    """(Back-compat) Interpret result as ADC bits for given length."""
    return length_to_adc_bits(length)


def percent_to_bits(percent: float) -> int:
    """(Back-compat) Map percent → DAC bits for speed commands."""
    return percent_to_dac_bits(percent)
