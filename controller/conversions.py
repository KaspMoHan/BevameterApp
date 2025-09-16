"""
Conversions between raw DAC/ADC bits, voltages, and actuator lengths.
Assumes a 13-bit DAC/ADC with 0–11 V range driving an actuator of 0–1200 mm stroke.
"""

# Constants for conversion
MAX_DAC_BITS = 2**13 - 1      # 13-bit resolution: 0..8191
MAX_VOLTAGE  = 10.0           # volts corresponding to full-scale bits
MAX_LENGTH   = 1200.0         # mm corresponding to full-scale voltage


def bits_to_voltage(bits: int) -> float:
    """
    Convert raw 13-bit DAC/ADC bits to voltage.

    Args:
        bits: integer in [0, MAX_DAC_BITS]
    Returns:
        Voltage in volts, clamped to [0, MAX_VOLTAGE].
    """
    b = max(0, min(MAX_DAC_BITS, bits))
    return (b / MAX_DAC_BITS) * MAX_VOLTAGE


def voltage_to_bits(voltage: float) -> int:
    """
    Convert voltage to raw 13-bit DAC/ADC bits.

    Args:
        voltage: volts in [0, MAX_VOLTAGE]
    Returns:
        Integer bits in [0, MAX_DAC_BITS].
    """
    v = max(0.0, min(MAX_VOLTAGE, voltage))
    return int(round((v / MAX_VOLTAGE) * MAX_DAC_BITS))


def voltage_to_length(voltage: float) -> float:
    """
    Convert voltage to actuator length in mm.

    Args:
        voltage: volts in [0, MAX_VOLTAGE]
    Returns:
        Length in mm in [0, MAX_LENGTH].
    """
    v = max(0.0, min(MAX_VOLTAGE, voltage))
    return (v / MAX_VOLTAGE) * MAX_LENGTH


def length_to_voltage(length: float) -> float:
    """
    Convert actuator length in mm to voltage.

    Args:
        length: mm in [0, MAX_LENGTH]
    Returns:
        Voltage in volts in [0, MAX_VOLTAGE].
    """
    l = max(0.0, min(MAX_LENGTH, length))
    return (l / MAX_LENGTH) * MAX_VOLTAGE


def bits_to_length(bits: int) -> float:
    """
    Convert raw 13-bit bits directly to actuator length in mm.

    Args:
        bits: integer in [0, MAX_DAC_BITS]
    Returns:
        Length in mm in [0, MAX_LENGTH].
    """
    return voltage_to_length(bits_to_voltage(bits))


def length_to_bits(length: float) -> int:
    """
    Convert actuator length in mm directly to raw 13-bit bits.

    Args:
        length: mm in [0, MAX_LENGTH]
    Returns:
        Integer bits in [0, MAX_DAC_BITS].
    """
    return voltage_to_bits(length_to_voltage(length))

def percent_to_bits(percent: float,
                    max_bits: int = MAX_DAC_BITS,
                    v_target: float = 10.0,
                    dac_fullscale: float = 11.0) -> int:
    """
    Map 0–100 % to DAC bits such that 100% = 10 V output,
    when the DAC’s full-scale is 11 V.
    """
    p = 0.0 if percent is None else float(percent)
    if p < 0.0: p = 0.0
    if p > 100.0: p = 100.0
    scale = (v_target / dac_fullscale)   # 10/11
    return int(round((p / 100.0) * max_bits * scale))