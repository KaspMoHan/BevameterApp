#!/usr/bin/env python3
import os
import sys
import time

# make sure we can import src/
sys.path.insert(0, os.path.abspath(os.path.join(__file__, "..", "..")))

from controller.opta_controller import OPTAController
from controller.conversions import length_to_bits

# — Configuration —
TARGET_MM     = 980.0   # mm to extend to
SPEED_PERCENT = 100.0    # % of full‐scale speed
POLL_INTERVAL = 0.1     # seconds between position reads

relay_str = "grouser" # what actuator do we want to control


def main():
    ctrl = OPTAController("COM3")

    # compute bit threshold for 500 mm
    threshold_bits = length_to_bits(TARGET_MM)

    # 1) set speed and start extending
    ctrl.set_speed_percent(SPEED_PERCENT, relay_str)
    ctrl.open_relay_fwd(relay_str)
    print(f"Extending to {TARGET_MM:.1f} mm ({threshold_bits} bits) @ {SPEED_PERCENT:.0f}% speed")

    # 2) loop until we hit or exceed threshold_bits
    while True:
        bits = ctrl.get_position_bits(relay_str)
        mm   = ctrl.get_position_mm(relay_str)
        V    = ctrl.get_position_voltage(relay_str)
        print(f"{bits:5d} bits / {mm:7.1f} mm / {V:5.2f} V", end="\r")

        if bits >= threshold_bits:
            break
        time.sleep(POLL_INTERVAL)

    # 3) stop fully (zero drive + drop relays) and final print
    ctrl.stop(relay_str)
    print(f"{bits:5d} bits / {mm:7.1f} mm / {V:5.2f} V  ✓")

if __name__ == "__main__":
    main()
