import serial
import csv
import time
from datetime import datetime

# --- USER SETTINGS ---
PORT = "COM3"  # <-- Change this to your Opta's COM port
BAUD = 115200
OUTPUT_FILE = f"samples_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
EXPECTED_ROWS = 1000  # how many samples your Arduino collects
ZERO_VOLTAGE = 0.02  # offset voltage for zero torque
ADD_TORQUE = True  # set to False if you don’t want Nm column


def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        time.sleep(2)  # let Opta reset on connection

        # --- Trigger sampling on the Opta ---
        print("Sending 's' to start sampling...")
        ser.write(b's')  # send the trigger
        ser.flush()

        # --- Prepare CSV file ---
        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["Index", "Time_ms", "Raw", "Voltage_V"]
            if ADD_TORQUE:
                header.append("Torque_Nm")
            writer.writerow(header)

            print("Waiting for data...")
            rows_written = 0
            start_time = time.time()

            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    # timeout safety
                    if time.time() - start_time > 30:
                        print("Timeout waiting for data.")
                        break
                    continue

                if line.startswith("Index") or line.startswith("Raw ADC"):
                    # skip header lines echoed from Arduino
                    continue

                if "," in line:
                    parts = line.split(",")
                    if len(parts) >= 4:
                        idx, t_ms, raw, voltage = parts[:4]
                        try:
                            idx = int(idx)
                            t_ms = int(t_ms)
                            raw = int(raw)
                            voltage = float(voltage)
                        except ValueError:
                            continue

                        row = [idx, t_ms, raw, voltage]
                        if ADD_TORQUE:
                            torque = 50.0 * (voltage - ZERO_VOLTAGE)
                            row.append(round(torque, 4))
                        writer.writerow(row)
                        rows_written += 1

                        if rows_written % 100 == 0:
                            print(f"{rows_written} samples logged...")

                        if rows_written >= EXPECTED_ROWS:
                            print("All expected samples received.")
                            break

            print(f"✅ Done. {rows_written} samples saved to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
