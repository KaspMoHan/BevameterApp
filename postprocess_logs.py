#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd

# ---- import your shared conversions (rubber / grouser) ----
from controller.conversions import (
    adc_bits_to_voltage,
    adc_bits_to_length,
    bits_to_torque,
)

# ==========================================================
# Pressure conversions (EXACTLY matching ManualPressureWindow)
# ==========================================================
def _clamp_05_45(v: float) -> float:
    if v < 0.5:
        return 0.5
    if v > 4.5:
        return 4.5
    return v


def pressure_bits_to_force_newton(bits: float) -> float:
    try:
        v = float(adc_bits_to_voltage(int(bits)))
        v = _clamp_05_45(v)
        return (v - 0.5) * 12500.0
    except Exception:
        return np.nan


def pressure_bits_to_pos_mm(bits: float) -> float:
    try:
        v = float(adc_bits_to_voltage(int(bits)))
        return (v - 0.38) * 336.0
    except Exception:
        return np.nan


def bits_to_volts(bits: float) -> float:
    try:
        return float(adc_bits_to_voltage(int(bits)))
    except Exception:
        return np.nan


# ==========================================================
# Column mapping
# raw_column -> (new_column, conversion_fn)
# ==========================================================
CONVERSIONS = {
    # ---- Pressure ----
    "pressure_force": ("pressure_force_N", pressure_bits_to_force_newton),
    "pressure_pos":   ("pressure_pos_mm",  pressure_bits_to_pos_mm),

    # ---- Rubber ----
    "rubber_pos":     ("rubber_pos_mm",
                       lambda b: adc_bits_to_length(int(b)) if pd.notna(b) else np.nan),
    "rubber_torque":  ("rubber_torque_Nm",
                       lambda b: bits_to_torque(int(b)) if pd.notna(b) else np.nan),

    # ---- Grouser ----
    "grouser_pos":    ("grouser_pos_mm",
                       lambda b: adc_bits_to_length(int(b)) if pd.notna(b) else np.nan),
    "grouser_torque": ("grouser_torque_Nm",
                       lambda b: bits_to_torque(int(b)) if pd.notna(b) else np.nan),
}


# ==========================================================
# CSV loading (semicolon-delimited)
# ==========================================================
def read_log_csv(path: Path) -> pd.DataFrame:
    """
    Read semicolon-delimited log CSV.
    Uses python engine to tolerate malformed rows.
    """
    return pd.read_csv(
        path,
        sep=";",
        engine="python",
        on_bad_lines="skip",   # skip broken rows instead of crashing
    )


# ==========================================================
# Processing
# ==========================================================
def process_file(path: Path, add_volts: bool = True, overwrite: bool = False) -> Path:
    df = read_log_csv(path)

    # Attempt numeric conversion where possible
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="ignore")

    converted_raw_cols: list[str] = []

    # Apply conversions
    for raw_col, (new_col, fn) in CONVERSIONS.items():
        if raw_col in df.columns:
            converted_raw_cols.append(raw_col)
            series = pd.to_numeric(df[raw_col], errors="coerce")
            df[new_col] = series.apply(fn)

    # Optional: add voltage columns for converted raw channels
    if add_volts:
        for col in converted_raw_cols:
            df[f"{col}_V"] = pd.to_numeric(df[col], errors="coerce").apply(bits_to_volts)

    # Output file
    out_path = path if overwrite else path.with_name(path.stem + "_processed" + path.suffix)
    df.to_csv(out_path, index=False, sep=";")

    return out_path


def iter_csv_files(p: Path, recursive: bool) -> list[Path]:
    if p.is_file():
        return [p]
    if not p.exists():
        return []
    pattern = "**/*.csv" if recursive else "*.csv"
    return sorted(p.glob(pattern))


# ==========================================================
# CLI
# ==========================================================
def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(
        description="Postprocess Bevameter logs: ADC bits → engineering units."
    )
    ap.add_argument("input", type=str, help="CSV file or folder")
    ap.add_argument("--recursive", action="store_true", help="Process folders recursively")
    ap.add_argument("--no-volts", action="store_true", help="Do not add *_V columns")
    ap.add_argument("--overwrite", action="store_true", help="Overwrite input CSV")
    args = ap.parse_args(argv)

    p = Path(args.input).expanduser().resolve()
    files = iter_csv_files(p, args.recursive)

    if not files:
        print(f"[ERROR] No CSV files found at: {p}")
        return 2

    for f in files:
        try:
            out = process_file(
                f,
                add_volts=(not args.no_volts),
                overwrite=args.overwrite,
            )
            print(f"[OK] {f.name} → {out.name}")
        except Exception as e:
            print(f"[FAIL] {f}: {e}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
