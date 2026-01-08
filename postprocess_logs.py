#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")  # ensure PNG saving works headless / on Windows
import matplotlib.pyplot as plt

# Your shared conversion module (rubber/grouser)
from controller.conversions import (
    adc_bits_to_voltage,
    adc_bits_to_length,
    bits_to_torque,
)

# -------------------------
# Pressure conversions (match ManualPressureWindow)
# -------------------------
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


# -------------------------
# CSV loading (comma-delimited)
# -------------------------
def read_log_csv(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=",",
        engine="python",
        on_bad_lines="skip",
    )


# -------------------------
# Detect test prefix from filename
# -------------------------
def detect_prefix(file_path: Path) -> str:
    name = file_path.name.lower()
    for p in ("rubber", "pressure", "grouser"):
        if name.startswith(p):
            return p
    stem = file_path.stem.lower()
    head = stem.split("_")[0].split("-")[0]
    return head if head in ("rubber", "pressure", "grouser") else "unknown"


# -------------------------
# Fuzzy column detection
# -------------------------
def _norm(s: str) -> str:
    return "".join(ch for ch in s.lower() if ch.isalnum() or ch == "_")


def find_first_col(df: pd.DataFrame, must_contain: list[str]) -> str | None:
    for c in df.columns:
        nc = _norm(str(c))
        if all(tok in nc for tok in must_contain):
            return c
    return None


def detect_time_col(df: pd.DataFrame) -> str | None:
    for tokens in (["time", "ms"], ["timestamp", "ms"], ["t", "ms"], ["ms"]):
        c = find_first_col(df, tokens)
        if c:
            return c
    return find_first_col(df, ["time"])


def add_time_seconds(df: pd.DataFrame) -> tuple[pd.DataFrame, str | None]:
    used = detect_time_col(df)
    if not used:
        return df, None

    s = pd.to_numeric(df[used], errors="coerce")
    if s.notna().sum() == 0:
        return df, None

    t0 = float(s.dropna().iloc[0])
    df["t_s"] = (s - t0) / 1000.0
    return df, used


# -------------------------
# Resolve raw bit columns
# -------------------------
def resolve_raw_columns(df: pd.DataFrame, prefix: str) -> dict[str, str]:
    if prefix == "rubber":
        return {
            "pos_bits": find_first_col(df, ["rubber", "pos"]),
            "y_bits":   find_first_col(df, ["rubber", "torque"]),
        }
    if prefix == "grouser":
        return {
            "pos_bits": find_first_col(df, ["grouser", "pos"]),
            "y_bits":   find_first_col(df, ["grouser", "torque"]),
        }
    if prefix == "pressure":
        return {
            "pos_bits": find_first_col(df, ["pressure", "pos"]),
            "y_bits":   find_first_col(df, ["pressure", "force"]),
        }
    return {"pos_bits": None, "y_bits": None}


def convert_columns(df: pd.DataFrame, prefix: str, raw: dict[str, str]):
    if prefix in ("rubber", "grouser"):
        pos_eng = f"{prefix}_pos_mm"
        y_eng   = f"{prefix}_torque_Nm"
        y_label = "Torque [Nm]"

        if raw["pos_bits"] in df:
            df[pos_eng] = pd.to_numeric(df[raw["pos_bits"]], errors="coerce").apply(
                lambda b: adc_bits_to_length(int(b)) if pd.notna(b) else np.nan
            )
        if raw["y_bits"] in df:
            df[y_eng] = pd.to_numeric(df[raw["y_bits"]], errors="coerce").apply(
                lambda b: bits_to_torque(int(b)) if pd.notna(b) else np.nan
            )

    elif prefix == "pressure":
        pos_eng = "pressure_pos_mm"
        y_eng   = "pressure_force_N"
        y_label = "Force [N]"

        if raw["pos_bits"] in df:
            df[pos_eng] = pd.to_numeric(df[raw["pos_bits"]], errors="coerce").apply(
                pressure_bits_to_pos_mm
            )
        if raw["y_bits"] in df:
            df[y_eng] = pd.to_numeric(df[raw["y_bits"]], errors="coerce").apply(
                pressure_bits_to_force_newton
            )
    else:
        return df, None

    return df, {
        "pos": pos_eng,
        "y": y_eng,
        "y_label": y_label,
    }


# -------------------------
# Plot helpers
# -------------------------
def _save_plot(path: Path, x, y, xlabel: str, ylabel: str, title: str):
    df = pd.DataFrame({"x": x, "y": y}).dropna()
    if len(df) < 2:
        return

    plt.figure()
    plt.plot(df["x"], df["y"])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def _save_dual_y_plot(
    path: Path,
    t,
    pos,
    y,
    y_label: str,
    title: str,
):
    df = pd.DataFrame({"t": t, "pos": pos, "y": y}).dropna()
    if len(df) < 2:
        return

    fig, ax1 = plt.subplots()

    # Position (blue)
    l1, = ax1.plot(df["t"], df["pos"], color="tab:blue", label="Position [mm]")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position [mm]", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")

    # Torque / Force (red)
    ax2 = ax1.twinx()
    l2, = ax2.plot(df["t"], df["y"], color="tab:red", label=y_label)
    ax2.set_ylabel(y_label, color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")

    ax1.legend(handles=[l1, l2], loc="best")
    fig.suptitle(title)
    fig.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close(fig)


# -------------------------
# Main processing
# -------------------------
def process_file(path: Path, overwrite: bool, make_pngs: bool):
    df = read_log_csv(path)
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="ignore")

    prefix = detect_prefix(path)
    df, _ = add_time_seconds(df)

    raw = resolve_raw_columns(df, prefix)
    df, meta = convert_columns(df, prefix, raw)

    out = path if overwrite else path.with_name(path.stem + "_processed.csv")
    df.to_csv(out, index=False)

    if make_pngs and meta and "t_s" in df:
        _save_plot(
            out.with_name(out.stem + f"_{prefix}_pos_vs_time.png"),
            df["t_s"], df[meta["pos"]],
            "Time [s]", "Position [mm]",
            f"{prefix.capitalize()}: Position vs Time",
        )

        _save_plot(
            out.with_name(out.stem + f"_{prefix}_y_vs_time.png"),
            df["t_s"], df[meta["y"]],
            "Time [s]", meta["y_label"],
            f"{prefix.capitalize()}: {meta['y_label']} vs Time",
        )

        _save_dual_y_plot(
            out.with_name(out.stem + f"_{prefix}_pos_and_y_vs_time.png"),
            df["t_s"], df[meta["pos"]], df[meta["y"]],
            meta["y_label"],
            f"{prefix.capitalize()}: Position & {meta['y_label']} vs Time",
        )


# -------------------------
# CLI
# -------------------------
def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--recursive", action="store_true")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)

    p = Path(args.input).resolve()
    files = [p] if p.is_file() else sorted(p.glob("**/*.csv" if args.recursive else "*.csv"))

    for f in files:
        process_file(f, args.overwrite, not args.no_plots)
        print(f"[OK] {f.name}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
