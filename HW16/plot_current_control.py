#!/usr/bin/env python3
"""Trigger the HW16 current-control run and plot the returned serial data."""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
import time
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "hw16_mplconfig"))
os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "hw16_cache"))

import matplotlib.pyplot as plt
import serial


DEFAULT_BAUD = 115200
DEFAULT_SAMPLES = 800


def auto_port() -> str | None:
    candidates = []
    for pattern in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*"):
        candidates.extend(glob.glob(pattern))
    return sorted(candidates)[0] if candidates else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-p", "--port", default=None, help="serial port, for example /dev/cu.usbmodem1101")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    parser.add_argument("-n", "--samples", type=int, default=DEFAULT_SAMPLES, help="samples expected from the Nucleo")
    parser.add_argument("--csv", default="current_control_capture.csv", help="CSV output path")
    parser.add_argument("--plot", default="current_control_plot.png", help="plot output path")
    parser.add_argument("--no-show", action="store_true", help="save the plot without opening a window")
    return parser.parse_args()


def collect_rows(port: str, baud: int, samples: int) -> tuple[list[dict[str, int | None]], bool, bool]:
    rows: list[dict[str, int | None]] = []
    header_seen = False
    done_seen = False
    protocol: str | None = None
    deadline = time.monotonic() + 15.0

    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(b"a")

        while time.monotonic() < deadline and len(rows) < samples:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line in {"done", "ready"}:
                done_seen = True
                break
            if line.startswith("idx,"):
                header_seen = True
                protocol = "local"
                continue
            if line.startswith("index,adc,"):
                header_seen = True
                protocol = "reference"
                continue
            if line.startswith("run_complete"):
                continue
            if not header_seen:
                continue

            fields = line.split(",")
            if protocol == "local" and len(fields) == 5:
                idx, desired, current, pwm, adc = (int(field) for field in fields)
                rows.append(
                    {
                        "idx": idx,
                        "desired_ma3": desired,
                        "current_ma3": current,
                        "pwm_cmd": pwm,
                        "adc": adc,
                    }
                )
            elif protocol == "reference" and len(fields) == 4:
                idx, adc, desired, current = (int(field) for field in fields)
                rows.append(
                    {
                        "idx": idx,
                        "desired_ma3": desired,
                        "current_ma3": current,
                        "pwm_cmd": None,
                        "adc": adc,
                    }
                )
            else:
                print(f"Skipping malformed line: {line}", file=sys.stderr)

    return rows, header_seen, done_seen


def write_csv(rows: list[dict[str, int | None]], path: Path) -> None:
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=["idx", "desired_ma3", "current_ma3", "pwm_cmd", "adc"])
        writer.writeheader()
        writer.writerows(rows)


def plot_rows(rows: list[dict[str, int | None]], path: Path, show: bool) -> None:
    time_ms = [int(row["idx"]) for row in rows]
    desired_ma = [int(row["desired_ma3"]) / 3.0 for row in rows]
    current_ma = [int(row["current_ma3"]) / 3.0 for row in rows]
    pwm_cmd = [row["pwm_cmd"] for row in rows]
    adc = [int(row["adc"]) for row in rows]
    has_pwm = any(value is not None for value in pwm_cmd)

    fig, (ax_current, ax_pwm) = plt.subplots(2, 1, sharex=True, figsize=(9, 6), constrained_layout=True)
    ax_current.plot(time_ms, desired_ma, label="desired current", linewidth=2)
    ax_current.plot(time_ms, current_ma, label="measured current", linewidth=2)
    ax_current.set_ylabel("current (mA)")
    ax_current.grid(True, alpha=0.3)
    ax_current.legend()

    if has_pwm:
        ax_pwm.plot(time_ms, [int(value) if value is not None else 0 for value in pwm_cmd],
                    label="PWM command", color="tab:green")
    ax_pwm.plot(time_ms, adc, label="ADC position", color="tab:orange", alpha=0.75)
    ax_pwm.set_xlabel("time (ms)")
    ax_pwm.set_ylabel("counts")
    ax_pwm.grid(True, alpha=0.3)
    ax_pwm.legend()

    fig.suptitle("HW16 Current Control")
    fig.savefig(path, dpi=180)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    args = parse_args()
    port = args.port or auto_port()
    if port is None:
        print("No /dev/cu.usbmodem* serial port found. Pass --port explicitly.", file=sys.stderr)
        return 2

    rows, header_seen, done_seen = collect_rows(port, args.baud, args.samples)
    if not rows:
        if header_seen and done_seen:
            print(
                "Controller replied but logged 0 samples. The ADC safety check probably tripped immediately; "
                "center the servo pot and check the PA0/3V3/GND wiring.",
                file=sys.stderr,
            )
            return 1
        print(f"No data received from {port}. Check firmware, wiring, and serial port.", file=sys.stderr)
        return 1

    csv_path = Path(args.csv)
    plot_path = Path(args.plot)
    write_csv(rows, csv_path)
    plot_rows(rows, plot_path, show=not args.no_show)
    print(f"Captured {len(rows)} samples from {port}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
