import argparse
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from serial.tools import list_ports


def default_port():
    ports = list(list_ports.comports())
    for port in ports:
        if "usbmodem" in port.device or "Pico" in port.description:
            return port.device
    return ports[0].device if ports else None


def read_samples(port, baudrate, samples, timeout):
    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        print(f"Using serial port {port}")
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(f"{samples}\n".encode("ascii"))
        ser.flush()

        collection_timeout = max(timeout, samples / 8.0 + 5.0)
        header_deadline = time.monotonic() + collection_timeout
        while time.monotonic() < header_deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line == "time_ms,raw,filtered":
                break
            if line.startswith("error,"):
                raise RuntimeError(line)
            if line:
                print(line)
        else:
            raise TimeoutError(
                "Timed out waiting for CSV data. The Pico replied, but it may still be collecting. "
                "Use fewer samples, increase --timeout, or check that the HX711 board is set to 80 Hz."
            )

        rows = []
        data_deadline = time.monotonic() + timeout + samples / 60.0
        while len(rows) < samples and time.monotonic() < data_deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line.startswith("error,"):
                raise RuntimeError(line)
            if line == "done":
                break
            parts = line.split(",")
            if len(parts) != 3:
                continue
            rows.append([float(part) for part in parts])

    if not rows:
        raise RuntimeError("No sample rows were received from the Pico.")
    if len(rows) < 2:
        raise RuntimeError("At least two samples are needed for the time and FFT plots.")

    return np.array(rows)


def save_time_plot(t, raw, filtered, filename):
    plt.figure()
    plt.plot(t, raw, label="raw")
    plt.plot(t, filtered, label="filtered")
    plt.xlabel("time (s)")
    plt.ylabel("HX711 counts")
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=200)


def save_fft_plot(t, raw, filtered, filename):
    dt = np.median(np.diff(t))
    raw_fft = np.abs(np.fft.rfft(raw - np.mean(raw)))
    filtered_fft = np.abs(np.fft.rfft(filtered - np.mean(filtered)))
    freq = np.fft.rfftfreq(len(t), dt)

    plt.figure()
    plt.plot(freq, raw_fft, label="raw")
    plt.plot(freq, filtered_fft, label="filtered")
    plt.xlabel("frequency (Hz)")
    plt.ylabel("magnitude")
    plt.xlim(0, min(45, freq[-1]))
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=200)

    return 1.0 / dt, freq[-1]


def main():
    parser = argparse.ArgumentParser(description="Collect and plot HW14 HX711 data.")
    parser.add_argument("-p", "--port", default=default_port(), help="serial port for the Pico")
    parser.add_argument("-n", "--samples", type=int, default=800, help="number of samples to collect")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="ignored by USB CDC, kept for pyserial")
    parser.add_argument("--timeout", type=float, default=10.0, help="serial read timeout in seconds")
    parser.add_argument("--no-show", action="store_true", help="save plots without opening a plot window")
    args = parser.parse_args()

    if args.port is None:
        print("No serial ports found. Plug in the Pico and run again.", file=sys.stderr)
        return 1

    data = read_samples(args.port, args.baudrate, args.samples, args.timeout)
    t = (data[:, 0] - data[0, 0]) / 1000.0
    raw = data[:, 1]
    filtered = data[:, 2]

    save_time_plot(t, raw, filtered, "hw14_data.png")
    sample_rate, nyquist = save_fft_plot(t, raw, filtered, "hw14_fft.png")

    print(f"Received {len(data)} samples from {args.port}")
    print(f"Estimated sample rate: {sample_rate:.2f} Hz, Nyquist: {nyquist:.2f} Hz")
    print("Saved hw14_data.png and hw14_fft.png")

    if not args.no_show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
