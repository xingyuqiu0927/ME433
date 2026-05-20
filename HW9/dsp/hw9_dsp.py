from pathlib import Path
import csv
import os

import numpy as np

ROOT = Path(__file__).resolve().parent
PLOT_DIR = ROOT / "plots"
CACHE_DIR = ROOT / ".cache"
CACHE_DIR.mkdir(exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(CACHE_DIR))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


FILES = ["sigA.csv", "sigB.csv", "sigC.csv", "sigD.csv"]

# Chosen by comparing the filtered time-domain signals and FFTs.
MAF_WINDOWS = {
    "sigA.csv": 100,
    "sigB.csv": 200,
    "sigC.csv": 100,
    "sigD.csv": 20,
}

IIR_WEIGHTS = {
    "sigA.csv": (0.95, 0.05),
    "sigB.csv": (0.98, 0.02),
    "sigC.csv": (0.97, 0.03),
    "sigD.csv": (0.93, 0.07),
}

FIR_SETTINGS = {
    "sigA.csv": {"cutoff": 60.0, "bandwidth": 100.0, "window": "hamming"},
    "sigB.csv": {"cutoff": 10.0, "bandwidth": 20.0, "window": "hamming"},
    "sigC.csv": {"cutoff": 15.0, "bandwidth": 20.0, "window": "hamming"},
    "sigD.csv": {"cutoff": 5.0, "bandwidth": 8.0, "window": "hamming"},
}


def read_csv(path):
    time = []
    signal = []
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            time.append(float(row[0]))
            signal.append(float(row[1]))
    return np.array(time), np.array(signal)


def sample_rate(time):
    return (len(time) - 1) / (time[-1] - time[0])


def one_sided_fft(time, signal):
    fs = sample_rate(time)
    y = signal - np.mean(signal)
    freq = np.fft.rfftfreq(len(y), d=1.0 / fs)
    mag = np.abs(np.fft.rfft(y)) / len(y)
    return freq, mag


def moving_average(signal, window_size):
    filtered = np.zeros_like(signal)
    running_sum = 0.0
    for i, value in enumerate(signal):
        running_sum += value
        if i >= window_size:
            running_sum -= signal[i - window_size]
        filtered[i] = running_sum / window_size
    return filtered


def iir_filter(signal, a_weight, b_weight):
    filtered = np.zeros_like(signal)
    for i, value in enumerate(signal):
        previous = filtered[i - 1] if i > 0 else 0.0
        filtered[i] = a_weight * previous + b_weight * value
    return filtered


def window_values(name, taps):
    if name == "hamming":
        return np.hamming(taps)
    if name == "hann":
        return np.hanning(taps)
    if name == "blackman":
        return np.blackman(taps)
    raise ValueError(f"Unsupported window: {name}")


def fir_tap_count(fs, bandwidth, window_name):
    scale = {
        "hann": 3.1,
        "hamming": 4.0,
        "blackman": 5.5,
    }[window_name]
    taps = int(np.ceil(scale * fs / bandwidth))
    if taps % 2 == 0:
        taps += 1
    return max(taps, 3)


def lowpass_sinc_weights(fs, cutoff, bandwidth, window_name):
    taps = fir_tap_count(fs, bandwidth, window_name)
    n = np.arange(taps)
    center = (taps - 1) / 2
    normalized_cutoff = cutoff / fs
    weights = 2 * normalized_cutoff * np.sinc(2 * normalized_cutoff * (n - center))
    weights *= window_values(window_name, taps)
    weights /= np.sum(weights)
    return weights


def fir_filter(signal, weights):
    filtered = np.zeros_like(signal)
    for i in range(len(signal)):
        count = min(i + 1, len(weights))
        recent_samples = signal[i - count + 1 : i + 1][::-1]
        filtered[i] = np.dot(weights[:count], recent_samples)
    return filtered


def style_axes(ax_time, ax_fft):
    ax_time.set_xlabel("Time (s)")
    ax_time.set_ylabel("Signal")
    ax_time.grid(True, alpha=0.25)

    ax_fft.set_xlabel("Frequency (Hz)")
    ax_fft.set_ylabel("FFT magnitude")
    ax_fft.grid(True, which="both", alpha=0.25)


def plot_signal_and_fft(csv_name, time, signal):
    freq, mag = one_sided_fft(time, signal)
    fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    ax_time.plot(time, signal, color="black", linewidth=1.0)
    ax_time.set_title(f"{csv_name} signal vs time")
    ax_fft.loglog(freq[1:], mag[1:], color="black", linewidth=1.0)
    ax_fft.set_title(f"{csv_name} FFT, sample rate = {sample_rate(time):.1f} Hz")
    style_axes(ax_time, ax_fft)
    fig.savefig(PLOT_DIR / f"part4_{csv_name[:-4]}_signal_fft.png", dpi=200)
    plt.close(fig)


def plot_filtered(csv_name, time, original, filtered, title, filename):
    freq, mag = one_sided_fft(time, original)
    freq_f, mag_f = one_sided_fft(time, filtered)

    fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    ax_time.plot(time, original, color="black", linewidth=1.0, label="unfiltered")
    ax_time.plot(time, filtered, color="red", linewidth=1.0, label="filtered")
    ax_time.set_title(title)
    ax_time.legend()

    ax_fft.loglog(freq[1:], mag[1:], color="black", linewidth=1.0, label="unfiltered FFT")
    ax_fft.loglog(freq_f[1:], mag_f[1:], color="red", linewidth=1.0, label="filtered FFT")
    ax_fft.set_title(f"{csv_name} before and after FFT")
    ax_fft.legend()
    style_axes(ax_time, ax_fft)

    fig.savefig(PLOT_DIR / filename, dpi=200)
    plt.close(fig)


def main():
    PLOT_DIR.mkdir(exist_ok=True)
    print("Sample rates:")

    for csv_name in FILES:
        time, signal = read_csv(ROOT / csv_name)
        fs = sample_rate(time)
        print(f"  {csv_name}: {fs:.1f} Hz")

        plot_signal_and_fft(csv_name, time, signal)

        window_size = MAF_WINDOWS[csv_name]
        maf = moving_average(signal, window_size)
        plot_filtered(
            csv_name,
            time,
            signal,
            maf,
            f"{csv_name} moving average filter, X = {window_size} samples",
            f"part5_{csv_name[:-4]}_maf.png",
        )

        a_weight, b_weight = IIR_WEIGHTS[csv_name]
        iir = iir_filter(signal, a_weight, b_weight)
        plot_filtered(
            csv_name,
            time,
            signal,
            iir,
            f"{csv_name} IIR filter, A = {a_weight:.2f}, B = {b_weight:.2f}",
            f"part6_{csv_name[:-4]}_iir.png",
        )

        settings = FIR_SETTINGS[csv_name]
        weights = lowpass_sinc_weights(
            fs, settings["cutoff"], settings["bandwidth"], settings["window"]
        )
        fir = fir_filter(signal, weights)
        plot_filtered(
            csv_name,
            time,
            signal,
            fir,
            (
                f"{csv_name} FIR low-pass sinc, {len(weights)} weights, "
                f"cutoff = {settings['cutoff']:.1f} Hz, "
                f"bandwidth = {settings['bandwidth']:.1f} Hz, "
                f"{settings['window']} window"
            ),
            f"part7_{csv_name[:-4]}_fir.png",
        )

    print(f"Plots saved in {PLOT_DIR}")


if __name__ == "__main__":
    main()
