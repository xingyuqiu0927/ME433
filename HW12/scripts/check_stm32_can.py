#!/usr/bin/env python3
import argparse
import glob
import os
import select
import sys
import termios
import time


BAUD = 115200


def choose_port(requested):
    if requested:
        return requested

    candidates = []
    for pattern in (
        "/dev/cu.usbmodem*",
        "/dev/tty.usbmodem*",
        "/dev/cu.debug-console",
        "/dev/tty.debug-console",
    ):
        candidates.extend(glob.glob(pattern))

    if not candidates:
        raise SystemExit("No STM32 serial port found. Pass --port /dev/cu.usbmodemXXX.")

    return sorted(candidates)[0]


def configure_serial(fd):
    attrs = termios.tcgetattr(fd)

    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                  termios.ISTRIP | termios.INLCR | termios.IGNCR |
                  termios.ICRNL | termios.IXON)
    attrs[1] &= ~termios.OPOST
    attrs[2] &= ~(termios.CSIZE | termios.PARENB)
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                  termios.ISIG | termios.IEXTEN)
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def update_state(line, state):
    if "CAN INTERNAL LOOPBACK PASS" in line:
        state["loopback_pass"] = True
    if "CAN INTERNAL LOOPBACK FAIL" in line:
        state["loopback_fail"] = True
    if line.startswith("TX ID="):
        state["tx_requested"] = True
    if "TX complete" in line:
        state["tx_complete"] = True
    if "CAN TX timeout" in line:
        state["tx_timeout"] = True
    if line.startswith("RX ID="):
        state["rx_seen"] = True


def print_summary(state):
    print("\nResult:")

    if state["loopback_pass"]:
        print("  PASS: STM32 CAN controller works in internal loopback.")
    elif state["loopback_fail"]:
        print("  FAIL: STM32 internal CAN loopback failed.")
    else:
        print("  UNKNOWN: did not see the internal loopback line. Reset the STM32 while this script is running.")

    if state["tx_complete"]:
        print("  PASS: external CAN bus ACKed a transmitted frame.")
    elif state["rx_seen"]:
        print("  PASS: STM32 received an external CAN frame.")
    elif state["tx_timeout"]:
        print("  FAIL: STM32 tried to transmit, but no external CAN ACK was received.")
    elif state["tx_requested"]:
        print("  UNKNOWN: TX was requested but no completion/timeout line was seen yet.")
    else:
        print("  WAITING: press the blue USER button to test the external CAN bus.")

    if state["tx_timeout"]:
        print("\nCheck CANH/CANL, common GND, TJA1051 S/STB to GND, 250 kbit/s on the other node, and bus termination.")


def main():
    parser = argparse.ArgumentParser(description="Monitor HW12 STM32 CAN diagnostic serial output.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem103")
    parser.add_argument("--seconds", type=float, default=20.0, help="How long to monitor")
    args = parser.parse_args()

    port = choose_port(args.port)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_serial(fd)

    state = {
        "loopback_pass": False,
        "loopback_fail": False,
        "tx_requested": False,
        "tx_complete": False,
        "tx_timeout": False,
        "rx_seen": False,
    }

    print(f"Monitoring {port} at {BAUD} baud for {args.seconds:g} seconds.")
    print("Reset the STM32 to see internal loopback, then press the blue USER button for external CAN.")

    deadline = time.monotonic() + args.seconds
    buffer = b""

    try:
        while time.monotonic() < deadline:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue

            chunk = os.read(fd, 1024)
            if not chunk:
                continue

            buffer += chunk
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode(errors="replace").strip()
                if line:
                    print(line)
                    update_state(line, state)
    finally:
        os.close(fd)

    if buffer.strip():
        line = buffer.decode(errors="replace").strip()
        print(line)
        update_state(line, state)

    print_summary(state)
    return 0 if (state["loopback_pass"] and (state["tx_complete"] or state["rx_seen"])) else 1


if __name__ == "__main__":
    sys.exit(main())
