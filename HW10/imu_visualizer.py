import math
import os
import random
import time

from pygame import Rect

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


WIDTH = 900
HEIGHT = 640
TITLE = "HW10 IMU Tilt"

BAUDRATE = 115200
BOARD = Rect(38, 38, 600, 560)
BALL_RADIUS = 17
TARGET_RADIUS = 24
TARGET_MARGIN = 44

TILT_COUNTS_FOR_FULL_SPEED = 10500.0
BALL_ACCELERATION = 780.0
BALL_FRICTION = 2.4
MAX_SPEED = 520.0

# These match the IMU axis mapping used in HW6.
SWAP_AXES = False
INVERT_X = True
INVERT_Y = False

BACKGROUND = (34, 33, 31)
BOARD_FILL = (232, 230, 218)
GRID = (203, 201, 190)
BORDER = (74, 72, 66)
TEXT = (242, 239, 229)
MUTED_TEXT = (181, 177, 164)
BALL = (26, 147, 136)
BALL_DARK = (18, 94, 88)
TARGET = (224, 128, 43)
TARGET_DARK = (129, 73, 30)
GAUGE_BG = (63, 61, 56)
GAUGE_POSITIVE = (99, 178, 106)
GAUGE_NEGATIVE = (207, 81, 72)

ser = None
serial_buffer = ""
serial_status = "Serial not opened"
last_open_attempt = 0.0
last_sample_time = 0.0
last_line = ""

ax = 0
ay = 0
az = 16384
gx = 0
gy = 0
gz = 0
tilt_x = 0.0
tilt_y = 0.0
roll_deg = 0.0
pitch_deg = 0.0

ball_x = float(BOARD.centerx)
ball_y = float(BOARD.centery)
velocity_x = 0.0
velocity_y = 0.0
score = 0


def clamp(value, low, high):
    return max(low, min(high, value))


def choose_serial_port():
    env_port = os.environ.get("PICO_PORT")
    if env_port:
        return env_port

    if list_ports is None:
        return None

    ports = list(list_ports.comports())
    for port in ports:
        device = str(port.device)
        text = " ".join(
            str(item or "")
            for item in (port.device, port.description, port.manufacturer)
        ).lower()
        if "usbmodem" in device.lower() or "pico" in text or "rp2040" in text or "rp2350" in text:
            return device

    return None


def open_serial():
    global ser, serial_status, last_open_attempt

    last_open_attempt = time.monotonic()

    if serial is None:
        serial_status = "Install pyserial"
        return

    port = choose_serial_port()
    if port is None:
        serial_status = "No Pico serial port"
        return

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0)
        ser.reset_input_buffer()
        serial_status = "Serial: " + port
    except (OSError, serial.SerialException) as exc:
        ser = None
        serial_status = "Serial error: " + str(exc)


def random_target():
    return (
        random.uniform(BOARD.left + TARGET_MARGIN, BOARD.right - TARGET_MARGIN),
        random.uniform(BOARD.top + TARGET_MARGIN, BOARD.bottom - TARGET_MARGIN),
    )


target_x, target_y = random_target()


def parse_sample(line):
    global ax, ay, az, gx, gy, gz, tilt_x, tilt_y
    global roll_deg, pitch_deg, last_sample_time, last_line, serial_status

    line = line.strip()
    if not line or line.startswith("#"):
        return

    parts = [part.strip() for part in line.split(",")]
    if parts[0].lower() == "error":
        serial_status = "Pico: " + line[:36]
        return
    if parts[0].lower() == "imu":
        parts = parts[1:]
    if len(parts) < 6:
        return

    try:
        values = [int(float(part)) for part in parts[:6]]
    except ValueError:
        return

    ax, ay, az, gx, gy, gz = values

    mapped_x = float(ax)
    mapped_y = float(ay)
    if SWAP_AXES:
        mapped_x, mapped_y = mapped_y, mapped_x
    if INVERT_X:
        mapped_x = -mapped_x
    if INVERT_Y:
        mapped_y = -mapped_y

    tilt_x = clamp(mapped_x / TILT_COUNTS_FOR_FULL_SPEED, -1.0, 1.0)
    tilt_y = clamp(mapped_y / TILT_COUNTS_FOR_FULL_SPEED, -1.0, 1.0)

    roll_deg = math.degrees(math.atan2(float(ay), float(az) if az != 0 else 1.0))
    pitch_deg = math.degrees(math.atan2(-float(ax), math.sqrt(float(ay * ay + az * az))))
    last_sample_time = time.monotonic()
    last_line = line


def poll_serial():
    global ser, serial_buffer, serial_status

    if ser is None:
        if time.monotonic() - last_open_attempt > 2.0:
            open_serial()
        return

    try:
        waiting = ser.in_waiting
        if waiting <= 0:
            return

        chunk = ser.read(waiting).decode("utf-8", errors="ignore")
        serial_buffer += chunk
        while "\n" in serial_buffer:
            line, serial_buffer = serial_buffer.split("\n", 1)
            parse_sample(line)
    except (OSError, serial.SerialException):
        serial_status = "Serial disconnected"
        try:
            ser.close()
        except OSError:
            pass
        ser = None


def keyboard_drive():
    x = 0.0
    y = 0.0
    if keyboard.left or keyboard.a:
        x -= 1.0
    if keyboard.right or keyboard.d:
        x += 1.0
    if keyboard.up or keyboard.w:
        y -= 1.0
    if keyboard.down or keyboard.s:
        y += 1.0
    return x, y


def update(dt=1.0 / 60.0):
    global ball_x, ball_y, velocity_x, velocity_y, target_x, target_y, score

    dt = min(dt, 0.05)
    poll_serial()

    key_x, key_y = keyboard_drive()
    drive_x = clamp(tilt_x + key_x, -1.0, 1.0)
    drive_y = clamp(tilt_y + key_y, -1.0, 1.0)

    velocity_x += drive_x * BALL_ACCELERATION * dt
    velocity_y += drive_y * BALL_ACCELERATION * dt

    damping = max(0.0, 1.0 - BALL_FRICTION * dt)
    velocity_x = clamp(velocity_x * damping, -MAX_SPEED, MAX_SPEED)
    velocity_y = clamp(velocity_y * damping, -MAX_SPEED, MAX_SPEED)

    ball_x += velocity_x * dt
    ball_y += velocity_y * dt

    left = BOARD.left + BALL_RADIUS
    right = BOARD.right - BALL_RADIUS
    top = BOARD.top + BALL_RADIUS
    bottom = BOARD.bottom - BALL_RADIUS

    if ball_x < left:
        ball_x = left
        velocity_x = abs(velocity_x) * 0.35
    elif ball_x > right:
        ball_x = right
        velocity_x = -abs(velocity_x) * 0.35

    if ball_y < top:
        ball_y = top
        velocity_y = abs(velocity_y) * 0.35
    elif ball_y > bottom:
        ball_y = bottom
        velocity_y = -abs(velocity_y) * 0.35

    if math.hypot(ball_x - target_x, ball_y - target_y) <= BALL_RADIUS + TARGET_RADIUS:
        score += 1
        target_x, target_y = random_target()
        velocity_x *= 0.45
        velocity_y *= 0.45


def draw_board():
    screen.draw.filled_rect(BOARD, BOARD_FILL)
    for x in range(BOARD.left + 60, BOARD.right, 60):
        screen.draw.line((x, BOARD.top), (x, BOARD.bottom), GRID)
    for y in range(BOARD.top + 60, BOARD.bottom, 60):
        screen.draw.line((BOARD.left, y), (BOARD.right, y), GRID)
    screen.draw.rect(BOARD, BORDER)


def draw_target():
    screen.draw.filled_circle((target_x + 3, target_y + 4), TARGET_RADIUS + 2, TARGET_DARK)
    screen.draw.filled_circle((target_x, target_y), TARGET_RADIUS, TARGET)
    screen.draw.circle((target_x, target_y), TARGET_RADIUS + 7, TARGET)
    screen.draw.filled_circle((target_x, target_y), 5, BOARD_FILL)


def draw_ball():
    screen.draw.filled_circle((ball_x + 4, ball_y + 5), BALL_RADIUS + 1, BALL_DARK)
    screen.draw.filled_circle((ball_x, ball_y), BALL_RADIUS, BALL)
    screen.draw.filled_circle((ball_x - 5, ball_y - 6), 5, (121, 218, 202))


def draw_bar(label, value, limit, x, y, width, height):
    screen.draw.text(label, (x, y - 22), fontsize=18, color=MUTED_TEXT)
    screen.draw.filled_rect(Rect(x, y, width, height), GAUGE_BG)
    center = x + width // 2
    screen.draw.line((center, y - 2), (center, y + height + 2), MUTED_TEXT)

    fraction = clamp(float(value) / float(limit), -1.0, 1.0)
    if fraction >= 0.0:
        fill = Rect(center, y, int((width // 2) * fraction), height)
        color = GAUGE_POSITIVE
    else:
        fill_width = int((width // 2) * -fraction)
        fill = Rect(center - fill_width, y, fill_width, height)
        color = GAUGE_NEGATIVE
    screen.draw.filled_rect(fill, color)
    screen.draw.rect(Rect(x, y, width, height), BORDER)


def shortened(text, max_chars):
    if len(text) <= max_chars:
        return text
    return text[:max_chars - 3] + "..."


def draw_sidebar():
    x = 680
    live = time.monotonic() - last_sample_time < 0.75
    live_color = GAUGE_POSITIVE if live else TARGET

    screen.draw.text("HW10 IMU Tilt", (x, 42), fontsize=34, color=TEXT)
    screen.draw.text("score " + str(score), (x, 91), fontsize=30, color=TARGET)
    screen.draw.text(shortened(serial_status, 28), (x, 135), fontsize=18, color=live_color)

    screen.draw.text("accel", (x, 180), fontsize=24, color=TEXT)
    draw_bar("x " + str(ax), ax, 16384, x, 232, 170, 18)
    draw_bar("y " + str(ay), ay, 16384, x, 286, 170, 18)
    draw_bar("z " + str(az), az, 16384, x, 340, 170, 18)

    screen.draw.text("tilt", (x, 390), fontsize=24, color=TEXT)
    screen.draw.text("roll  " + str(round(roll_deg, 1)), (x, 431), fontsize=20, color=MUTED_TEXT)
    screen.draw.text("pitch " + str(round(pitch_deg, 1)), (x, 461), fontsize=20, color=MUTED_TEXT)

    cx = x + 84
    cy = 535
    radius = 45
    screen.draw.circle((cx, cy), radius, MUTED_TEXT)
    screen.draw.line((cx - radius, cy), (cx + radius, cy), MUTED_TEXT)
    screen.draw.line((cx, cy - radius), (cx, cy + radius), MUTED_TEXT)
    screen.draw.filled_circle((cx + tilt_x * radius, cy + tilt_y * radius), 8, BALL)


def draw():
    screen.fill(BACKGROUND)
    draw_board()
    draw_target()
    draw_ball()
    draw_sidebar()


if __name__ == "__main__":
    import pgzrun

    open_serial()
    pgzrun.go()
