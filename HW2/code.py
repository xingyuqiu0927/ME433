import time

try:
    import board
    import pwmio
except ModuleNotFoundError as exc:
    raise SystemExit(
        "This script is for CircuitPython on the Pico 2. Copy code.py to the "
        "CIRCUITPY drive and run it there, not with desktop Python."
    ) from exc

SERVO_PIN = board.GP15
SERVO_FREQUENCY_HZ = 50
SERVO_MIN_DUTY = 0.015
SERVO_MAX_DUTY = 0.12
SERVO_STEP_DEG = 2
SERVO_STEP_DELAY_S = 0.02


def set_servo_angle(servo_pwm, angle_deg):
    if angle_deg < 0:
        angle_deg = 0
    elif angle_deg > 180:
        angle_deg = 180

    duty_fraction = SERVO_MIN_DUTY + (angle_deg / 180) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)
    servo_pwm.duty_cycle = int(duty_fraction * 65535)


servo = pwmio.PWMOut(SERVO_PIN, frequency=SERVO_FREQUENCY_HZ, duty_cycle=0)

while True:
    for angle in range(0, 181, SERVO_STEP_DEG):
        set_servo_angle(servo, angle)
        time.sleep(SERVO_STEP_DELAY_S)

    for angle in range(180, -1, -SERVO_STEP_DEG):
        set_servo_angle(servo, angle)
        time.sleep(SERVO_STEP_DELAY_S)
