"""
L298N motor driver for 4WD car.

Wiring (BCM pin numbers, all configurable):
  ENA → GPIO 18 (Pin 12)  — left motors PWM
  IN1 → GPIO 23 (Pin 16)  — left motors direction
  IN2 → GPIO 24 (Pin 18)  — left motors direction
  ENB → GPIO 13 (Pin 33)  — right motors PWM
  IN3 → GPIO 27 (Pin 13)  — right motors direction
  IN4 → GPIO 22 (Pin 15)  — right motors direction
  GND → GND (Pin 6/9/14/20/25/30/34/39)
"""

import lgpio


PWM_FREQ = 1000  # Hz


class L298NDriver:
    def __init__(self, ena=18, in1=23, in2=24, enb=13, in3=27, in4=22, speed=75):
        self._ena = ena
        self._in1 = in1
        self._in2 = in2
        self._enb = enb
        self._in3 = in3
        self._in4 = in4
        self._speed = max(0, min(100, speed))

        self._h = lgpio.gpiochip_open(0)

        for pin in (in1, in2, in3, in4, ena, enb):
            try:
                lgpio.gpio_free(self._h, pin)
            except lgpio.error:
                pass
            lgpio.gpio_claim_output(self._h, pin, 0)

        lgpio.tx_pwm(self._h, ena, PWM_FREQ, 0)
        lgpio.tx_pwm(self._h, enb, PWM_FREQ, 0)

    def forward(self):
        lgpio.gpio_write(self._h, self._in1, 1)
        lgpio.gpio_write(self._h, self._in2, 0)
        lgpio.gpio_write(self._h, self._in3, 1)
        lgpio.gpio_write(self._h, self._in4, 0)
        lgpio.tx_pwm(self._h, self._ena, PWM_FREQ, self._speed)
        lgpio.tx_pwm(self._h, self._enb, PWM_FREQ, self._speed)

    def backward(self):
        lgpio.gpio_write(self._h, self._in1, 0)
        lgpio.gpio_write(self._h, self._in2, 1)
        lgpio.gpio_write(self._h, self._in3, 0)
        lgpio.gpio_write(self._h, self._in4, 1)
        lgpio.tx_pwm(self._h, self._ena, PWM_FREQ, self._speed)
        lgpio.tx_pwm(self._h, self._enb, PWM_FREQ, self._speed)

    def stop(self):
        lgpio.tx_pwm(self._h, self._ena, PWM_FREQ, 0)
        lgpio.tx_pwm(self._h, self._enb, PWM_FREQ, 0)
        lgpio.gpio_write(self._h, self._in1, 0)
        lgpio.gpio_write(self._h, self._in2, 0)
        lgpio.gpio_write(self._h, self._in3, 0)
        lgpio.gpio_write(self._h, self._in4, 0)

    def set_motors(self, left_speed: int, right_speed: int):
        """Independent left/right speed control. Range -100 to 100 (negative = backward)."""
        left_speed = max(-100, min(100, int(left_speed)))
        right_speed = max(-100, min(100, int(right_speed)))

        if left_speed > 0:
            lgpio.gpio_write(self._h, self._in1, 1)
            lgpio.gpio_write(self._h, self._in2, 0)
        elif left_speed < 0:
            lgpio.gpio_write(self._h, self._in1, 0)
            lgpio.gpio_write(self._h, self._in2, 1)
        else:
            lgpio.gpio_write(self._h, self._in1, 0)
            lgpio.gpio_write(self._h, self._in2, 0)

        if right_speed > 0:
            lgpio.gpio_write(self._h, self._in3, 1)
            lgpio.gpio_write(self._h, self._in4, 0)
        elif right_speed < 0:
            lgpio.gpio_write(self._h, self._in3, 0)
            lgpio.gpio_write(self._h, self._in4, 1)
        else:
            lgpio.gpio_write(self._h, self._in3, 0)
            lgpio.gpio_write(self._h, self._in4, 0)

        lgpio.tx_pwm(self._h, self._ena, PWM_FREQ, abs(left_speed))
        lgpio.tx_pwm(self._h, self._enb, PWM_FREQ, abs(right_speed))

    def cleanup(self):
        self.stop()
        lgpio.gpiochip_close(self._h)
