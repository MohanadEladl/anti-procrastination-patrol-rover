# 4WD Car Wiring Guide

Raspberry Pi 4B + L298N Motor Driver + SG90 Servo + Pi Camera

---

## Raspberry Pi 4B — 40-Pin Header Reference

```
         3.3V  [1]  [2]  5V
    GPIO2/SDA  [3]  [4]  5V
    GPIO3/SCL  [5]  [6]  GND
        GPIO4  [7]  [8]  GPIO14
          GND  [9]  [10] GPIO15
       GPIO17  [11] [12] GPIO18  ← ENA (L298N)
       GPIO27  [13] [14] GND
       GPIO22  [15] [16] GPIO23
         3.3V  [17] [18] GPIO24
       GPIO10  [19] [20] GND
        GPIO9  [21] [22] GPIO25
       GPIO11  [23] [24] GPIO8
          GND  [25] [26] GPIO7
        GPIO0  [27] [28] GPIO1
        GPIO5  [29] [30] GND
        GPIO6  [31] [32] GPIO12
       GPIO13  [33] [34] GND
       GPIO19  [35] [36] GPIO16
       GPIO26  [37] [38] GPIO20
          GND  [39] [40] GPIO21
```

---

## L298N Motor Driver

### Power

| L298N       | Connect to              | Notes                        |
|-------------|-------------------------|------------------------------|
| 12V (VCC)   | Battery pack +          | 7–12V recommended            |
| GND         | Battery pack − AND Pi GND (Pin 6/9/14/20/25/30/34/39) | Common ground — must connect Pi GND to L298N GND |
| 5V (out)    | Not connected           | Can power Pi if no other 5V source |

### Motor Control → Raspberry Pi

| L298N Pin | Pi BCM  | Pi Physical Pin | Role                     |
|-----------|---------|-----------------|--------------------------|
| ENA       | GPIO 18 | Pin 12          | Left motors PWM speed    |
| IN1       | GPIO 23 | Pin 16          | Left motors direction A  |
| IN2       | GPIO 24 | Pin 18          | Left motors direction B  |
| ENB       | GPIO 13 | Pin 33          | Right motors PWM speed   |
| IN3       | GPIO 27 | Pin 13          | Right motors direction A |
| IN4       | GPIO 22 | Pin 15          | Right motors direction B |

> **Note:** ENA/ENB jumpers must be **removed** from the L298N board.

### Motor Outputs

| L298N Output | Connected to          |
|--------------|-----------------------|
| OUT1, OUT2   | Left motors (front + rear wired in parallel) |
| OUT3, OUT4   | Right motors (front + rear wired in parallel) |

---

## SG90 Servo (camera pan)

| SG90 Wire   | Connect to          | Notes                          |
|-------------|---------------------|--------------------------------|
| Orange (signal) | Pi Pin 32 (GPIO 12) | Hardware PWM — 50 Hz, 5–10 % duty |
| Red (VCC)   | Pi Pin 2 (5V)       | Servo needs 5V                 |
| Brown (GND) | Pi Pin 34 (GND)     |                                |

### Pulse widths

| Angle | Pulse width | Duty cycle |
|-------|-------------|------------|
| 0°    | 1000 µs     | 5.0 %      |
| 90°   | 1500 µs     | 7.5 %      |
| 180°  | 2000 µs     | 10.0 %     |

---

## LEDs and Buzzer

| Component      | Pi BCM   | Pi Physical Pin | Notes                         |
|----------------|----------|-----------------|-------------------------------|
| LED 1 anode    | GPIO 17  | Pin 11          | 330 Ω resistor to GND         |
| LED 2 anode    | GPIO 25  | Pin 22          | 330 Ω resistor to GND         |
| Buzzer (+)     | GPIO 5   | Pin 29          | Active buzzer — HIGH = on     |
| Buzzer (−)     | GND      | Pin 30          |                               |

---

## HC-SR04 Ultrasonic Sensor

| HC-SR04 Pin | Connect to                              | Notes                                                    |
|-------------|-----------------------------------------|----------------------------------------------------------|
| VCC         | Pi Pin 2 (5V)                           |                                                          |
| GND         | Pi Pin 9 (GND)                          |                                                          |
| TRIG        | Pi Pin 7 (GPIO 4)                       | 3.3V output is sufficient to trigger                     |
| ECHO        | Voltage divider → Pi Pin 31 (GPIO 6)   | ECHO is 5V — **must** use divider: ECHO → 1kΩ → GPIO6 → 2kΩ → GND |

> **Voltage divider required:** The HC-SR04 ECHO pin outputs 5V. The Pi GPIO is 3.3V max.
> Wire: `ECHO → 1kΩ resistor → GPIO6`, then `GPIO6 → 2kΩ resistor → GND`.

---

## Pi Camera

Connect via CSI ribbon cable to the Pi Camera port. No GPIO pins used.

---

## Full Pin Usage Summary

| Pi Physical Pin | BCM     | Used by        | Signal            |
|-----------------|---------|----------------|-------------------|
| Pin 2           | 5V      | SG90 VCC       | Servo power       |
| Pin 6 / 34      | GND     | L298N + SG90   | Common ground     |
| Pin 11          | GPIO 17 | LED 1          | Alert LED         |
| Pin 12          | GPIO 18 | L298N ENA      | Left motors PWM   |
| Pin 13          | GPIO 27 | L298N IN3      | Right dir A       |
| Pin 15          | GPIO 22 | L298N IN4      | Right dir B       |
| Pin 16          | GPIO 23 | L298N IN1      | Left dir A        |
| Pin 18          | GPIO 24 | L298N IN2      | Left dir B        |
| Pin 22          | GPIO 25 | LED 2          | Alert LED         |
| Pin 29          | GPIO 5  | Buzzer         | Active buzzer     |
| Pin 7           | GPIO 4  | HC-SR04 TRIG   | Sonar trigger     |
| Pin 31          | GPIO 6  | HC-SR04 ECHO   | Sonar echo (via 1kΩ/2kΩ divider) |
| Pin 32          | GPIO 12 | SG90 signal    | Servo PWM         |
| Pin 33          | GPIO 13 | L298N ENB      | Right motors PWM  |
