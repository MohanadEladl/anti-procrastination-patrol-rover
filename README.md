# 🚔 Anti Procrastination Patrol Rover

> An autonomous 4-wheeled ground robot that patrols an area, detects a person holding a mobile phone using on-device AI, and triggers a multi-modal alarm — entirely running on a Raspberry Pi 4B with no cloud connectivity.

---

## 📸 What It Does

The rover continuously patrols back and forth while scanning with a pan-mounted camera. The moment it spots **a person holding a phone**, it:

- 🛑 **Stops** immediately
- 🚨 **Flashes LEDs** in an alternating police-light pattern at 15 Hz
- 🔊 **Activates a buzzer**
- 📡 **Monitors proximity** with an ultrasonic sensor — reverses if someone gets too close

Detection uses **YOLO11s in NCNN format**, running fully offline on the Pi's ARM CPU.

---

## 🧰 Hardware

| Component | Details |
|---|---|
| **Brain** | Raspberry Pi 4B (4 GB RAM), Ubuntu 24.04 LTS |
| **Chassis** | 4WD platform with two L298N motor driver modules |
| **Camera** | Pi Camera (CSI), pan-controlled by SG90 servo |
| **Sonar** | HC-SR04 ultrasonic sensor (30 cm obstacle trigger) |
| **Alarm** | 2× LEDs + active buzzer |

### GPIO Pinout Summary

| Pi Pin | BCM | Connected To |
|---|---|---|
| Pin 12 | GPIO 18 | L298N ENA (Left motors PWM) |
| Pin 16 | GPIO 23 | L298N IN1 |
| Pin 18 | GPIO 24 | L298N IN2 |
| Pin 33 | GPIO 13 | L298N ENB (Right motors PWM) |
| Pin 13 | GPIO 27 | L298N IN3 |
| Pin 15 | GPIO 22 | L298N IN4 |
| Pin 32 | GPIO 12 | SG90 Servo (Hardware PWM) |
| Pin 11 | GPIO 17 | LED 1 |
| Pin 22 | GPIO 25 | LED 2 |
| Pin 29 | GPIO 5  | Buzzer |
| Pin 7  | GPIO 4  | HC-SR04 TRIG |
| Pin 31 | GPIO 6  | HC-SR04 ECHO (via 1kΩ/2kΩ divider) |

> See [`WIRING.md`](WIRING.md) for the full wiring diagram and voltage divider instructions.

---

## 🧠 Software Architecture

**Framework:** ROS2 Jazzy — `ament_python` package: `car_control`

The system runs as a single ROS2 node (`patrol`) using a multi-threaded architecture with shared state protected by `threading.Lock`:

| Thread | Rate | Role |
|---|---|---|
| `_camera_reader_loop` | Max pipe speed | Reads MJPEG from `rpicam-vid`, keeps latest frame |
| `_inference_loop` | ~1–2 fps | CLAHE → YOLO11s NCNN → detection logic |
| `_sonar_loop` | 20 Hz | HC-SR04 distance measurement |
| `_servo_loop` | 20 Hz | Drift-corrected 0°→180°→0° pan sweep |
| `_stream_server_loop` | On demand | MJPEG HTTP stream on port 8080 |
| ROS control timer | 20 Hz | State machine, LEDs, buzzer, motors |

### State Machine

```
SCANNING ──(person + phone detected)──► LOCKED
    │                                       │
    │◄─────────(8s window expires)──────────┘
```

- **SCANNING:** Patrols forward/backward; servo sweeps; inference runs
- **LOCKED:** Motors stop; LEDs flash; buzzer on; inference pauses to free CPU for sonar

---

## 👁️ Detection Pipeline

1. **Camera** — `rpicam-vid` subprocess → raw MJPEG at 640×480, 10 fps (`--gain 4 --exposure long` for low light)
2. **Preprocessing** — CLAHE on L channel (LAB colorspace) for edge contrast in dim conditions
3. **Model** — YOLO11s in NCNN format (`conf=0.10, iou=0.45, classes=[0, 67]` — person + cell phone)
4. **Spatial filter** — Phone box center must fall **within the person box** (expanded 25%) to confirm detection

> NCNN is used exclusively — PyTorch is not installed on the Pi (CUDA torch build causes `SIGILL` on Cortex-A72).

---

## 🚀 Getting Started

### Prerequisites

- Raspberry Pi 4B running Ubuntu 24.04
- ROS2 Jazzy installed
- Hardware PWM enabled: add `dtoverlay=pwm,pin=12,func=4` to `/boot/firmware/config.txt`

### Install & Run

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/MohanadEladl/anti-procrastination-patrol-rover.git
cd ../..

# Build
colcon build --packages-select car_control
source install/setup.bash

# Launch
ros2 launch car_control patrol.launch.py
```

---

## ⚙️ Tunable Parameters (`patrol.launch.py`)

| Parameter | Default | Description |
|---|---|---|
| `base_speed` | 45 | Patrol motor duty cycle (0–100) |
| `servo_step` | 4.0° | Degrees per servo tick at 20 Hz |
| `patrol_time` | 5.0 s | Duration of each forward/backward segment |
| `unlock_secs` | 8.0 s | Alarm duration after last detection |
| `proximity_cm` | 30.0 cm | Sonar obstacle trigger distance |
| `led_flash_hz` | 15 | LED alternation frequency during alarm |
| `model_path` | `/home/mohanad/yolo11s_ncnn_model` | NCNN model directory on Pi |

---

## 🗂️ Repository Structure

```
.
├── src/
│   └── car_control/              # ROS2 Python package
│       ├── car_control/
│       │   ├── patrol_node.py        # Main node — state machine, all threads
│       │   ├── l298n_driver.py       # L298N motor driver abstraction
│       │   ├── motor_controller_node.py
│       │   └── scan_to_range_node.py
│       ├── launch/
│       │   ├── patrol.launch.py      # Main launch file
│       │   └── car_control.launch.py
│       ├── setup.py
│       └── setup.cfg
├── yolo11s_ncnn_model/           # NCNN model files
│   ├── model.ncnn.bin
│   ├── model.ncnn.param
│   └── model_ncnn.py
├── yolo11s.pt                    # Original YOLO11s weights
├── documentation.md              # Full technical deep-dive
└── WIRING.md                     # Full GPIO wiring guide
```

---

## 🔧 Key Engineering Challenges

| Problem | Root Cause | Solution |
|---|---|---|
| Servo jitter | lgpio software PWM jitter under YOLO load | Hardware PWM via sysfs |
| Servo slow during YOLO | Servo thread starved by YOLO CPU usage | Moved servo to independent 20 Hz thread |
| Sonar latency 2–3 s | YOLO at 300% CPU pre-empts sonar busy-wait | Pause inference during LOCKED state |
| Phone not detected | Confidence 0.35 too high for held phones | Per-class thresholds: person ≥ 0.40, phone ≥ 0.12 |
| Low-light detection failures | Conservative camera defaults | `--gain 4 --exposure long` + CLAHE |
| SIGILL on Pi | CUDA torch build incompatible with Cortex-A72 | Export NCNN on Mac, rsync to Pi |

---

## 📄 License

MIT — see [LICENSE](LICENSE) for details.

---

*Built as a Year 4 Robotics project — Raspberry Pi 4B · ROS2 Jazzy · YOLO11s NCNN · lgpio*
