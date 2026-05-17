import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

_SCANNING = 'SCANNING'
_LOCKED   = 'LOCKED'

_PWM_ROOT = '/sys/class/pwm/pwmchip0'
_PWM_PATH = f'{_PWM_ROOT}/pwm0'


def _servo_ns(angle: float) -> int:
    """Convert angle (0–180°) to pulse width in nanoseconds for hardware PWM."""
    pw_us = 1000.0 + (angle / 180.0) * 1000.0  # 1000–2000 µs
    return int(pw_us * 1000)                     # µs → ns


def _person_with_phone(results) -> bool:
    """Return True if a phone center falls within an expanded person bbox."""
    person_boxes = [b for b in results.boxes
                    if int(b.cls[0]) == 0  and float(b.conf[0]) >= 0.40]
    phone_boxes  = [b for b in results.boxes
                    if int(b.cls[0]) == 67 and float(b.conf[0]) >= 0.12]
    if not person_boxes or not phone_boxes:
        return False
    for pb in person_boxes:
        x1, y1, x2, y2 = [float(v) for v in pb.xyxy[0]]
        mx = (x2 - x1) * 0.25
        my = (y2 - y1) * 0.25
        for ph in phone_boxes:
            cx = (float(ph.xyxy[0][0]) + float(ph.xyxy[0][2])) / 2
            cy = (float(ph.xyxy[0][1]) + float(ph.xyxy[0][3])) / 2
            if x1 - mx <= cx <= x2 + mx and y1 - my <= cy <= y2 + my:
                return True
    return False


def _has_phone_shape(frame, person_xyxy) -> bool:
    """Return True if a phone-proportioned rectangle is found in person's hand area."""
    import cv2
    x1 = int(float(person_xyxy[0]))
    y1 = int(float(person_xyxy[1]))
    x2 = int(float(person_xyxy[2]))
    y2 = int(float(person_xyxy[3]))
    h = y2 - y1
    crop = frame[y1 + h // 3 : y2, x1:x2]
    if crop.size == 0:
        return False

    gray    = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges   = cv2.Canny(blurred, 30, 100)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    crop_area = crop.shape[0] * crop.shape[1]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (crop_area * 0.01 <= area <= crop_area * 0.40):
            continue
        peri   = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        if not (4 <= len(approx) <= 6):
            continue
        _, _, bw, bh = cv2.boundingRect(approx)
        if bw == 0 or bh == 0:
            continue
        ratio = max(bw, bh) / min(bw, bh)
        if not (1.5 <= ratio <= 4.0):
            continue
        if area / (bw * bh) < 0.50:
            continue
        return True
    return False


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol')

        self.declare_parameter('servo_pin',    12)
        self.declare_parameter('led1_pin',     17)
        self.declare_parameter('led2_pin',     25)
        self.declare_parameter('buzzer_pin',    5)
        self.declare_parameter('ena_pin',      18)
        self.declare_parameter('in1_pin',      23)
        self.declare_parameter('in2_pin',      24)
        self.declare_parameter('enb_pin',      13)
        self.declare_parameter('in3_pin',      27)
        self.declare_parameter('in4_pin',      22)
        self.declare_parameter('servo_step',    4.0)
        self.declare_parameter('patrol_time',   5.0)
        self.declare_parameter('base_speed',   60)
        self.declare_parameter('lock_frames',   1)
        self.declare_parameter('unlock_frames', 15)   # kept for API compat, unused
        self.declare_parameter('confidence',    0.35)
        self.declare_parameter('unlock_secs',  15.0)
        self.declare_parameter('model_path',
            '/home/mohanad/yolov8n_ncnn_model'
        )
        self.declare_parameter('led_flash_hz', 15)
        self.declare_parameter('trig_pin',      4)
        self.declare_parameter('echo_pin',      6)
        self.declare_parameter('proximity_cm', 30.0)

        self._servo_pin  = self.get_parameter('servo_pin').value
        self._led1_pin   = self.get_parameter('led1_pin').value
        self._led2_pin   = self.get_parameter('led2_pin').value
        self._buzzer_pin = self.get_parameter('buzzer_pin').value
        self._servo_step = float(self.get_parameter('servo_step').value)
        self._patrol_sec = float(self.get_parameter('patrol_time').value)
        self._lock_frames   = self.get_parameter('lock_frames').value
        self._unlock_frames = self.get_parameter('unlock_frames').value
        self._confidence    = float(self.get_parameter('confidence').value)
        self._model_path    = self.get_parameter('model_path').value
        self._unlock_secs   = float(self.get_parameter('unlock_secs').value)
        self._led_flash_hz  = self.get_parameter('led_flash_hz').value
        self._trig_pin      = self.get_parameter('trig_pin').value
        self._echo_pin      = self.get_parameter('echo_pin').value
        self._proximity_cm  = float(self.get_parameter('proximity_cm').value)

        import lgpio
        from car_control.l298n_driver import L298NDriver

        # GPIO for servo / LEDs / buzzer / sonar
        self._h = lgpio.gpiochip_open(0)
        for pin in (self._led1_pin, self._led2_pin, self._buzzer_pin):
            try:
                lgpio.gpio_free(self._h, pin)
            except lgpio.error:
                pass
            lgpio.gpio_claim_output(self._h, pin, 0)

        # Hardware PWM servo via sysfs (GPIO 12 = PWM0, enabled by dtoverlay)
        if not os.path.exists(_PWM_PATH):
            with open(f'{_PWM_ROOT}/export', 'w') as f:
                f.write('0')
            time.sleep(0.1)
        with open(f'{_PWM_PATH}/period', 'w') as f:
            f.write('20000000')             # 20 ms = 50 Hz
        with open(f'{_PWM_PATH}/duty_cycle', 'w') as f:
            f.write(str(_servo_ns(0)))      # start at 0°
        with open(f'{_PWM_PATH}/enable', 'w') as f:
            f.write('1')
        self._pwm_duty = open(f'{_PWM_PATH}/duty_cycle', 'w')

        # HC-SR04 sonar
        try:
            lgpio.gpio_free(self._h, self._trig_pin)
        except lgpio.error:
            pass
        lgpio.gpio_claim_output(self._h, self._trig_pin, 0)
        try:
            lgpio.gpio_free(self._h, self._echo_pin)
        except lgpio.error:
            pass
        lgpio.gpio_claim_input(self._h, self._echo_pin)

        # Motor driver (opens its own lgpio handle internally)
        self._driver = L298NDriver(
            ena=self.get_parameter('ena_pin').value,
            in1=self.get_parameter('in1_pin').value,
            in2=self.get_parameter('in2_pin').value,
            enb=self.get_parameter('enb_pin').value,
            in3=self.get_parameter('in3_pin').value,
            in4=self.get_parameter('in4_pin').value,
            speed=self.get_parameter('base_speed').value,
        )

        # State
        self._state                 = _SCANNING
        self._servo_angle           = 0.0
        self._servo_dir             = 1
        self._patrol_start          = time.monotonic()
        self._patrol_elapsed_at_lock = 0.0
        self._led_tick              = 0

        # Thread-safe detection state
        self._det_lock          = threading.Lock()
        self._detect_count      = 0
        self._miss_count        = 0
        self._last_detected_t   = 0.0   # monotonic time of last positive detection

        # Latest decoded frame shared between camera-reader and inference threads
        self._frame_lock   = threading.Lock()
        self._latest_frame = None

        # Sonar state
        self._sonar_lock       = threading.Lock()
        self._distance_cm      = 999.0
        self._proximity_active = False

        # MJPEG stream — latest annotated frame as JPEG bytes
        self._stream_lock = threading.Lock()
        self._stream_jpeg = None

        self._inf_thread    = threading.Thread(target=self._inference_loop, daemon=True)
        self._stream_thread = threading.Thread(target=self._stream_server_loop, daemon=True)
        self._cam_thread    = threading.Thread(target=self._camera_reader_loop, daemon=True)
        self._sonar_thread  = threading.Thread(target=self._sonar_loop, daemon=True)
        self._servo_thread  = threading.Thread(target=self._servo_loop, daemon=True)
        self._inf_thread.start()
        self._stream_thread.start()
        self._cam_thread.start()
        self._sonar_thread.start()
        self._servo_thread.start()

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Patrol ready | stream=http://192.168.100.206:8080 '
            f'servo=GPIO{self._servo_pin} '
            f'led1=GPIO{self._led1_pin} led2=GPIO{self._led2_pin} '
            f'buzzer=GPIO{self._buzzer_pin} '
            f'trig=GPIO{self._trig_pin} echo=GPIO{self._echo_pin} '
            f'proximity={self._proximity_cm:.0f}cm'
        )

    # ------------------------------------------------------------------ #
    # Camera reader — drains pipe as fast as possible, keeps latest frame #
    # ------------------------------------------------------------------ #

    def _camera_reader_loop(self):
        import subprocess
        import numpy as np
        import cv2

        def _start_proc():
            return subprocess.Popen(
                ['rpicam-vid', '--codec', 'mjpeg',
                 '--width', '640', '--height', '480',
                 '--framerate', '10', '-t', '0', '-n',
                 '--gain', '4', '--exposure', 'long',
                 '-o', '-'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )

        proc = _start_proc()
        buf = b''
        try:
            while rclpy.ok():
                chunk = proc.stdout.read(65536)  # large reads to drain quickly
                if not chunk:
                    self.get_logger().warn('Camera process exited, retrying in 3s')
                    proc.wait()
                    time.sleep(3)
                    proc = _start_proc()
                    buf = b''
                    continue

                buf += chunk

                # Parse ALL complete JPEGs in the buffer; keep only the LAST one
                latest = None
                pos = 0
                while True:
                    s = buf.find(b'\xff\xd8', pos)
                    if s == -1:
                        break
                    e = buf.find(b'\xff\xd9', s + 2)
                    if e == -1:
                        break
                    jpeg = buf[s:e + 2]
                    frame = cv2.imdecode(
                        np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        latest = frame          # overwrite — only keep newest
                    pos = e + 2

                # Discard all fully-parsed data; keep trailing incomplete bytes
                last_end = buf.rfind(b'\xff\xd9')
                if last_end != -1:
                    buf = buf[last_end + 2:]
                elif len(buf) > 200_000:        # safety: prevent unbounded growth
                    buf = b''

                if latest is not None:
                    with self._frame_lock:
                        self._latest_frame = latest
        finally:
            proc.terminate()

    # ------------------------------------------------------------------ #
    # Inference thread — always runs on the freshest available frame      #
    # ------------------------------------------------------------------ #

    def _inference_loop(self):
        import cv2
        from ultralytics import YOLO
        os.nice(10)  # yield CPU to servo/control threads when contested

        model = YOLO(self._model_path, task='detect')
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.get_logger().info(f'YOLO model loaded ({self._model_path}), starting camera')

        while rclpy.ok():
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None   # consume it so we don't re-run same frame

            if frame is None:
                time.sleep(0.01)
                continue

            if self._state == _LOCKED:
                time.sleep(0.1)
                continue

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b_ch = cv2.split(lab)
            l = clahe.apply(l)
            frame = cv2.cvtColor(cv2.merge((l, a, b_ch)), cv2.COLOR_LAB2BGR)

            results = model(frame, conf=0.10, iou=0.45,
                            agnostic_nms=True, classes=[0, 67], verbose=False)[0]

            found = _person_with_phone(results)

            # Shape fallback disabled (too many false positives outdoors)
            # if not found:
            #     for pb in results.boxes:
            #         if int(pb.cls[0]) != 0 or float(pb.conf[0]) < 0.40:
            #             continue
            #         if _has_phone_shape(frame, pb.xyxy[0]):
            #             found = True
            #             self.get_logger().info('Phone detected via shape fallback')
            #             break

            # Update MJPEG stream with annotated frame
            annotated = results.plot()
            _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._stream_lock:
                self._stream_jpeg = buf.tobytes()

            if results.boxes:
                labels = [
                    f'{model.names[int(b.cls[0])]}({float(b.conf[0]):.2f})'
                    for b in results.boxes
                ]
                self.get_logger().info(f'Detected: {", ".join(labels)} | holding={found}')

            with self._det_lock:
                if found:
                    self._detect_count += 1
                    self._miss_count = 0
                    self._last_detected_t = time.monotonic()
                    self.get_logger().info(
                        f'Person+phone confirmed — detect_count={self._detect_count}'
                    )
                else:
                    self._miss_count += 1
                    self._detect_count = 0

    # ------------------------------------------------------------------ #
    # Main control loop (20 Hz)                                           #
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        with self._det_lock:
            detect      = self._detect_count
            miss        = self._miss_count
            last_det_t  = self._last_detected_t

        # Active as long as a detection happened within the unlock window
        secs_since = time.monotonic() - last_det_t
        target_active = last_det_t > 0 and secs_since < self._unlock_secs

        if self._state == _SCANNING:
            self._patrol_car()
            self._set_leds(False, False)
            import lgpio
            lgpio.gpio_write(self._h, self._buzzer_pin, 0)

            if target_active:
                self._patrol_elapsed_at_lock = time.monotonic() - self._patrol_start
                self._state = _LOCKED
                self._led_tick = 0
                self._driver.stop()
                self.get_logger().info('Target locked — stopping')

        elif self._state == _LOCKED:
            if not self._proximity_active:
                self._flash_leds()
                import lgpio
                lgpio.gpio_write(self._h, self._buzzer_pin, 1)

                with self._sonar_lock:
                    dist = self._distance_cm
                if dist < self._proximity_cm:
                    self.get_logger().info(f'Proximity {dist:.0f} cm — triggering alert')
                    self._proximity_active = True
                    threading.Thread(target=self._proximity_alert, daemon=True).start()

            if not target_active:
                with self._det_lock:
                    self._last_detected_t = 0.0
                self._state = _SCANNING
                self._patrol_start = time.monotonic() - self._patrol_elapsed_at_lock
                self.get_logger().info(
                    f'Target lost — alarm off after {self._unlock_secs:.0f}s, resuming patrol'
                )

    # ------------------------------------------------------------------ #
    # MJPEG HTTP stream — http://<pi-ip>:8080                             #
    # ------------------------------------------------------------------ #

    def _stream_server_loop(self):
        from http.server import BaseHTTPRequestHandler, HTTPServer

        node = self

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # suppress per-request console noise

            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while rclpy.ok():
                        with node._stream_lock:
                            jpeg = node._stream_jpeg
                        if jpeg:
                            self.wfile.write(
                                b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' +
                                jpeg + b'\r\n'
                            )
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        server = HTTPServer(('0.0.0.0', 8080), _Handler)
        server.serve_forever()

    # ------------------------------------------------------------------ #
    # HC-SR04 sonar                                                        #
    # ------------------------------------------------------------------ #

    def _read_distance(self) -> float:
        import lgpio
        lgpio.gpio_write(self._h, self._trig_pin, 1)
        time.sleep(10e-6)
        lgpio.gpio_write(self._h, self._trig_pin, 0)

        deadline = time.monotonic() + 0.05
        while lgpio.gpio_read(self._h, self._echo_pin) == 0:
            if time.monotonic() > deadline:
                return 999.0
        t0 = time.monotonic()
        deadline = t0 + 0.05
        while lgpio.gpio_read(self._h, self._echo_pin) == 1:
            if time.monotonic() > deadline:
                return 999.0
        return (time.monotonic() - t0) * 17150.0

    def _sonar_loop(self):
        while rclpy.ok():
            dist = self._read_distance()
            with self._sonar_lock:
                self._distance_cm = dist
            time.sleep(0.05)  # 20 Hz

    def _proximity_alert(self):
        REVERSE_SPEED = 60
        REVERSE_SECS  = 1.0

        import lgpio
        self._driver.set_motors(-REVERSE_SPEED, -REVERSE_SPEED)
            t_start = time.monotonic()
            for _ in range(3):
                lgpio.gpio_write(self._h, self._buzzer_pin, 1)
                time.sleep(0.08)
                lgpio.gpio_write(self._h, self._buzzer_pin, 0)
                time.sleep(0.05)
            remaining = REVERSE_SECS - (time.monotonic() - t_start)
            if remaining > 0:
                time.sleep(remaining)
            self._driver.set_motors(REVERSE_SPEED, REVERSE_SPEED)
            time.sleep(REVERSE_SECS)
            self._driver.stop()

        self._proximity_active = False

    # ------------------------------------------------------------------ #
    # Servo thread — drift-corrected 20 Hz, independent of ROS executor   #
    # ------------------------------------------------------------------ #

    def _servo_loop(self):
        interval  = 0.05  # 20 Hz
        next_tick = time.monotonic() + interval
        while rclpy.ok():
            if self._state == _SCANNING:
                self._sweep_servo()
            sleep_t = next_tick - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            next_tick += interval
            if next_tick < time.monotonic():  # fell behind — re-anchor
                next_tick = time.monotonic() + interval

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _sweep_servo(self):
        self._servo_angle += self._servo_step * self._servo_dir
        if self._servo_angle >= 180.0:
            self._servo_angle = 180.0
            self._servo_dir = -1
        elif self._servo_angle <= 0.0:
            self._servo_angle = 0.0
            self._servo_dir = 1
        self._pwm_duty.seek(0)
        self._pwm_duty.write(str(_servo_ns(self._servo_angle)))
        self._pwm_duty.flush()

    def _patrol_car(self):
        elapsed = time.monotonic() - self._patrol_start
        going_forward = int(elapsed / self._patrol_sec) % 2 == 0
        if going_forward:
            self._driver.forward()
        else:
            self._driver.backward()

    def _flash_leds(self):
        self._led_tick += 1
        half = max(1, 20 // (2 * self._led_flash_hz))
        led1_on = (self._led_tick // half) % 2 == 0
        self._set_leds(led1_on, not led1_on)

    def _set_leds(self, led1: bool, led2: bool):
        import lgpio
        lgpio.gpio_write(self._h, self._led1_pin, int(led1))
        lgpio.gpio_write(self._h, self._led2_pin, int(led2))

    # ------------------------------------------------------------------ #
    # Cleanup                                                              #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        import lgpio
        self._driver.stop()
        self._driver.cleanup()
        try:
            self._pwm_duty.close()
            with open(f'{_PWM_PATH}/enable', 'w') as f:
                f.write('0')
        except OSError:
            pass
        for pin in (self._led1_pin, self._led2_pin, self._buzzer_pin,
                    self._trig_pin, self._echo_pin):
            try:
                lgpio.gpio_free(self._h, pin)
            except lgpio.error:
                pass
        try:
            lgpio.gpiochip_close(self._h)
        except lgpio.error:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
