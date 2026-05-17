import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from car_control.l298n_driver import L298NDriver


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.declare_parameter('ena_pin', 18)
        self.declare_parameter('in1_pin', 23)
        self.declare_parameter('in2_pin', 24)
        self.declare_parameter('enb_pin', 13)
        self.declare_parameter('in3_pin', 27)
        self.declare_parameter('in4_pin', 22)
        self.declare_parameter('speed', 75)

        self._driver = L298NDriver(
            ena=self.get_parameter('ena_pin').value,
            in1=self.get_parameter('in1_pin').value,
            in2=self.get_parameter('in2_pin').value,
            enb=self.get_parameter('enb_pin').value,
            in3=self.get_parameter('in3_pin').value,
            in4=self.get_parameter('in4_pin').value,
            speed=self.get_parameter('speed').value,
        )

        self.create_subscription(String, '/car_cmd', self._cmd_callback, 10)
        self.get_logger().info('Motor controller ready, subscribing to /car_cmd')

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'forward':
            self._driver.forward()
            self.get_logger().info('Moving forward')
        elif cmd == 'backward':
            self._driver.backward()
            self.get_logger().info('Moving backward')
        elif cmd == 'stop':
            self._driver.stop()
            self.get_logger().info('Stopped')
        else:
            self.get_logger().warn(f'Unknown command: "{msg.data}" — use forward/backward/stop')

    def destroy_node(self):
        self._driver.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
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
