import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self):
        super().__init__('scan_to_range')
        self.create_subscription(LaserScan, 'scan', self._cb, 10)
        self._pub = self.create_publisher(Range, 'range', 10)

    def _cb(self, msg: LaserScan):
        r = Range()
        r.header = msg.header
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = 0.26   # ~15°, typical HC-SR04 beam angle
        r.min_range = msg.range_min
        r.max_range = msg.range_max
        r.range = msg.ranges[0] if msg.ranges else msg.range_max
        self._pub.publish(r)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
