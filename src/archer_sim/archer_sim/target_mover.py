"""
target_mover — oscillates the moving AprilTag target side to side.

Publishes a slow sine wave to /mover/slide_cmd (the moving target's prismatic
'slide' joint), so the panel glides back and forth and the archer has to
re-acquire it on each sweep. Pure demo dressing.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TargetMover(Node):
    def __init__(self):
        super().__init__('target_mover')
        self.declare_parameter('amplitude', 1.8)   # metres, peak slide
        self.declare_parameter('period', 10.0)     # seconds per full cycle

        self.pub = self.create_publisher(Float64, '/mover/slide_cmd', 10)
        self.t0 = self._now()
        self.create_timer(0.05, self.tick)         # 20 Hz
        self.get_logger().info('Target mover online: sliding the moving tag.')

    def tick(self):
        amp = self.get_parameter('amplitude').value
        period = self.get_parameter('period').value
        t = self._now() - self.t0
        pos = amp * math.sin(2.0 * math.pi * t / period)
        self.pub.publish(Float64(data=pos))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = TargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
