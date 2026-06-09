"""
arrow_launcher — fires the arrow when the brain releases.

On /archer/fire (the aim azimuth in radians, emitted by archer_brain at RELEASE):
  1. teleport the 'arrow' model to the bow tip, pointed along the aim
     (gz .../set_pose service, a blocking call so it lands reliably)
  2. apply a one-shot force impulse along the aim by publishing an EntityWrench
     to .../wrench (bridged to gz by the launch file)

Gazebo physics then flies the arrow on a real ballistic arc and collides it with
whatever it hits. Launch speed is set by force: v ~= force * physics_dt / mass.

The wrench goes through a persistent ROS publisher (not a one-shot gz CLI call)
because a publish-and-exit process drops the message before transport delivers it.
"""

import math
import subprocess

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
from std_msgs.msg import Float64


class ArrowLauncher(Node):
    def __init__(self):
        super().__init__('arrow_launcher')

        self.declare_parameter('world', 'archer_vision_world')
        self.declare_parameter('launch_speed', 12.0)    # m/s
        self.declare_parameter('elevation_deg', 6.0)     # upward tilt for arc
        self.declare_parameter('arrow_mass', 0.02)       # kg (matches the SDF)
        self.declare_parameter('physics_dt', 0.001)      # s (matches world step)
        self.declare_parameter('bow_offset', 0.30)       # m forward of base
        self.declare_parameter('bow_height', 0.60)       # m

        world = self._p('world')
        self.wrench_pub = self.create_publisher(
            EntityWrench, f'/world/{world}/wrench', 10)
        self.create_subscription(Float64, '/archer/fire', self.on_fire, 10)
        self.get_logger().info('Arrow launcher ready. Waiting for /archer/fire.')

    def on_fire(self, msg: Float64):
        theta = msg.data                       # aim azimuth (rad)
        world = self._p('world')

        # 1) teleport the arrow to the bow tip, yawed to the aim (blocking service)
        bx = self._p('bow_offset') * math.cos(theta)
        by = self._p('bow_offset') * math.sin(theta)
        bz = self._p('bow_height')
        qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)
        pose_req = (f'name: "arrow", position: {{x: {bx:.4f}, y: {by:.4f}, '
                    f'z: {bz:.4f}}}, orientation: {{x: 0, y: 0, z: {qz:.5f}, '
                    f'w: {qw:.5f}}}')
        try:
            subprocess.run(
                ['gz', 'service', '-s', f'/world/{world}/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '500', '--req', pose_req],
                timeout=2.0, capture_output=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'set_pose failed: {exc}')

        # 2) one-shot impulse along the aim (force ~ speed * mass / dt)
        force = self._p('launch_speed') * self._p('arrow_mass') / self._p('physics_dt')
        elev = math.radians(self._p('elevation_deg'))
        w = EntityWrench()
        w.entity = Entity(name='arrow::shaft', type=Entity.LINK)
        w.wrench.force = Vector3(
            x=force * math.cos(elev) * math.cos(theta),
            y=force * math.cos(elev) * math.sin(theta),
            z=force * math.sin(elev))
        self.wrench_pub.publish(w)

        self.get_logger().info(
            f'Fired at azimuth {math.degrees(theta):+.0f} deg (force {force:.0f} N).')

    def _p(self, name):
        return self.get_parameter(name).value


def main():
    rclpy.init()
    node = ArrowLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
