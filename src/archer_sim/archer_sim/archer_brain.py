"""
P2.3b archer_brain — continuous 180-degree sweep, fire when a tag is centered.

The camera rides on the rotating body (turret eye). The base sweeps back and
forth across the arc, NON-STOP. Whenever a tag passes through the center of the
camera's view (bearing ~ 0), the bow is pointed at it, so we fire: draw, hold,
release, cool down, then resume sweeping.

Why this design (vs. closed-loop centering):
  - No "rotate toward the tag" step, so no handedness/sign bug.
  - A moving tag is handled for free: the next sweep pass catches its new spot.
  - Multiple tags: each gets shot as the sweep crosses its center.

The detector publishes the bearing (deg, relative to camera) of the tag nearest
the image center on /archer/target_angle; no message means no tag in view.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ArcherBrain(Node):
    def __init__(self):
        super().__init__('archer_brain')

        self.declare_parameter('sweep_speed', 0.35)     # rad/s base sweep
        self.declare_parameter('sweep_limit', 1.4)      # rad, +/- arc edge (~80 deg)
        self.declare_parameter('fire_window_deg', 3.0)  # |bearing| to count as "aimed"
        self.declare_parameter('tag_timeout', 0.4)      # s without bearing = no tag
        self.declare_parameter('draw_time', 1.5)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('release_time', 0.6)
        self.declare_parameter('cooldown_time', 1.5)
        self.declare_parameter('draw_distance', -0.15)

        self.base_pub = self.create_publisher(Float64, '/archer/base_cmd', 10)
        self.draw_pub = self.create_publisher(Float64, '/archer/draw_cmd', 10)
        self.create_subscription(
            Float64, '/archer/target_angle', self.on_bearing, 10)

        self.dt = 0.05
        self.base_cmd = 0.0
        self.sweep_dir = 1.0
        self.last_bearing = 0.0
        self.last_bearing_time = -1e9
        # 'armed' stops us re-firing at the same centered tag right after a shot;
        # we must sweep off the tag (or lose it) before the next shot can arm.
        self.armed = True
        self.state = 'SWEEP'
        self.state_elapsed = 0.0

        self.create_timer(self.dt, self.tick)
        self.get_logger().info('Archer brain online. Sweeping 180; fires on a centered tag.')

    def on_bearing(self, msg: Float64):
        self.last_bearing = msg.data
        self.last_bearing_time = self._now()

    def _tag_visible(self) -> bool:
        return (self._now() - self.last_bearing_time) < self._p('tag_timeout')

    def tick(self):
        if self.state == 'SWEEP':
            self._sweep()
        else:
            self._timed_shot_states()

    def _sweep(self):
        visible = self._tag_visible()
        centered = visible and abs(self.last_bearing) <= self._p('fire_window_deg')

        # Re-arm once we're off the previous target (so we don't machine-gun it).
        if not self.armed and (not visible or not centered):
            self.armed = True

        if self.armed and centered:
            self.armed = False
            self.get_logger().info(
                f'Tag centered (bearing {self.last_bearing:+.1f} deg). Firing.')
            self._set_state('DRAW')
            return

        # keep sweeping back and forth across the arc
        self.base_cmd += self.sweep_dir * self._p('sweep_speed') * self.dt
        limit = self._p('sweep_limit')
        if self.base_cmd >= limit:
            self.base_cmd, self.sweep_dir = limit, -1.0
        elif self.base_cmd <= -limit:
            self.base_cmd, self.sweep_dir = -limit, 1.0
        self._send(self.base_pub, self.base_cmd)

    def _timed_shot_states(self):
        # base is held fixed (aimed) while we draw/hold/release
        self.state_elapsed += self.dt
        durations = {
            'DRAW': self._p('draw_time'),
            'HOLD': self._p('hold_time'),
            'RELEASE': self._p('release_time'),
            'COOLDOWN': self._p('cooldown_time'),
        }
        if self.state_elapsed < durations[self.state]:
            return
        nxt = {'DRAW': 'HOLD', 'HOLD': 'RELEASE',
               'RELEASE': 'COOLDOWN', 'COOLDOWN': 'SWEEP'}[self.state]
        self._set_state(nxt)

    def _set_state(self, state: str):
        self.state = state
        self.state_elapsed = 0.0
        if state == 'DRAW':
            self._send(self.draw_pub, self._p('draw_distance'))
        elif state == 'RELEASE':
            self._send(self.draw_pub, 0.0)
        elif state == 'SWEEP':
            self.get_logger().info('Resuming sweep.')
        self.get_logger().info(f'-> {state}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _p(self, name):
        return self.get_parameter(name).value

    def _send(self, pub, value: float):
        pub.publish(Float64(data=float(value)))


def main():
    rclpy.init()
    node = ArcherBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
