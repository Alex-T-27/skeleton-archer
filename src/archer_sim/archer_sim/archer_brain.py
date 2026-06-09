"""
P2.3b archer_brain — sweep, lock, TRACK-while-firing, resume.

The camera rides on the rotating body (turret eye). Behavior:

  SWEEP    sweep the base back and forth across the arc, non-stop, until a tag
           passes through the center of view (bow aligned) -> lock and fire.
  DRAW     pull the string  ... while CENTERING on the tag (so a moving target
  HOLD     hold at full draw  ... is followed instead of slipping away)
  RELEASE  let go             ...
  COOLDOWN brief pause (base held), then resume SWEEP from this position.

So a moving tag is tracked through the whole shot; after release the archer
keeps scanning from wherever it ended up.

The detector publishes the bearing (deg, relative to camera) of the tag nearest
image center on /archer/target_angle; no message = no tag in view.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class ArcherBrain(Node):
    def __init__(self):
        super().__init__('archer_brain')

        self.declare_parameter('sweep_speed', 0.35)     # rad/s base sweep
        self.declare_parameter('sweep_limit', 1.4)      # rad, +/- arc edge (~80 deg)
        self.declare_parameter('fire_window_deg', 3.0)  # |bearing| to lock/fire
        self.declare_parameter('center_gain', 0.4)      # tracking correction strength
        self.declare_parameter('tag_timeout', 0.4)      # s without bearing = no tag
        self.declare_parameter('draw_time', 1.5)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('release_time', 0.6)
        self.declare_parameter('cooldown_time', 1.5)
        self.declare_parameter('draw_distance', -0.15)

        self.base_pub = self.create_publisher(Float64, '/archer/base_cmd', 10)
        self.draw_pub = self.create_publisher(Float64, '/archer/draw_cmd', 10)
        self.create_subscription(
            Float64MultiArray, '/archer/target', self.on_target, 10)

        self.dt = 0.05
        self.base_cmd = 0.0
        self.sweep_dir = 1.0
        self.last_id = -1
        self.last_bearing = 0.0
        self.last_bearing_time = -1e9
        self.locked_id = None
        self.armed = True
        self.state = 'SWEEP'
        self.state_elapsed = 0.0

        # states during which we keep centering on the locked tag
        self.tracking_states = ('DRAW', 'HOLD', 'RELEASE')

        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            'Archer brain online. Sweep 180; lock, track through the shot, resume.')

    def on_target(self, msg: Float64MultiArray):
        self.last_id = int(msg.data[0])
        self.last_bearing = msg.data[1]
        self.last_bearing_time = self._now()

    def _tag_visible(self) -> bool:
        return (self._now() - self.last_bearing_time) < self._p('tag_timeout')

    def tick(self):
        if self.state == 'SWEEP':
            self._sweep()
        else:
            self._shot_tick()

    def _sweep(self):
        visible = self._tag_visible()
        centered = visible and abs(self.last_bearing) <= self._p('fire_window_deg')

        if not self.armed and (not visible or not centered):
            self.armed = True

        if self.armed and centered:
            self.armed = False
            self.locked_id = self.last_id
            self.get_logger().info(
                f'Locked tag id={self.last_id} (bearing {self.last_bearing:+.1f}). '
                'Tracking + firing.')
            self._set_state('DRAW')
            return

        self.base_cmd += self.sweep_dir * self._p('sweep_speed') * self.dt
        limit = self._p('sweep_limit')
        if self.base_cmd >= limit:
            self.base_cmd, self.sweep_dir = limit, -1.0
        elif self.base_cmd <= -limit:
            self.base_cmd, self.sweep_dir = -limit, 1.0
        self._send(self.base_pub, self.base_cmd)

    def _shot_tick(self):
        # keep the bow on the (possibly moving) tag during the draw/hold/release
        if self.state in self.tracking_states:
            self._track()

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

    def _track(self):
        # Follow ONLY the locked tag. If another tag is nearer center (e.g. a
        # static one the moving target slid past), or the tag is momentarily
        # lost, hold the last aim and let the shot finish -- never switch mid-shot.
        if not self._tag_visible() or self.last_id != self.locked_id:
            return
        self.base_cmd += self._p('center_gain') * math.radians(self.last_bearing)
        limit = self._p('sweep_limit')
        self.base_cmd = max(-limit, min(limit, self.base_cmd))
        self._send(self.base_pub, self.base_cmd)

    def _set_state(self, state: str):
        self.state = state
        self.state_elapsed = 0.0
        if state == 'DRAW':
            self._send(self.draw_pub, self._p('draw_distance'))
        elif state == 'RELEASE':
            self._send(self.draw_pub, 0.0)
        elif state == 'SWEEP':
            self.locked_id = None
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
