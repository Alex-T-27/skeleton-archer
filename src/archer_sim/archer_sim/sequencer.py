"""
P2.2 sequencer — the archer's "brain".

Listens for a target angle (in DEGREES) on /archer/target_angle, then runs the
shot as a timed state machine, commanding the two joint topics from P2.1:

    IDLE -> AIM -> DRAW -> HOLD -> RELEASE -> RECOVER -> IDLE

It never blocks: a timer ticks the state machine, so the node stays responsive.
Every transition is logged so you can watch the shot step through.

Run (with the sim already up from archer_sim.launch.py):
    ros2 run archer_sim sequencer
Then fire a shot from another terminal:
    ros2 topic pub --once /archer/target_angle std_msgs/msg/Float64 "{data: 30.0}"
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# What each state commands when entered, and which state follows it.
NEXT_STATE = {
    'AIM': 'DRAW',
    'DRAW': 'HOLD',
    'HOLD': 'RELEASE',
    'RELEASE': 'RECOVER',
    'RECOVER': 'IDLE',
}


class Sequencer(Node):
    def __init__(self):
        super().__init__('sequencer')

        # Tunable timings (seconds) + how far the draw hand pulls (metres).
        # Override at runtime, e.g.  --ros-args -p aim_time:=3.0
        self.declare_parameter('aim_time', 2.0)
        self.declare_parameter('draw_time', 1.5)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('release_time', 0.6)
        self.declare_parameter('recover_time', 2.0)
        self.declare_parameter('draw_distance', -0.15)

        self.base_pub = self.create_publisher(Float64, '/archer/base_cmd', 10)
        self.draw_pub = self.create_publisher(Float64, '/archer/draw_cmd', 10)
        self.create_subscription(
            Float64, '/archer/target_angle', self.on_target, 10)

        self.state = 'IDLE'
        self.state_elapsed = 0.0
        self.target_rad = 0.0
        self.dt = 0.05  # 20 Hz tick
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            'Sequencer ready. Publish degrees to /archer/target_angle to fire.')

    # --- trigger ------------------------------------------------------------
    def on_target(self, msg: Float64):
        if self.state != 'IDLE':
            self.get_logger().warn(
                f'Busy ({self.state}); ignoring target {msg.data:+.1f}.')
            return
        self.target_rad = math.radians(msg.data)
        self.get_logger().info(f'Target {msg.data:+.1f} deg received.')
        self.enter('AIM')

    # --- state machine ------------------------------------------------------
    def enter(self, state: str):
        self.state = state
        self.state_elapsed = 0.0
        if state == 'AIM':
            self._send(self.base_pub, self.target_rad)
        elif state == 'DRAW':
            self._send(self.draw_pub, self._p('draw_distance'))
        elif state == 'RELEASE':
            self._send(self.draw_pub, 0.0)        # let the string go
        elif state == 'RECOVER':
            self._send(self.base_pub, 0.0)        # swivel back to center
        # HOLD commands nothing — it just waits, string drawn.
        self.get_logger().info(f'-> {state}')

    def tick(self):
        if self.state == 'IDLE':
            return
        self.state_elapsed += self.dt
        if self.state_elapsed < self._duration(self.state):
            return
        nxt = NEXT_STATE[self.state]
        if nxt == 'IDLE':
            self.state = 'IDLE'
            self.get_logger().info('Shot complete. Back to IDLE.')
        else:
            self.enter(nxt)

    # --- helpers ------------------------------------------------------------
    def _duration(self, state: str) -> float:
        return self._p(f'{state.lower()}_time')

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _send(self, pub, value: float):
        pub.publish(Float64(data=float(value)))


def main():
    rclpy.init()
    node = Sequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
