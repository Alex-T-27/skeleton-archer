"""
P2.3 target_detector — the archer's eye, in sim.

Subscribes to the simulated camera (/archer/camera), runs the same AprilTag
detection as the P1 webcam script, converts the tag's pixel position into an
aim angle (degrees), and publishes it to /archer/target_angle for the sequencer.

Because this is a simulated camera we KNOW its field of view exactly
(camera_hfov_deg matches the SDF), so the angle is accurate, not a guess.

Run (with archer_vision.launch.py already up):
    ros2 run archer_sim target_detector
Watch what it sees:
    ros2 topic echo /archer/target_angle
"""

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')

        # Must match <horizontal_fov> in the SDF camera (1.2 rad = 68.75 deg).
        self.declare_parameter('camera_hfov_deg', 68.75)
        # Flip if the archer aims the WRONG way (camera frame handedness).
        self.declare_parameter('angle_sign', -1.0)

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.angle_pub = self.create_publisher(Float64, '/archer/target_angle', 10)
        # [id, bearing] of the nearest-center tag, for the brain to lock onto
        self.target_pub = self.create_publisher(
            Float64MultiArray, '/archer/target', 10)
        self.create_subscription(Image, '/archer/camera', self.on_image, 10)

        self._last_log = -1.0
        self.get_logger().info('Target detector ready. Watching /archer/camera.')

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return

        # With multiple tags, report the one nearest the image center -- that's
        # the one the bow is closest to aiming at.
        mid = frame_width / 2.0
        ids = ids.flatten()
        centers = [float(c[0][:, 0].mean()) for c in corners]
        best = min(range(len(centers)), key=lambda i: abs(centers[i] - mid))
        center_x = centers[best]
        tag_id = int(ids[best])
        angle = self.pixel_to_angle(center_x, frame_width)

        # bearing alone (manual/sequencer) + [id, bearing] (for the brain to lock)
        self.angle_pub.publish(Float64(data=angle))
        self.target_pub.publish(Float64MultiArray(data=[float(tag_id), angle]))

        # Log at most ~once/sec so we don't flood the terminal.
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_log > 1.0:
            self.get_logger().info(f'tag id={tag_id}  angle={angle:+.1f} deg')
            self._last_log = now

    def pixel_to_angle(self, center_x: float, frame_width: int) -> float:
        hfov = self.get_parameter('camera_hfov_deg').value
        sign = self.get_parameter('angle_sign').value
        half_width = frame_width / 2.0
        offset_fraction = (center_x - half_width) / half_width   # -1..+1
        return sign * offset_fraction * (hfov / 2.0)


def main():
    rclpy.init()
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
