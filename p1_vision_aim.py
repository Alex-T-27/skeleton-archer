"""
P1 — Vision -> target angle  (Skeleton Archer)

What this does:
  webcam frame -> detect AprilTag (36h11) -> find its center -> turn that center
  into an aim angle: how far left/right of straight-ahead the target is.

Run:  python3 p1_vision_aim.py        (press q in the window to quit)
"""

import cv2

# ---------------------------------------------------------------------------
# Camera settings. /dev/video0 is your webcam. If the window is black, try
# CAM_INDEX = 1 instead.
# ---------------------------------------------------------------------------
CAM_INDEX = 0

# Horizontal field of view of the camera, in degrees. A guess for now — most
# webcams are roughly 60-70 deg across. You'll need this for the angle math.
# (Later: measure it properly via camera calibration. Not today.)
CAMERA_HFOV_DEG = 65.0


def pixel_to_angle(center_x: float, frame_width: int) -> float:
    """
    Turn the tag's horizontal pixel position into an aim angle.

      center_x     : tag's horizontal pixel (0 = far left, frame_width = far right)
      frame_width  : total frame width in pixels

    Returns degrees:  negative = LEFT of straight-ahead, 0 = center, positive = RIGHT.

    How it works: measure how far off-center the tag is as a fraction of half the
    frame, then scale that fraction onto half the camera's field of view. The
    "half" on both sides is because center-to-edge is half the frame AND half the
    total viewing angle. This is the rough linear version (good enough for
    left/center/right + an approximate angle; an uncalibrated camera can't do
    better anyway).
    """
    half_width = frame_width / 2.0
    offset_fraction = (center_x - half_width) / half_width   # -1.0 .. +1.0
    return offset_fraction * (CAMERA_HFOV_DEG / 2.0)


def main():
    # --- Layer 1: open the webcam --------------------------------------------
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open camera index {CAM_INDEX}. Try the other index.")
        return

    # --- Layer 2: set up the AprilTag detector -------------------------------
    # OpenCV speaks AprilTag through its "aruco" module. We tell it the tag
    # family (36h11) and it hands back the 4 corners of any tag it sees.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()

    print("Camera open. Hold a tag36h11 AprilTag in view. Press q to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Dropped a frame.")
            break

        frame_height, frame_width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # corners: a list, one entry per detected tag. Each entry is the tag's
        # 4 corner points, in order: top-left, top-right, bottom-right, bottom-left.
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=params
        )

        if ids is not None:
            # draw the outlines + ids so you can see what it locked onto
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for tag_corners, tag_id in zip(corners, ids.flatten()):
                pts = tag_corners[0]  # shape (4, 2): the four corner points

                # center = average of the four corners
                center_x = float(pts[:, 0].mean())
                center_y = float(pts[:, 1].mean())

                # --- Layer 3: the part you implement ---------------------
                angle = pixel_to_angle(center_x, frame_width)

                label = f"id={tag_id}  x={center_x:.0f}  angle={angle:+.1f}"
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame, label, (10, 30 + 25 * int(tag_id % 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                )
                print(label)

        # a vertical line marking dead-center, so you can eyeball your angle
        cx = frame.shape[1] // 2
        cv2.line(frame, (cx, 0), (cx, frame.shape[0]), (0, 255, 0), 1)

        cv2.imshow("P1 vision aim", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
