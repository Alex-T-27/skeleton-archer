# Skeleton Archer

> Two robot arms, Minecraft-skeleton style, that find a target anywhere across a
> **180° arc** and loose a single toy arrow at it — on command.

A summer robotics build: a dual-arm archer on a rotating base that uses a camera
+ AprilTags to spot a target, swivels to aim, draws, and shoots. The "brain" runs
in **ROS 2**, so the logic proven in simulation carries straight over to the real
hardware.

**Scope, locked:** one shot · close range · toy arrow · printed bullseye with an
AprilTag on it · looks-first (it should *read* as a skeleton archer).

See [`plan.html`](plan.html) for the full design rationale and the four
feasibility tricks the project hinges on.

---

## Status

| Phase | What | State |
|------|------|-------|
| **P1** | Webcam → AprilTag → aim angle | ✅ done |
| **P2.1** | Archer modeled in Gazebo, joints driven from ROS 2 | ✅ done |
| **P2.2** | Shot sequencer (aim → draw → hold → release) | ✅ done |
| **P2.3** | Autonomous 180° vision: sweep → detect → aim → fire | ✅ done |
| **P2.4** | Projectile physics (the arrow actually flies) | ⏳ next |
| **P3** | Real arms + servos (SO-ARM100 style) | ⬜ hardware |
| **P4** | Integrate + skeleton styling | ⬜ |

Everything through P2.3 runs in pure software — no hardware required.

---

## How it works (the five parts)

1. **Launcher** — light elastic "bow", a toy arrow, a clean release.
2. **Two arms** — one holds the bow steady, one draws & releases (single release point).
3. **Aiming base** — a turntable swivels the body to face the target, so the arms
   always perform the same fixed draw motion.
4. **Vision brain** — camera + AprilTag → the target's bearing.
5. **Sequencer** — ROS 2 ties it together: detect → aim → draw → hold → shoot → recover.

---

## Running it

### P1 — webcam vision (standalone)

Needs Python 3 + OpenCV (with the `aruco` module — bundled in `opencv-python`).

```bash
python3 p1_vision_aim.py
```

Point your webcam at a `tag36h11` AprilTag (search "apriltag 36h11 generator").
It outlines the tag and prints how many degrees left/right of center it sits.

### P2 — Gazebo simulation

Needs **ROS 2 Jazzy**, **Gazebo Sim 8 (Harmonic)**, and the `ros_gz` packages.
Build the workspace once:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

**Manual control (P2.1 / P2.2)** — drive the archer, or trigger a shot by angle:

```bash
ros2 launch archer_sim archer_sim.launch.py
# in another terminal:
ros2 topic pub --once /archer/base_cmd std_msgs/msg/Float64 "{data: 0.7}"     # swivel
ros2 run archer_sim sequencer                                                  # then:
ros2 topic pub --once /archer/target_angle std_msgs/msg/Float64 "{data: 30}"   # full shot
```

**Autonomous 180° targeting (P2.3)** — the archer finds and engages a tag on its own:

```bash
ros2 launch archer_sim archer_vision.launch.py   # sim + camera + detector
ros2 run archer_sim archer_brain                 # sweep → detect → aim → fire → repeat
```

The camera rides on the turret and sweeps the arc non-stop; when a tag crosses the
center of view (bow aligned) it fires, then resumes sweeping. Moving targets and
multiple tags are handled for free.

> **Heads up:** `ros2 run` nodes keep running in the background. Before a fresh
> test, stop old ones (`pkill -f archer_brain`) — otherwise two nodes fight over
> the joints. Sanity check: `ros2 topic info /archer/base_cmd` should show
> publisher count `1`.

---

## Stack

- **ROS 2 Jazzy** — nodes, topics, launch
- **Gazebo Sim 8 (Harmonic)** + `ros_gz` — physics + the ROS↔Gz bridge
- **OpenCV** (`aruco`) — AprilTag detection
- **Python**

## Layout

```
p1_vision_aim.py            # P1: webcam → AprilTag → angle
plan.html                   # full design write-up
src/archer_sim/
  worlds/                   # Gazebo worlds (archer, + vision world with camera/target)
  launch/                   # bring-up launch files
  archer_sim/
    sequencer.py            # P2.2: timed shot state machine
    target_detector.py      # P2.3: AprilTag detection on the sim camera
    archer_brain.py         # P2.3: autonomous 180° sweep-and-fire
```

## Troubleshooting

- **Gazebo camera renders a blank grey gradient** (seen on RTX 50-series / recent
  NVIDIA drivers): `ogre2` fails to render the sensor off-screen. The vision world
  uses the legacy `ogre` engine and the launch pins EGL to the NVIDIA vendor
  (`__EGL_VENDOR_LIBRARY_FILENAMES`) — both are needed.
