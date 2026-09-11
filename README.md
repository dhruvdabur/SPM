# 3RRR simulation and orientation inverse kinematics

## Setup

```bash
cd /home/dhruv/mujoco_3rrr
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Launch the physics viewer with `python main.py`. It reads accepted motor targets
from `web_control.py` at http://127.0.0.1:8000 and moves the robot using position
servos. Start the two programs in separate terminals, in either order. Restart
any viewer opened before this command connection was added.
Use `python main.py --manual` for the original standalone interactive viewer.

## Calculate angles directly

Arguments are **yaw, pitch, roll**, in radians unless `--degrees` is supplied:

```bash
python ik.py 10 5 3 --degrees
```

This returns the three motor angles (`q1`, `q2`, `q3`) in radians and degrees,
plus the passive-joint angles and maximum attachment-point closure error.
For this example the motor angles are approximately `[-5.53665, -17.60948,
-6.19278]` degrees. Positive joint angles follow the XML axes; all three motor
axes point along negative Z, so positive platform yaw gives negative motor angles.

To inspect the calculated pose in a separate static viewer:

```bash
python ik.py 10 5 3 --degrees --view
```

This viewer displays the solved configuration without stepping physics. It does
not send commands to an already running `main.py` viewer.

## ROS 2 Humble node

From the project directory, terminal 1:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python ik_node.py --ros-args -p degrees:=true
```

Terminal 2, send yaw=10°, pitch=5°, roll=3°:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /platform_ypr std_msgs/msg/Float64MultiArray '{data: [10.0, 5.0, 3.0]}'
```

Terminal 3, start this before sending a target to see its result:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_commands
```

Omit `-p degrees:=true` for radian input. Output angles are **always radians**.
The node publishes once per successful target, not continuously.

| Topic | Message | Contents |
| --- | --- | --- |
| `/platform_ypr` | `std_msgs/msg/Float64MultiArray` | Input `[yaw, pitch, roll]` |
| `/joint_commands` | `sensor_msgs/msg/JointState` | Calculated positions named `level-1-gear-rev`, `level-2-gear-rev`, `level-3-gear-rev` |
| `/motor_angles` | `std_msgs/msg/Float64MultiArray` | Calculated `[q1, q2, q3]` for the MuJoCo actuators |
| `/ik_status` | `std_msgs/msg/String` | JSON success/closure error, or failure reason |

These are calculated commands, not measured joint states. The node does not
actuate hardware or connect to `main.py`; a controller can subscribe to the
outputs. Invalid or unsolved targets produce a failure status and no angle
command. The last successful branch is retained.

A different compatible model can be supplied with
`--ros-args -p model_path:=/absolute/path/to/model.xml`.

## Geometry, conventions and limitations

Orientation uses `Rz(yaw) @ Ry(pitch) @ Rx(roll)` relative to the initial platform
orientation, with axes expressed in the world/base frame. In this model the
initial platform rotation is identity. The platform body's origin is at its
first attachment, not at the spherical center; its translation must change
when it rotates.

The solver reads `models/3rrr.xml`, which carries the exported URDF joint
origins/axes and adds the three loop connections absent from the URDF tree.
It solves all nine attachment-position equations for six hinge angles and
three platform translation coordinates, holding the requested rotation fixed.
An analytic MuJoCo site Jacobian is used with SciPy least squares. Small
orientation continuation steps start from the previous solution (home on
startup) to follow one assembly branch. Closure tolerance is 1 micrometre.
Failure means no acceptable solution was found along that branch; it is not
proof that all assembly branches are unreachable.

This implementation was informed by
[mateofernandezmier19/3-RRR_SPM](https://github.com/mateofernandezmier19/3-RRR_SPM),
especially `MATLAB_CODE/inverse_kinematics.m`, `define_coefficients.m`, and
`define_Q.m`. That implementation uses an ideal spherical geometry and
`Rz @ Rx @ Ry`; its angle convention and geometry constants are not copied
into this model-specific yaw–pitch–roll solver.

The current XML uses point (`connect`) constraints at the platform, not full
revolute-axis constraints, and two attachment coordinates are marked as
symmetry estimates. Results match that simulation geometry; they do not
validate the physical robot's top hinge axes. The URDF's zero joint limits
are export placeholders; this solver uses the XML limits (currently none).
IK also does not perform collision/path clearance checks or enforce motor
speed/torque limits. It is a simulation IK calculator, not a calibrated
hardware motion controller.

## Verification

```bash
python -m unittest discover -s tests -v
```

Checks cover home/pure yaw, mixed orientations, invalid/unsolved targets,
branch-state preservation, and physics-driven tracking from home.

## Joystick website

```bash
cd /home/dhruv/mujoco_3rrr
source .venv/bin/activate
python web_control.py
```

Open **http://127.0.0.1:8000**. Drag the joystick horizontally for roll and
vertically for pitch; use the yaw slider for rotation. All interface values
are degrees. Release holds the selected tilt. The sliders also support
keyboard control. Turn off **Send while moving** to compose a pose, then
click **Send target**. **Return to zero** always sends a zero target.

In a second terminal, run the live simulation:

```bash
source .venv/bin/activate
python main.py
```

The terminal prints `Connected to .../api/state` when the connection succeeds.
The simulator polls the last accepted motor angles in the background and ramps
the servo targets at up to 0.5 rad/s while stepping physics. If the server is
unavailable, physics continues with the last target and reconnects automatically.
This ramp is not a collision-checked motion planner. The website preview remains
a target schematic, not simulation feedback.

The browser sends at most ten sequential requests per second, keeping the
latest pending target. The server calculates IK and displays the last accepted
motor angles. The preview is an orientation schematic, not live MuJoCo feedback.
UI ranges are yaw ±180° and pitch/roll ±20°; these are interface limits, not a
guarantee that every combination is reachable. Failed IK leaves the last
accepted command unchanged.

To publish the calculated joint commands to ROS 2 as well:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python web_control.py --ros
```

In this mode the web server performs IK itself and publishes `/joint_commands`
(`JointState`, URDF motor names) and `/motor_angles` (`Float64MultiArray`,
`[q1,q2,q3]`), both in radians. Run this instead of `ik_node.py` to avoid two
publishers commanding the same output topics. `main.py` reads commands through
the web API whether ROS output is enabled or disabled. Hardware requires a
separate controller.

Use `--port 8001` for another port and start the viewer with
`python main.py --web-url http://127.0.0.1:8001`. The server binds to localhost only.
