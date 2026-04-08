# Laboratory 7 — Coordinate transforms (TF2), robot description (URDF/Xacro), and the RTR manipulator

## Theory - RTR Manipulator and Kinematics Explanation

The **RTR** (Revolute–Translational–Revolute) manipulator is a three-degree-of-freedom serial chain: base rotation about a vertical axis, translation along that axis, and a revolute elbow in a vertical plane. The photograph below illustrates a representative configuration; the simplified schematic under it matches the joint ordering used in the provided URDF.

<img src="docs/images/rtr_manipulator_photo.jpg" width="30%">

**Joint variables** (notation used in this laboratory):

| Symbol | Type | Role in the provided URDF |
|--------|------|---------------------------|
| $\theta_1$ | Revolute | Base yaw about the vertical axis |
| $\theta_2$ | Prismatic | Vertical translation |
| $\theta_3$ | Revolute | Elbow in the vertical plane |

**Link lengths** (default parameters in the package): $l_2 = 0.9\,\mathrm{m}$, $l_3 = 1.0\,\mathrm{m}$.

**Forward kinematics** — end-effector position $\mathbf{p} = (x,y,z)^T$ in the base frame:

$$ \mathbf{p} =
\begin{pmatrix}
\cos\theta_1\,(l_3\cos\theta_3 + l_2) \\
\sin\theta_1\,(l_3\cos\theta_3 + l_2) \\
l_3\sin\theta_3 + \theta_2
\end{pmatrix}.$$

Python helpers for $\mathbf{p}$, the orientation quaternion, the combined pose ``rtr_end_effector_transform`` (used by the TF2 demos), and TF-vs-analytic checks live in `lab7/rtr_kinematics.py`.

---

## Software and Package Contents

Work in the **container** workspace `/opt/ws` with sources under `/opt/ws/src/code`.

| Path | Description |
|------|-------------|
| `lab7/tf2_demo_cli.py` | Parses `theta_1 theta_2 theta_3 [l2] [l3]` for the TF2 demos |
| `lab7/tf2_broadcaster_demo.py` | **Dynamic** broadcaster: fixed pose from CLI (`world` → `rtr_ee_demo`) |
| `lab7/tf2_listener_demo.py` | Same CLI; recomputes analytic pose and checks TF vs theory |
| `lab7/rtr_kinematics.py` | Forward kinematics, ``rtr_end_effector_transform``, TF pose matching |
| `urdf/rtr_manipulator.xacro` | RTR model and `ros2_control` mock hardware block |
| `launch/rtr_visualize.launch.py` | `joint_state_publisher_gui`, `robot_state_publisher`, RViz2 |
| `launch/rtr_ros2_control.launch.py` | `ros2_control_node`, **joint_state_broadcaster**, **forward_position_controller** |
| `config/rtr_controllers.yaml` | Controller manager and forward-command parameters |
| `tests/test_rtr_kinematics.py` | Kinematics unit tests (no ROS graph) |
| `tests/test_tf2_analytic_agreement.py` | Stepwise pose vs `forward_position` / TF matcher (no ROS graph) |

## Build The Package

```bash
cd /opt/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lab7 --symlink-install
source install/setup.bash
```

If `docker/Dockerfile` was updated on the repository, rebuild the image on the host and start a new container:

```bash
./scripts/cmd build-docker
./scripts/cmd run
```

## TF2 broadcaster and listener

Arguments: **`theta_1 theta_2 theta_3`** (rad, rad, rad), optional **`l2`** **`l3`** (m, default `0.9` `1.0`). Here `theta_2` is the prismatic coordinate in metres, matching the laboratory RTR model.

**Terminal 1 — broadcaster**

```bash
ros2 run lab7 tf2_broadcaster_demo -- 0.2 0.5 0.35
```

**Terminal 2 — listener** (use the **same** numbers so the analytic check succeeds)

```bash
ros2 run lab7 tf2_listener_demo -- 0.2 0.5 0.35
```

**Optional verification**

```bash
ros2 run tf2_ros tf2_echo world rtr_ee_demo
```

## URDF/Xacro, joint GUI, and TF

```bash
ros2 launch lab7 rtr_visualize.launch.py
```

In **RViz2**, set **Fixed Frame** to `world` if needed and enable **TF**.

## `joint_state_broadcaster` and TF queries

```bash
ros2 launch lab7 rtr_ros2_control.launch.py
```

Send a position command (revolute joints in **radians**, prismatic **joint_theta2** in **metres**):

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0.6, 0.4]}"
```

Example TF query:

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

**Warning.** Do not run `rtr_visualize.launch.py` and `rtr_ros2_control.launch.py` **simultaneously**; both would publish conflicting data on `/joint_states`.

## Automated check (kinematics)

```bash
colcon test --packages-select lab7
```

The suite checks forward kinematics, ``rtr_end_effector_transform`` (shared by the TF demos), and the TF-vs-analytic matcher the listener uses (without starting a ROS graph).

