# Lab 6: Motion Planning for Mobile Robots (Nav2)

## Description

Nav2 stack containing:
- Map Server;
- Localization;
- Planner;
- Controller;
- Costmaps;
- Behavior tree.

## Setup

Rebuild the Docker image the first time (so Nav2 packages are installed):

```bash
./scripts/cmd build-docker
```

Inside the container:

```bash
cd /opt/ws
colcon build --packages-select lab6
source install/setup.bash
```

## Launch

```bash
ros2 launch lab6 nav2_room_bringup.launch.py
```

This starts:

- Gazebo with **`room_nav2`** (8×8 m room, perimeter walls, and ten fixed box obstacles).
- TurtleBot3 at (0, 0).
- Nav2: **map_server**, **AMCL**, planner, controller, behavior tree, etc.

## Run Navigation

1. Do **Setup** and **Launch** so the package builds and Nav2 starts.
2. In RViz, set **Fixed Frame** to `map` if needed so the map, path, and robot line up.
3. Use **2D Pose Estimate** so the robot in RViz matches Gazebo. Wait if needed; **2D Pose Estimate** again if the map and room do not match.
4. When the pose looks stable, use **Nav2 Goal** for a target. If nothing happens, wait, fix **2D Pose Estimate**, try again. Read the terminal for errors.
5. Watch the global path, local plan, and costmaps.

## Tweak the Parameters

### Global vs Local Planning

**Global** vs **local** (short):

- **Global planner** (`planner_server`): Uses the **saved map** on disk and draws the **global path** in RViz. It does **not** build that path from the laser step by step.

- **Local controller** (`controller_server` / **FollowPath**): Runs **while the robot moves**, follows the global path, uses the **local costmap** (laser, small rolling map) for nearby obstacles, and sends **`/cmd_vel`**.

So: **global** = path on the **saved map**; **local** = **use sensors** around the robot while driving.

### Parameters Description 

Change different parameters in `lab6/config/nav2_params.yaml` to alter the behaviour of the robot and local costmap. The description of the most important parameters is given below.

**Inflation** (in `nav2_params.yaml`, under `global_costmap` / `local_costmap` → `inflation_layer`)

- **`inflation_radius`** — How wide the “cushion” around walls is. **Bigger** → the path stays **farther from walls**; the route may get **longer**; the robot may pass **narrow places** more safely.
- **`cost_scaling_factor`** — How fast the cost drops as you move away from a wall. **Bigger** → the cushion looks **stronger** in RViz; the planner **hates** squeezing next to walls **more**.

**Costmap `resolution`** (same file, `global_costmap` / `local_costmap`)

- **Smaller** numbers (e.g. `0.05` → `0.03` m) → **smaller cells**, finer grid in RViz, more CPU work.
- **Bigger** numbers (e.g. `0.1` m) → **bigger cells**, blockier grid; path may look more **jagged**; thin gaps can **vanish** on the grid.
- The map file uses **`0.05` m** (`lab6/maps/room_nav2.yaml`). If changing **global** resolution causes problems, try only **`local_costmap`** first, or keep **global** at **`0.05`**.

**Costmap rates** (`global_costmap` / `local_costmap` → top-level `ros__parameters`)

- **`update_frequency`** — How often the costmap **recomputes** (Hz). **Higher** → grid **tracks** the laser and the robot **faster** (often **less** smearing when turning); **lower** → less CPU, but obstacles can look **laggy** or **drift**.
- **`publish_frequency`** — How often the costmap is **sent to RViz** (Hz). **Higher** → **smoother** display; **lower** → can look **choppy** even if the map is fine internally.

**Controller server** (`controller_server` → `ros__parameters`)

- **`controller_frequency`** — How often the local controller **runs** (Hz). **Higher** → **snappier** `/cmd_vel` updates (can help next to a fast **local** costmap); **lower** → simpler load, motion can feel **softer** or **late**.

**FollowPath** (`controller_server` → `FollowPath`; default in this lab is **DWB** — other plugins use different names)

- **`max_speed_xy`** — Top speed in the plane (m/s). On a **diff-drive** robot, **`max_vel_x`** is the main forward limit. **Higher** → **faster** motion if nothing else blocks it.
- **`max_vel_theta`** — Max **turn rate** (rad/s). **Higher** → **faster** spins (can make laser/costmap look **worse** when rotating); **lower** → **slower** turns, often **calmer** near walls.
- **`acc_lim_theta`** — Max **angular acceleration** (rad/s²). **Higher** → reaches top spin speed **faster**; **lower** → **gentler** starts/stops of rotation.
- **`decel_lim_theta`** — Max **angular deceleration** (negative rad/s² in YAML). **Higher magnitude** → **sharper** braking of rotation; **lower** → **smoother** stop.

**Goal checker** (`controller_server` → `goal_checker`)

- **`xy_goal_tolerance`** — How close (meters) the robot must sit on the goal point. **Bigger** → “good enough” stop **sooner**; **smaller** → takes **longer** to finish (sim noise can make this hard).
- **`yaw_goal_tolerance`** — How close the final angle must be (radians). **Bigger** → may stop **more sideways**; **smaller** → more **turning in place** at the end.

You can also change **other** keys in `nav2_params.yaml` (speed limits, lookahead, planner options, etc.). The list above is just a guide.

After you edit YAML, rebuild and launch again:

```bash
cd /opt/ws
colcon build --packages-select lab6
source install/setup.bash
ros2 launch lab6 nav2_room_bringup.launch.py
```