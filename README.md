# Arctos Arm — Gazebo Simulation

Full-physics simulation of the Arctos 6-DOF arm in Gazebo Harmonic with
`ros2_control`.  The same `joint_trajectory_controller` pipeline runs in
simulation and on real hardware — the only thing that changes is the
hardware interface (Gazebo vs CAN bus).

---

## Architecture

```
Gazebo Harmonic (gz-sim 8)
  └─ gz_ros2_control plugin
       └─ controller_manager (inside Gazebo process)
            ├─ joint_state_broadcaster  ──► /joint_states ──► robot_state_publisher ──► TF
            ├─ arctos_arm_controller    ◄── /arctos_arm_controller/follow_joint_trajectory
            └─ arctos_hand_controller   ◄── /arctos_hand_controller/gripper_cmd
```

**Same interface on real hardware** — only `<hardware><plugin>` changes in
`arctos.ros2_control.xacro` (from `GazeboSimSystem` to `ArctosInterface`).

---

## Prerequisites

| Package | Version | Notes |
|---------|---------|-------|
| ROS 2 Humble | any | `source /opt/ros/humble/setup.bash` |
| Gazebo Harmonic | gz-sim 8 | `sudo apt install gz-harmonic` |
| gz_ros2_control | **built from source** | the apt binary is compiled for gz-plugin 1.x — see below |

---

## One-time setup: build `gz_ros2_control` from source

The apt package `ros-humble-gz-ros2-control` (v0.7.17) was compiled against
**gz-plugin 1.x** (Ignition Fortress).  gz-sim 8 requires **gz-plugin 2.x**.
The plugin silently fails to load, `/controller_manager` never appears, and
the arm falls under gravity with no control.

```bash
# From the repo root — run once, takes ~2 minutes
bash scripts/setup_gz_ros2_control.sh
```

The script:
1. Removes the broken apt package
2. Creates `~/ros2_ws/src/`
3. Clones `gz_ros2_control` (humble branch) and builds it against gz-sim 8
4. Verifies the built `.so` links `gz-plugin2` (not IgnitionPlugin)
5. Symlinks `arctos_gazebo` into the workspace and builds it

### If the build fails with "gz-sim8 not found in CMakeLists"

The `humble` branch may target gz-sim 7 (Garden).  Patch it:

```bash
sed -i \
  's/find_package(gz-sim7/find_package(gz-sim8/g;
   s/gz-sim7::/gz-sim8::/g' \
  ~/ros2_ws/src/gz_ros2_control/CMakeLists.txt
cd ~/ros2_ws
colcon build --symlink-install --packages-select gz_ros2_control
```

---

## Launching the simulation — step by step

### Step 1: Source the workspace

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

> **Important**: source `~/ros2_ws` second so the source-built
> `gz_ros2_control` takes priority over any system install.

### Step 2: Launch

```bash
ros2 launch arctos_gazebo sim.launch.py
```

What happens in the first 35 seconds:

| Time | Event |
|------|-------|
| 0 s | Gazebo loads `arctos_world.sdf` (with robot embedded via `<include>`) |
| 2–5 s | gz_ros2_control plugin loads; `controller_manager` starts inside Gazebo |
| 5 s | `robot_state_publisher` + `ros_gz_bridge` start |
| 30 s | `joint_state_broadcaster` spawner fires (30 s timer) |
| 31 s | `arctos_arm_controller` + `arctos_hand_controller` spawn after JSB exits |
| 35 s | All controllers should be `active` |

### Step 3: Checkpoint A — gz_ros2_control loaded (open a new terminal)

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 node list | grep controller_manager
```

Expected:
```
/controller_manager
```

If `/controller_manager` is absent after 10 s, the plugin did not load.
Check Gazebo stdout for errors containing `gz_ros2_control`.

### Step 4: Checkpoint B — controllers active (~35 s)

```bash
ros2 control list_controllers
```

Expected:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
arctos_arm_controller  [joint_trajectory_controller/JointTrajectoryController] active
arctos_hand_controller [position_controllers/GripperActionController] active
```

### Step 5: Test — move the arm against gravity

```bash
ros2 action send_goal \
  /arctos_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [X_joint, Y_joint, Z_joint, A_joint, B_joint, C_joint],
      points: [{
        positions: [0.0, 0.5, -0.3, 0.0, 0.3, 0.0],
        time_from_start: {sec: 3}
      }]
    }
  }'
```

The arm should smoothly move to the commanded position and hold it against
gravity.  Watch `/joint_states` in RViz to confirm.

---

## Debugging

### Plugin fails to load (`IgnitionPluginHook` error)

```
[gz] [Err] [] Failed to load system plugin [gz_ros2_control-system] ...
           undefined symbol: _ZN8ignition6plugin...IgnitionPluginHook
```

The source-built plugin was still linked against gz-plugin 1.x.  Verify:

```bash
ldd ~/ros2_ws/install/gz_ros2_control/lib/libgz_ros2_control*.so \
  | grep -i 'gz-plugin\|ign-plugin'
```

If it shows `libignition-plugin1` → the build picked up gz-sim 7 headers.
Re-run the CMakeLists patch above and rebuild.

### Controller spawner times out

```
[spawner] Timeout waiting for controller_manager
```

`/controller_manager` didn't start — the plugin didn't load.  Go back to
the plugin debug above.

### Robot falls immediately (no gravity hold)

Controllers are active but PID gains are too low (or missing).  For a quick
test add `state_tolerance_check: false` (already set in `sim_controllers.yaml`)
and check `allow_partial_joints_goal: true`.  For long-term stability tune
PID gains in `sim_controllers.yaml`.

### WSL2: Gazebo hangs on startup

The `ogre` render engine is set in `arctos_world.sdf` (not `ogre2`) to work
with llvmpipe software rendering.  If it still hangs, try:

```bash
LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 \
  ros2 launch arctos_gazebo sim.launch.py
```

---

## Real hardware

On the physical arm, swap the hardware interface in
`arctos_description/urdf/arctos.ros2_control.xacro`:

```xml
<!-- Simulation -->
<plugin>gz_ros2_control/GazeboSimSystem</plugin>

<!-- Real hardware (CAN bus via arctos_hardware_interface) -->
<plugin>arctos_hardware_interface/ArctosInterface</plugin>
```

All controllers, topic names, and action interfaces remain identical.
