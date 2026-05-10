# OpenArm Bringup

This package provides launch files to bring up the OpenArm robot system.

## Quick Start

Launch the OpenArm with v1.0 configuration and fake hardware:

```bash
ros2 launch openarm_bringup openarm.launch.py arm_type:=v10 hardware_type:=real
```

## Launch Files

- `openarm.launch.py` - Single arm configuration
- `openarm.bimanual.launch.py` - Dual arm configuration
- `openarm.bimanual.mobile_base.launch.py` - Dual arm configuration with an omnidirectional mobile base

## Bimanual Mobile Base

Launch the bimanual robot with the omnidirectional mobile base:

```bash
source /home/cw/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.mobile_base.launch.py
```

The launch file uses `openarm_v10_bimanual_mobile_base_controllers.yaml` and starts `mobile_base_controller`.

Move the base forward:

```bash
ros2 topic pub --rate 10 /mobile_base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
'{header: {frame_id: "mobile_base_link"}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```

Move the base sideways:

```bash
ros2 topic pub --rate 10 /mobile_base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
'{header: {frame_id: "mobile_base_link"}, twist: {linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```

Rotate the base:

```bash
ros2 topic pub --rate 10 /mobile_base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
'{header: {frame_id: "mobile_base_link"}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}'
```

## Key Parameters

- `arm_type` - Arm type (default: v10)
- `hardware_type` - Use real/mock/mujoco hardware (default: real)
- `can_interface` - CAN interface to use (default: can0)
- `robot_controller` - Controller type: `joint_trajectory_controller` or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization
