# Innovation Cup Demo

A self-contained "wander around" demo for the SORA mecanum robot, driven by an
Xbox controller and the YDLidar for obstacle avoidance.

## What it does

1. On launch it starts the LiDAR, the Xbox `joy` node, the mecanum kinematics,
   the motor driver and the `demo_wander` behaviour node.
2. The robot **waits**, holding still, until you press **A** on the controller.
3. After **A** it roams: drives forward, wanders gently, and steers / rotates
   away from obstacles seen by the LiDAR. If something is very close it backs
   off while turning toward the more open side.
4. Press **B** at any time to stop in place. Press **A** again to resume.

## Run it

```bash
source /opt/ros/jazzy/setup.bash
source ~/sora_ws/install/setup.bash
ros2 launch sora_base_control demo.launch.py
```

Then connect the Xbox controller and press **A**.

Optional overrides:

```bash
ros2 launch sora_base_control demo.launch.py joy_dev:=/dev/input/js1
```

## Autostart on the Raspberry Pi

Build once on the Pi after pulling this branch:

```bash
cd ~/sora_ws
colcon build --packages-select sora_base_control sora_motor_driver
```

Then point your boot service (systemd unit / rc.local) at:

```bash
bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/sora_ws/install/setup.bash && ros2 launch sora_base_control demo.launch.py'
```

## Tuning

All behaviour is parameterised on the `demo_wander` node — adjust in
`launch/demo.launch.py` or override at launch. Key ones:

| parameter | default | meaning |
|---|---|---|
| `forward_speed` | 0.35 m/s | cruise speed |
| `turn_speed` | 1.0 rad/s | in-place / avoidance turn rate |
| `slow_distance` | 1.0 m | start slowing & steering below this |
| `stop_distance` | 0.50 m | rotate in place below this |
| `backup_distance` | 0.30 m | reverse while turning below this |
| `front_half_angle_deg` | 35 | half-width of the front cone |

Two control-scheme improvements back this demo (both backward compatible):

* `mecanum_kinematics` now scales wheel commands against a tunable
  `max_wheel_speed` (default 6.0 rad/s) so speed is proportional to `/cmd_vel`
  instead of saturating to full throttle on any input. Robot dimensions are
  unchanged.
* `motor_driver_node` gained an optional `min_move_throttle` (default 0.0,
  i.e. disabled) that keeps slow commands above the stiction threshold; the
  demo launch sets it to 0.22.
