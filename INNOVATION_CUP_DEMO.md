# Innovation Cup Demo

A self-contained "wander around" demo for the SORA mecanum robot, using the
YDLidar for obstacle avoidance. It can be started/stopped from a **phone web
page** (recommended) and/or an **Xbox controller**.

## What it does

1. On launch it starts the LiDAR, the mecanum kinematics, the motor driver, the
   `demo_wander` behaviour node, the `web_control` web server, and (optionally)
   the Xbox `joy` node.
2. The robot **waits**, holding still, until it receives a **start** command.
3. After **start** it roams: drives forward, wanders gently, and steers /
   rotates away from obstacles seen by the LiDAR. If something is very close it
   backs off while turning toward the more open side.
4. A **stop** command halts it in place at any time; **start** resumes.

Start/stop can come from either input, which both feed the same behaviour:

| Input | Start | Stop |
|---|---|---|
| Phone web page | **START ROAMING** button | **STOP** button |
| Xbox controller | **A** | **B** |

## Run it

```bash
source /opt/ros/jazzy/setup.bash
source ~/sora_ws/install/setup.bash
ros2 launch sora_base_control demo.launch.py
```

### Phone web control

With the phone joined to the robot's WiFi, open:

```
http://<robot-ip>:8080/
```

`<robot-ip>` is the robot's address on its own network — e.g. `192.168.4.1`
when the Pi is its own access point (see below). The page shows a live
STOPPED / ROAMING status and big START / STOP buttons. It is fully
self-contained (no internet needed on the AP).

Useful launch overrides:

```bash
# change the web port (e.g. 80 so the URL needs no :port — see autostart notes)
ros2 launch sora_base_control demo.launch.py web_port:=8080

# web-only, don't start the joystick node at all
ros2 launch sora_base_control demo.launch.py use_joy:=false

# different joystick device
ros2 launch sora_base_control demo.launch.py joy_dev:=/dev/input/js1
```

### Topic interface

`demo_wander` is driven by (and reports on) plain topics, so you can also test
from a terminal:

```bash
ros2 topic pub --once /demo/command std_msgs/String "{data: start}"
ros2 topic pub --once /demo/command std_msgs/String "{data: stop}"
ros2 topic echo /demo/state      # STOPPED / ROAMING (latched)
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

## Robot WiFi access point (Pi side)

For the phone web control, the Pi should broadcast its own WiFi so the phone can
join it with no router/internet. On Raspberry Pi OS (NetworkManager), a simple
reference setup — run once on the Pi:

```bash
sudo nmcli device wifi hotspot ifname wlan0 ssid SORA-DEMO password "sora12345"
# make it come up automatically on every boot:
sudo nmcli connection modify Hotspot connection.autoconnect yes connection.autoconnect-priority 100
```

The Pi is then reachable at `192.168.4.1` (NetworkManager's default hotspot
address). Join the `SORA-DEMO` network on your phone and open
`http://192.168.4.1:8080/`.

Check the address the hotspot actually assigned with:

```bash
ip -4 addr show wlan0
```

> Note: enabling the hotspot takes `wlan0` off your normal WiFi, so SSH over the
> LAN will drop — reconnect by joining the robot's own `SORA-DEMO` network (or
> keep a wired/other path for admin).

### Serving on port 80 (optional, nicer URL)

To drop the `:8080` so the phone can just open `http://192.168.4.1/`, run the
web node on port 80. Binding port 80 needs the capability — in the systemd unit
add `AmbientCapabilities=CAP_NET_BIND_SERVICE` and launch with `web_port:=80`.

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
