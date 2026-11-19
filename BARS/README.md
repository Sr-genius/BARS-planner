# BARS

Reactive 3-D safety-corridor planner for quadrotors operating among static
and dynamic obstacles. BARS builds a time-decayed voxel map, searches a
three-dimensional global guide, expands it into an overlapping free-space
corridor, and bounds the closing speed against obstacles with a braking-aware
barrier. It runs entirely onboard and is designed to be adapted to a new
airframe through configuration rather than code changes.

## Features

- Time-decayed voxel map with bounded size and nearest-neighbour lookups.
- 3-D A* guide over a 26-connected free-cell grid, including altitude and
  clearance costs.
- Free-space corridor with hard-projected smoothing, so the smooth
  path never leaves collision-free cells.
- Braking-aware safety barrier that keeps every approach speed reachable
  under the acceleration limit, with independent clearance for dynamic
  objects.
- Acceleration-limited reference tracking and velocity-aligned heading.
- All topics, frames and parameters are configurable.

## Architecture

```
point cloud ──► time-decayed voxel map ──► free-cell grid ──► 3-D A* guide
                                                                      │
dynamic objects ───────────────────────────────────────► free-space corridor
                                                                      │
odometry, goal ─────────────────────────────────────────► hard-projected
                                                                      │
                                                              safety barrier
                                                                      │
                                                          tracking + heading
                                                                      │
                                                          PositionCommand
```

The planner node runs at 50 Hz. The guide is recomputed at the configurable
replanning interval; the barrier and tracking layers run on every cycle.

## Dependencies

- ROS Noetic (catkin workspace)
- PCL 1.10+
- Eigen 3
- [`quadrotor_msgs`](https://github.com/AutonomousFlight/quadrotor_msgs)

The other dependencies are standard ROS message packages. If your
autopilot already accepts `quadrotor_msgs/PositionCommand`, no command
conversion is required.

## Build

Clone the package into a catkin workspace and build:

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone <your-repository-url> bars_planner
# also place quadrotor_msgs in src if it is not installed
cd ..
catkin_make
source devel/setup.bash
```

## Launch

```bash
roslaunch bars_planner bars.launch
```

Point the launch at the topics produced by your state estimation stack:

```bash
roslaunch bars_planner bars.launch \
  cloud_topic:=/your_cloud \
  odom_topic:=/your_odometry \
  frame_id:=map
```

Set a goal with the RViz `Set Goal` tool or publish a `geometry_msgs/PoseStamped`
on the goal topic. Publish `std_msgs/Bool` with `data: true` on the cancel
topic to halt and clear the current path.

## Topics

### Subscribed

| Topic (default) | Type | Meaning |
|---|---|---|
| `pointcloud` | `sensor_msgs/PointCloud2` | World-frame obstacle cloud |
| `dynamic_objects` | `sensor_msgs/PointCloud2` | Dynamic-object points |
| `odometry` | `nav_msgs/Odometry` | Current pose |
| `goal` | `geometry_msgs/PoseStamped` | Navigation goal |
| `cancel` | `std_msgs/Bool` | Stop and clear the path |

### Published

| Topic (default) | Type | Meaning |
|---|---|---|
| `position_command` | `quadrotor_msgs/PositionCommand` | Position, velocity, acceleration and yaw setpoint |

The optional command bridge additionally publishes `command_pose`
(`geometry_msgs/PoseStamped`) and `command_twist` (`geometry_msgs/Twist`).

## Parameters

The full set of parameters and their onboard values is in
`config/bars_params.yaml`. The most commonly tuned entries are:

| Parameter | Meaning |
|---|---|
| `v_max`, `a_max` | Velocity and acceleration limits |
| `z_min`, `z_max`, `z_cruise` | Admissible altitude band |
| `r_infl` | Free-cell clearance threshold |
| `res_g` | Guide grid resolution |
| `cbf_r_safe`, `cbf_alpha` | Static safety radius and barrier gain |
| `ped_r_safe`, `ped_alpha` | Dynamic-object safety radius and gain |
| `lookahead`, `plan_int` | Path lookahead and replanning interval |
| `t_keep`, `map_voxel_size` | Map retention time and voxel size |
| `grid_ox`, `grid_oy`, `grid_sx`, `grid_sy` | Guide grid origin and extent |

## Adapting to a new platform

1. **Frames.** Provide the obstacle cloud and odometry in a single static
   world frame and set `frame_id` to match.
2. **Topics.** Remap `cloud_topic` and `odom_topic` to your state estimation
   outputs, or publish on the default names.
3. **Control interface.** If your controller does not accept
   `PositionCommand`, edit `src/command_bridge.cpp` at the marked section and
   forward the position, velocity, acceleration and yaw to your platform's
   message or API. The node already publishes standard `PoseStamped` and
   `Twist` messages as a working reference.
4. **Limits.** Set `v_max`, `a_max` and the altitude band to the verified
   envelope of the airframe, then tune the safety radii to its size.
5. **Dynamic objects.** Either run the included segmentation node or supply
   your own dynamic-object cloud on `dynamic_objects`.

## License

BSD 3-Clause. See [LICENSE](LICENSE).
