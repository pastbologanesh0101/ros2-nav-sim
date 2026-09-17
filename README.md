# ROS2 Navigation Project (ros2-nav-sim)

> **IMPORTANT — READ THIS FIRST**
>
> **This is a from-scratch ROS2-CONCEPT SIMULATION, written in pure Python
> standard library, for demonstrating navigation algorithms and pub/sub
> node architecture. It does NOT run on real ROS2 / rclpy / DDS, and no
> ROS2 install is required or used anywhere in this repo.** It re-implements
> the *shape* of ROS2's node/topic/publisher/subscriber/executor API closely
> enough to be recognizable and educational, and builds a genuine navigation
> stack (costmap, A* planner, pure-pursuit controller, differential-drive
> robot model) on top of that simulated middleware. See
> [Porting to real ROS2 / rclpy](#porting-to-real-ros2--rclpy) below for
> exactly what would change to run this on an actual ROS2 install.

## Why simulate instead of using real ROS2

ROS2 is a large, OS-specific install (a full DDS middleware stack, colcon
build tooling, a specific supported OS/Python combination, etc.) that isn't
practical to install in a lot of environments — including the one this
project was built in. Rather than fake it or silently skip the
implementation, this project honestly reimplements the *architectural
pattern* ROS2 is built on — nodes that communicate only through named
topics, typed messages, publishers, subscribers, and an executor that drives
callbacks — in plain Python, and then builds a real, working, testable
navigation stack on top of it. The algorithms (A*, pure pursuit, costmap,
differential-drive kinematics) are the genuine article; only the
"middleware" layer underneath them is simulated.

## Architecture

```
                         +-------------------+
                         |   TopicRegistry    |   (shared "DDS": name -> Topic)
                         +----------+---------+
                                    |
        +----------------+---------+---------+----------------+
        |                |                   |                |
   creates topic     creates topic       creates topic    creates topic
     "/path"           "/odom"            "/cmd_vel"       (all of them)
        |                |                   |                |
+---------------+  +-------------+     +---------------+  +-------------+
| PlannerNode   |  | RobotNode   |      | ControllerNode|  |  (any node) |
|---------------|  |-------------|     |---------------|  +-------------+
| costmap +     |  | integrates  |      | subscribes:   |
| A* search     |  | cmd_vel ->  |      |  /path, /odom |
| -------       |  | diff-drive  |      | publishes:    |
| publishes:    |  | kinematics  |      |  /cmd_vel     |
|   /path  ---->+->| subscribes: |<-----+  (pure        |
|  (once, at    |  |  /cmd_vel   |      |   pursuit)    |
|   startup)    |  | publishes:  +----->|               |
+---------------+  |  /odom      |      +---------------+
                    +-------------+
                            ^
                            |
                     Executor.spin(max_iterations)
              (drives ControllerNode + RobotNode timers,
               once per iteration, closing the control loop)
```

Data flow, once per spin iteration:

1. **ControllerNode** reads the latest `/odom` pose and the (once-published)
   `/path`, picks the next look-ahead waypoint, runs the pure-pursuit control
   law, and publishes a `/cmd_vel` (linear, angular) command.
2. **RobotNode** receives that `/cmd_vel`, integrates differential-drive
   kinematics forward by one timestep, and publishes its new pose on
   `/odom`.
3. Repeat until the controller reports the goal has been reached (within
   tolerance) or the iteration budget runs out.

### Layers, mapped onto real ROS2 concepts

| This project (`ros2_nav_sim/core.py`) | Real ROS2 equivalent |
|---|---|
| `Node` | `rclpy.node.Node` |
| `Topic` / `TopicRegistry` | a DDS topic, discovered via `rmw`/DDS discovery |
| `Node.create_publisher()` | `rclpy` `Node.create_publisher()` |
| `Node.create_subscription()` | `rclpy` `Node.create_subscription()` |
| `Node.create_timer()` (fires once per spin iteration) | `rclpy` `Node.create_timer(period_sec, cb)` (fires on wall-clock period) |
| `Executor` / `Executor.spin(max_iterations)` | `rclpy.executors.SingleThreadedExecutor` / `rclpy.spin()` |
| `messages.py` dataclasses (`Pose2D`, `Twist`, `Path`, `OccupancyGrid`) | generated message classes from `geometry_msgs`, `nav_msgs` |
| synchronous, in-process `Topic.publish()` fan-out | asynchronous DDS transport (pub/sub over the network, with QoS, serialization, etc.) |
| topic "latching" (a late subscriber gets the last message) | ROS2 `TRANSIENT_LOCAL` durability QoS |

### Navigation stack, on top of the simulated middleware

- **`costmap.py`** — a 2D occupancy grid (`Costmap`) built from a small
  bundled ASCII map (`BUNDLED_MAP_LINES`): `.` = free, `#` = occupied. Mirrors
  `nav_msgs/OccupancyGrid`.
- **`planner.py`** — grid A* search (4-connected, Manhattan heuristic) over
  the costmap. `PlannerNode` runs it once at startup and publishes the
  result on `/path` (mirrors `nav_msgs/Path`).
- **`controller.py`** — `compute_control()` is a standalone pure-pursuit-style
  control law: given the robot's pose and one look-ahead target point, it
  returns `(linear, angular)`. `PurePursuitControllerNode` wraps it,
  advancing a waypoint index along `/path` as the robot gets close to each
  one, and stops the robot once it's within `goal_tolerance` of the final
  waypoint.
- **`robot.py`** — `integrate_pose()` is a standalone differential-drive /
  unicycle kinematics integrator (forward Euler). `DiffDriveRobotNode` wraps
  it: subscribes to `/cmd_vel`, integrates every spin iteration, publishes
  `/odom`.
- **`demo.py`** — wires a `PlannerNode`, `PurePursuitControllerNode`, and
  `DiffDriveRobotNode` together on the bundled map's default start/goal
  cells, runs `Executor.spin()` for up to `max_iterations` steps, and reports
  whether the robot reached the goal.

## Running the demo

No dependencies beyond the Python standard library (3.11+):

```bash
cd ros2-nav-sim
python3 -m ros2_nav_sim.demo
```

Example output:

```
ROS2-concept navigation simulation
===================================
Start cell: (1, 1)   Goal cell: (8, 8)
A* path (15 cells): [(1, 1), (2, 1), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (7, 4), (8, 4), (8, 5), (8, 6), (8, 7), (8, 8)]
Goal (world coords): (8.50, 8.50)
Final pose: x=8.24, y=8.50, theta=-0.00 rad
Iterations run: 230 / 1000
Final distance to goal: 0.259 m
Result: GOAL REACHED
```

## Running the tests

```bash
python3 -m unittest discover -s tests -v
```

36 unit tests cover: pub/sub delivery (including that a subscriber on a
different topic doesn't receive a message, and that the executor calls
registered timer callbacks exactly the requested number of times), A*
correctness on a small crafted grid with a known shortest path, A*'s
no-path-found case on a walled-off goal, costmap occupied/free cell parsing
from the bundled map, the pure-pursuit controller's steering sign/direction
on crafted geometric cases, the differential-drive integrator against known
inputs, and the full end-to-end demo reaching the goal within its iteration
budget. CI (`.github/workflows/tests.yml`) runs the same suite on push/PR
against Python 3.11 and 3.12.

## The bundled map

A 10x10 grid (`ros2_nav_sim/costmap.py: BUNDLED_MAP_LINES`) with two partial
walls, each with a single gap, forcing the planner to weave rather than walk
a straight line:

```
..........
..........
..........
####.#####
..........
..........
..........
.....####.
..........
..........
```

Default start cell `(1, 1)`, default goal cell `(8, 8)` (row, col).

## Porting to real ROS2 / rclpy

This simulation was deliberately built so the *navigation algorithms* are
directly reusable, and the *node/topic layer* is the only part that would
need to change. To port this onto a real ROS2 install:

1. **Messages**: swap the plain dataclasses in `messages.py` for real
   message types — `geometry_msgs/msg/Pose2D`, `geometry_msgs/msg/Twist`,
   `nav_msgs/msg/Path` (using `geometry_msgs/msg/PoseStamped` per pose
   instead of a raw `(x, y)` tuple), and `nav_msgs/msg/OccupancyGrid`.
2. **Node base class**: change each `Node` subclass here
   (`PlannerNode`, `PurePursuitControllerNode`, `DiffDriveRobotNode`) to
   inherit from `rclpy.node.Node` instead of `ros2_nav_sim.core.Node`, and
   call `rclpy.init()` / `rclpy.shutdown()` around them.
3. **Publishers/subscriptions**: `self.create_publisher(MsgType, topic)` and
   `self.create_subscription(MsgType, topic, callback)` already match
   `rclpy`'s call signatures (minus the QoS profile argument `rclpy`
   requires) — this is close to a drop-in rename once the message types are
   real.
4. **Timers**: replace `Node.create_timer(period, cb)` (called once per
   simulated spin iteration here) with `rclpy`'s real
   `self.create_timer(period_sec, cb)`, which fires on a wall-clock period
   instead of a discrete simulation step.
5. **Executor**: replace `ros2_nav_sim.core.Executor` / `spin()` with
   `rclpy.executors.SingleThreadedExecutor` (or `MultiThreadedExecutor`) and
   `rclpy.spin()`, and run each node in its own process (the normal ROS2
   deployment model) rather than in one Python process.
6. **Topic latching**: replace this project's "always replay the last
   message to a late subscriber" behavior with an explicit
   `QoSDurabilityPolicy.TRANSIENT_LOCAL` QoS profile on the `/path`
   publisher/subscription, since real ROS2 topics don't latch by default.
7. **Costmap source**: instead of a bundled ASCII map, subscribe to a real
   `nav_msgs/OccupancyGrid` published by a SLAM/mapping stack (e.g.
   `slam_toolbox` or `nav2`'s costmap layers) instead of building it from a
   hardcoded string.
8. **Robot model**: replace `DiffDriveRobotNode`'s kinematics integrator with
   a real robot driver node (or a simulator like Gazebo/Ignition) publishing
   real `/odom` from wheel encoders or a simulated physics engine.

None of `costmap.py`'s A* search, `controller.py`'s `compute_control()`, or
`robot.py`'s `integrate_pose()` would need to change at all — they only
depend on plain data (grids, poses, floats), not on the node/topic layer.

## Project layout

```
ros2-nav-sim/
├── ros2_nav_sim/
│   ├── core.py         # Node / Topic / TopicRegistry / Executor / spin()
│   ├── messages.py     # Pose2D, Twist, Path, OccupancyGrid dataclasses
│   ├── costmap.py      # Costmap + bundled ASCII map
│   ├── planner.py      # astar() + PlannerNode
│   ├── controller.py   # compute_control() + PurePursuitControllerNode
│   ├── robot.py        # integrate_pose() + DiffDriveRobotNode
│   └── demo.py         # wires everything together, run_demo() / main()
├── tests/               # 36 unittest cases
├── .github/workflows/tests.yml
├── LICENSE (MIT)
└── requirements.txt     # empty on purpose: stdlib only
```

## License

MIT — see [LICENSE](LICENSE).
