# Intelligent Delivery Rover

An autonomous warehouse delivery robot in ROS 2 Humble. It drives itself across a
depot, uses a camera to confirm the cargo is where it should be, and recovers on
its own when it isn't.

Nine processes, one launch command, one service call.

```bash
ros2 launch delivery_rover bringup.launch.py
ros2 service call /start_mission std_srvs/srv/Trigger '{}'
```

<!-- TODO: drop the 60-second clip here -->
<!-- ![the rover delivering](docs/media/mission.gif) -->

---

## What it does

The rover is given one instruction — *deliver* — and works out the rest:

1. Plans a route from **home** to the **pickup** bay, around the shelving.
2. Looks for the cargo with a camera running YOLO.
3. If it sees the cargo, carries on to the **dropoff** point.
4. If it doesn't, it says so, turns in place, and looks again.
5. If it still can't see it, it fails cleanly and reports why — it never hangs.

Throughout, it publishes what it is doing on `/mission/status`, so anything
— a dashboard, a fleet manager, a test script — can follow along without
reaching inside the robot.

## Architecture

```
          /start_mission (std_srvs/Trigger)
                     │
             ┌───────▼────────┐   the only component that reads perception
             │   /mission     │   AND commands navigation — a py_trees
             │  behavior tree │   behavior tree ticking at 2 Hz
             └───┬────────┬───┘
   detections    │        │   NavigateToPose action
 ┌───────────────▼─┐   ┌──▼──────────────────────────┐
 │ /robot_vision_  │   │ Nav2 — 6 lifecycle processes│
 │     brain       │   │ map · planner · controller  │
 │ camera → YOLO   │   │ behavior · BT · lifecycle   │
 └─────────────────┘   └──────────────┬──────────────┘
                                      │ /cmd_vel @ 20 Hz
                        ┌─────────────▼──────────────┐
                        │     /diff_drive_sim        │
                        │ the base and lidar, stood  │
                        │ in for by a kinematic sim  │
                        └──────────────┬─────────────┘
                                       │ /odom · /scan · TF
                                       └──► back to Nav2
```

**Perception and navigation never talk to each other.** YOLO publishes labels;
Nav2 consumes coordinates. The behavior tree is the only bridge — which is where
all of the decision-making in this project actually lives.

**The robot is coupled to its hardware by exactly four names:** `/cmd_vel`,
`/odom`, `/scan`, and TF. Replace the simulator with a real base publishing
those four, and nothing else in the stack changes.

| Component | What it is | Written here? |
|---|---|---|
| `/diff_drive_sim` | Kinematic diff-drive base + ray-cast lidar | Yes |
| `/robot_vision_brain` | Camera → YOLO11n → `Detection2DArray` | Yes |
| `/mission` | py_trees behavior tree + `/start_mission` service | Yes |
| Nav2 | NavFn planner, MPPI controller, recovery behaviours | Configured |
| YOLO, py_trees, tf2 | Libraries | Used |

## Stack

ROS 2 Humble · Nav2 1.1.20 (MPPI) · Ultralytics YOLO11n · py_trees 2.4 ·
Python 3.12 · Pixi/RoboStack · macOS (Apple Silicon) and Linux

## Quickstart

```bash
git clone https://github.com/<you>/delivery-rover.git
cd delivery-rover
pixi install && pixi shell

colcon build --symlink-install
source install/setup.zsh          # setup.bash on Linux

ros2 run delivery_rover make_map src/delivery_rover/config/map   # once
colcon build --symlink-install

ros2 launch delivery_rover bringup.launch.py
```

In a second terminal:

```bash
ros2 service call /start_mission std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
```

No webcam? Build a test feed from the sample images that ship with
`ultralytics`, and point the vision node at it:

```bash
python3 tests/make_feed.py
ros2 launch delivery_rover bringup.launch.py source:=assets/feed.mp4 cargo:=person
```

### Launch arguments

| Argument | Default | Use |
|---|---|---|
| `perception` | `true` | `false` to run navigation alone |
| `mission` | `true` | `false` to drive it yourself |
| `cargo` | `bottle` | any COCO class the camera can see |
| `source` | `0` | camera index, or a path to a video file |
| `auto_start` | `false` | `true` to deliver as soon as Nav2 is ready |
| `viz` | `false` | `true` starts rosbridge for Foxglove |

## Tests

Unit tests for pure logic, integration checks against a running system — the
same split used in industry.

```bash
colcon test && colcon test-result --verbose    # lint + unit tests
```

With the stack already running, each check subscribes like any other ROS client
— no privileged access, so anything it can verify, a reviewer can too:

| Check | Verifies |
|---|---|
| `tests/check_base.py` | TF chain, scan geometry, odometry frames |
| `tests/check_nav.py` | Nav2 reaches ready, then drives both legs |
| `tests/check_perception.py` | Detections arrive, images valid, staleness expires |
| `tests/check_mission.py` | Full delivery end to end, with the status trail |

Two scripts make the awkward cases repeatable:

```bash
# cargo never appears → clean failure, not a hang
python3 tests/fake_perception.py --label ""

# cargo revealed mid-recovery → the rover turns, looks again, and delivers
python3 tests/fake_perception.py --label bottle --delay 36

# what happens when two things command /cmd_vel at once (spoiler: it fails)
python3 tests/experiment_two_publishers.py
```

## Design decisions worth defending

**A kinematic simulator, not a physics engine.** Nav2 consumes topics, not
physics. Modelling mass and friction would have cost days and changed nothing
about what the stack had to get right.

**`ACTIVE` is not the same as ready.** Every Nav2 server reporting ACTIVE still
isn't enough — a goal sent before the costmaps have digested their first scan
dies with *"plan has 0 poses"*. `navigator.py` waits for both costmap topics as
well. This is the single most useful line in the file.

**Inference runs on a worker thread, and catches everything.** The ROS timer
grabs a frame and returns immediately; a worker runs YOLO. An unhandled
exception there would kill the thread and leave the node *alive but
permanently silent* — the worst failure mode in the system — so it logs and
carries on.

**Detections expire.** `max_age` is what makes "I saw it" mean "I see it".
Without it a crashed vision node leaves its last message in the mission's memory
forever, and the robot keeps believing it can see cargo that isn't there.

**One commander on `/cmd_vel`.** Topics are many-to-many with no arbitration.
Nav2 already has four publishers on that topic — the controller plus three
recovery behaviours — and it is safe only because its behavior tree guarantees
one is active at a time. Arbitration by sequencing, not by transport.

## MPPI tuning

Starting from the Nav2 reference config, the settings that mattered:

| Parameter | Value | Why |
|---|---|---|
| `batch_size` | 2000 | Trajectories sampled per cycle. Halve it on a weak CPU — below ~1000 the paths get visibly worse |
| `time_steps` × `model_dt` | 56 × 0.05 s | A 2.8 s lookahead — enough to commit to going around a shelf rather than stopping at it |
| `vx_max` / `wz_max` | 0.5 m/s · 1.9 rad/s | Warehouse-appropriate; higher and the arcs clip shelving |
| `PathAlignCritic` | 14.0 | Highest weight in the set. Lower it and the rover cuts corners into the inflation layer |
| `PreferForwardCritic` | 5.0 | Stops it reversing out of situations a turn would solve |
| `temperature` | 0.3 | Lower is greedier. Raising it made the path wander without avoiding anything new |

*"Control loop missed its desired rate"* means the CPU lost the 20 Hz loop, not
that the configuration is wrong. Reduce `batch_size` first.

## Repository layout

```
src/delivery_rover/        the robot — everything here is installed
  delivery_rover/          nodes and their libraries
  launch/                  robot · navigation · perception · bringup
  config/                  nav2_params.yaml + the generated map
  test/                    lint + unit tests
tests/                     check scripts — run by hand, never installed
rviz/                      the RViz layout
```

The split is deliberate: **if it would not be installed on the robot, it does
not belong in the robot's package.** The check scripts talk to the running
system the same way any other ROS client would.

## Known limits

- The base is simulated. The swap to hardware is four topics wide, but it has
  not been done.
- `map → odom` is published as identity. A real robot needs AMCL or SLAM here;
  the consumers do not change.
- Detection gives presence, not position. Acting on *where* the cargo is would
  need depth projection into `base_link`.
- Single robot, single mission at a time.

## License

MIT
