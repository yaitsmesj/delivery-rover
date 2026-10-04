# M6 capture checklist

Follow top to bottom. The order matches the video, so one session gets you
both the stills and the recording.

Save every screenshot into `docs/media/` with the **exact filename** given;
the README and the visual manual reference them by those names.

---

## Three reasons the rover was invisible in RViz

All three are fixed in the committed config. They are worth understanding,
because two of them are classic ROS traps that fail silently:

1. **There was no robot to see.** This project has no URDF, so nothing was ever
   drawn for the rover body — only a small TF triad. `sim.py` now publishes a
   `Marker` (an orange box with a nose arrow) in the `base_link` frame. It is
   published **once**, transient-local, and RViz moves it through TF every
   frame — so the body follows the robot for free.
2. **The laser was invisible because of QoS.** The saved config asked for
   `Reliable` on `/scan`, which publishes `Best Effort`. Incompatible QoS means
   DDS never connects the two — no error, no warning, silence. Incompatible QoS is
   silent by design.
3. **No display had a topic set.** Every `Value:` field was empty.

`rviz/rover.rviz` has been rewritten with all of it correct, plus a top-down
camera framed on the depot. Rebuild once and it just works.

---

## Step 0 · Setup (once)

```bash
cd ~/Developer/Production/delivery-rover

pixi shell
colcon build --symlink-install     # picks up the new marker + rviz config
source install/setup.zsh

python3 tests/make_feed.py          # test video, so the run is repeatable
mkdir -p docs/media
```

**Terminal layout.** You want three, side by side, plus RViz:

| | Runs |
|---|---|
| **T1** | the robot — `ros2 launch …` |
| **T2** | you — every inspection command |
| **T3** | the perception stub, for the recovery shot |

Every terminal needs `pixi shell` and `source install/setup.zsh` first.

---

## Step 1 · Start the robot and check it came up

**T1:**

```bash
ros2 launch delivery_rover bringup.launch.py
```

Wait for these two lines — they are your go/no-go:

```
[INFO] [vision-8]: process started with pid [...]        ← NINE processes
[lifecycle_manager]: Managed nodes are active            ← Nav2 is up
```

📸 **`01-launch.png`** — scroll to the top of T1 and frame all nine
`process started` lines, `[sim-1]` through `[vision-8]`. This one picture proves
the whole system starts on one command.

**T2:**

```bash
ros2 node list
```

📸 **`02-node-list.png`** — the full list. Yours are `/diff_drive_sim`,
`/robot_vision_brain`, `/mission`, `/mission_navigator`; the rest is Nav2.

```bash
ros2 node info /diff_drive_sim
```

📸 **`03-node-info.png`** — frame Subscribers + Publishers. This is the M1
contract, printed by the robot itself.

---

## Step 2 · RViz

**T2:**

```bash
rviz2 -d rviz/rover.rviz
```

Give it ~5 seconds. You should see, immediately:

- the depot floor plan in grey/black
- an **orange box with a black nose** at bottom-left — the rover at `home`
- an orange ring of laser points around it
- the TF triad and frame names
- the camera view in a panel

**If the map is there but the rover is not**, you did not rebuild — the marker
is new. `colcon build --symlink-install`, relaunch T1, reopen RViz.

**If nothing appears at all**, check the Displays panel on the left for a red
error, and check `Fixed Frame` says `map`.

Maximise the window before capturing.

📸 **`04-rviz-idle.png`** — **the hero image**, the README's first picture.
The whole depot, the rover at home, the laser ring, nothing selected. Spend a
minute framing it.

### Send a goal by hand

Click **2D Goal Pose** in the toolbar, then click-and-drag somewhere across the
depot — the drag direction sets the final heading. A green path appears and the
rover starts moving.

📸 **`05-rviz-plan.png`** — catch it mid-journey: green path ahead, rover
partway along, blue local costmap around it.

---

## Step 3 · The observation tools

All in **T2**, with the robot still running.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

📸 **`06-tf-echo.png`** — a few cycles of translation + rotation. Ctrl-C after.

```bash
ros2 topic hz /scan
# Ctrl-C, then:
ros2 topic info /scan --verbose
```

📸 **`07-topic-hz.png`** — both in one frame if you can: the ~10 Hz rate, and
the `BEST_EFFORT` QoS underneath. That QoS line is the thing that was breaking
your RViz.

```bash
ros2 run rqt_image_view rqt_image_view
```

Pick `/robot/camera/annotated` from the dropdown. Hold a bottle up.

📸 **`08-yolo.png`** — boxes and labels drawn on the live feed.

```bash
ros2 run rqt_graph rqt_graph
```

Set the dropdown to **Nodes/Topics (active)**, and untick **Debug** and
**Dead sinks**. Hit the refresh arrow.

📸 **`09-rqt-graph.png`** — the whole graph. This is the picture that shows
nine processes and the four wires between them.

---

## Step 4 · A full delivery — no camera, deterministic

Do not film a delivery through a live webcam. A missed frame or bad lighting
ruins the take, and you cannot repeat it. Swap perception for the stub: it
publishes the same `Detection2DArray` on the same topic, so **the mission node
cannot tell the difference** — the run is identical and it works every time.

**Capture `08-yolo.png` first** (Step 3) while the real camera is still
running, then stop T1 and restart it this way.

**T1 — the robot, with real perception switched off:**

```bash
ros2 launch delivery_rover bringup.launch.py perception:=false cargo:=bottle
```

Wait for `Managed nodes are active` and
`ready. call /start_mission … to deliver`.

**T3 — the stub, standing in for the vision node:**

```bash
python3 tests/fake_perception.py --label bottle
```

It prints a heartbeat every 2 s — that is how you know it is alive:

```
[INFO] [robot_vision_brain]: publishing 'bottle' at score 0.9 on
                             /robot/ai/detections at 5 Hz. Ctrl-C to stop.
[INFO] [robot_vision_brain]: t=  2.2s  publishing 1 detection(s)
```

`perception:=false` is **required** — the stub deliberately takes the same node
name as the real vision node, so running both puts two identically-named nodes
in the graph.

**T2 — watch the mission state. Leave this running; it is the screenshot:**

```bash
ros2 topic echo /mission/status
```

**T4 — start the delivery:**

```bash
ros2 service call /start_mission std_srvs/srv/Trigger '{}'
```

Watch RViz: the rover plans, drives to pickup, sees the bottle immediately,
and continues to dropoff. ~40 s end to end.

📸 **`10-mission-status-start.png`** — T2 as the run begins:

```
data: idle
data: running
data: 'running: go to pickup'
```

📸 **`10-mission-status-stop.png`** — T2 as it finishes:

```
data: 'running: go to dropoff'
data: 'running: SUCCESS'
data: SUCCESS
```

Two shots rather than one, because the trail is longer than a terminal window
once the tree has ticked for forty seconds. The pair reads better in the
write-up anyway: one frame showing the mission accepted and underway, one
showing it finished.

📸 **`10b-rviz-success.png`** — RViz at the moment it stops at dropoff,
with the completed path still drawn.

> **Only have three terminals?** Launch with `auto_start:=true` and drop T4 —
> the mission runs as soon as Nav2 is ready, and the status trail is identical.
> You lose only the service-call shot, which you can take separately later.

---

## Step 5 · The checks

Keep T1 and the stub in T3 running — `check_mission.py` drives a second
delivery, and it needs something publishing detections.

**T4:**

```bash
python3 tests/check_base.py
python3 tests/check_nav.py
python3 tests/check_mission.py
```

📸 **`11-checks.png`** — the PASS lines and `0 failures` for at least two.

Then stop T1 (Ctrl-C) and run:

```bash
colcon test && colcon test-result --verbose
```

📸 **`12-colcon-test.png`** — the `0 errors, 0 failures` summary.

---

## Step 6 · The two-publishers experiment

**T1:**

```bash
ros2 launch delivery_rover bringup.launch.py mission:=false
```

**T2 — run it twice, because the two results together are the lesson:**

```bash
python3 tests/experiment_two_publishers.py --rate 10   # slower than Nav2
python3 tests/experiment_two_publishers.py             # 40 Hz, faster
```

A topic has **no arbitration**. Every message is delivered and the robot acts
on whichever arrived last — so the winner is decided by **publish rate**, not
by priority. Nav2's controller publishes at 20 Hz:

| Interferer | vs Nav2 | What you see |
|---|---|---|
| `--rate 10` | loses 1:2 | the rover wobbles badly, loses time, usually still arrives |
| default 40 Hz | wins 2:1 | the rover is driven off its path; the goal usually fails |

📸 **`13-two-publishers-slow.png`** — the 10 Hz run. Frame
`SECOND PUBLISHER ON` and the pose column during the interference: the
heading jumping around (71° → 89° → 80° → 47°) while `remaining` still falls.

📸 **`13-two-publishers-fast.png`** — the 40 Hz run. Frame the same moment
plus the final `result:` line.

**The 10 Hz run is the more valuable screenshot**, and it is worth knowing
why: "it still arrived" is exactly how this bug survives code review on a
real robot. Nothing errored, nothing warned, the goal succeeded — and the
robot was being driven by two minds at once the whole way.

---

## Step 7 · The 60-second video

Record with **Cmd-Shift-5 → Record Selected Portion**. Frame RViz large with T1
tucked into a corner. One take, no narration.

**Set up the recovery shot first** — it is the whole point of the video, and the
deterministic version is easier to film than juggling a real bottle:

```bash
# T3 — the stub. Start this FIRST, before the launch.
python3 tests/fake_perception.py --label bottle --delay 36

# T2 — the status trail, tucked into a corner of the frame
ros2 topic echo /mission/status

# T1 — the robot. auto_start delivers as soon as Nav2 is ready.
ros2 launch delivery_rover bringup.launch.py \
  perception:=false cargo:=bottle auto_start:=true
```

**`auto_start:=true` is what makes the rover move.** Without it nothing calls
`/start_mission`, so the behavior tree sits in `idle` forever — that is why it
stood still. Step 4 had the call in a fourth terminal; for filming you do not
want to reach across the screen mid-take, so let the launch do it.

Order matters: start the stub **before** T1. Its 36-second clock begins when it
starts, and you want the "no bottle" window to still be open when the rover
arrives at pickup.

The stub reports "no bottle" for 36 seconds, which is long enough for the rover
to reach pickup, fail, and begin its look-around — then the bottle "appears"
and it recovers. Identical behaviour to the live-camera version.

| Seconds | On screen |
|---|---|
| 0–8 | `ros2 launch …` — eight processes, lifecycle walking to ACTIVE |
| 8–16 | RViz: depot, rover at home, laser ring, TF |
| 16–26 | The plan appears on its own and the rover arcs around the shelving |
| 26–40 | **The money shot.** Reaches pickup, scans, announces failure, turns in place, looks again — recovers |
| 40–52 | Drives to dropoff and stops; `/mission/status` reaches `SUCCESS` |
| 52–60 | `check_mission.py` printing `0 failures` |

> **Want the service call on camera instead?** Drop `auto_start:=true`, add a
> fourth terminal, and run
> `ros2 service call /start_mission std_srvs/srv/Trigger '{}'` when you are
> ready. Same run, one more window to manage.

Save as `docs/media/mission.mp4`, then make the README GIF:

```bash
ffmpeg -i docs/media/mission.mp4 -vf "fps=10,scale=720:-1" -t 20 docs/media/mission.gif
```

---

## When you're done

Every image referenced by the README lives in `docs/media/`, and every one of
them is reproducible by following the steps above against a clean checkout.
That is the point of this file: the screenshots are evidence, not decoration,
and anyone can regenerate them.

```bash
git add -A
git commit -m "docs: capture evidence for the README"
git push
```
