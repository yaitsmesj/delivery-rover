# Third-party notices

The MIT licence in `LICENSE` covers **only the source written in this
repository** — the nodes, launch files, configuration and check scripts.
It does not extend to the dependencies listed below, each of which carries its
own licence.

Nothing from these projects is vendored here. They are resolved at build time
by `pixi` from the manifest in `pixi.toml`, and the trained model weights
(`*.pt`) are downloaded on first run and are **not** committed.

| Dependency | Licence | Used for |
|---|---|---|
| **Ultralytics YOLO** (`ultralytics`) | **AGPL-3.0** | Object detection in `vision.py` |
| ROS 2 Humble | Apache-2.0 | Middleware, `rclpy`, `tf2`, message packages |
| Navigation2 (Nav2) | Apache-2.0 / BSD-3-Clause | Planner, MPPI controller, recovery behaviours, `nav2_simple_commander` |
| py_trees / py_trees_ros | BSD-3-Clause | Behaviour tree in `mission.py` |
| PyTorch, torchvision | BSD-3-Clause | Inference backend (Metal/MPS on Apple Silicon) |
| OpenCV (`opencv-python`) | Apache-2.0 | Camera capture and annotation |
| NumPy | BSD-3-Clause | Array maths throughout |
| PyYAML | MIT | Map and parameter files |

## A note on Ultralytics and AGPL-3.0

`ultralytics` is distributed under the **GNU Affero General Public License
v3.0**, which is a strong copyleft licence. Ultralytics' own published position
is that software built on their package falls under AGPL-3.0 unless a separate
commercial licence is obtained from them.

This repository is a personal learning project, published for reading rather
than for deployment, and its own source is offered under MIT. **Anyone intending
to use this code in a product should read the Ultralytics licence terms
directly and take their own legal advice** — the combination of an MIT project
with an AGPL dependency is a question this file flags rather than settles, and
nothing here is legal advice.

The detector is isolated behind one node (`vision.py`) and one message type
(`vision_msgs/Detection2DArray`). Swapping YOLO for a permissively licensed
detector would not require changes anywhere else in the stack — which was a
design goal, and is the practical answer if this ever needed to ship.
