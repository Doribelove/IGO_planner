# IGO Ackermann controller for Nav2 Jazzy

`igo_nav2_controller::IGOController` is a CPU `nav2_core::Controller` plugin for
Ubuntu24.04 / ROS2 Jazzy. It adapts the unchanged IGO optimizer from
Doribelove/IGO_planner commit `ac87ef450bde039564b2d555cc27f53b7c5c7221`.
This is the Ackermann local controller, not a global or coverage planner.

This self-contained ROS2 source layout is prepared for the `ros2-jazzy` branch
in the existing repository. The main branch retains the original ROS1 Noetic
package. ROS2 source publication is separate from an official ROS distribution
release, and does not imply ROS/Nav2 maintainer acceptance.

## Reproduce from this self-contained source root

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro jazzy -y
colcon build --paths . --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --paths . --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
ros2 run igo_nav2_controller igo_nav2_plugin_load_check
python3 scripts/controller_server_smoke.py --domain 198 --output /tmp/igo-smoke-new
```

The smoke test must use an unused ROS domain that is different from any live
robot domain. The output directory must be new or empty. It launches only a
synthetic plant, TF/odometry, sensor-free costmap and the real ControllerServer.
It does not launch a chassis driver. Localhost discovery is not absolute
network isolation.

## Integration contract

Merge `config/controller_server.yaml` into the robot's existing Nav2 settings;
it is a controller fragment, not complete robot bringup. Preserve localization,
planner, costmaps and the real chassis configuration.

- Output angular.z is yaw rate, not steering angle; no in-place rotation.
- Pose, footprint, base frame and Twist must share the Ackermann reference
  point, normally the rear-axle center. No lever-arm correction is performed.
- The server-supplied body-frame Twist is the velocity feedback; the plugin
  does not subscribe to a second odometry stream.
- Only finite XY/yaw projection is used, not 3D clearance or terrain planning.
- A padded, ordered convex footprint is captured during configure; runtime
  footprint/radius/padding changes require cleanup/configure. Startup read-only
  parameter values require restarting the node with new overrides.
- Standard Nav2 GoalChecker owns goal completion; its tolerances guide braking.
  Use a suitable stopped-goal checker and preserve low-speed odometry feedback.
- Jazzy zero speed-limit sentinel restores original limits; it does not command
  a stop. Lower limits are applied before collision-checked optimization and
  cannot remove physical momentum instantaneously.
- Cancel uses a checked braking rollout. Invalid paths/TF/unsafe controls raise
  specific Nav2 exceptions. There is no unvalidated emergency command fallback.
- The core models a local static costmap snapshot, not future obstacle motion.
  Actual emergency stop and certified safety remain the robot system's job.

## Evidence and limits

Clean real Jazzy builds, install-only pluginlib loading, 56 plugin gtests,
original core scenarios, rosdep resolution and real ControllerServer synthetic
Forward/Reverse/cancel actions passed. Both the development layout and a clean
self-contained source build were checked. Full results and limits are in
`docs/VALIDATION.md` and `test/results/2026-11-10/`.

These are software/synthetic tests, not new-wrapper hardware validation,
ROS certification, official binary distribution or Nav2 maintainer acceptance.
ARM64, other ROS distros, actuator/brake behavior and long-duration tests remain
unverified. The locally prepared GitHub workflow has not run on GitHub.

The existing public issue tracker is
https://github.com/Doribelove/IGO_planner/issues . Repository ownership and
maintainer contact/release decisions remain with the owner. No new long-term
support commitment is made by this candidate.

License: BSD-3-Clause, retained in LICENSE. Core provenance hashes are recorded
in CORE_PROVENANCE.json. See docs/RELEASE_CHECKLIST.md for the distinct source,
ROS Index, apt binary and optional Nav2-upstream steps.
