# Synthetic Nav2 controller tests

`test_controller.cpp` uses Jazzy's actual `nav2_core::Controller` interface and
pluginlib package export. Its in-memory `Costmap2DROS` subclass provides a real
`LayeredCostmap`, occupancy costs and a front-heavy convex footprint without
starting sensor subscriptions, a controller server, or hardware interfaces.

Coverage includes plugin discovery/loading, lifecycle reuse, dependency and
parameter validation, invalid/new paths, TF failure and success, command frame,
GoalChecker tolerance handling, absolute/percentage speed limiting, unknown and
interior footprint collisions, infeasible stopping, map bounds, forward/reverse
and curved kinematic loops, cancellation braking and reset.

The kinematic plant integrates returned constant Twist commands independently
of `Optimizer::advance()`. This is deterministic software regression coverage;
it is not evidence of sensor, TF latency, actuator, physical braking or robot
safety validation. Core delayed-plant scenarios are run separately.

Run in a sourced Ubuntu 24.04 / ROS 2 Jazzy / Nav2 workspace:

```bash
colcon build --paths . --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --paths . --event-handlers console_direct+
colcon test-result --verbose
```

`test_prepare_release.py` is a standard-library Python unittest registered
with CTest. It uses synthetic VCS metadata to check both source layouts,
metadata/cache exclusion, core provenance, source preservation and output
protection. It can also run without ROS:

```bash
python3 test/test_prepare_release.py -v
```

For an installed-package plugin check, source only that workspace's install
setup and run `ros2 run igo_nav2_controller igo_nav2_plugin_load_check`.
Do not claim the tests passed merely because this source compiles or the
ROS-independent core scenarios pass.

Official Jazzy API references used when authoring this fixture:

- https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_core/include/nav2_core/controller.hpp
- https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_core/include/nav2_core/goal_checker.hpp
- https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_core/include/nav2_core/controller_exceptions.hpp
- https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d_ros.hpp
