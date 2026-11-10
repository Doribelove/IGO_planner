# Nav2 Jazzy validation, 2026-11-10

This report concerns the new local ROS2 adaptation only. The root ROS1
`docs/VALIDATION.md` is historical upstream evidence, not a fresh Noetic or
hardware test of this wrapper. No real robot was commanded.

## Environment

- Host: cloud Debian 13 x86_64, user-namespace Ubuntu 24.04.5 rootfs.
- Official `ros:jazzy-ros-base` amd64 OCI manifest:
  `sha256:c26c86a28e1addf9d10a81232869791974a3f7907943d5634742b58b2f26e1a3`.
- Nav2 core/costmap/controller 1.3.13, rclcpp 28.1.22, pluginlib 5.4.6,
  CMake 3.28.3, colcon-core 0.21.3. Exact installed deb versions are in the
  accompanying result metadata.
- Rootfs, package/build caches and logs stay in the workspace, about 1.5 GiB.
  Nothing was installed into the user's home or host system.
- Separate user/PID/IPC/UTS namespaces, synthetic /dev, no chassis devices.
  A fresh network namespace could not bring up loopback on this kernel;
  localhost ROS discovery, domain 197 for unit tests and 198 for integration
  were used instead. This is not an absolute network isolation claim.
- Real same-process and cross-process ROS2 DDS publish/subscribe passed.
  Kernel getifaddrs/NETLINK_ROUTE warnings did not prevent these checks.

## Final passed checks

- Inspected upstream CPU core compiled with host g++/C++17 and passed all
  eight printed closed-loop/delayed-stop scenarios plus inline safety asserts.
- Real Jazzy colcon Release build and install passed, no compiler warnings
  in the final build.
- 56/56 gtests passed against actual Nav2/pluginlib libraries, including
  lifecycle reuse, TF, path/parameter validation, full-footprint collision,
  unknown cells, bounds/giant-coordinate protection, goal tolerances, speed
  limits, cancellation and three independent-Twist kinematic loops.
- Final aggregate colcon/CTest passed 3/3 CTest entries: core scenarios,
  a standalone plugin loader, and the gtest suite. colcon reported 59 results,
  zero failures/errors/skips; its total includes the suite bookkeeping, not
  59 separately designed unit cases.
- Install-space plugin discovery and creation passed without linking the
  check executable directly against the IGO library. ldd found no unresolved
  installed shared-library dependencies.
- rosdep check reported all declared system dependencies satisfied, including
  the real ControllerServer smoke-test dependencies. Rootfs TLS initially did
  not trust the cloud proxy; metadata was downloaded through the host's existing
  verified TLS path into a workspace cache. No TLS checks were disabled and
  no certificate/security settings were changed.

## Real ControllerServer synthetic integration

The real Jazzy `nav2_controller/controller_server` loaded two installed IGO
instances (fixed Forward and Reverse), completed configure/activate, and ran
FollowPath actions on a sensor-free 12x12m rolling costmap. A separate rclpy
plant integrated emitted Twist commands, publishing TF and odometry.

- Forward 1.8m: SUCCEEDED, error_code 0, position error 0.1435m, 4.52s.
- Reverse 1.0m: SUCCEEDED, error_code 0, position error 0.1291m, 5.72s;
  reverse commands reached -0.3m/s.
- Cancellation requested while moving at 0.5m/s: CANCELED, error_code 0;
  a zero command was observed after 0.296s and final simulated speed was zero.
- 110 emitted commands, final lifecycle deactivate/cleanup passed, server exit 0.
- Installed library SHA256 for this run:
  `30235f2f855ee38b3e4ccf7ab111b788f1b178af0ce9f9007ce9b83753f77308`.

This uses an ideal command-following plant, not real actuators, sensor latency,
slip, steering feedback or physical brakes. No obstacle-avoidance claim follows
from the empty-map server smoke. Separate core/gtest synthetic cases cover
software footprint/obstacle behavior, not dynamic-obstacle prediction.

## Standalone source

The generated self-contained source was built from scratch in a separate build
and install prefix, with no source dependency on the original ROS1 checkout.
Release build/install, 3/3 CTest entries, 56/56 gtests and install-space plugin
loading all passed. The delivered script was then run against that independent
installed library: Forward and Reverse both SUCCEEDED, moving cancel CANCELED,
113 emitted commands, server exit 0. Cancellation observed zero after 0.297s.
Its binary SHA256 was
`c09a6e15f83cf93b1b6935d20b81b0f3cddf4f7babe082db311bc97831a75114`.
Build-path-sensitive logging/metadata can change binary hashes between builds;
core source hashes are recorded in CORE_PROVENANCE.json. Only this validation
paragraph was updated after the build; implementation/test/script bytes stayed
unchanged. This source is prepared locally, not yet on an authorized ROS2
release branch or public distribution.

## Corrections found during validation

The first CMake configure exposed mixed plain/keyword target_link_libraries
signatures with Jazzy ament; it was corrected. Two initial test fixtures used
ROS2's valid default quaternion (w=1) while intending an invalid all-zero
quaternion; they were corrected explicitly. Final tests above are from the
corrected final C++ source, including finite/map-bounds checks before the
unchanged core's integer grid conversion. Earlier failing logs are not hidden
as passes.

## Not verified or performed

- No fresh ROS1/Noetic build in this environment; original ROS1 source/build
  files and core were left unchanged except for a README pointer to ROS2.
- No ARM64 build, other ROS2 distro, real robot, actuator/brake characterization,
  long-duration endurance, moving-obstacle prediction or certified safety test.
- At the time of the local validation above, no remote commit/push, PR, tag,
  publication or new legal agreement had been performed. Any subsequent source
  publication is a separate step and does not change these test limits.
- The CI workflow is defined locally but has not run on GitHub Actions.
- No ROS Index/apt release or Nav2-maintainer acceptance. The official Jazzy
  rosdistro distribution file checked on this date contained no IGO entry or
  Doribelove/IGO_planner source URL (blob
  `7eb484de5dcfdf2872706790f9e8ddc104a64f4c`).

Raw build/test XML, ControllerServer action/command logs, rosdep result and
machine-readable summary accompany the local deliverable. See README for
reproduction and RELEASE_CHECKLIST.md for the remaining publication stages.
