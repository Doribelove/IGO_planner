# ROS2 / Nav2 release preparation

Verified against official sources on 2026-11-10. This checklist was prepared
before the first authorized source publication. Source publication, tags,
registration, release repositories and binary release are separate steps;
check their actual verified status rather than assuming they occurred.

## Four separate outcomes

1. A standard plugin: implements the Jazzy `nav2_core::Controller` contract,
   exported through pluginlib/ament, and actually builds/loads/runs.
2. ROS Index entry: public source with valid package metadata, followed by a
   reviewed source entry in the Jazzy rosdistro distribution file.
3. Official apt binaries: separate release repository and bloom track, reviewed
   release entry, successful buildfarm jobs, testing and a distro sync. An Index
   entry or merged PR alone does not mean apt packages are available.
4. Inclusion in navigation2 itself: a separate contribution/maintenance decision
   by Nav2 maintainers. An independently hosted controller need not merge there.

None of these constitutes hardware validation or safety certification.

## Prepared locally

- SPDX `BSD-3-Clause` declaration and the original LICENSE retained.
- Lowercase ROS package name `igo_nav2_controller`, version `0.1.0`.
- Explicit build/runtime/test dependencies and ament plugin export.
- Lifecycle API, real pluginlib tests and unchanged core regression tests.
- Jazzy/Noble CI definition; whether it has run is separately recorded.
- Self-contained source generator with core provenance hashes.
- Integration configuration, stated interface limits and validation report.

## Before publishing source

- Review the local diff, test results and declared support matrix.
- Confirm maintainer identity and a working reachable maintainer email. The
  existing GitHub noreply address is retained from upstream but deliverability
  has not been tested; release contact validity remains an owner decision.
- Audit authorship and licenses of all included sources. The BSD text exists,
  but its presence is not proof of third-party ownership or patent clearance.
- Review versioning and release notes. This adaptation is not yet published.
- Place generated self-contained source at the root of an authorized dedicated
  ROS2 branch/repository before bloom. The nested development layout keeps the
  original Noetic package usable, but is not directly suitable for bloom's
  package discovery. Never silently release only nav2/ without its shared core.
- Perform clean ROS1 regression before claiming unchanged Noetic compatibility
  has been reverified, and the intended robot's safety-gated integration before
  claiming hardware operation of this new wrapper.

## Before ROS Index / apt release

- Confirm package name availability in the target distribution, not just GitHub.
- Public HTTPS source URL, correct branch, maintenance status, REP144 naming.
- Valid package.xml metadata: name, version, description, maintainer email,
  matching OSI/SPDX license and declared dependencies using ROS/rosdep keys.
- Dependencies must resolve in the official buildfarm without undeclared pip
  downloads. CMake must not fetch network dependencies.
- Prepare CHANGELOG and an authorized version tag. Release preparation tools
  may commit/tag/push: do not run them as a supposedly read-only check.
- Arrange a release repository and team. Rolling requires ros2-gbp; stable
  distributions currently permit external release repositories but centralized
  hosting is strongly recommended. Jazzy-only release is not automatically
  forbidden; releasing Rolling for future distributions is recommended.
- With specific publication authorization, run bloom with the proper track,
  review its generated distribution PR and monitor buildfarm/testing/sync.
- Confirm exact upstream and release commits, binary availability and a clean
  apt install before announcing official distribution availability.

The user has not authorized this local workflow to accept new legal agreements
or make financial/contract commitments. Community communication and technical
publication decisions remain subject to their applicable authorization scope.

## Optional Nav2 upstream contribution

Discuss a new controller/algorithm with maintainers before a large PR. Include
rationale, tests, docs, supported platforms and a maintenance plan. Follow DCO
sign-off and lint/static checks. Nav2's current contribution rules expect Apache
2.0 for entirely new packages; the existing IGO code is BSD-3-Clause, so do not
relicense it unilaterally. Independent standard-plugin distribution remains a
valid route and does not require upstream acceptance.

## Official sources

- Jazzy API: https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_core/include/nav2_core/controller.hpp
- Jazzy tutorial: https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/
- Server/exception behavior: https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/src/controller_server.cpp
- ROS Index: https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Releasing/Index-Your-Packages.rst
- First release: https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Releasing/First-Time-Release.rst
- Release repository/team: https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Releasing/Release-Team-Repository.rst
- Review guidelines: https://github.com/ros/rosdistro/blob/master/REVIEW_GUIDELINES.md
- Package metadata: https://reps.openrobotics.org/rep-0149/
- Names: https://reps.openrobotics.org/rep-0144/
- Supported platforms: https://reps.openrobotics.org/rep-2000/
- Nav2 contributions: https://github.com/ros-navigation/navigation2/blob/jazzy/CONTRIBUTING.md
- Noetic final release: https://discourse.ros.org/t/new-packages-for-noetic-2025-05-29-final/44003

Some docs.ros.org pages presented an access challenge during verification; the
corresponding official ros2_documentation Jazzy source was read instead.
