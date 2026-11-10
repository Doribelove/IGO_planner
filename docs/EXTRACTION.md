# 独立插件来源

本仓库提取自 `Doribelove/autolabor-robot-nav` 工程中的
`src/navigation_arena/igo_local_planner`，提取日期为 2026-11-10。
使用的是当日工作树，包含尚未提交到原仓库 HEAD 的最新制动、转角反馈和优化器更新。
逐文件原始摘要见 [source_snapshot.json](source_snapshot.json)。

独立化保留了优化器、ROS 插件类名和控制接口。`core.cpp`、`core.h` 的内容与
提取时完全一致；ROS 包装源码只补充了显式的 `<algorithm>` 头文件。

仓库补齐了依赖声明、安装规则、完整参数示例、独立仿真入口、插件加载测试、
使用文档及原包已声明的 BSD-3-Clause 许可证。原工程专用的
`check_launch.py` 和 `test_coverage_adapter.py` 依赖整车 launch 与覆盖任务管理器，
未纳入独立插件；自包含的运动学场景测试随包保留。

`source_snapshot.json` 记录提取前文件摘要；独立化调整过的构建、配置、文档和测试入口
应以本仓库版本为准。仓库不包含整车地图、相机模型、录包或编译产物。
