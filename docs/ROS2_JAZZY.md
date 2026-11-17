# IGO / ROS 2 Jazzy 源码安装与验证

IGO 的 ROS 2 包名为 `igo_nav2_controller`，提供 `igo_nav2_controller::IGOController` 这一 Nav2 局部控制器插件。它接收全局路径并输出阿克曼车辆的速度指令；全局规划、定位和底盘驱动由现有导航系统提供。

源码位于同一仓库的 **[`ros2-jazzy` 分支](https://github.com/Doribelove/IGO_planner/tree/ros2-jazzy)**，默认 `main` 分支是 ROS 1 Noetic 包。本页提供从仓库首页到 ROS 2 安装的入口，完整接口约定与更新见 [ROS 2 README](https://github.com/Doribelove/IGO_planner/blob/ros2-jazzy/README.md)。

## 1. 准备环境与克隆正确分支

以下命令在 Ubuntu 24.04、已安装 ROS 2 Jazzy、`rosdep` 和 `colcon` 的 Bash 环境执行。若刚安装 `rosdep`，先按 ROS 安装说明完成一次 `sudo rosdep init`。使用新目录并在同一个终端依次执行：

```bash
source /opt/ros/jazzy/setup.bash
git clone --branch ros2-jazzy --single-branch \
  https://github.com/Doribelove/IGO_planner.git igo_jazzy
cd igo_jazzy
rosdep update --rosdistro jazzy
rosdep install --from-paths . --ignore-src --rosdistro jazzy -y
```

## 2. 构建、测试与插件装载

仍在 `igo_jazzy` 源码根目录中执行。该分支已经包含优化核心，无需先编译 ROS 1 包。

```bash
colcon build --paths . --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --paths . --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
ros2 pkg prefix igo_nav2_controller
ros2 run igo_nav2_controller igo_nav2_plugin_load_check
```

预期结果：构建与测试命令成功退出，测试汇总没有失败，`ros2 pkg prefix` 显示当前工作区的安装路径，插件装载检查成功退出。若找不到包，先核对分支为 `ros2-jazzy`，并确认当前终端已执行 `source install/setup.bash`。

## 3. 可选的合成控制闭环

在独立测试环境中，选择与现有机器人不同且未占用的 ROS domain。下例使用 198；输出目录必须不存在或为空。

```bash
python3 scripts/controller_server_smoke.py --domain 198 --output /tmp/igo-jazzy-smoke-new
```

脚本启动真实 Nav2 ControllerServer、合成底盘和 TF/里程计，验证前进、倒车与取消任务，并将结果写入输出目录。该示例只启动模拟节点；ROS domain 用于隔离发现，不替代实际网络隔离。

接入现有机器人时，使用分支中的 [controller_server.yaml](https://github.com/Doribelove/IGO_planner/blob/ros2-jazzy/config/controller_server.yaml) 合并控制器参数，并保留现有定位、代价地图、全局规划与底盘设置。合并配置时保留 `StoppedGoalChecker` 和低速里程计阈值设置，以支持终点制动与停车判定。`Twist.angular.z` 表示偏航角速度，车体几何与速度反馈应采用同一阿克曼参考点；完整约定见 [Integration contract](https://github.com/Doribelove/IGO_planner/blob/ros2-jazzy/README.md#integration-contract)。

## ROS 登记与验证记录

- [rosdistro #55324](https://github.com/ros/rosdistro/pull/55324) 已于 2026-11-16 合并，将 `ros2-jazzy` 分支登记为 Jazzy 的源码来源。
- [ROS Index 搜索](https://index.ros.org/?pkgs=igo_nav2_controller&search_packages=true#jazzy)：选择 Jazzy，使用包名 `igo_nav2_controller`；索引页面随网站构建更新。
- 当前发布方式为源码构建。二进制发布进度按 [发布清单](https://github.com/Doribelove/IGO_planner/blob/ros2-jazzy/docs/RELEASE_CHECKLIST.md) 跟进。
- [已记录的验证结果](https://github.com/Doribelove/IGO_planner/blob/ros2-jazzy/docs/VALIDATION.md) 与 [Jazzy 自动化构建](https://github.com/Doribelove/IGO_planner/actions/workflows/nav2-jazzy.yml?query=branch%3Aros2-jazzy) 可用于核对具体提交。当前 ROS 2 验证覆盖软件与合成场景，实车适配仍需验证实际转向、速度反馈与制动响应。

返回 [项目首页 / ROS 1 使用说明](../README.md)。
