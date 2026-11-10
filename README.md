# IGO 局部规划器

`igo_local_planner` 是面向阿克曼车辆的 ROS 1 `nav_core::BaseLocalPlanner` 插件，支持 ROS Noetic / catkin。它接收现有全局规划器的路径，在局部代价地图上使用对角高斯 IGO（Information-Geometric Optimization）优化速度和曲率序列，向 `move_base` 返回 `geometry_msgs/Twist`。

规划器在 **CPU** 上运行，无需 BPU、GPU 或神经网络模型。仓库包含优化核心、ROS 插件、动态参数和运动学仿真，接入时保留现有全局规划、定位、代价地图及底盘驱动。

- 前进、倒车和自动方向选择；换向先经过零速，遵守阿克曼转弯半径。
- 联合线速度、角速度、加速度和前轮转角变化率约束。
- 对整个凸多边形车体外廓做碰撞检查，并检查预测时域之后的延迟与制动轨迹。
- 包含参考跟踪、左右绕行、50% / 25% 低速参考以及保持轮角的制动候选；每个周期复用上次优化结果。

实现原理、代价和终点处理见 [docs/ALGORITHM.md](docs/ALGORITHM.md)。

## 构建与安装

以下命令在已安装 ROS Noetic 的 Ubuntu / catkin 环境执行。GitHub 仓库名为 `IGO_planner`，ROS 包名为 `igo_local_planner`。

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/igo_ws/src
cd ~/igo_ws/src
git clone https://github.com/Doribelove/IGO_planner.git igo_local_planner
cd ~/igo_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
rospack find igo_local_planner
```

需要使用安装目录时：

```bash
cd ~/igo_ws
catkin_make install
source install/setup.bash
```

C++ 核心采用 C++14。ROS 依赖由 `package.xml` 声明，包括 `nav_core`、`costmap_2d`、`pluginlib`、TF2、消息包和 `dynamic_reconfigure`。**TEB 不是编译或运行依赖**。

## 接入已有 move_base

在现有 `move_base` 节点的 launch 配置中加入或替换以下项。原来的 `base_global_planner`、全局/局部代价地图和定位配置继续使用。

```xml
<!-- 位于已有 <node pkg="move_base" type="move_base" name="move_base"> 内 -->
<param name="base_local_planner"
       value="igo_local_planner/IGOLocalPlannerROS"/>
<rosparam file="$(find igo_local_planner)/config/igo.yaml"/>
<param name="IGOLocalPlannerROS/odom_topic" value="/odom"/>
<!-- 可选的实际前轮转角反馈，std_msgs/Float64，单位 rad -->
<param name="IGOLocalPlannerROS/steering_topic" value="/steering_angle"/>
```

参数加载在 `move_base` 的私有命名空间中，典型完整路径为 `/move_base/IGOLocalPlannerROS/...`。`controller_frequency` 是 `move_base` 的参数，IGO 初始化时据此计算首步控制周期；默认读取 10 Hz，对应 0.1 s。

### 输入与输出约定

| 接口 | 数据与用途 |
|---|---|
| `setPlan()` | `move_base` 传入全局 `PoseStamped` 路径；插件在局部地图坐标系下截取参考段 |
| 局部 `Costmap2DROS` | 提供机器人当前位姿、地图快照及实际车体 footprint |
| `odom_topic` | `nav_msgs/Odometry`，读取 `twist.twist.linear.x` 与 `twist.twist.angular.z` |
| `steering_topic` | 可选 `std_msgs/Float64`，实际前轮转角，单位 rad，正值对应左转 |
| `computeVelocityCommands()` | 返回 `Twist.linear.x`（m/s）和 `Twist.angular.z`（rad/s），由 `move_base` 经原有控制链发送 |

`/odom` 必须包含与车体参考坐标一致的**实际前进速度和偏航角速度**，不能只提供位姿、却让 `twist` 长期为零。IGO 从代价地图/TF 获取位姿，不使用该消息的 `pose` 替代定位。它也不会自动将 `twist` 从其他坐标系旋转到车体系。

尤其需要核对已有 launch 的节点级 `remap`：即使 `odom_topic` 写成 `/odom`，该订阅仍可能被 ROS 重映射。若车辆在动而 status 中的 `input_v` 一直是零，先检查实际连接与速度字段。错误的零速度反馈会让每次优化都从静止状态计算可达速度。

前轮转角消息按接收墙钟时间判定：最近 0.5 s 内的有限值用于初始化轮角，包含零速时的真实转角；没有有效消息时，根据里程计的速度与角速度估计轮角，零速时估计为零。转角反馈不是额外启动条件。输出 `angular.z` 始终是**偏航角速度**，不是前轮转角。

### 车体几何

配置 `wheelbase`、`min_turning_radius` 和代价地图的 `footprint`，使其对应同一车体参考系。`footprint` 应是按轮廓顺序排列、至少三个点的凸多边形，包含车头附加装置等真实外廓；坐标为米，通常 x 向前、y 向左。

IGO 读取 `Costmap2DROS::getRobotFootprint()` 返回的 footprint，其中包含代价地图配置的 padding，不再额外添加一层车体余量。碰撞时同时检查多边形内部和边缘。越出局部地图的轨迹也判为不可行，因此局部地图需能容纳车体、预测运动和后续停车范围。

本仓库仿真中的几何仅用于复现示例。新底盘应使用自己的实际轴距、转弯能力和外廓。

## 参数

完整动态参数定义见 [cfg/IGO.cfg](cfg/IGO.cfg)，启动配置见 [config/igo.yaml](config/igo.yaml)。下表为当前动态参数默认值；已有 launch 显式设置及下文的兼容继承会影响最终值。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `max_vel_x` / `max_vel_x_backwards` | 0.8 / 0.3 | 前进/倒车速度上限，m/s；倒车参数填写正数 |
| `max_vel_theta` | 0.6 | 偏航角速度上限，rad/s |
| `acc_lim_x` / `acc_lim_theta` | 2.0 / 0.5 | 线加速度 / 偏航角加速度上限 |
| `decel_lim_x` | 0.18 | 预测物理制动减速度，m/s²，与起步加速度分开 |
| `stopping_delay` | 1.2 | 终点减速与停车延续检查采用的响应延迟，s |
| `wheelbase` / `min_turning_radius` | 0.65 / 1.35 | 轴距 / 最小转弯半径，m |
| `max_steering_rate` | 0.6 | 预测前轮转角变化率上限，rad/s |
| `steering_change_weight` | 0.5 | 前轮转角变化率平方项权重 |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.5 / 0.3 | 终点位置 / 朝向容差，m / rad |
| `max_global_plan_lookahead_dist` | 4.0 | 截取的局部全局路径长度，m |
| `motion_direction_mode` | 0 | -1 固定倒车、0 自动、+1 固定前进 |
| `allow_init_with_backwards_motion` | true | 是否允许倒车候选；固定倒车时也需要开启 |
| `global_plan_overwrite_orientation` | true | 将参考段中间点朝向改为路径切向；末点朝向保留 |
| `treat_unknown_as_obstacle` | true | 未知栅格是否作为硬障碍 |
| `horizon` / `knots` | 4.0 / 10 | 预测时域秒数 / 速度与曲率控制结点数 |
| `samples` / `iterations` | 48 / 4 | 每个方向、每轮的高斯样本数 / 更新轮数 |
| `mean_rate` / `variance_rate` | 0.65 / 0.25 | 均值 / 对角方差自然梯度更新步长 |
| `elite_fraction` | 0.25 | 排名效用使用的最优分位比例 |
| `path_weight` / `heading_weight` | 5.0 / 1.0 | 横向偏离 / 参考朝向代价权重 |
| `obstacle_weight` / `progress_weight` | 5.0 / 5.0 | 膨胀代价权重 / 有序路径进度奖励权重 |

`odom_topic` 和 `steering_topic` 为初始化时读取的 ROS 参数，不属于动态参数服务。代码默认转角话题为 `/m2_driver/wheel_angle`，可按上面的例子改为自己的反馈话题。

运行中可通过 `rqt_reconfigure` 或动态参数服务调整速度、方向、容差及优化参数，例如：

```bash
rosrun dynamic_reconfigure dynparam get /move_base/IGOLocalPlannerROS
rosrun dynamic_reconfigure dynparam set /move_base/IGOLocalPlannerROS motion_direction_mode 1
```

这些设置直接作用于当前规划器；固定前进/倒车适合任务执行器明确知道行驶方向的阶段。方向、结点数量或预测时域变化时会清除上次优化的初值。

## 路径、状态与兼容接口

以节点名 `move_base` 为例：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/move_base/IGOLocalPlannerROS/global_plan` | `nav_msgs/Path` | 当前截取并转换后的局部参考段，不是完整全局路线 |
| `/move_base/IGOLocalPlannerROS/local_plan` | `nav_msgs/Path` | 选中候选在预测时域内的轨迹 |
| `/move_base/IGOLocalPlannerROS/status` | `std_msgs/String` | JSON 诊断，如 `valid`、`solve_ms`、`evaluations`、`input_v`、`v`、`omega`、`goal_braking`、`reason` |

```bash
rosparam get /move_base/base_local_planner
rosnode info /move_base
rostopic echo -n 1 /move_base/IGOLocalPlannerROS/status
rostopic echo -n 1 /odom/twist
```

`solve_ms` 统计 ROS 控制回调中的地图拷贝、参考处理和优化时间，不是传感器采集到电机执行的端到端延迟。停车延续段参与碰撞检查，但不附加到发布的 `local_plan` 中。

为接入既有导航配置，初始化时会在 IGO 未显式设置相应参数的情况下，从 `~/TebLocalPlannerROS` 继承通用速度、加速度、轴距、转弯半径、目标容差和前瞻距离；`odom_topic` 也可继承。本仓库 `config/igo.yaml` 已完整列出默认值，加载该文件时以其中的 IGO 参数为准，不需要其他规划器命名空间。TEB 的图优化权重、迭代参数和转角输出方式不参与 IGO。

另外发布 `~/TebLocalPlannerROS/global_plan` 和 `~/TebLocalPlannerROS/local_plan` 两个 `Path` 显示别名，方便复用已有 RViz 配置。它们不加载 TEB，也不建立 TEB 动态参数服务。规划器身份以 `base_local_planner` 和 IGO status 为准。

### 可选的终点朝向自由协议

普通路径默认同时检查终点位置和朝向。若上层任务确实允许忽略终点 yaw，当前源码支持以下路径约定，三个条件必须同时成立：

1. `motion_direction_mode == 0`；
2. 路径每个 `PoseStamped.header.seq` 完全相同；
3. `(seq & 0xC0000000) == 0x80000000`，即最高两位为二进制 `10`。

该标志需由全局路径生产方保留到 `setPlan()`；仅修改 `MoveBaseGoal` 的序号不会自动让所有全局规划器透传。低 30 位可用于区分任务。普通 seq=0、固定档位或带有其他高位模式的路径都保留终点朝向约束。`global_plan_overwrite_orientation=true` 也不会自动免除终点朝向要求。

## 验证内容

本次独立包构建、安装、核心场景和 ROS 模拟闭环的结果见 [验证记录](docs/VALIDATION.md)。

核心场景可执行文件 `igo_scenarios` 不需要 ROS master，覆盖直行、倒车、半径弯道、自动倒车、局部绕障、带延迟制动，以及外廓碰撞、未知格、转角变化率和换向约束。它使用模拟车辆和合成地图，输出求解统计与测试终态。

```bash
rosrun igo_local_planner igo_scenarios
mkdir -p /tmp/igo-core-results
rosrun igo_local_planner igo_scenarios /tmp/igo-core-results 0.18 1.2 31
```

后一条命令依次指定输出目录、制动减速度、响应延迟和随机种子，并保存轨迹、参考线及障碍 CSV。

插件装载检查使用真实 `pluginlib` 按导出的名称创建插件实例，不初始化代价地图，也不需要 ROS master：

```bash
rosrun igo_local_planner igo_plugin_load_check
```

这两个检查注册为 CTest 测试，可在上述工作区构建后运行：

```bash
cd ~/igo_ws/build
ctest -R '^igo_' --output-on-failure
```

完整 ROS 模拟闭环使用 `move_base + Navfn + IGO + 模拟底盘`，自动执行前进、倒车和自动方向三个目标：

```bash
rosrun igo_local_planner run_simulation.py --output /tmp/igo_simulation
```

入口为 [scripts/run_simulation.py](scripts/run_simulation.py)，内部启动 [launch/simulation.launch](launch/simulation.launch)。脚本分配新的回环 ROS master 端口，覆盖继承的 master/IP 设置，使用合成空地图和模拟速度反馈。可以用 `--port 11951` 指定空闲端口；省略 `--output` 则创建新的临时结果目录。

`ros_simulation.json` 保存三个目标的 action 状态、误差、诊断和轨迹，ROS 日志保存在同一输出目录。再次运行时使用新目录，避免与已有结果混合。示例只启动模拟节点，不包含实车驱动或传感器启动项。

当前求解采用局部静态代价地图快照；移动障碍物的未来运动预测不在优化模型中。核心场景和 ROS 模拟闭环用于验证接口、约束与实现回归，实车效果还取决于实际反馈、转向执行与制动响应。
