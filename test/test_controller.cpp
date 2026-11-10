// SPDX-License-Identifier: BSD-3-Clause
// These tests exercise the installed Nav2 plugin interface on a synthetic map.
// No controller server, external navigation stack, sensors or robot are started.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "igo_local_planner/core.h"
#include "igo_nav2_controller/controller.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{
constexpr double kPeriod = 0.1;
constexpr double kRadius = 1.35;
constexpr double kWheelbase = 0.65;
constexpr char kPlugin[] = "igo_nav2_controller::IGOController";
constexpr char kName[] = "FollowPath";
static_assert(std::is_base_of<nav2_core::Controller,
  igo_nav2_controller::IGOController>::value, "Jazzy controller interface required");

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
[[maybe_unused]] auto * const ros_environment =
  ::testing::AddGlobalTestEnvironment(new RosEnvironment());

// Use the real Costmap2D/LayeredCostmap and footprint APIs without calling
// Costmap2DROS::on_activate(), which would wait for live robot TF and sensors.
class SyntheticCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  SyntheticCostmap()
  : nav2_costmap_2d::Costmap2DROS("igo_test_costmap", false)
  {
    global_frame_ = "map";
    robot_base_frame_ = "base_link";
    footprint_padding_ = 0.0;
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      global_frame_, false, false);
    layered_costmap_->resizeMap(240, 240, 0.1, -12.0, -12.0);
    std::vector<geometry_msgs::msg::Point> points(4);
    points[0].x = 1.32; points[0].y = 0.5;
    points[1].x = 1.32; points[1].y = -0.5;
    points[2].x = -0.62; points[2].y = -0.5;
    points[3].x = -0.62; points[3].y = 0.5;
    setRobotFootprint(points);
    clear();
  }

  void clear()
  {
    auto * map = getCostmap();
    std::fill_n(map->getCharMap(),
      map->getSizeInCellsX() * map->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
  }

  void fill(double x0, double y0, double x1, double y1, unsigned char cost)
  {
    auto * map = getCostmap();
    for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
        double wx, wy;
        map->mapToWorld(x, y, wx, wy);
        if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) {
          map->setCost(x, y, cost);
        }
      }
    }
  }

  igo_local_planner::Grid snapshot()
  {
    auto * map = getCostmap();
    igo_local_planner::Grid grid;
    grid.width = map->getSizeInCellsX();
    grid.height = map->getSizeInCellsY();
    grid.resolution = map->getResolution();
    grid.origin_x = map->getOriginX();
    grid.origin_y = map->getOriginY();
    grid.cells.assign(map->getCharMap(), map->getCharMap() + grid.width * grid.height);
    grid.prepare();
    return grid;
  }
};

class TestGoalChecker : public nav2_core::GoalChecker
{
public:
  double xy = 0.5;
  double yaw = 0.3;
  bool valid_tolerances = true;
  unsigned int goal_queries = 0;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {goal_queries = 0;}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    ++goal_queries;
    return false;
  }
  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    pose = geometry_msgs::msg::Pose();
    pose.position.x = xy;
    pose.position.y = xy;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
    velocity = geometry_msgs::msg::Twist();
    velocity.linear.x = velocity.linear.y = velocity.angular.z = 0.05;
    return valid_tolerances;
  }
};

geometry_msgs::msg::PoseStamped poseAt(
  double x = 0.0, double y = 0.0, double yaw = 0.0,
  const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

nav_msgs::msg::Path line(double length = 5.0, int direction = 1)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double d = 0; d < length; d += 0.1) {
    path.poses.push_back(poseAt(direction * d));
  }
  path.poses.push_back(poseAt(direction * length));
  return path;
}

class ControllerTest : public ::testing::Test
{
protected:
  // Loader must outlive every instance it creates, including teardown paths.
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  nav2_core::Controller::Ptr controller_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<SyntheticCostmap> costmap_;
  TestGoalChecker checker_;
  bool configured_ = false;
  bool active_ = false;

  void SetUp() override
  {
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    controller_ = loader_->createSharedInstance(kPlugin);
    costmap_ = std::make_shared<SyntheticCostmap>();
    makeNode({});
  }

  void TearDown() override
  {
    if (active_) {
      EXPECT_NO_THROW(controller_->deactivate());
    }
    if (configured_) {
      EXPECT_NO_THROW(controller_->cleanup());
    }
    controller_.reset();
    loader_.reset();
    tf_.reset();
    costmap_.reset();
    node_.reset();
  }

  void makeNode(std::vector<rclcpp::Parameter> overrides)
  {
    tf_.reset();
    node_.reset();
    overrides.emplace_back("controller_frequency", 1.0 / kPeriod);
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("igo_controller_test", options);
    // Match ControllerServer, which declares this parent-level parameter.
    node_->declare_parameter("controller_frequency", 1.0 / kPeriod);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // No ROS listener/thread is needed: test transforms are inserted directly.
    tf_->setUsingDedicatedThread(true);
  }

  void configure(std::vector<rclcpp::Parameter> overrides = {})
  {
    if (!overrides.empty()) {
      makeNode(std::move(overrides));
    }
    controller_->configure(node_, kName, tf_, costmap_);
    configured_ = true;
  }

  void start(std::vector<rclcpp::Parameter> overrides = {})
  {
    configure(std::move(overrides));
    controller_->activate();
    active_ = true;
  }

  geometry_msgs::msg::TwistStamped command(
    const geometry_msgs::msg::PoseStamped & pose = poseAt(),
    const geometry_msgs::msg::Twist & velocity = geometry_msgs::msg::Twist())
  {
    return controller_->computeVelocityCommands(pose, velocity, &checker_);
  }

  void expectAckermann(
    const geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & previous, double forward = 0.8, double reverse = 0.3)
  {
    EXPECT_TRUE(std::isfinite(command.linear.x));
    EXPECT_TRUE(std::isfinite(command.angular.z));
    EXPECT_GE(command.linear.x, -reverse - 1e-8);
    EXPECT_LE(command.linear.x, forward + 1e-8);
    EXPECT_LE(std::abs(command.angular.z), std::abs(command.linear.x) / kRadius + 1e-8);
    EXPECT_LE(std::abs(command.angular.z), 0.6 + 1e-8);
    EXPECT_LE(std::abs(command.linear.x - previous.linear.x), 2.0 * kPeriod + 1e-8);
    EXPECT_LE(std::abs(command.angular.z - previous.angular.z), 0.5 * kPeriod + 1e-8);
    EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.linear.z, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.y, 0.0);
    if (std::abs(command.linear.x) < 1e-9) {
      EXPECT_NEAR(command.angular.z, 0.0, 1e-9);
    }
  }

  // Independent constant-Twist integration, not Optimizer::advance().
  static void integrate(
    igo_local_planner::State & state, const geometry_msgs::msg::Twist & velocity)
  {
    const double next_yaw = state.yaw + velocity.angular.z * kPeriod;
    if (std::abs(velocity.angular.z) > 1e-9) {
      state.x += velocity.linear.x / velocity.angular.z *
        (std::sin(next_yaw) - std::sin(state.yaw));
      state.y -= velocity.linear.x / velocity.angular.z *
        (std::cos(next_yaw) - std::cos(state.yaw));
    } else {
      state.x += velocity.linear.x * std::cos(state.yaw) * kPeriod;
      state.y += velocity.linear.x * std::sin(state.yaw) * kPeriod;
    }
    state.yaw = next_yaw;
    state.v = velocity.linear.x;
    state.omega = velocity.angular.z;
  }

  void closedLoop(const nav_msgs::msg::Path & path, int gear, unsigned int steps)
  {
    controller_->setPlan(path);
    igo_local_planner::State state;
    geometry_msgs::msg::Twist velocity;
    auto grid = costmap_->snapshot();
    std::vector<igo_local_planner::Point> footprint;
    for (const auto & point : costmap_->getRobotFootprint()) {
      footprint.push_back({point.x, point.y});
    }
    const auto & goal = path.poses.back().pose.position;
    const auto & orientation = path.poses.back().pose.orientation;
    const double goal_yaw = std::atan2(2.0 * orientation.w * orientation.z,
      1.0 - 2.0 * orientation.z * orientation.z);
    bool reached = false;
    unsigned int moving_commands = 0;
    double previous_steering = 0.0;
    for (unsigned int step = 0; step < steps; ++step) {
      SCOPED_TRACE(step);
      const auto result = command(poseAt(state.x, state.y, state.yaw), velocity);
      ASSERT_EQ(result.header.frame_id, "base_link");
      expectAckermann(result.twist, velocity);
      if (gear != 0) {
        ASSERT_GE(result.twist.linear.x * gear, -1e-8);
      }
      if (std::abs(result.twist.linear.x) > 1e-6) {
        const double steering = std::atan(
          kWheelbase * result.twist.angular.z / result.twist.linear.x);
        EXPECT_LE(std::abs(steering - previous_steering), 0.6 * kPeriod + 1e-7);
        previous_steering = steering;
      }
      moving_commands += std::abs(result.twist.linear.x) > 0.01;
      velocity = result.twist;
      integrate(state, velocity);
      ASSERT_TRUE(grid.footprint(state, footprint)) << "command collides after integration";
      if (std::hypot(state.x - goal.x, state.y - goal.y) <= checker_.xy + 0.02 &&
        std::abs(igo_local_planner::angle(state.yaw - goal_yaw)) <= checker_.yaw + 0.02 &&
        std::abs(state.v) <= 0.01 && std::abs(state.omega) <= 0.01)
      {
        reached = true;
        break;
      }
    }
    EXPECT_TRUE(reached) << "final pose " << state.x << ", " << state.y;
    EXPECT_GT(moving_commands, 5u);
    EXPECT_EQ(checker_.goal_queries, 0u) << "server owns stateful goal completion";
  }
};

TEST_F(ControllerTest, LoadsExactPluginThroughNav2CoreExport)
{
  const auto classes = loader_->getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kPlugin), classes.end());
  ASSERT_NE(controller_, nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<igo_nav2_controller::IGOController>(controller_), nullptr);
}

TEST_F(ControllerTest, LifecycleRejectsInactiveCommandsAndSupportsReconfigure)
{
  EXPECT_THROW(command(), nav2_core::InvalidController);
  for (int cycle = 0; cycle < 2; ++cycle) {
    configure();
    controller_->setPlan(line());
    EXPECT_THROW(command(), nav2_core::InvalidController);
    controller_->activate(); active_ = true;
    EXPECT_NO_THROW(command());
    controller_->deactivate(); active_ = false;
    EXPECT_THROW(command(), nav2_core::InvalidController);
    controller_->cleanup(); configured_ = false;
    EXPECT_THROW(command(), nav2_core::InvalidController);
  }
}

TEST_F(ControllerTest, MissingDependenciesAreRejected)
{
  rclcpp_lifecycle::LifecycleNode::WeakPtr expired;
  EXPECT_THROW(controller_->configure(expired, kName, tf_, costmap_), std::exception);
  EXPECT_THROW(controller_->configure(node_, kName, nullptr, costmap_), std::exception);
  EXPECT_THROW(controller_->configure(node_, kName, tf_, nullptr), std::exception);
}

TEST_F(ControllerTest, DegenerateAndNonConvexFootprintsAreRejected)
{
  std::vector<geometry_msgs::msg::Point> points(4);
  // A bow-tie is self-intersecting rather than an ordered convex polygon.
  points[0].x = -0.5; points[0].y = -0.5;
  points[1].x = 0.5; points[1].y = 0.5;
  points[2].x = -0.5; points[2].y = 0.5;
  points[3].x = 0.5; points[3].y = -0.5;
  costmap_->setRobotFootprint(points);
  EXPECT_THROW(configure(), nav2_core::ControllerException);
  points.assign(4, geometry_msgs::msg::Point());
  costmap_->setRobotFootprint(points);
  EXPECT_THROW(configure(), nav2_core::ControllerException);
}

TEST_F(ControllerTest, CleanupConfigureReloadsChangedFootprint)
{
  start(); controller_->setPlan(line());
  costmap_->fill(0.80, 0.10, 1.00, 0.30, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(command(), nav2_core::NoValidControl);
  controller_->deactivate(); active_ = false;
  controller_->cleanup(); configured_ = false;
  auto points = costmap_->getRobotFootprint();
  for (auto & point : points) {point.x *= 0.2; point.y *= 0.2;}
  costmap_->setRobotFootprint(points);
  start(); controller_->setPlan(line());
  EXPECT_NO_THROW(command());
}

TEST_F(ControllerTest, MissingPlanAndGoalCheckerAreRejected)
{
  start();
  EXPECT_THROW(command(), nav2_core::InvalidPath);
  controller_->setPlan(line());
  EXPECT_THROW(controller_->computeVelocityCommands(poseAt(), geometry_msgs::msg::Twist(), nullptr),
    nav2_core::ControllerException);
}

TEST_F(ControllerTest, EmptyNewPlanInvalidatesOldPlan)
{
  start();
  controller_->setPlan(line());
  EXPECT_NO_THROW(command());
  EXPECT_THROW(controller_->setPlan(nav_msgs::msg::Path()), nav2_core::InvalidPath);
  EXPECT_THROW(command(), nav2_core::InvalidPath);
}

TEST_F(ControllerTest, InvalidPathCoordinatesAndOrientationsAreRejected)
{
  start();
  for (int invalid = 0; invalid < 4; ++invalid) {
    SCOPED_TRACE(invalid);
    auto path = line();
    if (invalid == 0) {path.poses[2].pose.position.x = std::numeric_limits<double>::quiet_NaN();}
    if (invalid == 1) {path.poses[2].pose.position.y = std::numeric_limits<double>::infinity();}
    if (invalid == 2) {path.poses[2].pose.orientation.w = 0.0;}
    if (invalid == 3) {path.poses[2].pose.orientation.z = std::numeric_limits<double>::quiet_NaN();}
    EXPECT_THROW(controller_->setPlan(path), nav2_core::InvalidPath);
    EXPECT_THROW(command(), nav2_core::InvalidPath);
  }
}

TEST_F(ControllerTest, EmptyPathFrameIsRejected)
{
  start();
  auto path = line();
  path.header.frame_id.clear();
  for (auto & point : path.poses) {point.header.frame_id.clear();}
  EXPECT_THROW(controller_->setPlan(path), nav2_core::InvalidPath);
}

TEST_F(ControllerTest, MixedPathFramesInvalidateTheReplacement)
{
  start(); controller_->setPlan(line());
  auto path = line(); path.poses[2].header.frame_id = "odom";
  EXPECT_THROW(controller_->setPlan(path), nav2_core::InvalidPath);
  EXPECT_THROW(command(), nav2_core::InvalidPath);
}

TEST_F(ControllerTest, SinglePosePlanIsAValidStationaryGoal)
{
  start();
  nav_msgs::msg::Path path; path.header.frame_id = "map";
  path.poses.push_back(poseAt());
  controller_->setPlan(path);
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, StartupParameterAndFrequencyChangesAreRejected)
{
  start();
  EXPECT_FALSE(node_->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.max_vel_x", 1.0)}).successful);
  EXPECT_FALSE(node_->set_parameters_atomically({
    rclcpp::Parameter("controller_frequency", 5.0)}).successful);
}

TEST_F(ControllerTest, ExcessivePathLengthIsRejected)
{
  start({rclcpp::Parameter("FollowPath.max_plan_poses", 10)});
  EXPECT_THROW(controller_->setPlan(line()), nav2_core::InvalidPath);
}

TEST_F(ControllerTest, InvalidRobotFeedbackIsRejected)
{
  start(); controller_->setPlan(line());
  auto pose = poseAt();
  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(command(pose), nav2_core::ControllerException);
  pose = poseAt(); pose.pose.orientation.w = 0.0;
  EXPECT_THROW(command(pose), nav2_core::ControllerException);
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(command(poseAt(), velocity), nav2_core::ControllerException);
  velocity = geometry_msgs::msg::Twist();
  velocity.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(command(poseAt(), velocity), nav2_core::ControllerException);
}

TEST_F(ControllerTest, VeryLargeFiniteCoordinatesFailBeforeGridIndexConversion)
{
  start();
  for (const double huge : {1e300, 1e308}) {
    SCOPED_TRACE(huge);
    auto path = line();
    for (auto & point : path.poses) {point.pose.position.x = huge;}
    EXPECT_THROW(controller_->setPlan(path), nav2_core::InvalidPath);
    controller_->setPlan(line());
    EXPECT_THROW(command(poseAt(huge, huge)), nav2_core::ControllerTFError);
  }
  // Both geometries are finite and accepted by the coordinate guard. Their
  // common location is still far outside the map, so reject in double before
  // core footprint rasterization can convert a cell coordinate to int.
  auto path = line();
  for (auto & point : path.poses) {point.pose.position.x += 1e8;}
  controller_->setPlan(path);
  EXPECT_THROW(command(poseAt(1e8)), nav2_core::NoValidControl);
}

TEST_F(ControllerTest, MissingRobotTransformThrowsControllerTFError)
{
  start(); controller_->setPlan(line());
  EXPECT_THROW(command(poseAt(0.0, 0.0, 0.0, "unconnected_robot_frame")),
    nav2_core::ControllerTFError);
}

TEST_F(ControllerTest, MissingPlanTransformThrowsControllerTFError)
{
  start();
  auto path = line(); path.header.frame_id = "unconnected_plan_frame";
  for (auto & point : path.poses) {point.header.frame_id = path.header.frame_id;}
  controller_->setPlan(path);
  EXPECT_THROW(command(), nav2_core::ControllerTFError);
}

TEST_F(ControllerTest, TransformsRobotAndPlanAndReturnsBaseFrame)
{
  start();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = 1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "igo_test", true));
  auto path = line(); path.header.frame_id = "odom";
  for (auto & point : path.poses) {
    point.header.frame_id = "odom";
    point.pose.position.x -= 2.0;
    point.pose.position.y -= 1.0;
  }
  controller_->setPlan(path);
  const auto result = command(poseAt(-2.0, -1.0, 0.0, "odom"));
  EXPECT_EQ(result.header.frame_id, "base_link");
  EXPECT_GT(rclcpp::Time(result.header.stamp).nanoseconds(), 0);
  EXPECT_GT(result.twist.linear.x, 0.0);
  expectAckermann(result.twist, geometry_msgs::msg::Twist());
}

TEST_F(ControllerTest, RobotTooFarFromPathIsRejected)
{
  start(); controller_->setPlan(line());
  EXPECT_THROW(command(poseAt(0.0, 4.0)), nav2_core::InvalidPath);
}

TEST_F(ControllerTest, StationaryGoalProducesZeroWithoutSpin)
{
  start(); controller_->setPlan(line(1.0));
  const auto result = command(poseAt(1.0));
  EXPECT_DOUBLE_EQ(result.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.twist.angular.z, 0.0);
  EXPECT_EQ(checker_.goal_queries, 0u);
}

TEST_F(ControllerTest, GoalCheckerTolerancesOverrideFallback)
{
  start(); checker_.xy = 0.05;
  controller_->setPlan(line(0.4));
  EXPECT_GT(command().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, UnavailableGoalCheckerTolerancesUseFallback)
{
  start(); checker_.valid_tolerances = false;
  controller_->setPlan(line(0.4));
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, AbsoluteAndPercentageSpeedLimitsAndReset)
{
  start(); controller_->setPlan(line());
  geometry_msgs::msg::Twist velocity;
  controller_->setSpeedLimit(0.12, false);
  for (int step = 0; step < 10; ++step) {
    const auto result = command(poseAt(), velocity);
    expectAckermann(result.twist, velocity, 0.12, 0.12);
    velocity = result.twist;
  }
  EXPECT_GT(velocity.linear.x, 0.01);
  velocity = geometry_msgs::msg::Twist();
  controller_->setSpeedLimit(25.0, true);
  for (int step = 0; step < 10; ++step) {
    const auto result = command(poseAt(), velocity);
    expectAckermann(result.twist, velocity, 0.2, 0.075);
    velocity = result.twist;
  }
  controller_->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  double peak_speed = 0.0;
  for (int step = 0; step < 10; ++step) {
    const auto result = command(poseAt(), velocity);
    expectAckermann(result.twist, velocity);
    velocity = result.twist;
    peak_speed = std::max(peak_speed, velocity.linear.x);
  }
  EXPECT_GT(peak_speed, 0.2);
}

TEST_F(ControllerTest, LoweredLimitBrakesMeasuredOverspeedWithoutClipping)
{
  start(); controller_->setPlan(line());
  geometry_msgs::msg::Twist velocity; velocity.linear.x = 0.6;
  controller_->setSpeedLimit(0.1, false);
  const auto result = command(poseAt(), velocity);
  expectAckermann(result.twist, velocity);
  EXPECT_GT(result.twist.linear.x, 0.1);
  EXPECT_LE(result.twist.linear.x, velocity.linear.x);
}

TEST_F(ControllerTest, PercentageLimitAlsoScalesReverseSpeed)
{
  start({rclcpp::Parameter("FollowPath.motion_direction_mode", -1),
    rclcpp::Parameter("FollowPath.global_plan_overwrite_orientation", false)});
  controller_->setPlan(line(2.0, -1));
  controller_->setSpeedLimit(50.0, true);
  geometry_msgs::msg::Twist velocity;
  for (int step = 0; step < 10; ++step) {
    const auto result = command(poseAt(), velocity);
    expectAckermann(result.twist, velocity, 0.4, 0.15);
    EXPECT_LE(result.twist.linear.x, 0.0);
    velocity = result.twist;
  }
  EXPECT_LT(velocity.linear.x, -0.01);
}

TEST_F(ControllerTest, InvalidSpeedLimitsAreRejected)
{
  start();
  EXPECT_THROW(controller_->setSpeedLimit(-1.0, false), nav2_core::ControllerException);
  EXPECT_THROW(controller_->setSpeedLimit(101.0, true), nav2_core::ControllerException);
  EXPECT_THROW(controller_->setSpeedLimit(std::numeric_limits<double>::quiet_NaN(), false),
    nav2_core::ControllerException);
  EXPECT_THROW(controller_->setSpeedLimit(std::numeric_limits<double>::infinity(), true),
    nav2_core::ControllerException);
}

TEST_F(ControllerTest, CollisionInsideLargeFrontAttachmentIsRejected)
{
  start(); controller_->setPlan(line());
  // The centre and all footprint vertices remain free.
  costmap_->fill(0.80, 0.10, 1.00, 0.30, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(command(), nav2_core::NoValidControl);
}

TEST_F(ControllerTest, UnknownInsideFootprintIsBlockedByDefault)
{
  start(); controller_->setPlan(line());
  costmap_->fill(0.80, 0.10, 1.00, 0.30, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_THROW(command(), nav2_core::NoValidControl);
}

TEST_F(ControllerTest, UnknownPolicyMayBeExplicitlyRelaxed)
{
  start({rclcpp::Parameter("FollowPath.treat_unknown_as_obstacle", false)});
  controller_->setPlan(line());
  costmap_->fill(0.80, 0.10, 1.00, 0.30, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_NO_THROW(command());
}

TEST_F(ControllerTest, UnavoidableCollisionAtSpeedThrowsNoValidControl)
{
  start(); controller_->setPlan(line());
  costmap_->fill(1.4, -2.0, 1.5, 2.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist velocity; velocity.linear.x = 0.8;
  EXPECT_THROW(command(poseAt(), velocity), nav2_core::NoValidControl);
}

TEST_F(ControllerTest, BlockedPathBrakesWithoutCrossingWall)
{
  start(); controller_->setPlan(line());
  costmap_->fill(1.4, -2.0, 1.5, 2.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto result = command();
  EXPECT_GE(result.twist.linear.x, 0.0);
  EXPECT_LE(result.twist.linear.x, 0.1);
  expectAckermann(result.twist, geometry_msgs::msg::Twist());
}

TEST_F(ControllerTest, CostmapBoundaryViolationIsRejected)
{
  start();
  auto path = line();
  for (auto & point : path.poses) {point.pose.position.x += 11.0;}
  controller_->setPlan(path);
  EXPECT_THROW(command(poseAt(11.0)), nav2_core::NoValidControl);
}

TEST_F(ControllerTest, ForwardSyntheticClosedLoopReachesAndStops)
{
  start({rclcpp::Parameter("FollowPath.motion_direction_mode", 1),
    rclcpp::Parameter("FollowPath.global_plan_overwrite_orientation", false)});
  closedLoop(line(3.0), 1, 250);
}

TEST_F(ControllerTest, ReverseSyntheticClosedLoopReachesAndStops)
{
  start({rclcpp::Parameter("FollowPath.motion_direction_mode", -1),
    rclcpp::Parameter("FollowPath.global_plan_overwrite_orientation", false)});
  closedLoop(line(2.0, -1), -1, 250);
}

TEST_F(ControllerTest, AutomaticDirectionCanReverseWithoutChangingBodyYaw)
{
  start({rclcpp::Parameter("FollowPath.global_plan_overwrite_orientation", false)});
  controller_->setPlan(line(2.0, -1));
  const auto result = command();
  EXPECT_LT(result.twist.linear.x, 0.0);
  expectAckermann(result.twist, geometry_msgs::msg::Twist());
}

TEST_F(ControllerTest, CurvedSyntheticClosedLoopRespectsAckermannBounds)
{
  start({rclcpp::Parameter("FollowPath.motion_direction_mode", 1),
    rclcpp::Parameter("FollowPath.global_plan_overwrite_orientation", false)});
  nav_msgs::msg::Path path; path.header.frame_id = "map";
  for (double yaw = 0.0; yaw <= 1.2 + 1e-8; yaw += 0.04) {
    path.poses.push_back(poseAt(2.0 * std::sin(yaw), 2.0 * (1.0 - std::cos(yaw)), yaw));
  }
  closedLoop(path, 1, 300);
}

TEST_F(ControllerTest, CancelBrakesUntilFeedbackStopsAndNewPlanResumes)
{
  start(); controller_->setPlan(line());
  geometry_msgs::msg::Twist velocity; velocity.linear.x = 0.5;
  // First record real feedback through the normal control API.
  velocity = command(poseAt(), velocity).twist;
  EXPECT_FALSE(controller_->cancel());
  bool stopped = false;
  for (int step = 0; step < 100; ++step) {
    const auto result = command(poseAt(), velocity);
    expectAckermann(result.twist, velocity);
    EXPECT_GE(result.twist.linear.x, -1e-8);
    EXPECT_LE(result.twist.linear.x, velocity.linear.x + 1e-8);
    velocity = result.twist;
    if (controller_->cancel()) {stopped = true; break;}
  }
  ASSERT_TRUE(stopped);
  EXPECT_LE(std::abs(velocity.linear.x), 0.05);
  EXPECT_LE(std::abs(velocity.angular.z), 0.05);
  controller_->setPlan(line(4.0));
  EXPECT_GT(command().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, ResetDiscardsPlanAndCancellationState)
{
  start(); controller_->setPlan(line());
  controller_->cancel();
  controller_->reset();
  EXPECT_THROW(command(), nav2_core::InvalidPath);
  controller_->setPlan(line());
  EXPECT_GT(command().twist.linear.x, 0.0);
}

class InvalidParameterTest : public ControllerTest,
  public ::testing::WithParamInterface<rclcpp::Parameter> {};

TEST_P(InvalidParameterTest, ConfigureRejectsInvalidValue)
{
  const auto & parameter = GetParam();
  SCOPED_TRACE(parameter.get_name());
  EXPECT_THROW(configure({parameter}), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  SafetyAndOptimizerBounds, InvalidParameterTest,
  ::testing::Values(
    rclcpp::Parameter("FollowPath.wheelbase", 0.0),
    rclcpp::Parameter("FollowPath.min_turning_radius", -1.0),
    rclcpp::Parameter("FollowPath.max_vel_x", -0.1),
    rclcpp::Parameter("FollowPath.acc_lim_x", 0.0),
    rclcpp::Parameter("FollowPath.decel_lim_x", 0.0),
    rclcpp::Parameter("FollowPath.max_steering_rate", 0.0),
    rclcpp::Parameter("FollowPath.horizon", 0.0),
    rclcpp::Parameter("FollowPath.stopping_delay", -0.1),
    rclcpp::Parameter("FollowPath.samples", 0),
    rclcpp::Parameter("FollowPath.iterations", 0),
    rclcpp::Parameter("FollowPath.knots", 0),
    rclcpp::Parameter("FollowPath.motion_direction_mode", 2),
    rclcpp::Parameter("FollowPath.elite_fraction", 0.0),
    rclcpp::Parameter("FollowPath.mean_rate", 1.1),
    rclcpp::Parameter("FollowPath.transform_tolerance", -0.1),
    rclcpp::Parameter("FollowPath.max_robot_pose_search_dist", 0.0),
    rclcpp::Parameter("FollowPath.max_plan_poses", 0),
    rclcpp::Parameter("FollowPath.wheelbase", std::numeric_limits<double>::quiet_NaN())));
}  // namespace
