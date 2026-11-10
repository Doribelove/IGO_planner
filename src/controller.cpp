// Copyright 2026 IGO contributors
// SPDX-License-Identifier: BSD-3-Clause
#include "igo_nav2_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace igo_nav2_controller
{
namespace
{
using igo_local_planner::Point;
using igo_local_planner::State;
constexpr double kPi = 3.14159265358979323846;
constexpr double kStopped = 1e-3;
constexpr std::size_t kMaxMapCells = 10000000;

void range(const std::string & name, double value, double low, double high)
{
  if (!std::isfinite(value) || value < low || value > high) {
    throw nav2_core::ControllerException(
            "IGO parameter '" + name + "' must be finite and in [" +
            std::to_string(low) + ", " + std::to_string(high) + "]");
  }
}

template<typename T>
T parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "IGO startup configuration; restart the controller node with new overrides to change.";
    node->declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  }
  return node->get_parameter(name).get_value<T>();
}

bool normalizePose(geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  auto & q = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
    std::abs(p.x) > 1e9 || std::abs(p.y) > 1e9 || std::abs(p.z) > 1e6)
  {
    return false;
  }
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  // Reject missing/invalid orientation instead of treating an all-zero quaternion as yaw=0.
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    return false;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return true;
}

double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool validFrame(const std::string & frame)
{
  return !frame.empty() && frame.front() != '/' &&
         frame.find_first_of(" \t\n\r") == std::string::npos;
}

bool finiteTwist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

std::vector<Point> checkedFootprint(const std::vector<geometry_msgs::msg::Point> & input)
{
  std::vector<Point> polygon;
  for (const auto & p : input) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      std::hypot(p.x, p.y) > 100.0)
    {
      throw nav2_core::ControllerException("IGO footprint contains an invalid or oversized point");
    }
    polygon.push_back({p.x, p.y});
  }
  if (polygon.size() > 1 && std::hypot(
      polygon.front().x - polygon.back().x, polygon.front().y - polygon.back().y) < 1e-9)
  {
    polygon.pop_back();
  }
  if (polygon.size() < 3 || polygon.size() > 128) {
    throw nav2_core::ControllerException("IGO requires a convex footprint with 3 to 128 vertices");
  }
  double area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    if (std::hypot(b.x - a.x, b.y - a.y) < 1e-9) {
      throw nav2_core::ControllerException("IGO footprint has repeated adjacent vertices");
    }
    area += a.x * b.y - b.x * a.y;
  }
  if (std::abs(area) < 1e-9) {
    throw nav2_core::ControllerException("IGO footprint has zero area");
  }
  // All vertices must lie on the same side of every edge. This also rejects
  // self-intersecting/star-shaped input, which a local turn-sign test can miss.
  const double sign = area > 0.0 ? 1.0 : -1.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    for (const auto & c : polygon) {
      if (sign * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) < -1e-9) {
        throw nav2_core::ControllerException("IGO footprint must be an ordered convex polygon");
      }
    }
  }
  return polygon;
}

State interpolate(const State & a, const State & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
    igo_local_planner::angle(a.yaw + t * igo_local_planner::angle(b.yaw - a.yaw)), 0.0, 0.0};
}
}  // namespace

void IGOController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (configured_) {
    throw nav2_core::InvalidController("IGO is already configured; cleanup before reconfiguration");
  }
  auto node = parent.lock();
  if (!node || !tf || !costmap_ros || !costmap_ros->getLayeredCostmap() ||
    !costmap_ros->getCostmap() || name.empty())
  {
    throw nav2_core::InvalidController("IGO configure requires node, TF, configured costmap, and name");
  }
  const std::string prefix = name + ".";
  igo_local_planner::Parameters p;
  try {
    auto real = [&](const std::string & key, double value) {
        return parameter<double>(node, prefix + key, value);
      };
    auto flag = [&](const std::string & key, bool value) {
        return parameter<bool>(node, prefix + key, value);
      };
    auto integer = [&](const std::string & key, int value, int low, int high) {
        const auto result = parameter<int64_t>(node, prefix + key, value);
        range(key, static_cast<double>(result), low, high);
        return static_cast<int>(result);
      };
    if (!node->has_parameter("controller_frequency")) {
      // The real controller server declares this. Declare only for standalone
      // hosts as well, so NodeOptions overrides are honored rather than lost.
      node->declare_parameter("controller_frequency", 20.0);
    }
    const double frequency = node->get_parameter("controller_frequency").as_double();
    range("controller_frequency", frequency, 1.0, 1000.0);
    p.period = 1.0 / frequency;
    p.forward = real("max_vel_x", p.forward);
    p.reverse = real("max_vel_x_backwards", p.reverse);
    p.angular = real("max_vel_theta", p.angular);
    p.acceleration = real("acc_lim_x", p.acceleration);
    p.deceleration = real("decel_lim_x", p.deceleration);
    p.stopping_delay = real("stopping_delay", p.stopping_delay);
    p.angular_acceleration = real("acc_lim_theta", p.angular_acceleration);
    p.wheelbase = real("wheelbase", p.wheelbase);
    p.radius = real("min_turning_radius", p.radius);
    p.steering_rate = real("max_steering_rate", p.steering_rate);
    p.steering_weight = real("steering_change_weight", p.steering_weight);
    p.xy_tolerance = real("xy_goal_tolerance", p.xy_tolerance);
    p.yaw_tolerance = real("yaw_goal_tolerance", p.yaw_tolerance);
    p.lookahead = real("max_global_plan_lookahead_dist", p.lookahead);
    p.path_weight = real("path_weight", p.path_weight);
    p.heading_weight = real("heading_weight", p.heading_weight);
    p.obstacle_weight = real("obstacle_weight", p.obstacle_weight);
    p.progress_weight = real("progress_weight", p.progress_weight);
    p.horizon = real("horizon", p.horizon);
    p.mean_rate = real("mean_rate", p.mean_rate);
    p.variance_rate = real("variance_rate", p.variance_rate);
    p.elite_fraction = real("elite_fraction", p.elite_fraction);
    p.direction = integer("motion_direction_mode", p.direction, -1, 1);
    p.samples = integer("samples", p.samples, 4, 512);
    p.iterations = integer("iterations", p.iterations, 1, 64);
    p.knots = integer("knots", p.knots, 2, 128);
    p.allow_reverse = flag("allow_init_with_backwards_motion", p.allow_reverse);
    p.overwrite_orientation = flag("global_plan_overwrite_orientation", p.overwrite_orientation);
    p.unknown_obstacle = flag("treat_unknown_as_obstacle", p.unknown_obstacle);
    p.goal_yaw_required = flag("goal_yaw_required", p.goal_yaw_required);
    const double transform_tolerance = real("transform_tolerance", 0.1);
    const double steering_timeout = real("steering_timeout", 0.5);
    const double search_dist = real("max_robot_pose_search_dist", 6.0);
    const double path_distance = real("max_path_distance", 2.0);
    const int max_plan_poses = integer("max_plan_poses", 100000, 1, 1000000);
    const auto steering_topic = parameter<std::string>(node, prefix + "steering_topic", "");

    range("max_vel_x", p.forward, 0.0, 20.0);
    range("max_vel_x_backwards", p.reverse, 0.0, 20.0);
    range("max_vel_theta", p.angular, 0.001, 20.0);
    range("acc_lim_x", p.acceleration, 0.01, 20.0);
    range("decel_lim_x", p.deceleration, 0.01, 20.0);
    range("stopping_delay", p.stopping_delay, 0.0, 10.0);
    range("acc_lim_theta", p.angular_acceleration, 0.001, 20.0);
    range("wheelbase", p.wheelbase, 0.01, 20.0);
    range("min_turning_radius", p.radius, 0.01, 100.0);
    range("max_steering_rate", p.steering_rate, 0.001, 20.0);
    range("xy_goal_tolerance", p.xy_tolerance, 0.001, 20.0);
    range("yaw_goal_tolerance", p.yaw_tolerance, 0.0, kPi);
    range("max_global_plan_lookahead_dist", p.lookahead, 0.01, 100.0);
    for (const auto & weight : std::vector<std::pair<std::string, double>>{
        {"steering_change_weight", p.steering_weight}, {"path_weight", p.path_weight},
        {"heading_weight", p.heading_weight}, {"obstacle_weight", p.obstacle_weight},
        {"progress_weight", p.progress_weight}})
    {
      range(weight.first, weight.second, 0.0, 10000.0);
    }
    range("horizon", p.horizon, std::max(0.05, p.period), 30.0);
    range("mean_rate", p.mean_rate, 0.001, 1.0);
    range("variance_rate", p.variance_rate, 0.001, 1.0);
    range("elite_fraction", p.elite_fraction, 1.0 / p.samples, 1.0);
    range("transform_tolerance", transform_tolerance, 0.0, 5.0);
    range("steering_timeout", steering_timeout, 0.001, 10.0);
    range("max_robot_pose_search_dist", search_dist, 0.01, 1000.0);
    range("max_path_distance", path_distance, 0.01, 100.0);
    if (p.samples * p.iterations * p.knots > 2000000) {
      throw nav2_core::ControllerException("IGO optimizer sample/iteration/knot product is too large");
    }
    const bool forward_enabled = p.direction >= 0 && p.forward > 0.0;
    const bool reverse_enabled = p.direction <= 0 && p.allow_reverse && p.reverse > 0.0;
    if ((!forward_enabled && !reverse_enabled) ||
      (p.direction < 0 && (!p.allow_reverse || p.reverse == 0.0)))
    {
      throw nav2_core::ControllerException("IGO direction and velocity limits disable all motion");
    }
    const auto global_frame = costmap_ros->getGlobalFrameID();
    const auto base_frame = costmap_ros->getBaseFrameID();
    if (!validFrame(global_frame) || !validFrame(base_frame)) {
      throw nav2_core::ControllerException("IGO costmap global/base frame must be valid TF frame IDs");
    }
    // Jazzy's Costmap2DROS footprint getter has no synchronization against its
    // footprint-topic/dynamic-parameter writers. Read geometry only at configure,
    // as part of the lifecycle setup; runtime footprint changes are unsupported.
    std::vector<Point> footprint;
    {
      auto * map = costmap_ros->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> guard(*map->getMutex());
      footprint = checkedFootprint(costmap_ros->getRobotFootprint());
    }
    auto local_pub = node->create_publisher<nav_msgs::msg::Path>(name + "/local_plan", 1);
    auto global_pub = node->create_publisher<nav_msgs::msg::Path>(name + "/global_plan", 1);
    auto feedback = std::make_shared<SteeringFeedback>();
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription;
    if (!steering_topic.empty()) {
      const std::weak_ptr<SteeringFeedback> weak_feedback(feedback);
      const double max_angle = std::atan(p.wheelbase / p.radius);
      subscription = node->create_subscription<std_msgs::msg::Float64>(
        steering_topic, rclcpp::SensorDataQoS().keep_last(1),
        [weak_feedback, max_angle](const std_msgs::msg::Float64::SharedPtr msg) {
          auto state = weak_feedback.lock();
          if (!state || !std::isfinite(msg->data) || std::abs(msg->data) > max_angle + 1e-6) {
            return;
          }
          std::lock_guard<std::mutex> guard(state->mutex);
          if (!state->enabled) {
            return;
          }
          state->angle = msg->data;
          state->stamp = std::chrono::steady_clock::now();
          state->received = true;
        });
    }
    auto parameter_guard = node->add_on_set_parameters_callback(
      [prefix](const std::vector<rclcpp::Parameter> & updates) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & update : updates) {
          if (update.get_name().compare(0, prefix.size(), prefix) == 0 ||
            update.get_name() == "controller_frequency")
          {
            result.successful = false;
            result.reason = "IGO startup parameters are fixed; restart the controller node with updated configuration";
            break;
          }
        }
        return result;
      });

    plugin_name_ = std::move(name);
    global_frame_ = global_frame;
    base_frame_ = base_frame;
    parameters_ = p;
    transform_tolerance_ = transform_tolerance;
    steering_timeout_ = steering_timeout;
    max_robot_pose_search_dist_ = search_dist;
    max_path_distance_ = path_distance;
    max_plan_poses_ = static_cast<std::size_t>(max_plan_poses);
    footprint_ = std::move(footprint);
    local_plan_pub_ = std::move(local_pub);
    global_plan_pub_ = std::move(global_pub);
    steering_feedback_ = std::move(feedback);
    steering_sub_ = std::move(subscription);
    parameter_guard_ = std::move(parameter_guard);
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    speed_scale_ = 1.0;
    absolute_speed_limit_ = 0.0;
    clearPlan();
    configured_ = true;
    active_ = false;
    RCLCPP_INFO(
      logger_, "%s configured for Nav2 Jazzy, %.1f Hz, fixed padded footprint (%zu vertices)",
      plugin_name_.c_str(), frequency, footprint_.size());
  } catch (const nav2_core::ControllerException &) {
    throw;
  } catch (const std::exception & error) {
    throw nav2_core::ControllerException(std::string("IGO configure failed: ") + error.what());
  }
}

void IGOController::requireConfigured() const
{
  if (!configured_) {
    throw nav2_core::InvalidController("IGO controller has not been configured");
  }
}

void IGOController::clearPlan()
{
  plan_ = nav_msgs::msg::Path();
  cursor_ = 0;
  goal_braking_ = false;
  cancelling_ = false;
  cancel_complete_ = false;
  optimizer_.reset();
}

void IGOController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (steering_feedback_) {
    std::lock_guard<std::mutex> guard(steering_feedback_->mutex);
    steering_feedback_->enabled = false;
    steering_feedback_->received = false;
  }
  if (active_) {
    local_plan_pub_->on_deactivate();
    global_plan_pub_->on_deactivate();
  }
  active_ = false;
  configured_ = false;
  clearPlan();
  steering_sub_.reset();
  steering_feedback_.reset();
  parameter_guard_.reset();
  local_plan_pub_.reset();
  global_plan_pub_.reset();
  costmap_ros_.reset();
  tf_.reset();
  clock_.reset();
  footprint_.clear();
}

void IGOController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  requireConfigured();
  if (active_) {
    return;
  }
  local_plan_pub_->on_activate();
  global_plan_pub_->on_activate();
  {
    std::lock_guard<std::mutex> guard(steering_feedback_->mutex);
    steering_feedback_->received = false;
    steering_feedback_->enabled = true;
  }
  active_ = true;
}

void IGOController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || !active_) {
    return;
  }
  active_ = false;
  local_plan_pub_->on_deactivate();
  global_plan_pub_->on_deactivate();
  {
    std::lock_guard<std::mutex> guard(steering_feedback_->mutex);
    steering_feedback_->enabled = false;
    steering_feedback_->received = false;
  }
  clearPlan();
}

void IGOController::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  clearPlan();
}

void IGOController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  requireConfigured();
  // A rejected replacement must not leave a stale executable plan installed.
  clearPlan();
  if (path.poses.empty() || path.poses.size() > max_plan_poses_ ||
    !validFrame(path.header.frame_id))
  {
    throw nav2_core::InvalidPath("IGO requires a nonempty bounded path with a valid header frame");
  }
  auto plan = path;
  for (auto & pose : plan.poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = path.header.frame_id;
    }
    if (pose.header.frame_id != path.header.frame_id || !normalizePose(pose.pose)) {
      throw nav2_core::InvalidPath("IGO path poses must have finite coordinates, unit quaternions, and one frame");
    }
  }
  plan_ = std::move(plan);
}

geometry_msgs::msg::PoseStamped IGOController::transformPose(
  const geometry_msgs::msg::PoseStamped & input) const
{
  auto pose = input;
  if (!validFrame(pose.header.frame_id) || !normalizePose(pose.pose)) {
    throw nav2_core::ControllerTFError("IGO current pose has an invalid frame, coordinate, or quaternion");
  }
  try {
    if (pose.header.frame_id != global_frame_) {
      const auto transform = tf_->lookupTransform(
        global_frame_, pose.header.frame_id,
        rclcpp::Time(pose.header.stamp, clock_->get_clock_type()),
        rclcpp::Duration::from_seconds(transform_tolerance_));
      tf2::doTransform(pose, pose, transform);
      pose.header.stamp = input.header.stamp;
    }
  } catch (const tf2::TransformException & error) {
    throw nav2_core::ControllerTFError(std::string("IGO current-pose TF failed: ") + error.what());
  }
  if (!normalizePose(pose.pose)) {
    throw nav2_core::ControllerTFError("IGO transformed current pose is not finite and normalized");
  }
  return pose;
}

std::vector<State> IGOController::makeReference(
  const geometry_msgs::msg::PoseStamped & pose, bool & terminal)
{
  std::vector<State> transformed;
  transformed.reserve(plan_.poses.size());
  geometry_msgs::msg::TransformStamped transform;
  const bool needs_transform = plan_.header.frame_id != global_frame_;
  try {
    if (needs_transform) {
      // Path geometry is static in its own frame. Use the control pose time,
      // not the potentially obsolete global-plan publication time. Zero means latest.
      transform = tf_->lookupTransform(
        global_frame_, plan_.header.frame_id,
        rclcpp::Time(pose.header.stamp, clock_->get_clock_type()),
        rclcpp::Duration::from_seconds(transform_tolerance_));
    }
    for (auto point : plan_.poses) {
      if (needs_transform) {
        tf2::doTransform(point, point, transform);
      }
      if (!normalizePose(point.pose)) {
        throw nav2_core::InvalidPath("IGO transformed path contains invalid geometry");
      }
      transformed.push_back({point.pose.position.x, point.pose.position.y,
          yaw(point.pose.orientation), 0.0, 0.0});
    }
  } catch (const tf2::TransformException & error) {
    throw nav2_core::ControllerTFError(std::string("IGO path TF failed: ") + error.what());
  }
  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;
  std::size_t nearest = std::min(cursor_, transformed.size() - 1);
  State projected = transformed[nearest];
  double best = std::hypot(robot_x - projected.x, robot_y - projected.y);
  double walked = 0.0;
  for (std::size_t i = nearest; i + 1 < transformed.size(); ++i) {
    const auto & a = transformed[i];
    const auto & b = transformed[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    const double max_t = length > 1e-9 ?
      std::min(1.0, std::max(0.0, max_robot_pose_search_dist_ - walked) / length) : 1.0;
    const double t = length > 1e-9 ? std::clamp(
      ((robot_x - a.x) * dx + (robot_y - a.y) * dy) / (length * length), 0.0, max_t) : 0.0;
    const State candidate = interpolate(a, b, t);
    const double distance = std::hypot(robot_x - candidate.x, robot_y - candidate.y);
    if (distance < best) {
      best = distance;
      nearest = i;
      projected = candidate;
    }
    walked += length;
    if (walked >= max_robot_pose_search_dist_) {
      break;
    }
  }
  if (best > max_path_distance_) {
    throw nav2_core::InvalidPath("IGO robot is too far from the ordered reference path");
  }
  cursor_ = nearest;
  std::vector<State> reference{projected};
  double length = 0.0;
  terminal = nearest == transformed.size() - 1;
  for (std::size_t i = nearest + 1; i < transformed.size(); ++i) {
    const auto & a = reference.back();
    const auto & b = transformed[i];
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    if (length + segment > parameters_.lookahead) {
      reference.push_back(interpolate(a, b, (parameters_.lookahead - length) / segment));
      terminal = false;
      break;
    }
    reference.push_back(b);
    length += segment;
    terminal = i == transformed.size() - 1;
  }
  if (parameters_.overwrite_orientation) {
    for (std::size_t i = 0; i + 1 < reference.size(); ++i) {
      const double dx = reference[i + 1].x - reference[i].x;
      const double dy = reference[i + 1].y - reference[i].y;
      if (std::hypot(dx, dy) > 1e-9) {
        reference[i].yaw = std::atan2(dy, dx);
      }
    }
    if (!terminal && reference.size() > 1) {
      reference.back().yaw = reference[reference.size() - 2].yaw;
    }
  }
  return reference;
}

igo_local_planner::Grid IGOController::snapshotCostmap() const
{
  igo_local_planner::Grid grid;
  auto * map = costmap_ros_->getCostmap();
  if (!map || !map->getMutex()) {
    throw nav2_core::NoValidControl("IGO costmap is unavailable");
  }
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> guard(*map->getMutex());
    grid.width = map->getSizeInCellsX();
    grid.height = map->getSizeInCellsY();
    grid.resolution = map->getResolution();
    grid.origin_x = map->getOriginX();
    grid.origin_y = map->getOriginY();
    grid.unknown_obstacle = parameters_.unknown_obstacle;
    // Core integral-image indices are unsigned. Bound both allocation and all
    // intermediate products before invoking it; never multiply unchecked dimensions.
    const std::uint64_t cells = static_cast<std::uint64_t>(grid.width) * grid.height;
    const std::uint64_t integral_cells =
      (static_cast<std::uint64_t>(grid.width) + 1) * (static_cast<std::uint64_t>(grid.height) + 1);
    const double max_x = grid.origin_x + static_cast<double>(grid.width) * grid.resolution;
    const double max_y = grid.origin_y + static_cast<double>(grid.height) * grid.resolution;
    if (!grid.width || !grid.height || cells > kMaxMapCells ||
      integral_cells > std::numeric_limits<unsigned>::max() ||
      !std::isfinite(grid.resolution) || grid.resolution < 1e-6 || grid.resolution > 1000.0 ||
      !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y) ||
      std::abs(grid.origin_x) > 1e9 || std::abs(grid.origin_y) > 1e9 ||
      !std::isfinite(max_x) || !std::isfinite(max_y) ||
      max_x <= grid.origin_x || max_y <= grid.origin_y || !map->getCharMap())
    {
      throw nav2_core::NoValidControl("IGO costmap dimensions, resolution, origin, or storage are invalid");
    }
    grid.cells.assign(map->getCharMap(), map->getCharMap() + static_cast<std::size_t>(cells));
  }
  grid.prepare();
  return grid;
}

geometry_msgs::msg::TwistStamped IGOController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & input_pose,
  const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  requireConfigured();
  if (!active_) {
    throw nav2_core::InvalidController("IGO cannot compute commands while inactive");
  }
  if (plan_.poses.empty()) {
    throw nav2_core::InvalidPath("IGO has no valid global plan");
  }
  if (!goal_checker) {
    throw nav2_core::InvalidController("IGO requires the current Nav2 GoalChecker");
  }
  if (!finiteTwist(velocity) || std::abs(velocity.linear.x) > 40.0 ||
    std::abs(velocity.angular.z) > 40.0 || std::abs(velocity.linear.y) > 0.1)
  {
    throw nav2_core::NoValidControl("IGO received invalid or non-Ackermann base-frame velocity feedback");
  }
  if (costmap_ros_->getGlobalFrameID() != global_frame_ ||
    costmap_ros_->getBaseFrameID() != base_frame_)
  {
    throw nav2_core::InvalidController("IGO costmap frames changed; cleanup/configure is required");
  }
  const auto pose = transformPose(input_pose);
  bool terminal = false;
  auto reference = makeReference(pose, terminal);
  State start{pose.pose.position.x, pose.pose.position.y, yaw(pose.pose.orientation),
    velocity.linear.x, velocity.angular.z};
  {
    std::lock_guard<std::mutex> guard(steering_feedback_->mutex);
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - steering_feedback_->stamp).count();
    if (steering_feedback_->received && age >= 0.0 && age <= steering_timeout_) {
      start.steering = steering_feedback_->angle;
    }
  }
  // Measured yaw rate is never silently replaced with wheel-derived yaw rate.
  // Feedback disagreement can make a rollout infeasible and must be surfaced.
  auto p = parameters_;
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    const double x = pose_tolerance.position.x;
    const double y = pose_tolerance.position.y;
    if (std::isfinite(x) && x >= 0.0 && std::isfinite(y) && y >= 0.0) {
      p.xy_tolerance = std::max(1e-6, std::min(x, y));
    }
    geometry_msgs::msg::Pose tolerance_pose;
    tolerance_pose.orientation = pose_tolerance.orientation;
    if (normalizePose(tolerance_pose)) {
      p.yaw_tolerance = std::abs(yaw(tolerance_pose.orientation));
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "IGO could not read GoalChecker tolerances; using configured approach tolerances");
  }
  p.forward *= speed_scale_;
  p.reverse *= speed_scale_;
  if (absolute_speed_limit_ > 0.0) {
    p.forward = std::min(p.forward, absolute_speed_limit_);
    p.reverse = std::min(p.reverse, absolute_speed_limit_);
  }
  const auto & goal = reference.back();
  const bool in_goal_region = terminal &&
    std::hypot(start.x - goal.x, start.y - goal.y) <= p.xy_tolerance &&
    (!p.goal_yaw_required || std::abs(igo_local_planner::angle(start.yaw - goal.yaw)) <= p.yaw_tolerance);
  const bool stopped = std::abs(start.v) <= kStopped && std::abs(start.omega) <= kStopped;
  if (goal_braking_ && stopped && !in_goal_region) {
    goal_braking_ = false;
  }
  if (in_goal_region) {
    goal_braking_ = true;
  }
  if (goal_braking_ || cancelling_) {
    p.forward = 0.0;
    p.reverse = 0.0;
  }
  auto grid = snapshotCostmap();
  // Reject an out-of-map current footprint before the core converts its
  // bounding box to signed grid indices. This also avoids out-of-range float
  // to int conversions for valid-but-very-distant robot coordinates.
  for (const auto & point : footprint_) {
    const double x = start.x + std::cos(start.yaw) * point.x - std::sin(start.yaw) * point.y;
    const double y = start.y + std::sin(start.yaw) * point.x + std::cos(start.yaw) * point.y;
    const double cell_x = (x - grid.origin_x) / grid.resolution;
    const double cell_y = (y - grid.origin_y) / grid.resolution;
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) || cell_x < 0.0 || cell_y < 0.0 ||
      cell_x >= grid.width || cell_y >= grid.height)
    {
      throw nav2_core::NoValidControl("IGO current footprint is outside the local costmap");
    }
  }
  auto result = optimizer_.solve(start, reference, grid, footprint_, p, terminal);
  if (!result.valid) {
    throw nav2_core::NoValidControl("IGO: " + result.reason);
  }
  const double v = result.command.v;
  const double omega = result.omega;
  const double previous_angle = std::isfinite(start.steering) ? start.steering :
    (std::abs(start.v) > 1e-8 ? std::atan(p.wheelbase * start.omega / start.v) : 0.0);
  const double next_angle = std::abs(v) > 1e-8 ? std::atan(p.wheelbase * omega / v) : previous_angle;
  const double tolerance = 1e-6;
  // Validate, do not clip: clipping a single component after collision checking
  // would invalidate the Ackermann geometry and the checked stopping continuation.
  if (!std::isfinite(v) || !std::isfinite(omega) || !std::isfinite(result.cost) ||
    std::abs(v - start.v) > p.acceleration * p.period + tolerance ||
    std::abs(omega - start.omega) > p.angular_acceleration * p.period + tolerance ||
    std::abs(omega) > p.angular + tolerance || std::abs(omega) > std::abs(v) / p.radius + tolerance ||
    v > std::max(start.v, p.direction >= 0 ? p.forward : 0.0) + tolerance ||
    v < std::min(start.v, p.direction <= 0 && p.allow_reverse ? -p.reverse : 0.0) - tolerance ||
    std::abs(next_angle - previous_angle) > p.steering_rate * p.period + tolerance ||
    (start.v * v < -tolerance))
  {
    optimizer_.reset();
    throw nav2_core::NoValidControl("IGO result violates finite-output or Ackermann transition constraints");
  }
  cancel_complete_ = cancelling_ && stopped && std::abs(v) <= kStopped && std::abs(omega) <= kStopped;
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = base_frame_;
  command.header.stamp = clock_->now();
  command.twist.linear.x = v;
  command.twist.angular.z = omega;
  publishPaths(result, reference, rclcpp::Time(command.header.stamp, clock_->get_clock_type()));
  return command;
}

void IGOController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  requireConfigured();
  if (!std::isfinite(speed_limit) || speed_limit < 0.0 || (percentage && speed_limit > 100.0)) {
    throw nav2_core::ControllerException("IGO speed limit must be finite, nonnegative, and at most 100 percent");
  }
  // Nav2's 0.0 sentinel means NO_SPEED_LIMIT, not an emergency stop request.
  speed_scale_ = 1.0;
  absolute_speed_limit_ = 0.0;
  if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
    if (percentage) {
      speed_scale_ = speed_limit / 100.0;
    } else {
      absolute_speed_limit_ = speed_limit;
    }
  }
  // Limits apply inside the core before rollout/collision checking. When the
  // measured speed exceeds a new limit, braking is gradual under core dynamics.
  optimizer_.reset();
}

bool IGOController::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || !active_) {
    return true;
  }
  if (!cancelling_) {
    cancelling_ = true;
    cancel_complete_ = false;
    optimizer_.reset();
    return false;  // obtain fresh velocity feedback in at least one braking compute cycle
  }
  return cancel_complete_;
}

void IGOController::publishPaths(
  const igo_local_planner::Result & result, const std::vector<State> & reference,
  const rclcpp::Time & stamp)
{
  auto path = [&](const std::vector<State> & states) {
      nav_msgs::msg::Path message;
      message.header.frame_id = global_frame_;
      message.header.stamp = stamp;
      for (const auto & state : states) {
        geometry_msgs::msg::PoseStamped point;
        point.header = message.header;
        point.pose.position.x = state.x;
        point.pose.position.y = state.y;
        point.pose.orientation.z = std::sin(0.5 * state.yaw);
        point.pose.orientation.w = std::cos(0.5 * state.yaw);
        message.poses.push_back(point);
      }
      return message;
    };
  if (local_plan_pub_->get_subscription_count() > 0) {
    local_plan_pub_->publish(path(result.trajectory));
  }
  if (global_plan_pub_->get_subscription_count() > 0) {
    global_plan_pub_->publish(path(reference));
  }
}

}  // namespace igo_nav2_controller

PLUGINLIB_EXPORT_CLASS(igo_nav2_controller::IGOController, nav2_core::Controller)
