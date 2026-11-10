// Copyright 2026 IGO contributors
// SPDX-License-Identifier: BSD-3-Clause
#ifndef IGO_NAV2_CONTROLLER__CONTROLLER_HPP_
#define IGO_NAV2_CONTROLLER__CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "igo_local_planner/core.h"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace igo_nav2_controller
{

/// Nav2 Jazzy adapter. Parameters are startup-only; changing them requires node restart.
/// Padded footprint is captured at configure; change geometry only while unconfigured.
/// The velocity reference frame is the costmap's robot base frame (Ackermann rear axle).
class IGOController : public nav2_core::Controller
{
public:
  IGOController() = default;
  ~IGOController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  bool cancel() override;
  void reset() override;

private:
  struct SteeringFeedback
  {
    std::mutex mutex;
    bool enabled{false};
    bool received{false};
    double angle{0.0};
    std::chrono::steady_clock::time_point stamp{};
  };

  void requireConfigured() const;
  void clearPlan();  // caller holds mutex_
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose) const;
  std::vector<igo_local_planner::State> makeReference(
    const geometry_msgs::msg::PoseStamped & pose, bool & terminal);
  igo_local_planner::Grid snapshotCostmap() const;
  void publishPaths(
    const igo_local_planner::Result & result,
    const std::vector<igo_local_planner::State> & reference, const rclcpp::Time & stamp);

  // Serializes lifecycle, plan, solve, cancellation, and speed-limit updates.
  std::mutex mutex_;
  bool configured_{false};
  bool active_{false};
  bool goal_braking_{false};
  bool cancelling_{false};
  bool cancel_complete_{false};
  std::size_t cursor_{0};
  std::size_t max_plan_poses_{100000};
  double transform_tolerance_{0.1};
  double steering_timeout_{0.5};
  double max_robot_pose_search_dist_{6.0};
  double max_path_distance_{2.0};
  double speed_scale_{1.0};
  double absolute_speed_limit_{0.0};
  std::string plugin_name_;
  std::string global_frame_;
  std::string base_frame_;
  nav_msgs::msg::Path plan_;
  igo_local_planner::Parameters parameters_;
  igo_local_planner::Optimizer optimizer_;
  std::vector<igo_local_planner::Point> footprint_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("igo_nav2_controller")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<SteeringFeedback> steering_feedback_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
};

}  // namespace igo_nav2_controller
#endif  // IGO_NAV2_CONTROLLER__CONTROLLER_HPP_
