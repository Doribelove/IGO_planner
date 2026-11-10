#pragma once
#include "igo_local_planner/core.h"
#include <igo_local_planner/IGOConfig.h>
#include <nav_core/base_local_planner.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <mutex>
#include <memory>

namespace igo_local_planner {
class IGOLocalPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  IGOLocalPlannerROS()=default;
  void initialize(std::string,tf2_ros::Buffer*,costmap_2d::Costmap2DROS*) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>&) override;
  bool computeVelocityCommands(geometry_msgs::Twist&) override;
  bool isGoalReached() override;
 private:
  void reconfigure(IGOConfig&,uint32_t);
  void odometry(const nav_msgs::Odometry::ConstPtr&);
  void steering(const std_msgs::Float64::ConstPtr&);
  bool reference(const geometry_msgs::PoseStamped&,std::vector<State>&,bool&);
  void publish(const Result&,const std::vector<State>&,const ros::Time&);
  std::mutex mutex_;
  bool initialized_=false,reached_=false;
  bool goal_braking_=false;
  Parameters params_;
  Optimizer optimizer_;
  tf2_ros::Buffer* tf_=nullptr;
  costmap_2d::Costmap2DROS* costmap_=nullptr;
  std::vector<geometry_msgs::PoseStamped> plan_;
  size_t cursor_=0;
  State twist_;
  ros::WallTime steering_received_;
  std::string frame_;
  std::unique_ptr<dynamic_reconfigure::Server<IGOConfig>> server_;
  ros::Subscriber odom_sub_,steering_sub_;
  ros::Publisher local_pub_,global_pub_,legacy_local_pub_,legacy_global_pub_,status_pub_;
};
}
