#include "igo_local_planner/ros_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(igo_local_planner::IGOLocalPlannerROS,nav_core::BaseLocalPlanner)
namespace igo_local_planner {
void IGOLocalPlannerROS::initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* map) {
  if(initialized_) return;
  tf_=tf;costmap_=map;frame_=map->getGlobalFrameID();ros::NodeHandle nh("~/"+name),parent("~");
  // Preserve the resolved existing launch profile (including CLI overrides).
  // These are shared chassis/navigation settings, not a TEB solver dependency.
  ros::NodeHandle old("~/TebLocalPlannerROS");
  for(const std::string& key:{"max_vel_x","max_vel_x_backwards","max_vel_theta","acc_lim_x","acc_lim_theta",
      "wheelbase","min_turning_radius","xy_goal_tolerance","yaw_goal_tolerance","max_global_plan_lookahead_dist"}) {
    XmlRpc::XmlRpcValue value;if(!nh.hasParam(key)&&old.getParam(key,value)) nh.setParam(key,value);
  }
  double hz=10.;parent.param("controller_frequency",hz,10.);params_.period=1./std::max(1.,hz);
  // Existing coverage verification reads these compatibility semantics.
  nh.setParam("use_proportional_saturation",true);nh.setParam("cmd_angle_instead_rotvel",false);
  std::string odom="/odom";old.param("odom_topic",odom,odom);nh.param("odom_topic",odom,odom);
  odom_sub_=nh.subscribe(odom,1,&IGOLocalPlannerROS::odometry,this);
  std::string steering_topic="/m2_driver/wheel_angle";
  nh.param("steering_topic",steering_topic,steering_topic);
  steering_sub_=nh.subscribe(steering_topic,1,&IGOLocalPlannerROS::steering,this);
  local_pub_=nh.advertise<nav_msgs::Path>("local_plan",1);
  global_pub_=nh.advertise<nav_msgs::Path>("global_plan",1);
  // The existing embedded RViz files subscribe to these two legacy display
  // names. Only path visualization is aliased; all control/config is IGO.
  legacy_local_pub_=old.advertise<nav_msgs::Path>("local_plan",1);
  legacy_global_pub_=old.advertise<nav_msgs::Path>("global_plan",1);
  status_pub_=nh.advertise<std_msgs::String>("status",1,true);
  server_.reset(new dynamic_reconfigure::Server<IGOConfig>(nh));
  server_->setCallback(boost::bind(&IGOLocalPlannerROS::reconfigure,this,_1,_2));
  initialized_=true;
  std_msgs::String status;status.data="{\"planner\":\"IGO\",\"state\":\"initialized\",\"compute_unit\":\"CPU\"}";
  status_pub_.publish(status);
  ROS_INFO("IGO local planner initialized: Gaussian natural gradient, Ackermann forward/reverse; odom=%s",odom.c_str());
}
void IGOLocalPlannerROS::reconfigure(IGOConfig& c,uint32_t) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(params_.direction!=c.motion_direction_mode || params_.knots!=c.knots || params_.horizon!=c.horizon) optimizer_.reset();
  params_.forward=c.max_vel_x;params_.reverse=c.max_vel_x_backwards;params_.angular=c.max_vel_theta;
  params_.acceleration=c.acc_lim_x;params_.angular_acceleration=c.acc_lim_theta;
  params_.deceleration=c.decel_lim_x;params_.stopping_delay=c.stopping_delay;
  params_.wheelbase=c.wheelbase;params_.radius=c.min_turning_radius;
  params_.steering_rate=c.max_steering_rate;params_.steering_weight=c.steering_change_weight;
  params_.xy_tolerance=c.xy_goal_tolerance;params_.yaw_tolerance=c.yaw_goal_tolerance;
  params_.lookahead=c.max_global_plan_lookahead_dist;params_.direction=c.motion_direction_mode;
  params_.allow_reverse=c.allow_init_with_backwards_motion;params_.unknown_obstacle=c.treat_unknown_as_obstacle;
  params_.overwrite_orientation=c.global_plan_overwrite_orientation;
  params_.samples=c.samples;params_.iterations=c.iterations;params_.knots=c.knots;params_.horizon=c.horizon;
  params_.mean_rate=c.mean_rate;params_.variance_rate=c.variance_rate;params_.elite_fraction=c.elite_fraction;
  params_.path_weight=c.path_weight;params_.heading_weight=c.heading_weight;
  params_.obstacle_weight=c.obstacle_weight;params_.progress_weight=c.progress_weight;
}
void IGOLocalPlannerROS::odometry(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(std::isfinite(msg->twist.twist.linear.x)&&std::isfinite(msg->twist.twist.angular.z)) {
    twist_.v=msg->twist.twist.linear.x;twist_.omega=msg->twist.twist.angular.z;
  }
}
void IGOLocalPlannerROS::steering(const std_msgs::Float64::ConstPtr& msg) {
  if(!std::isfinite(msg->data)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  twist_.steering=msg->data;steering_received_=ros::WallTime::now();
}
bool IGOLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  std::lock_guard<std::mutex> lock(mutex_);if(!initialized_||plan.empty()) return false;
  for(const auto& p:plan) if(!std::isfinite(p.pose.position.x+p.pose.position.y+tf2::getYaw(p.pose.orientation))) return false;
  bool changed=plan_.empty() || plan.front().header.seq!=plan_.front().header.seq;
  bool new_target=plan_.empty();
  if(!plan_.empty()) {
    const auto& a=plan.back().pose;const auto& b=plan_.back().pose;
    new_target=std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)>.05 ||
            std::abs(angle(tf2::getYaw(a.orientation)-tf2::getYaw(b.orientation)))>.05;
    changed=changed || new_target;
  }
  bool new_origin=plan_.empty() || std::hypot(plan.front().pose.position.x-plan_.front().pose.position.x,
                                            plan.front().pose.position.y-plan_.front().pose.position.y)>.05;
  if(changed) optimizer_.reset();
  if(new_target) goal_braking_=false;
  cursor_=(changed||new_origin)?0:std::min(cursor_,plan.size()-1);
  plan_=plan;reached_=false;return true;
}
bool IGOLocalPlannerROS::reference(const geometry_msgs::PoseStamped& pose,std::vector<State>& out,bool& terminal) {
  if(plan_.empty()) return false;
  std::vector<State> transformed;transformed.reserve(plan_.size());
  try {
    // Path geometry is static within its frame. Resolve frame transforms at the
    // current control time, not at the obsolete global-plan publication time.
    std::string source;geometry_msgs::TransformStamped transform;
    for(auto p:plan_) {
      p.header.stamp=ros::Time(0);
      if(p.header.frame_id!=frame_) {
        if(source!=p.header.frame_id) {
          source=p.header.frame_id;transform=tf_->lookupTransform(frame_,source,ros::Time(0),ros::Duration(.05));
        }
        tf2::doTransform(p,p,transform);
      }
      transformed.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation),0,0});
    }
  } catch(const tf2::TransformException& e) {ROS_WARN_THROTTLE(1.,"IGO reference TF: %s",e.what());return false;}
  double best=1e100,walk=0;size_t nearest=cursor_;
  for(size_t i=cursor_;i<transformed.size();++i) {
    if(i>cursor_) walk+=std::hypot(transformed[i].x-transformed[i-1].x,transformed[i].y-transformed[i-1].y);
    if(walk>params_.lookahead+2.) break;
    double d=std::hypot(transformed[i].x-pose.pose.position.x,transformed[i].y-pose.pose.position.y);
    if(d<best) {best=d;nearest=i;}
  }
  cursor_=nearest;double length=0;size_t end=nearest;
  out.push_back(transformed[nearest]);
  for(size_t i=nearest+1;i<transformed.size();++i) {
    length+=std::hypot(transformed[i].x-transformed[i-1].x,transformed[i].y-transformed[i-1].y);
    out.push_back(transformed[i]);end=i;if(length>=params_.lookahead) break;
  }
  terminal=end==transformed.size()-1;
  if(params_.overwrite_orientation) for(size_t i=0;i+1<out.size();++i)
    if(std::hypot(out[i+1].x-out[i].x,out[i+1].y-out[i].y)>1e-6)
      out[i].yaw=std::atan2(out[i+1].y-out[i].y,out[i+1].x-out[i].x);
  return true;
}
void IGOLocalPlannerROS::publish(const Result& r,const std::vector<State>& ref,const ros::Time& stamp) {
  auto path=[&](const std::vector<State>& states) {
    nav_msgs::Path msg;msg.header.frame_id=frame_;msg.header.stamp=stamp;
    for(const auto& s:states) {
      geometry_msgs::PoseStamped p;p.header=msg.header;p.pose.position.x=s.x;p.pose.position.y=s.y;
      tf2::Quaternion q;q.setRPY(0,0,s.yaw);p.pose.orientation=tf2::toMsg(q);msg.poses.push_back(p);
    }return msg;
  };
  auto local=path(r.trajectory),global=path(ref);
  local_pub_.publish(local);global_pub_.publish(global);legacy_local_pub_.publish(local);legacy_global_pub_.publish(global);
  std::ostringstream out;out<<"{\"planner\":\"IGO\",\"valid\":"<<(r.valid?"true":"false")
    <<",\"solve_ms\":"<<r.elapsed_ms<<",\"evaluations\":"<<r.evaluations
    <<",\"cost\":"<<r.cost<<",\"v\":"<<r.command.v<<",\"omega\":"<<r.omega
    <<",\"max_steering_rate\":"<<params_.steering_rate
    <<",\"input_v\":"<<twist_.v<<",\"goal_braking\":"<<(goal_braking_?"true":"false")
    <<",\"decel_lim_x\":"<<params_.deceleration
    <<",\"direction_mode\":"<<params_.direction<<",\"goal_reached\":"<<(reached_?"true":"false")
    <<",\"reason\":\""<<r.reason<<"\"}";
  std_msgs::String msg;msg.data=out.str();status_pub_.publish(msg);
}
bool IGOLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd) {
  const auto begin=std::chrono::steady_clock::now();std::lock_guard<std::mutex> lock(mutex_);cmd=geometry_msgs::Twist();
  if(!initialized_) return false;
  geometry_msgs::PoseStamped pose;if(!costmap_->getRobotPose(pose)) return false;
  std::vector<State> ref;bool terminal=false;if(!reference(pose,ref,terminal)) return false;
  State s{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation),twist_.v,twist_.omega};
  // Use existing physical wheel feedback, including while stopped. Simulators
  // without this topic retain the odometry-based estimate; no new readiness gate.
  if(!steering_received_.isZero() && (ros::WallTime::now()-steering_received_).toSec()<.5) {
    s.steering=twist_.steering;
    s.omega=s.v*std::tan(s.steering)/params_.wheelbase;
  }
  Grid grid;
  {
    auto* map=costmap_->getCostmap();boost::unique_lock<costmap_2d::Costmap2D::mutex_t> guard(*map->getMutex());
    grid.width=map->getSizeInCellsX();grid.height=map->getSizeInCellsY();grid.resolution=map->getResolution();
    grid.origin_x=map->getOriginX();grid.origin_y=map->getOriginY();grid.unknown_obstacle=params_.unknown_obstacle;
    grid.cells.assign(map->getCharMap(),map->getCharMap()+grid.width*grid.height);
  }
  grid.prepare();std::vector<Point> footprint;
  for(auto p:costmap_->getRobotFootprint()) footprint.push_back({p.x,p.y});
  const State goal=ref.back();
  // Preserve the global planner's existing per-goal yaw-free protocol, while
  // exact coverage/Hybrid actions retain their terminal body orientation.
  const bool yaw_free=params_.direction==0 && (plan_.front().header.seq&0xC0000000u)==0x80000000u &&
    std::all_of(plan_.begin(),plan_.end(),[&](const geometry_msgs::PoseStamped& p){return p.header.seq==plan_.front().header.seq;});
  const bool in_goal=terminal && std::hypot(s.x-goal.x,s.y-goal.y)<=params_.xy_tolerance &&
                     (yaw_free || std::abs(angle(s.yaw-goal.yaw))<=params_.yaw_tolerance);
  const bool stopped=std::abs(s.v)<=.05 && std::abs(s.omega)<=.05;
  // Finish a goal-related braking manoeuvre before reconsidering the approach.
  // Crossing the tolerance boundary while still rolling must not reaccelerate
  // or alternate gears. A stop outside tolerance still requires a correction.
  if(goal_braking_ && stopped && !in_goal) goal_braking_=false;
  if(in_goal) goal_braking_=true;
  Result result;
  if(in_goal && stopped) {
    reached_=true;result.valid=true;result.trajectory={s};optimizer_.reset();
  } else {
    auto p=params_;
    p.goal_yaw_required=!yaw_free;
    // Braking in the goal region remains a collision-checked feasible rollout.
    if(goal_braking_) {p.forward=0;p.reverse=0;}
    result=optimizer_.solve(s,ref,grid,footprint,p,terminal);
    if(result.valid) {cmd.linear.x=result.command.v;cmd.angular.z=result.omega;}
  }
  result.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  publish(result,ref,ros::Time::now());return result.valid;
}
bool IGOLocalPlannerROS::isGoalReached() {std::lock_guard<std::mutex> lock(mutex_);return reached_;}
} // namespace
