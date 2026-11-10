#include <nav_core/base_local_planner.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <exception>
#include <iostream>

// Load through the package export, without initialize(), ROS topics or goals.
// This also works after sourcing only the installed workspace.
int main(int argc, char** argv) {
  ros::init(argc, argv, "igo_plugin_load_check", ros::init_options::AnonymousName |
            ros::init_options::NoSigintHandler);
  try {
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> loader(
        "nav_core", "nav_core::BaseLocalPlanner");
    auto planner = loader.createInstance("igo_local_planner/IGOLocalPlannerROS");
    if (!planner) return 1;
    std::cout << "Loaded igo_local_planner/IGOLocalPlannerROS; initialize() was not called."
              << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "IGO plugin load failed: " << error.what() << std::endl;
    return 1;
  }
}
