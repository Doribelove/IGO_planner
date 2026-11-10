#include <iostream>
#include <memory>
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"

int main()
{
  try {
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    auto controller = loader.createSharedInstance("igo_nav2_controller::IGOController");
    if (!controller) {return 1;}
    std::cout << "{\"plugin_loaded\":true,\"hardware_motion\":false}" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
