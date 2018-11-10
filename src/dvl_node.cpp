#include "dvl.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dvl_node");
  ros::NodeHandle nh;

  try {
    nortek_dvl::DvlInterface dvl;

    ros::spin();
  } catch (const std::runtime_error& e) {
    ROS_ERROR("%s", e.what());
  }
}
