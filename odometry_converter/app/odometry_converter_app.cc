#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-ros-common/gflags-interface.h>

#include <odometry_converter/node.h>

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "odometry_converter");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::OdometryConverter odometry_converter(nh, nh_private);

  if (!odometry_converter.run()) {
   ROS_FATAL("Failed to start running the odometry converter node!");
   ros::shutdown();
   return 1;
  }

  ros::spin();

  odometry_converter.shutdown();
  return 0;
}
