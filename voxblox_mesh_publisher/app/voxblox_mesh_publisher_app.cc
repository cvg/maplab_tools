#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-ros-common/gflags-interface.h>

#include <voxblox_mesh_publisher/node.h>

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "voxblox_mesh_publisher");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::MeshPublisher mesh_publisher(nh, nh_private);

  if (!mesh_publisher.run()) {
   ROS_FATAL("Failed to start running the mesh publisher node!");
   ros::shutdown();
   return 1;
  }

  std::atomic<bool>& end_of_days_signal_received = mesh_publisher.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
   std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  mesh_publisher.shutdown();
  return 0;
}
