#ifndef ODOMETRY_CONVERTER_NODE_H_
#define ODOMETRY_CONVERTER_NODE_H_

#include <glog/logging.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <atomic>
#include <functional>

namespace maplab {

class OdometryConverter {
 public:
  explicit OdometryConverter(
      ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool run();
  void shutdown();
    std::atomic<bool>& shouldExit();

 private:
  bool initializeServicesAndSubscribers();

  void odomCallback(
      const nav_msgs::Odometry::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;

  ros::Subscriber odom_sub_;
  ros::Publisher maplab_odom_pub_;

    std::atomic<bool> should_exit_;
};

}

#endif  // ODOMETRY_CONVERTER_NODE_H_
