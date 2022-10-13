#ifndef ODOMETRY_CONVERTER_NODE_H_
#define ODOMETRY_CONVERTER_NODE_H_

#include <glog/logging.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace maplab {

class OdometryConverter {
 public:
  explicit OdometryConverter(
      ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool run();
  void shutdown();

 private:
  bool initializeServicesAndSubscribers();

  void odomCallback(
      const nav_msgs::Odometry::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber odom_sub_;
  ros::Publisher maplab_odom_pub_;
};

}

#endif  // ODOMETRY_CONVERTER_NODE_H_
