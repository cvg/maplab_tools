#include "odometry_converter/node.h"

#include <maplab_msgs/OdometryWithImuBiases.h>

DEFINE_string(
    odom_topic, "/ros_odom",
    "Defines the topic of the input odometry message");

namespace maplab {

OdometryConverter::OdometryConverter(ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private) {

  LOG(INFO)
      << "[OdometryConverter] Initializing publisher...";
  if (!initializeServicesAndSubscribers()) {
    LOG(FATAL) << "[OdometryConverter] "
               << "Failed initialize subscribers and services.";
  }
}

bool OdometryConverter::run() {
  LOG(INFO) << "[OdometryConverter] Starting...";
  return true;
}

void OdometryConverter::shutdown() {
  // noop
}

bool OdometryConverter::initializeServicesAndSubscribers() {
  boost::function<void(const nav_msgs::Odometry::ConstPtr&)> odom_callback =
    boost::bind(&OdometryConverter::odomCallback, this, _1);
  odom_sub_ = nh_.subscribe(FLAGS_odom_topic, 10, odom_callback);
  maplab_odom_pub_ = nh_private_.advertise<maplab_msgs::OdometryWithImuBiases>("maplab_odom", 1);
  return true;
}

void OdometryConverter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  maplab_msgs::OdometryWithImuBiases maplab_msg;
  maplab_msg.header = msg->header;
  maplab_msg.child_frame_id = msg->child_frame_id;
  maplab_msg.pose = msg->pose;
  maplab_msg.twist = msg->twist;
  maplab_odom_pub_.publish(maplab_msg);
}

}
