#include "odometry_converter/node.h"

#include <maplab_msgs/OdometryWithImuBiases.h>

DEFINE_string(
    odom_topic, "/ros_odom",
    "Defines the topic of the input odometry message");

namespace maplab {

OdometryConverter::OdometryConverter(ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  spinner_(1),
  should_exit_(false) {

  LOG(INFO)
      << "[OdometryConverter] Initializing publisher...";
  if (!initializeServicesAndSubscribers()) {
    LOG(FATAL) << "[OdometryConverter] "
               << "Failed initialize subscribers and services.";
  }
}

bool MaplabCameraInfoPublisher::run() {
  LOG(INFO) << "[MaplabCameraInfoPublisher] Starting...";
  return true;
}

void MaplabCameraInfoPublisher::shutdown() {
  // noop
}

bool MaplabCameraInfoPublisher::initializeServicesAndSubscribers() {
  odom_sub_ = nh_.subscribe(FLAGS_odom_topic, 10, &OdometryConverter::odomCallback);
  maplab_odom_pub_ = nh_private_.advertise<maplab_msgs::OdometryWithImuBiases>("maplab_odom", 1);
}

void OdometryConverter::odomCallback(const nav_msgs::OdometryMsgPtr& msg) {
  maplab_msgs::OdometryWithImuBiases maplab_msg;
  maplab_msg.header = msg->header;
  maplab_msg.child_frame_id = msg->child_frame_id;
  maplab_msg.pose = msg->pose;
  maplab_msg.twist = msg->twist;
  maplab_odom_pub_.publish(maplab_msg);
}

}
