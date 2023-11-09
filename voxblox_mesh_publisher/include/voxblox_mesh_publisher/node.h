#ifndef VOXBLOX_MESH_PUBLISHER_H_
#define VOXBLOX_MESH_PUBLISHER_H_

#include <atomic>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_tf.h>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/io/mesh_ply.h"
#include "voxblox/mesh/mesh.h"
#include "voxblox/mesh/mesh_utils.h"

namespace maplab {

class MeshPublisher {
 public:
  explicit MeshPublisher(
      ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool run();
  void shutdown();
  std::atomic<bool>& shouldExit();

 private:
  bool initializeServicesAndSubscribers();

  void voxbloxMeshCallback(const voxblox_msgs::Mesh::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::AsyncSpinner spinner_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber odom_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher marker_pub_;

  std::atomic<bool> should_exit_;
};

}  // namespace maplab

#endif  // VOXBLOX_MESH_PUBLISHER_H_
