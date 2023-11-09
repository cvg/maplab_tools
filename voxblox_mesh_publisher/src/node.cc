#include "voxblox_mesh_publisher/node.h"

#include <boost/bind.hpp>
#include <glog/logging.h>
#include <maplab_msgs/OdometryWithImuBiases.h>

DEFINE_string(
    voxblox_mesh_topic, "/voxblox_node/mesh",
    "Defines the topic of the voxblox mesh message that will be converted");
DEFINE_string(
    target_frame, "unity",
    "Defines the topic of the voxblox mesh message that will be converted");

namespace maplab {

MeshPublisher::MeshPublisher(
    ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      spinner_(1),
      should_exit_(false),
      tf_listener_(tf_buffer_) {
  LOG(INFO) << "[MeshPublisher] Initializing publisher...";
  if (!initializeServicesAndSubscribers()) {
    LOG(FATAL) << "[MeshPublisher] "
               << "Failed initialize subscribers and services.";
  }
}

bool MeshPublisher::run() {
  LOG(INFO) << "[MeshPublisher] Starting...";
  spinner_.start();
  return true;
}

void MeshPublisher::shutdown() {
  // noop
}
std::atomic<bool>& MeshPublisher::shouldExit() {
  return should_exit_;
}

bool MeshPublisher::initializeServicesAndSubscribers() {
  boost::function<void(const voxblox_msgs::Mesh::ConstPtr&)> mesh_callback =
      boost::bind(&MeshPublisher::voxbloxMeshCallback, this, _1);
  odom_sub_ = nh_.subscribe(FLAGS_voxblox_mesh_topic, 10, mesh_callback);
  mesh_pub_ = nh_private_.advertise<shape_msgs::Mesh>("mesh", 1, true);
  marker_pub_ =
      nh_private_.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);
  return true;
}

void MeshPublisher::voxbloxMeshCallback(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  voxblox::Mesh full_mesh;
  bool first = true;

  kindr::minimal::QuatTransformationTemplate<float> T_O_I;
  try {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        FLAGS_target_frame, msg->header.frame_id, ros::Time(0));
    tf::transformMsgToKindr(transform.transform, &T_O_I);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    const voxblox::BlockIndex index(
        mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);

    if (mesh_block.x.size() == 0) {
      continue;
    }

    size_t vertex_index = 0u;
    voxblox::Mesh mesh;
    mesh.vertices.reserve(mesh_block.x.size());
    mesh.indices.reserve(mesh_block.x.size());

    // translate vertex data from message to voxblox mesh
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      // Each vertex is given as its distance from the blocks origin in units of
      // (2*block_size), see mesh_vis.h for the slightly convoluted
      // justification of the 2.
      constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      const float mesh_x =
          (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
           static_cast<float>(index[0])) *
          msg->block_edge_length;
      const float mesh_y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(index[1])) *
          msg->block_edge_length;
      const float mesh_z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(index[2])) *
          msg->block_edge_length;

      mesh.indices.push_back(vertex_index++);
      mesh.vertices.emplace_back(mesh_x, mesh_y, mesh_z);
    }

    // calculate normals
    mesh.normals.reserve(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
      const voxblox::Point dir0 = mesh.vertices[i] - mesh.vertices[i + 1];
      const voxblox::Point dir1 = mesh.vertices[i] - mesh.vertices[i + 2];
      const voxblox::Point normal = dir0.cross(dir1).normalized();

      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
    }

    // add color information
    mesh.colors.reserve(mesh.vertices.size());
    const bool has_color = mesh_block.x.size() == mesh_block.r.size();
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      voxblox::Color color;
      if (has_color) {
        color.r = mesh_block.r[i];
        color.g = mesh_block.g[i];
        color.b = mesh_block.b[i];

      } else {
        // reconstruct normals coloring
        color.r = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].x() * 0.5f + 0.5f);
        color.g = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].y() * 0.5f + 0.5f);
        color.b = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].z() * 0.5f + 0.5f);
      }
      color.a = 1.0;
      mesh.colors.push_back(color);
    }

    // connect mesh

    if (first) {
      voxblox::createConnectedMesh(mesh, &full_mesh);
      first = false;
    } else {
      voxblox::Mesh connected_mesh;
      voxblox::createConnectedMesh(mesh, &connected_mesh);
      full_mesh.concatenateMesh(connected_mesh);
    }

    // convert to ROS MeshMarker
    voxblox::Mesh connected_mesh;
    voxblox::createConnectedMesh(mesh, &connected_mesh);
    visualization_msgs::Marker marker;
    marker.header.frame_id = FLAGS_target_frame;
    marker.header.stamp = msg->header.stamp;
    marker.ns = "voxblox";
    voxblox::AnyIndexHash hasher;
    marker.id = hasher(index);
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    for (size_t i : connected_mesh.indices) {
      Eigen::Vector3f point_eigen = T_O_I * connected_mesh.vertices[i];
      geometry_msgs::Point point;
      point.x = point_eigen.x();
      point.y = point_eigen.y();
      point.z = point_eigen.z();
      marker.points.push_back(point);

      std_msgs::ColorRGBA color;
      color.r = connected_mesh.colors[i].r / 255.0;
      color.g = connected_mesh.colors[i].g / 255.0;
      color.b = connected_mesh.colors[i].b / 255.0;
      color.a = 1.0;
      marker.colors.push_back(color);
    }
    marker_pub_.publish(marker);
  }

  // convert to ROS Mesh
  shape_msgs::Mesh out_mesh;
  for (auto vertex : full_mesh.vertices) {
    geometry_msgs::Point point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = vertex.z();
    out_mesh.vertices.push_back(point);
  }
  for (size_t i = 0; i < full_mesh.indices.size(); i += 3) {
    shape_msgs::MeshTriangle triangle;
    triangle.vertex_indices[0] = full_mesh.indices[i];
    triangle.vertex_indices[1] = full_mesh.indices[i + 1];
    triangle.vertex_indices[2] = full_mesh.indices[i + 2];
    out_mesh.triangles.push_back(triangle);
  }
  mesh_pub_.publish(out_mesh);
}

}  // namespace maplab
