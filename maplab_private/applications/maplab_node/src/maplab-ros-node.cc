#include "maplab-node/maplab-ros-node.h"

#include <atomic>
#include <memory>
#include <signal.h>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <aslam/cameras/ncamera.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <message-flow/message-topic-registration.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>

#include <std_srvs/Empty.h>
#include <vi-map/vi-map-serialization.h>
#include <vio-common/vio-types.h>

#include "maplab-node/maplab-node.h"

// === SENSORS ===

DEFINE_string(
    sensor_calibration_file, "",
    "Yaml file with all the sensor calibrations. Determines which sensors are "
    "used by MaplabNode.");

// ==== INPUT LOCALIZATION MAPS ====

DEFINE_string(
    visual_localization_map_folder, "",
    "Path to a visual localization summary map or a full VI-map used for "
    "localization.");

DEFINE_string(
    lidar_localization_map_folder, "", "Path to a lidar localization map.");

// === OUTPUT MAPS ===

DEFINE_string(
    map_output_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
    map_overwrite_enabled, false,
    "If set to true, an existing map will be overwritten on save. Otherwise, a "
    "number will be appended to map_output_folder to obtain an available "
    "folder.");
DEFINE_bool(
    map_compute_localization_map, false,
    "Optimize and process the map into a localization map before "
    "saving it.");

// === MAPLAB NODE ===

DEFINE_bool(
    enable_visual_localization, true,
    "The maplab node will use vision to localize, if enabled. Requires "
    "--enable_visual_inertial_data=true");

DEFINE_bool(
    enable_lidar_localization, false,
    "The maplab node will use the lidar data to localize, if enabled. Requires "
    "--enable_lidar_data=true "
    "[NOT IMPLEMENTED YET]");

DEFINE_bool(
    enable_online_mapping, false,
    "The maplab node will try to optimize and loop close the pose graph online "
    "and provide the localization with new maps. [NOT IMPLEMENTED YET]");

namespace maplab {

MaplabRosNode::MaplabRosNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      map_output_folder_(""),
      maplab_spinner_(common::getNumHardwareThreads()) {
  LOG(INFO) << "[MaplabROSNode] Initializing message flow...";
  message_flow_.reset(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          common::getNumHardwareThreads()));

  // If a map will be saved (i.e., if the save map folder is not empty), append
  // a number to the name until a name is found that is free.
  map_output_folder_ = FLAGS_map_output_folder;
  if (!FLAGS_map_overwrite_enabled && !map_output_folder_.empty()) {
    map_output_folder_ = common::getUniqueFolderName(map_output_folder_);
  }
  if (map_output_folder_.empty()) {
    LOG(INFO) << "[MaplabROSNode] No output map folder was provided, map "
              << "building disable.";
  } else {
    LOG(INFO) << "[MaplabROSNode] Set output map folder to: '"
              << map_output_folder_ << "', map building enabled.";
  }

  if (FLAGS_map_builder_save_image_as_resources && map_output_folder_.empty()) {
    LOG(FATAL) << "If you would like to save the resources, "
               << "please also set a map folder with: --map_output_folder";
  }

  // === MAPLAB NODE ===
  LOG(INFO) << "[MaplabROSNode] Initializing MaplabNode...";
  maplab_node_.reset(new MaplabNode(
      FLAGS_sensor_calibration_file, map_output_folder_, message_flow_.get()));

  // === LOCALIZATION MAPS ===
  // === VISION ===
  if (!FLAGS_visual_localization_map_folder.empty() &&
      FLAGS_enable_visual_localization) {
    LOG(INFO) << "[MaplabROSNode] Loading visual localization map from: "
              << FLAGS_visual_localization_map_folder << "'.";

    std::unique_ptr<summary_map::LocalizationSummaryMap> summary_map(
        new summary_map::LocalizationSummaryMap);
    summary_map::loadLocalizationSummaryMapFromAnyMapFile(
        FLAGS_visual_localization_map_folder, summary_map.get());
    maplab_node_->enableVisualLocalization(std::move(summary_map));
    CHECK(!summary_map);
  } else {
    LOG(INFO) << "[MaplabROSNode] No visual localization map provided.";
  }
  // === LIDAR ===
  if (!FLAGS_lidar_localization_map_folder.empty() &&
      FLAGS_enable_lidar_localization) {
    LOG(INFO) << "[MaplabROSNode] Loading lidar localization map from: "
              << FLAGS_lidar_localization_map_folder << "'.";

    // TODO(LBern): load lidar localization map.

    maplab_node_->enableLidarLocalization();
  } else {
    LOG(INFO) << "[MaplabROSNode] No lidar localization map provided.";
  }
}

bool MaplabRosNode::run() {
  LOG(INFO) << "[MaplabROSNode] Starting...";
  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  maplab_spinner_.start();
  maplab_node_->start();
  return true;
}

bool MaplabRosNode::saveMap(const std::string& map_folder) {
  LOG(INFO) << "[MaplabROSNode] Saving map to: '" << map_folder << "'.";
  if (!map_folder.empty()) {
    CHECK(maplab_node_);
    maplab_node_->saveMapAndOptionallyOptimize(
        map_folder, FLAGS_map_overwrite_enabled,
        FLAGS_map_compute_localization_map);
    return true;
  } else {
    return false;
  }
}

bool MaplabRosNode::saveMap() {
  return saveMap(map_output_folder_);
}

std::atomic<bool>& MaplabRosNode::shouldExit() {
  return maplab_node_->isDataSourceExhausted();
}

void MaplabRosNode::shutdown() {
  LOG(INFO) << "[MaplabROSNode] Shutting down...";
  maplab_node_->shutdown();
  message_flow_->shutdown();
  message_flow_->waitUntilIdle();
}

// Save map over ROS service, in case save_map_on_shutdown is disabled.
bool MaplabRosNode::saveMapCallback(
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  return saveMap();
}

// Optional output.
std::string MaplabRosNode::printDeliveryQueueStatistics() const {
  return message_flow_->printDeliveryQueueStatistics();
}

}  // namespace maplab
