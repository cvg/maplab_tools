#include "online-map-builders/stream-map-builder.h"

#include <aslam/common/stl-helpers.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/vi-map.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>

DEFINE_bool(
    map_builder_save_image_as_resources, false,
    "Store the images associated with the visual frames to the map resource "
    "folder.");

DEFINE_bool(
    map_builder_save_point_clouds_as_resources, false,
    "Store the point clouds associated with a lidar sensor to the map resource "
    "folder.");

namespace online_map_builders {

const vi_map::VIMap* StreamMapBuilder::constMap() const {
  return map_;
}

pose_graph::VertexId StreamMapBuilder::getRootVertexId() const {
  CHECK(mission_id_.isValid());
  return map_->getMission(mission_id_).getRootVertexId();
}

pose_graph::VertexId StreamMapBuilder::getLastVertexId() const {
  CHECK(last_vertex_.isValid());
  return last_vertex_;
}

StreamMapBuilder::StreamMapBuilder(
    const vi_map::SensorManager& sensor_manager, vi_map::VIMap* map)
    : map_(CHECK_NOTNULL(map)),
      manipulation_(map),
      mission_id_(aslam::createRandomId<vi_map::MissionId>()),
      is_first_baseframe_estimate_processed_(false) {
  // clone sensor manager into the new map
  map_->getSensorManager().merge(sensor_manager);

  // add new mission
  map_->addNewMissionWithBaseframe(
      mission_id_, aslam::Transformation(),
      Eigen::Matrix<double, 6, 6>::Identity(),
      vi_map::Mission::BackBone::kViwls);

  // associate sensors to the newly created mission
  aslam::SensorIdSet sensor_ids;
  sensor_manager.getAllSensorIds(&sensor_ids);
  map_->associateMissionSensors(sensor_ids, mission_id_);
}

void StreamMapBuilder::apply(const vio::MapUpdate& update) {
  CHECK(update.check());
  constexpr bool kDeepCopyNFrame = true;
  apply(update, kDeepCopyNFrame);
}

void StreamMapBuilder::apply(
    const vio::MapUpdate& update, bool deep_copy_nframe) {
  CHECK(update.check());
  std::shared_ptr<aslam::VisualNFrame> nframe_to_insert;
  if (deep_copy_nframe) {
    const aslam::VisualNFrame& nframe_original = *update.keyframe->nframe;
    nframe_to_insert = aligned_shared<aslam::VisualNFrame>(
        nframe_original.getId(), nframe_original.getNumCameras());
    *nframe_to_insert = *update.keyframe->nframe;
  } else {
    nframe_to_insert = update.keyframe->nframe;
  }
  CHECK(nframe_to_insert);

  if (!last_vertex_.isValid()) {
    addRootViwlsVertex(nframe_to_insert, update.vinode);
  } else {
    CHECK(mission_id_.isValid());
    if (!is_first_baseframe_estimate_processed_ &&
        (update.localization_state == common::LocalizationState::kLocalized ||
         update.localization_state ==
             common::LocalizationState::kMapTracking)) {
      T_M0_G_ = update.T_G_M.inverse();
      map_->getMissionBaseFrameForMission(mission_id_).set_T_G_M(update.T_G_M);
      is_first_baseframe_estimate_processed_ = true;
    }

    addViwlsVertexAndEdge(
        nframe_to_insert, update.vinode, update.T_G_M, update.imu_timestamps,
        update.imu_measurements);
  }
}

void StreamMapBuilder::addRootViwlsVertex(
    const aslam::VisualNFrame::Ptr& nframe,
    const vio::ViNodeState& vinode_state) {
  CHECK(nframe);
  CHECK(!last_vertex_.isValid()) << "Root vertex has already been set!";

  const aslam::Transformation T_G_M;
  pose_graph::VertexId root_vertex_id =
      addViwlsVertex(nframe, vinode_state, T_G_M);
  CHECK(root_vertex_id.isValid());

  CHECK(!constMap()->getMission(mission_id_).getRootVertexId().isValid())
      << "Root vertex has already been set for this mission.";
  map_->getMission(mission_id_).setRootVertexId(root_vertex_id);

  last_vertex_ = root_vertex_id;
}

void StreamMapBuilder::addViwlsVertexAndEdge(
    const aslam::VisualNFrame::Ptr& nframe,
    const vio::ViNodeState& vinode_state, const aslam::Transformation& T_G_M,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data) {
  CHECK(nframe);
  CHECK(last_vertex_.isValid())
      << "Need to have a previous vertex to connect to!";

  pose_graph::VertexId new_vertex_id =
      addViwlsVertex(nframe, vinode_state, T_G_M);

  addImuEdge(new_vertex_id, imu_timestamps, imu_data);

  if (kKeepNMostRecentImages > 0u) {
    manipulation_.releaseOldVisualFrameImages(
        last_vertex_, kKeepNMostRecentImages);
  }
}

pose_graph::VertexId StreamMapBuilder::addViwlsVertex(
    const aslam::VisualNFrame::Ptr& nframe,
    const vio::ViNodeState& vinode_state, const aslam::Transformation& T_G_M) {
  CHECK_EQ(
      map_->getMissionNCamera(mission_id_).getId(),
      nframe->getNCamera().getId())
      << "Can only add nframes that correspond to the mission camera";

  // Initialize all keypoint <-> landmark associations to invalid.
  std::vector<vi_map::LandmarkIdList> invalid_landmark_ids;
  const size_t num_frames = nframe->getNumFrames();
  for (size_t frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
    if (nframe->isFrameSet(frame_idx)) {
      const aslam::VisualFrame& frame = nframe->getFrame(frame_idx);
      invalid_landmark_ids.emplace_back(frame.getNumKeypointMeasurements());
    }
  }

  // Create and add the new map vertex.
  pose_graph::VertexId vertex_id =
      aslam::createRandomId<pose_graph::VertexId>();
  vi_map::Vertex* map_vertex = new vi_map::Vertex(
      vertex_id, vinode_state.getImuBias(), nframe, invalid_landmark_ids,
      mission_id_);

  // Set pose and velocity.
  const aslam::Transformation T_M0_M = T_M0_G_ * T_G_M;
  map_vertex->set_T_M_I(T_M0_M * vinode_state.get_T_M_I());
  map_vertex->set_v_M(T_M0_M * vinode_state.get_v_M_I());
  map_->addVertex(vi_map::Vertex::UniquePtr(map_vertex));

  // Optionally dump the image to disk.
  if (FLAGS_map_builder_save_image_as_resources) {
    CHECK(map_->hasMapFolder())
        << "Cannot store resources to a map that has no associated map folder, "
        << "please set the map folder in the VIMap constructor or by using "
        << "map.setMapFolder()!";
    map_->useMapResourceFolder();
    for (size_t frame_idx = 0u; frame_idx < nframe->getNumFrames();
         ++frame_idx) {
      if (nframe->isFrameSet(frame_idx)) {
        map_->storeFrameResource(
            nframe->getFrame(frame_idx).getRawImage(), frame_idx,
            backend::ResourceType::kRawImage, map_vertex);
      }
    }
  }
  return vertex_id;
}

void StreamMapBuilder::removeAllVerticesAfterVertexId(
    const pose_graph::VertexId& vertex_id_from,
    pose_graph::VertexIdList* removed_vertex_ids) {
  CHECK_NOTNULL(removed_vertex_ids);
  manipulation_.removePosegraphAfter(vertex_id_from, removed_vertex_ids);
  last_vertex_ = vertex_id_from;
}

void StreamMapBuilder::addImuEdge(
    pose_graph::VertexId target_vertex_id,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements) {
  CHECK(last_vertex_.isValid());
  CHECK(target_vertex_id.isValid());
  CHECK_EQ(imu_timestamps.cols(), imu_measurements.cols());

  // Make sure the vertices are not yet connected.
  pose_graph::EdgeIdSet edges;
  constMap()->getVertex(last_vertex_).getOutgoingEdges(&edges);
  for (const pose_graph::EdgeId& outgoing_edge_id : edges) {
    CHECK(
        constMap()->getEdgeType(outgoing_edge_id) !=
        pose_graph::Edge::EdgeType::kViwls)
        << "Source vertex already has an outgoing ViwlsEdge.";
  }
  constMap()->getVertex(target_vertex_id).getIncomingEdges(&edges);
  for (const pose_graph::EdgeId& incoming_edge_id : edges) {
    CHECK(
        constMap()->getEdgeType(incoming_edge_id) !=
        pose_graph::Edge::EdgeType::kViwls)
        << "Source vertex already has an incoming ViwlsEdge.";
  }

  // The vi_map data structures requires that the last IMU measurement of
  // the previous edge to be be duplicated as the first measurement of the
  // following edge.
  pose_graph::EdgeIdList source_vertex_edge_incoming_ids;
  constMap()->getIncomingOfType(
      pose_graph::Edge::EdgeType::kViwls, last_vertex_,
      &source_vertex_edge_incoming_ids);
  CHECK_LE(source_vertex_edge_incoming_ids.size(), 1u)
      << "More than one ViwlsEdge found.";

  const bool first_edge = source_vertex_edge_incoming_ids.empty();
  if (!first_edge) {
    const vi_map::ViwlsEdge& prev_edge = map_->getEdgeAs<vi_map::ViwlsEdge>(
        source_vertex_edge_incoming_ids.front());
    const double kTolerance = 1.0e-12;
    const std::string kErrorMsg(
        "The vi_map data structures requires that the last IMU"
        "measurement of the previous edge to be be duplicated as the first "
        "measurement of the following edge.");
    CHECK_LT(
        (prev_edge.getImuTimestamps().tail<1>() - imu_timestamps.head(1))
                .cwiseAbs()
                .maxCoeff() +
            (prev_edge.getImuData().rightCols<1>() -
             imu_measurements.leftCols(1))
                .cwiseAbs()
                .maxCoeff(),
        kTolerance)
        << kErrorMsg;
  }
  // Add the edge.
  pose_graph::EdgeId edge_id = aslam::createRandomId<pose_graph::EdgeId>();
  map_->addEdge(aligned_unique<vi_map::ViwlsEdge>(
      edge_id, last_vertex_, target_vertex_id, imu_timestamps,
      imu_measurements));

  last_vertex_ = target_vertex_id;
}

bool StreamMapBuilder::checkConsistency() const {
  return vi_map::checkMapConsistency(*CHECK_NOTNULL(constMap()));
}

void StreamMapBuilder::attachArtifactMeasurement(
    const vi_map::ArtifactMeasurement& detection) {
  CHECK_NOTNULL(map_);
  CHECK(detection.isValid())
      << "[StreamMapBuilder] Artifact detection measurement is invalid!";
  const aslam::SensorId& artifact_sensor_id = detection.getSensorId();

  CHECK(map_->getSensorManager().hasSensor(artifact_sensor_id))
      << "[StreamMapBuilder] The artifact detection sensor of this detection is "
         "not yet present in the map's sensor manager!";

  resources::Artifact artifact;
  backend::convertDetectionToArtifact(detection, &artifact);

  backend::ResourceType artifact_type = 
    backend::ResourceType::kArtifactDetection;
  vi_map::VIMission& mission = map_->getMission(mission_id_);
  map_->addSensorResource(
      artifact_type, artifact_sensor_id,
      detection.getTimestampNanoseconds(), artifact, &mission);
}

}  // namespace online_map_builders
