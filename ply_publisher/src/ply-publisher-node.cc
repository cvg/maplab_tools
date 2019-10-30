#include "ply-publisher/ply-publisher-node.h"
#include <map-resources/resource-conversion.h>

#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>                                                 
#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/bind.hpp>
#include <sstream>

DEFINE_string(publish_topic, "/ply",
    "Defines the topic used for republishing the converted submaps.");

DEFINE_string(PLY_directory, "",
    "If set will store the extracted submap as PLY to the given path.");

namespace maplab {

PlyPublisher::PlyPublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), processed_plys_(0), 
    ply_directory_(FLAGS_PLY_directory), publish_topic_(FLAGS_publish_topic) {
  LOG(INFO) << "[PlyPublisher] Initializing publisher...";

  // Normalize input.
  ply_directory_.erase(ply_directory_.find_last_not_of("/") + 1);
  publish_topic_.erase(0, publish_topic_.find_first_not_of("/"));


  // Initialize the submap republish.
  ply_pub_ = nh.advertise<sensor_msgs::PointCloud2>(publish_topic_, 1);
}

bool PlyPublisher::run() {
  LOG(INFO) << "[PlyPublisher] Starting...";
  spinner_.start();
  readPointclouds(FLAGS_PLY_directory);
  return true;
}

void PlyPublisher::shutdown(){

}

std::atomic<bool>& PlyPublisher::shouldExit() {
  return should_exit_;
}

std::string PlyPublisher::printStatistics() const {
  std::stringstream ss;
  ss << "[PlyPublisher] Publishing to " << publish_topic_ << "\n";
  ss << " \t Processed " << processed_plys_ << " so far. \n";
  return ss.str();
}

void PlyPublisher::readDirectory(const std::string& directory,
     std::vector<std::string>* files) {
   boost::filesystem::path p(directory);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(*files),
        [] (const boost::filesystem::directory_entry& entry) {
          return entry.path().leaf().string();
    });
}

void PlyPublisher::readPointclouds(const std::string& dir) {
  CHECK(!dir.empty());

  VLOG(1) << "Retrieving PLY files from " << dir;
  std::vector<std::string> files;
  readDirectory(dir, &files);

  if (files.empty()) { 
    LOG(FATAL) << "Given directory is empty"; 
  }
  for (const std::string& file : files) {
    const std::string full_path = dir + file;
    resources::PointCloud maplab_pointcloud;
    maplab_pointcloud.loadFromFile(full_path);
    VLOG(1) << "point cloud size: " << maplab_pointcloud.xyz.size();
  }

}

} // namespace maplab
