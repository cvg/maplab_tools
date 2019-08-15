#ifndef MAPLAB_CAMERA_INFO_PUBLISHER_NODE_H_
#define MAPLAB_CAMERA_INFO_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>
#include <std_srvs/Empty.h>
#include <vi-map/sensor-manager.h>
#include <image_transport/image_transport.h>


#include <vector>
#include <atomic>
#include <functional>
#include <memory>

namespace maplab {

class MaplabCameraInfoPublisher {
  public:
    explicit MaplabCameraInfoPublisher(ros::NodeHandle& nh, 
        const ros::NodeHandle& nh_private);

    bool run();
    void shutdown();
    std::atomic<bool>& shouldExit();
    std::string printStatistics() const;


  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;
    std::atomic<bool> should_exit_;
    ros::ServiceServer service_start_;
    ros::ServiceServer service_stop_;
    image_transport::ImageTransport image_transport_;
    std::vector<image_transport::Subscriber> sub_images_;
    std::vector<ros::ServiceServer> services_;
    std::unique_ptr<vi_map::SensorManager> sensor_manager_;
    bool should_publish_ = false;
    aslam::NCamera::Ptr ncamera_rig_;
    std::vector<ros::Publisher> info_pubs_;

    const std::string kStartServiceTopic = "/cam_info_start_publishing";
    const std::string kStopServiceTopic = "/cam_info_stop_publishing";

    bool initializeServicesAndSubscribers();
    bool initializeNCamera();
    void imageCallback(const sensor_msgs::ImageConstPtr &image, 
        std::size_t camera_idx);
    bool startPublishing(std_srvs::Empty::Request&, 
      std_srvs::Empty::Response&);
    bool stopPublishing(std_srvs::Empty::Request&, 
      std_srvs::Empty::Response&);
    bool retrieveCameraIntrinsics(const aslam::Camera& camera, 
        double* fu, double* fv, double* cu, double *cv) const;
    bool retrieveDistortionParameters(const aslam::Camera& camera, 
        double* k1, double* k2, double* k3, double *k4, 
        std::string* type) const;
    void createAndPublishCameraInfo(const std::size_t idx,
        const sensor_msgs::ImageConstPtr &image) const;
};

} // namespace maplab

#endif // MAPLAB_CAMERA_INFO_PUBLISHER_NODE_H_