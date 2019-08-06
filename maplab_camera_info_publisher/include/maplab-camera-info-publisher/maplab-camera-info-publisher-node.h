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

    std::unique_ptr<vi_map::SensorManager> sensor_manager_;
    bool should_publish_ = false;

    bool initialize_services_and_subscribers();
    void imageCallback(const sensor_msgs::ImageConstPtr &image, 
        std::size_t camera_idx);
    bool start_publishing(std_srvs::Empty::Request&, 
      std_srvs::Empty::Response&);
    bool stop_publishing(std_srvs::Empty::Request&, 
      std_srvs::Empty::Response&);


};

} // namespace maplab

#endif // MAPLAB_CAMERA_INFO_PUBLISHER_NODE_H_
