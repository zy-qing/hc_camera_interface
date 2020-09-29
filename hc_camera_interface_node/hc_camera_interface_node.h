#ifndef HC_CAMERA_INTERFACE_NODE_H_
#define HC_CAMERA_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <vector>
#include "hc_camera_interface.h"

namespace driver {
class HcCameraInterfaceNode {
 public:
  HcCameraInterfaceNode() {}
  ~HcCameraInterfaceNode() {}

  void Run();

 private:
  bool Init();

  bool SetCameraInfo(const int& camera_num, const std::string& image_topics,
                     const std::string& camera_ips,
                     const std::string& camera_usernames,
                     const std::string& camera_passwords,
                     const std::string& camera_ports);

  std::vector<std::string> SplitParameter(const std::string& parameters);

  void StartCamera();

  void Publish();

 private:
  const int kFPS = 30;
  HcCameraInterface hc_camera_interface_;
  ros::NodeHandle private_handle_ = ros::NodeHandle("~");
  std::vector<std::shared_ptr<ros::Publisher>> camera_publisher;
  std::vector<long> camera_handle_;
  int is_show_ = true;
  std::vector<std::string> camera_ip_;
  std::vector<std::string> camera_username_;
  std::vector<std::string> camera_password_;
  std::vector<std::string> camera_port_;
};
}
#endif  // HC_CAMERA_INTERFACE_NODE_H_