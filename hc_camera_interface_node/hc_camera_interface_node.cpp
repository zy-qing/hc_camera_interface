#include "hc_camera_interface_node.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <string>
#include "sensor_msgs/Image.h"

namespace driver {

void HcCameraInterfaceNode::Run() {
  Init();
  std::string image_topic;
  int camera_num;
  std::string camera_ip;
  std::string camera_username;
  std::string camera_password;
  std::string camera_port;

  private_handle_.param<std::string>("image_topic", image_topic,
                                     "/sensor_raw/camera");
  private_handle_.param("camera_num", camera_num, 1);
  private_handle_.param<std::string>("camera_ip", camera_ip, "192.168.1.64");
  private_handle_.param<std::string>("camera_username", camera_username,
                                     "admin");
  private_handle_.param<std::string>("camera_password", camera_password,
                                     "root1234");
  private_handle_.param<std::string>("camera_port", camera_port, "8000");
  private_handle_.param("show_image", is_show_, 1);
  if (SetCameraInfo(camera_num, image_topic, camera_ip, camera_username,
                    camera_password, camera_port)) {
    StartCamera();
    Publish();
  }
}

bool HcCameraInterfaceNode::Init() {
  hc_camera_interface_.InitHcNetSdk();
  return true;
};

bool HcCameraInterfaceNode::SetCameraInfo(const int& camera_num,
                                          const std::string& image_topics,
                                          const std::string& camera_ips,
                                          const std::string& camera_usernames,
                                          const std::string& camera_passwords,
                                          const std::string& camera_ports) {
  if (camera_ips.empty() || camera_usernames.empty() ||
      camera_passwords.empty() || camera_ports.empty()) {
    printf("Parameter empty\n");
    return false;
  }
  camera_ip_ = SplitParameter(camera_ips);
  if (camera_ip_.size() != camera_num) {
    printf("Parameter error| camera_ip num != camera_num\n");
    return false;
  }
  camera_username_ = SplitParameter(camera_usernames);
  camera_password_ = SplitParameter(camera_passwords);
  camera_port_ = SplitParameter(camera_ports);
  if (camera_username_.size() == 0 || camera_password_.size() == 0 ||
      camera_port_.size() == 0) {
    printf(
        "Parameter empty: camera_port_||camera_password_||camera_username_ \n");
  }
  for (size_t i = 0; i < camera_num; i++) {
    std::string image_topic = image_topics + std::to_string(i + 1);
    std::shared_ptr<ros::Publisher> image_output =
        std::make_shared<ros::Publisher>(
            private_handle_.advertise<sensor_msgs::Image>(image_topic, 1));
    camera_publisher.emplace_back(image_output);
  }
  return true;
}

std::vector<std::string> HcCameraInterfaceNode::SplitParameter(
    const std::string& parameters) {
  std::vector<std::string> parameters_splited;
  int nCount = 0;
  std::string temp;
  size_t pos = 0, offset = 0;
  std::string separator = ";";
  while ((pos = parameters.find_first_of(separator, offset)) !=
         std::string::npos) {
    temp = parameters.substr(offset, pos - offset);
    if (temp.length() > 0) {
      parameters_splited.emplace_back(temp);
      nCount++;
    }
    offset = pos + 1;
  }
  temp = parameters.substr(offset, parameters.length() - offset);
  if (temp.length() > 0) {
    parameters_splited.push_back(temp);
    nCount++;
  }
  return parameters_splited;
}

void HcCameraInterfaceNode::StartCamera() {
  static const size_t kMAX = 80;
  if (camera_ip_.size() == camera_password_.size() &&
      camera_ip_.size() == camera_username_.size() &&
      camera_ip_.size() == camera_port_.size()) {
    char ip_char[kMAX];
    char username_char[kMAX];
    char password_char[kMAX];
    for (size_t i = 0; i < camera_ip_.size(); i++) {
      memset(ip_char, '\0', kMAX);
      memset(username_char, '\0', kMAX);
      memset(password_char, '\0', kMAX);
      camera_ip_.at(i).copy(ip_char, camera_ip_.at(i).size());
      camera_username_.at(i).copy(username_char, camera_username_.at(i).size());
      camera_password_.at(i).copy(password_char, camera_password_.at(i).size());
      camera_handle_.emplace_back(hc_camera_interface_.StartCamera(
          ip_char, username_char, password_char,
          atoi(camera_port_.at(i).c_str())));
    }
  }
}

void HcCameraInterfaceNode::Publish() {
  cv::Mat video[MAX_CAMERA_NUM];
  ros::Rate loop_rate(kFPS);
  sensor_msgs::ImagePtr msg;
  while (ros::ok) {
    for (size_t idx = 0; idx < camera_handle_.size(); ++idx) {
      if (camera_handle_[idx] >= 0 &&
          hc_camera_interface_.get_image_mat(video[idx], camera_handle_[idx]) &&
          video[idx].cols > 0 && video[idx].rows > 0) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        msg = cv_bridge::CvImage(header, "bgr8", video[idx]).toImageMsg();
        camera_publisher.at(idx)->publish(msg);
        if (is_show_) cv::imshow(std::to_string(idx), video[idx]);
      }
    }
    if (is_show_) cv::waitKey(kFPS);
    loop_rate.sleep();
  }
}
}