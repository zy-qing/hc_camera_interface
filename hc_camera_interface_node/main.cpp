#include <unistd.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "hc_camera_interface_node.h"
#include "opencv2/core/cvstd.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hc_camera_interface");

  driver::HcCameraInterfaceNode node;

  node.Run();

  return 0;
  // driver::HcCameraInterface camera_interface;
  // camera_interface.InitHcNetSdk();

  // std::vector<long> camera_id;
  // camera_id.emplace_back(
  //     camera_interface.StartCamera("192.168.1.64", "admin", "root1234",
  //     8000));
  // sleep(1);
  // camera_id.emplace_back(
  //     camera_interface.StartCamera("10.14.2.201", "admin", "root1234",
  //     8000));

  // cv::Mat video[MAX_CAMERA_NUM];
  // while (1) {
  //   for (size_t idx = 0; idx < camera_id.size(); ++idx) {
  //     if (camera_id[idx] >= 0 &&
  //         camera_interface.get_image_mat(video[idx], camera_id[idx])) {
  //       cv::imshow(std::to_string(idx), video[idx]);
  //     }
  //   }
  //   cv::waitKey(30);
  // }
  // sleep(100);
}