#ifndef CAMERA_INTERFACE_H_
#define CAMERA_INTERFACE_H_
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include "HCNetSDK.h"
#include "LinuxPlayM4.h"
#include "opencv2/core/core.hpp"

#define HC_CAMERA_IP "192.168.1.64"
#define HC_DATA_PORT 8000
#define HC_USER_NAME "admin"
#define HC_PASSWORD "root1234"
#define HC_CHANEL 1
#define HC_CONNECT_WAIT_TIME 2000
#define HC_CONNECT_INTERVAL 10000

#define HC_MAX_BUFFER 1024 * 1024
#define MAX_CAMERA_NUM 10

namespace driver {
typedef std::shared_ptr<IplImage> IplImagePtr;
class HcCameraInterface {
 public:
  HcCameraInterface();

  void InitHcNetSdk();

  long StartCamera(char *in_camera_ip, char *in_user_name, char *in_password,
                   int in_port);

  void StopCamera();

  int get_image_mat(cv::Mat &Img, long camera_id);

  ~HcCameraInterface();

 private:
  bool running_ = false;
  int camera_num_ = 0;
  std::vector<long> camera_handles_;

  static LONG n_port_[MAX_CAMERA_NUM];
  static IplImagePtr pImg_[MAX_CAMERA_NUM];
  static std::mutex data_mutex_[MAX_CAMERA_NUM];
  static bool image_update_[MAX_CAMERA_NUM];

  static void Yv12ToYuv(char *outYuv, char *inYv12, int width, int height,
                        int widthStep);
  static void CALLBACK RealDataCallBack(LONG lRealHandle, DWORD dwDataType,
                                        BYTE *pBuffer, DWORD dwBufSize,
                                        void *pUser);
  static void CALLBACK DecCBFun(int nPort, char *pBuf, int nSize,
                                FRAME_INFO *pFrameInfo, void *nReserved1,
                                int nReserved2);
};

}  // namespace driver

#endif