#include "hc_camera_interface.h"
#include <opencv2/opencv.hpp>

namespace driver {

LONG HcCameraInterface::n_port_[MAX_CAMERA_NUM] = {0};
IplImagePtr HcCameraInterface::pImg_[MAX_CAMERA_NUM] = {nullptr};
std::mutex HcCameraInterface::data_mutex_[MAX_CAMERA_NUM] = {};
bool HcCameraInterface::image_update_[MAX_CAMERA_NUM] = {false};

HcCameraInterface::HcCameraInterface() {}

HcCameraInterface::~HcCameraInterface() {}

void HcCameraInterface::InitHcNetSdk() {
  NET_DVR_Init();
  NET_DVR_SetConnectTime(HC_CONNECT_WAIT_TIME, 1);
  NET_DVR_SetReconnect(HC_CONNECT_INTERVAL, true);
  printf("Init HcNetSdk Finish!\n");
};

long HcCameraInterface::StartCamera(char *in_camera_ip, char *in_user_name,
                                    char *in_password, int in_port) {
  NET_DVR_DEVICEINFO_V30 struDeviceInfo;
  int user_id = NET_DVR_Login_V30(in_camera_ip, in_port, in_user_name,
                                  in_password, &struDeviceInfo);
  if (user_id < 0) {
    printf("Login ip:%s,user_name:%s,password:%s,port:%d| Error:%d!\n",
           in_camera_ip, in_user_name, in_password, in_port,
           NET_DVR_GetLastError());
    return -1;
  } else
    printf("Login ip:%s,user_name:%s,password:%s,port:%d| Succed!\n",
           in_camera_ip, in_user_name, in_password, in_port,
           NET_DVR_GetLastError());

  NET_DVR_PREVIEWINFO struPreviewInfo = {0};
  struPreviewInfo.lChannel = HC_CHANEL;
  struPreviewInfo.hPlayWnd = NULL;
  struPreviewInfo.dwStreamType = 0;  // 0 main stream
  struPreviewInfo.dwLinkMode = 1;    // 0 tcp,1 udp
  struPreviewInfo.bBlocked = 0;      // 0 unblocked, 1 blocked

  long camera_handle =
      NET_DVR_RealPlay_V40(user_id, &struPreviewInfo, RealDataCallBack, NULL);
  if (camera_handle < 0) {
    printf("NET_DVR_RealPlay_V40 :RealPlay %s:%d failed, error = %d\n",
           in_camera_ip, HC_CHANEL, NET_DVR_GetLastError());
    NET_DVR_Logout(user_id);
    return camera_handle;
  } else {
    camera_handles_.emplace_back(camera_handle);
    printf("RealStream from camera id:%ld,camera num:%lu\n", camera_handle,
           camera_handles_.size());
  }
  this->running_ = true;
  return camera_handle;
};

void CALLBACK HcCameraInterface::RealDataCallBack(LONG lRealHandle,
                                                  DWORD dwDataType,
                                                  BYTE *pBuffer,
                                                  DWORD dwBufSize,
                                                  void *pUser) {
  DWORD dRet;
  if (lRealHandle > MAX_CAMERA_NUM) {
    printf("Camera id:%d LG %d\n", lRealHandle, MAX_CAMERA_NUM);
    return;
  }
  switch (dwDataType) {
    case NET_DVR_SYSHEAD:                          // SYSTEM HEAD
      if (!PlayM4_GetPort(&n_port_[lRealHandle]))  // get device port id
      {
        printf("Camera id:%d, n_port %d\n", lRealHandle, n_port_[lRealHandle]);
        break;
      }
      if (dwBufSize > 0) {
        if (!PlayM4_SetStreamOpenMode(n_port_[lRealHandle], STREAME_REALTIME)) {
          std::cout << PlayM4_GetLastError(n_port_[lRealHandle]) << std::endl;
          break;
        }
        if (!PlayM4_OpenStream(n_port_[lRealHandle], pBuffer, dwBufSize,
                               1024 * 1024)) {
          std::cout << PlayM4_GetLastError(n_port_[lRealHandle]) << std::endl;
          break;
        }
        // set decode record
        if (!PlayM4_SetDecCallBack(n_port_[lRealHandle], DecCBFun)) {
          std::cout << PlayM4_GetLastError(n_port_[lRealHandle]) << std::endl;
          break;
        }
        // make I frame
        NET_DVR_MakeKeyFrame(lRealHandle, HC_CHANEL);

        if (!PlayM4_Play(n_port_[lRealHandle], NULL)) {
          std::cout << PlayM4_GetLastError(n_port_[lRealHandle]) << std::endl;
          break;
        }
      }
      break;

    case NET_DVR_STREAMDATA:  // stream
      if (dwBufSize > 0 && n_port_[lRealHandle] != -1) {
        BOOL inData =
            PlayM4_InputData(n_port_[lRealHandle], pBuffer, dwBufSize);
      }
      break;
  }
}

void CALLBACK HcCameraInterface::DecCBFun(int nPort, char *pBuf, int nSize,
                                          FRAME_INFO *pFrameInfo,
                                          void *nReserved1, int nReserved2) {
  static IplImagePtr pImgYCrCb[MAX_CAMERA_NUM];
  if (pFrameInfo->nType == T_YV12) {
    if (pImgYCrCb[nPort] == nullptr) {
      pImgYCrCb[nPort] = IplImagePtr(cvCreateImage(
          cvSize(pFrameInfo->nWidth, pFrameInfo->nHeight), IPL_DEPTH_8U, 3));
    }
    if (pImg_[nPort] == nullptr) {
      pImg_[nPort] = IplImagePtr(cvCreateImage(
          cvSize(pFrameInfo->nWidth, pFrameInfo->nHeight), IPL_DEPTH_8U, 3));
      cvResize(pImgYCrCb[nPort].get(), pImg_[nPort].get(), CV_INTER_LINEAR);
    }
    Yv12ToYuv(pImgYCrCb[nPort]->imageData, pBuf, pFrameInfo->nWidth,
              pFrameInfo->nHeight, pImgYCrCb[nPort]->widthStep);
    if (data_mutex_[nPort].try_lock()) {
      image_update_[nPort] = true;
      cvCvtColor(pImgYCrCb[nPort].get(), pImg_[nPort].get(), CV_YCrCb2RGB);
      data_mutex_[nPort].unlock();
    }
  }
}

void HcCameraInterface::Yv12ToYuv(char *outYuv, char *inYv12, int width,
                                  int height, int widthStep) {
  int col, row;
  unsigned int Y, U, V;
  int tmp;
  int idx;

  for (row = 0; row < height; row++) {
    idx = row * widthStep;
    int rowptr = row * width;

    for (col = 0; col < width; col++) {
      // int colhalf=col>>1;
      tmp = (row / 2) * (width / 2) + (col / 2);
      Y = (unsigned int)inYv12[row * width + col];
      U = (unsigned int)inYv12[width * height + width * height / 4 + tmp];
      V = (unsigned int)inYv12[width * height + tmp];

      outYuv[idx + col * 3] = Y;
      outYuv[idx + col * 3 + 1] = U;
      outYuv[idx + col * 3 + 2] = V;
    }
  }
}
int HcCameraInterface::get_image_mat(cv::Mat &Img, long camera_id) {
  int iPort = n_port_[camera_id];
  if (iPort != -1 && pImg_[iPort] && image_update_[camera_id] &&
      pImg_[iPort]->width > 1 && pImg_[iPort]->height > 1) {
    if (data_mutex_[camera_id].try_lock()) {
      Img.release();
      image_update_[camera_id] = false;
      cv::cvarrToMat(pImg_[camera_id].get()).copyTo(Img);
      data_mutex_[camera_id].unlock();
      return 1;
    }
  }
  return 0;
};
}  // namespace driver