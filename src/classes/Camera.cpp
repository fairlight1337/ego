#include "Camera.h"


Camera::Camera(ros::NodeHandle handleNode, string strTopic) {
  m_imgLastFrame.Data = NULL;
  image_transport::ImageTransport itImageTransport(handleNode);
  m_subCameraTopic = itImageTransport.subscribe(strTopic, 1, &Camera::frameCallback, this);
}

Camera::~Camera() {
}

GLFWimage Camera::convertImgMsgToImage(const sensor_msgs::ImageConstPtr& msg) {
  GLFWimage imgConverted;

  imgConverted.Width = msg->width;
  imgConverted.Height = msg->height;
  imgConverted.BytesPerPixel = 3;
  imgConverted.Data = (unsigned char*)malloc(imgConverted.Height * imgConverted.Width * imgConverted.BytesPerPixel);
  
  if(msg->encoding.find("bayer") != string::npos) {
    cv::Mat matTemp = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    IplImage iplImgGray = matTemp;
    IplImage *iplImg = cvCreateImage(cvSize(imgConverted.Width, imgConverted.Height), 8, 3);
    cvCvtColor(&iplImgGray, iplImg, CV_GRAY2BGR);

    for(int nY = 0; nY < imgConverted.Height; nY++) {
      for(int nX = 0; nX < imgConverted.Width; nX++) {
	for(int nRGB = 0; nRGB < imgConverted.BytesPerPixel; nRGB++) {
	  int nIndex = (nY * imgConverted.Width + nX) * imgConverted.BytesPerPixel + nRGB;
	  imgConverted.Data[nIndex] = iplImg->imageData[nIndex];
	}
      }
    }
  } else {
    for(int nY = 0; nY < imgConverted.Height; nY++) {
      for(int nX = 0; nX < imgConverted.Width; nX++) {
	if(msg->encoding.find("bgr") != string::npos) {
	  for(int nRGB = 0; nRGB < imgConverted.BytesPerPixel; nRGB++) {
	    int nIndexTo = (nY * imgConverted.Width + nX) * imgConverted.BytesPerPixel + nRGB;
	    int nIndexFrom = (nY * imgConverted.Width + nX) * imgConverted.BytesPerPixel + (imgConverted.BytesPerPixel - (nRGB + 1));
	    imgConverted.Data[nIndexTo] = msg->data[nIndexFrom];
	  }
	} else {
	  for(int nRGB = 0; nRGB < imgConverted.BytesPerPixel; nRGB++) {
	    int nIndex = (nY * imgConverted.Width + nX) * imgConverted.BytesPerPixel + nRGB;
	    imgConverted.Data[nIndex] = msg->data[nIndex];
	  }
	}
      }
    }
  }

  return imgConverted;
}

GLFWimage Camera::getCameraFrame() {
  GLFWimage imgTempFrame;

  m_mtxCameraFrame.lock();
  imgTempFrame = m_imgLastFrame;
  m_mtxCameraFrame.unlock();
  
  return imgTempFrame;
}

void Camera::frameCallback(const sensor_msgs::ImageConstPtr& msg) {
  m_mtxCameraFrame.lock();
  
  if(m_imgLastFrame.Data != NULL) {
    free(m_imgLastFrame.Data);
  }

  //ROS_INFO("%d/%d", msg->width, msg->height);
  //  ROS_INFO("%s", msg->encoding);
  //cout << msg->encoding << endl;

  m_imgLastFrame = convertImgMsgToImage(msg);
  m_mtxCameraFrame.unlock();
}
