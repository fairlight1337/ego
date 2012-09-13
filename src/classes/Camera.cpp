#include "Camera.h"


Camera::Camera(ros::NodeHandle handleNode, string strTopic) {
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
  imgConverted.Data = (unsigned char*)malloc(imgConverted.Height * imgConverted.Width * 3);

  for(int nY = 0; nY < imgConverted.Height; nY++) {
    for(int nX = 0; nX < imgConverted.Width; nX++) {
      int nIndex = nY * imgConverted.Width + nX;
      imgConverted.Data[nIndex] = msg->data[nIndex];
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
  m_imgLastFrame = convertImgMsgToImage(msg);
  m_mtxCameraFrame.unlock();

  /*// May want to view raw bayer data, which CvBridge doesn't know about
  if (msg->encoding.find("bayer") != std::string::npos) {
    last_image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                          const_cast<uint8_t*>(&msg->data[0]), msg->step);
			  }*/
  /*// We want to scale floating point images so that they display nicely
  else if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image_bridge = img_bridge_.imgMsgToCv(msg, "passthrough");
    cv::Mat_<float> float_image = float_image_bridge;
    float max_val = 0;
    for(int i = 0; i < float_image.rows; ++i)
    {
      for(int j = 0; j < float_image.cols; ++j)
      {
        max_val = std::max(max_val, float_image(i, j));
      }
    }

    if(max_val > 0)
    {
      float_image /= max_val;
    }
    last_image_ = float_image;
  }
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = img_bridge_.imgMsgToCv(msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8",
                             msg->encoding.c_str());
    }
    }*/
}
