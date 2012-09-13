#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <GL/glfw.h>
#include <string>
#include <cv_bridge/CvBridge.h>
#include <opencv2/opencv.hpp>

using namespace std;


class Camera {
 private:
  boost::mutex m_mtxCameraFrame;
  GLFWimage m_imgLastFrame;
  image_transport::Subscriber m_subCameraTopic;
  
 public:
  Camera(ros::NodeHandle handleNode, string strTopic);
  ~Camera();

  GLFWimage getCameraFrame();
  GLFWimage convertImgMsgToImage(const sensor_msgs::ImageConstPtr& msg);
  void frameCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif /* __CAMERA_H__ */
