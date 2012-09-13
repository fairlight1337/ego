// Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
