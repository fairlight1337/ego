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

#include "RobotBase.h"


RobotBase::RobotBase(ros::NodeHandle handleNode, string strTopic, string strTFTopic) {
  m_pubVelocity = handleNode.advertise<geometry_msgs::Twist>(strTopic, 1);
  m_subTFTopic = handleNode.subscribe<tf::tfMessage>(strTFTopic, 10, &RobotBase::robotPoseCallback, this);
}

RobotBase::~RobotBase() {
}

void RobotBase::robotPoseCallback(const tf::tfMessage::ConstPtr &msg) {
  string strFrameID = msg->transforms[0].header.frame_id;
  string strChildFrameID = msg->transforms[0].child_frame_id;

  if(strFrameID == "/map" && strChildFrameID == "/odom_combined") {
    m_tfRobotPose.translation = msg->transforms[0].transform.translation;
  } else if(strFrameID == "/odom_combined" && strChildFrameID == "/base_footprint") {
    m_tfRobotPose.rotation = msg->transforms[0].transform.rotation;
  }
}

geometry_msgs::Transform RobotBase::currentRobotPose() {
  return m_tfRobotPose;
}

void RobotBase::sendVelocity(float fX, float fY, float fW) {
  geometry_msgs::Twist cmdvel;
  
  cmdvel.linear.x = fX;
  cmdvel.linear.y = fY;
  cmdvel.angular.z = fW;

  m_pubVelocity.publish(cmdvel);
}
