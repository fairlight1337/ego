#ifndef __ROBOT_HEAD_H__
#define __ROBOT_HEAD_H__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHead {
 private:
  PointHeadClient* point_head_client_;
  
 public:
  //! Action client initialization 
  RobotHead();
  ~RobotHead();
  
  //! Points the high-def camera frame at a point in a given frame  
  void lookAt(std::string frame_id, double x, double y, double z);
  
  //! Shake the head from left to right n times  
  void shakeHead(int n);
};

#endif /* __ROBOT_HEAD_H__ */
