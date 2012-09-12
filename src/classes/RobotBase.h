#ifndef __ROBOT_BASE_H__
#define __ROBOT_BASE_H__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>

class RobotBase {
 private:
  ros::NodeHandle m_handleNode;
  ros::Publisher m_pubVelocity;

 public:
  RobotBase(ros::NodeHandle handleNode);
  ~RobotBase();

  void sendVelocity(float fX, float fY, float fW);
};

#endif /* __ROBOT_BASE_H__ */
