#include "RobotBase.h"


RobotBase::RobotBase(ros::NodeHandle handleNode) {
  m_handleNode = handleNode;
  m_pubVelocity = m_handleNode.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

RobotBase::~RobotBase() {
}

void RobotBase::sendVelocity(float fX, float fY, float fW) {
  geometry_msgs::Twist cmdvel;
  
  cmdvel.linear.x = fX;
  cmdvel.linear.y = fY;
  cmdvel.angular.z = fW;

  m_pubVelocity.publish(cmdvel);
}
