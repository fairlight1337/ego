#include "RobotHead.h"


RobotHead::RobotHead() {
  //Initialize the client for the Action interface to the head controller
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  
  //wait for head controller action server to come up 
  while(!point_head_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the point_head_action server to come up");
  }
}

RobotHead::~RobotHead() {
  delete point_head_client_;
}

//! Points the high-def camera frame at a point in a given frame  
void RobotHead::lookAt(std::string frame_id, double x, double y, double z) {
  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;
  
  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; point.point.y = y; point.point.z = z;
  goal.target = point;
  
  //we are pointing the high-def camera frame 
  //(pointing_axis defaults to X-axis)
  //goal.pointing_frame = "high_def_frame";
  goal.pointing_frame = "/head_pan_link";
  
  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.5);
  
  //and go no faster than 1 rad/s
  goal.max_velocity = 10.0;
  
  //send the goal
  point_head_client_->sendGoal(goal);
  
  //wait for it to get there (abort after 2 secs to prevent getting stuck)
  point_head_client_->waitForResult(ros::Duration(2));
}

//! Shake the head from left to right n times  
void RobotHead::shakeHead(int n) {
  int count = 0;
  while (ros::ok() && ++count <= n )
    {
      //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
      lookAt("base_link", 5.0, 1.0, 1.2);
      
      //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
      lookAt("base_link", 5.0, -1.0, 1.2);
    }
}
