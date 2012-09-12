#include <iostream>
#include <ros/ros.h>

#include "classes/RobotHead.h"
#include "classes/Visualization.h"

using namespace std;


int main(int argc, char **argv) {
  ros::init(argc, argv, "ego");
  
  // Initialize robot components
  RobotHead head;
  ROS_INFO("Initialized robot components.");

  // Initialize visualization and gui
  Visualization vis;
  ROS_INFO("Initialized visualization and gui.");
  
  ros::spin();
  
  return EXIT_SUCCESS;
}
