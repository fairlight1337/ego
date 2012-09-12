#include <iostream>
#include <ros/ros.h>

#include "classes/RobotHead.h"

using namespace std;


int main(int argc, char **argv) {
  ros::init(argc, argv, "ego");
  
  RobotHead head;
  
  ros::spin();
  
  return EXIT_SUCCESS;
}
