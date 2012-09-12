#include <iostream>
#include <ros/ros.h>
#include <GL/glfw.h>

#include "classes/RobotHead.h"
#include "classes/Camera.h"
#include "classes/RobotBase.h"
#include "classes/Visualization.h"

using namespace std;


int guiQuit() {
  ROS_INFO("Quit requested.");

  return GL_FALSE;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ego");
  ros::NodeHandle handleNode;
  
  // Initialize robot components
  RobotHead head();
  RobotBase base(handleNode);
  ROS_INFO("Initialized robot components.");

  // Initialize visualization and gui
  Visualization vis;
  vis.startVisualization();
  vis.setQuitCallback(guiQuit);
  ROS_INFO("Initialized visualization and gui.");
  
  bool bRunning = true;
  while(bRunning) {
    ros::spinOnce();
    vis.drawFrame();
  }
  
  return EXIT_SUCCESS;
}
