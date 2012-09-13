#include <iostream>
#include <ros/ros.h>
#include <GL/glfw.h>
#include <string>

#include "classes/RobotHead.h"
#include "classes/Camera.h"
#include "classes/RobotBase.h"
#include "classes/Visualization.h"

using namespace std;


bool bRunning = true;


int guiQuit() {
  bRunning = false;

  return GL_TRUE;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ego");
  ros::NodeHandle handleNode;
  
  // Initialize robot components
  RobotHead head;
  RobotBase base(handleNode, "base_controller/command");
  Camera cam(handleNode, "/wide_stereo/left/image_color");
  ROS_INFO("Initialized robot components.");

  // Initialize visualization and gui
  Visualization vis;
  vis.startVisualization();
  vis.setQuitCallback(guiQuit);
  ROS_INFO("Initialized visualization and gui.");
    
  base.sendVelocity(1.0, 1.0, 0.0);
  
  bRunning = true;
  while(bRunning) {
    // ROS cycle
    ros::spinOnce();

    // Data allocation and memory operations
    vis.setCameraFrame(cam.getCameraFrame());

    // Drawing camera image and interface
    vis.drawFrame();
  }
  
  base.sendVelocity(0.0, 0.0, 0.0);

  return EXIT_SUCCESS;
}
