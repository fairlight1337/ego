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
bool bArrayKeyState[318]; // Check on this value if something fails
			  // during execution. The maximum number of
			  // keys possibly addressed in GLFW could
			  // change from release to release. Got to
			  // find a different solution to this.

int guiQuit() {
  bRunning = false;

  return GL_TRUE;
}

void GLFWCALL guiInputKeyboard(int nKey, int nAction) {
  if(nKey == GLFW_KEY_ESC && nAction == GLFW_PRESS) {
    bRunning = false;
  } else {
    bArrayKeyState[nKey] = nAction;
  }
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
  vis.setKeyboardInputCallback(guiInputKeyboard);
  ROS_INFO("Initialized visualization and gui.");
    
  //base.sendVelocity(1.0, 1.0, 0.0);
  
  bRunning = true;
  while(bRunning) {
    // ROS cycle
    ros::spinOnce();

    // Data allocation and memory operations
    vis.setCameraFrame(cam.getCameraFrame());

    // Drawing camera image and interface
    vis.drawFrame();

    // Calculate velocity to apply
    float fForward = 0;
    float fSidewards = 0;
    float fTurning = 0;

    if(bArrayKeyState['W']) {
      fForward = 1.0;
    } else if(bArrayKeyState['S']) {
      fForward = -1.0;
    }

    if(bArrayKeyState['A']) {
      fSidewards = 1.0;
    } else if(bArrayKeyState['D']) {
      fSidewards = -1.0;
    }
    
    if(bArrayKeyState['Q']) {
      fTurning = 1.0;
    } else if(bArrayKeyState['E']) {
      fTurning = -1.0;
    }

    base.sendVelocity(fForward, fSidewards, fTurning);
  }
  
  base.sendVelocity(0.0, 0.0, 0.0);

  return EXIT_SUCCESS;
}
