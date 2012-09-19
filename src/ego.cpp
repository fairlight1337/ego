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
  RobotBase base(handleNode, "base_controller/command", "/odom_combined");
  Camera cam(handleNode, "/wide_stereo/left/image_color");
  ROS_INFO("Initialized robot components.");

  // Initialize visualization and gui
  Visualization vis(handleNode, "/map");
  vis.startVisualization();
  vis.setQuitCallback(guiQuit);
  vis.setKeyboardInputCallback(guiInputKeyboard);
  ROS_INFO("Initialized visualization and gui.");

  // Set ROS and environment parameters
  ros::Rate rate(10.0);
    
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

    // Make sure we're not sending velocity commands when there is
    // nothing to send. This way, we don't interfere with other nodes
    // that actually send commands. This is useful for just observing
    // what's going on.
    if(fForward != 0.0 || fSidewards != 0.0 || fTurning != 0.0) {
      base.sendVelocity(fForward, fSidewards, fTurning);
    }
  }
  
  return EXIT_SUCCESS;
}
