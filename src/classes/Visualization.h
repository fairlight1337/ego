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

#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <ros/ros.h>
#include <GL/glfw.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <boost/thread.hpp>
#include <string>

#include "Map.h"

using namespace std;


class Visualization {
 private:
  bool m_bInitialized;
  boost::mutex m_mtxCameraFrame;
  GLFWimage m_imgCameraFrame;
  GLuint m_unTexture;
  Map *m_mapMap;

 public:
  Visualization(ros::NodeHandle handleNode, string strMapTopic);
  ~Visualization();

  bool startVisualization();
  void setQuitCallback(GLFWwindowclosefun fncQuitCallback);
  void setKeyboardInputCallback(GLFWkeyfun fncKeyboardInputCallback);
  
  void setCameraFrame(GLFWimage imgCameraFrame);

  void drawFrame();
  void drawCameraFrame();
  void drawInterface();
  void drawMap();
};

#endif /* __VISUALIZATION_H__ */
