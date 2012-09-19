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

#ifndef __MAP_H__
#define __MAP_H__

#include <ros/ros.h>
#include <GL/glfw.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <malloc.h>
#include <boost/thread.hpp>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;


class Map {
 private:
  unsigned int m_unXDimension;
  unsigned int m_unYDimension;
  char *m_cMapData;
  unsigned char *m_ucTextureData;
  GLuint m_unTextureCollisionMap;
  boost::mutex m_mtxMapTexture;
  boost::mutex m_mtxTextureData;
  bool m_bInitialized;
  ros::Subscriber m_subCollisionMapTopic;
  
 public:
  Map(ros::NodeHandle handleNode, string strCollisionMapTopic, unsigned int unXDimension, unsigned int unYDimension);
  ~Map();

  void collisionMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void initializeMapDisplay();
  void clearMap(char cValue = -1);
  void setMapTile(unsigned int unX, unsigned int unY, char cValue);
  char getMapTile(unsigned int unX, unsigned int unY);
  void regenerateMapTexture();
  void drawMap();
};

#endif /* __MAP_H__ */
