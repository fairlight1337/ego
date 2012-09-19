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

#include "Map.h"


Map::Map(ros::NodeHandle handleNode, string strMapTopic, unsigned int unXDimension, unsigned int unYDimension) {
  // Initialize an unknown map
  m_unXDimension = unXDimension;
  m_unYDimension = unYDimension;
  
  m_cMapData = (char*)malloc(m_unXDimension * m_unYDimension);  
  m_ucTextureData = (unsigned char*)malloc(m_unXDimension * m_unYDimension * 3);

  m_subMapTopic = handleNode.subscribe<nav_msgs::OccupancyGrid>(strMapTopic, 10, &Map::mapCallback, this);
  
  m_bInitialized = false;
}

Map::~Map() {
  if(m_bInitialized) {
    free(m_cMapData);
    free(m_ucTextureData);
    
    glDeleteTextures(1, &m_unTextureMap);
  }
}

void Map::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  float fResolution = msg->info.resolution;
  unsigned int nWidth = msg->info.width;
  unsigned int nHeight = msg->info.height;

  cout << "Map received. Dimensions: " << nWidth << "x" << nHeight << endl;
  cout << "Resolution: " << fResolution << endl;
  cout << "Origin: " << msg->info.origin.position.x / fResolution << ", " << msg->info.origin.position.y / fResolution << endl;

  int nEffectiveMapWidth = min(nWidth, m_unXDimension);
  int nEffectiveMapHeight = min(nHeight, m_unYDimension);
  
  int nLeftRightPadding = (nWidth - nEffectiveMapWidth) / 2;
  int nTopBottomPadding = (nHeight - nEffectiveMapHeight) / 2;
  
  for(unsigned int unX = 0; unX < nEffectiveMapWidth; unX++) {
    for(unsigned int unY = 0; unY < nEffectiveMapHeight; unY++) {
      int nDataIndex = (nTopBottomPadding * nWidth) + ((unY + 1) * nLeftRightPadding) + (unY * (nLeftRightPadding + nEffectiveMapWidth)) + unX;
      //int nDataIndex = (nLeftRightPadding * nHeight) + ((unX + 1) * nTopBottomPadding) + (unX * (nTopBottomPadding + nEffectiveMapHeight)) + unY;
      
      char cValue = msg->data[nDataIndex];
      this->setMapTile((m_unYDimension - 1) - unY, (m_unXDimension - 1) - unX, cValue);
    }
  }

  regenerateMapTexture();
}

void Map::initializeMapDisplay() {
  // Prepare the GL texture for the map display
  glGenTextures(1, &m_unTextureMap);
  glBindTexture(GL_TEXTURE_2D, m_unTextureMap);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_unXDimension, m_unYDimension, 0, GL_RGB, GL_UNSIGNED_BYTE, m_ucTextureData);

  clearMap();

  m_bInitialized = true;
}

void Map::clearMap(char cValue) {
  // -1 is the default value for cValue.
  for(unsigned int unX = 0; unX < m_unXDimension; unX++) {
    for(unsigned int unY = 0; unY < m_unYDimension; unY++) {
      setMapTile(unX, unY, cValue);
    }
  }

  regenerateMapTexture();
}

void Map::setMapTile(unsigned int unX, unsigned int unY, char cValue) {
  m_mtxTextureData.lock();
  
  // Per definition: -1: unknown, 0: free, 1: obstructed
  m_cMapData[unY * m_unYDimension + unX] = cValue;

  char cR = 0, cG = 0, cB = 0;
  
  if(cValue == -1) {
    cR = 55;
    cG = 55;
    cB = 55;
  } else if(cValue == 0) {
    cR = 200;
    cG = 200;
    cB = 200;
  } else {
    cR = 0;
    cG = 0;
    cB = 0;
  }

  m_ucTextureData[(unY * m_unYDimension + unX) * 3 + 0] = cR;
  m_ucTextureData[(unY * m_unYDimension + unX) * 3 + 1] = cG;
  m_ucTextureData[(unY * m_unYDimension + unX) * 3 + 2] = cB;
  
  m_mtxTextureData.unlock();
}

char Map::getMapTile(unsigned int unX, unsigned int unY) {
  char cReturnvalue;
  
  m_mtxTextureData.lock();
  cReturnvalue = m_cMapData[unY * m_unYDimension + unX];
  m_mtxTextureData.unlock();
  
  return cReturnvalue;
}

void Map::regenerateMapTexture() {
  m_mtxMapTexture.lock();
  m_mtxTextureData.lock();
  
  glBindTexture(GL_TEXTURE_2D, m_unTextureMap);
  gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, m_unXDimension, m_unYDimension, GL_RGB, GL_UNSIGNED_BYTE, m_ucTextureData);
  
  m_mtxTextureData.unlock();
  m_mtxMapTexture.unlock();
}

void Map::drawMap() {
  float fQuadHeight = 0.65;
  float fQuadWidth = fQuadHeight;
  float fXOffset = -0.89;
  float fYOffset = 0.575;
  
  m_mtxMapTexture.lock();
  
  glLoadIdentity();
  glBindTexture(GL_TEXTURE_2D, m_unTextureMap);
  glLoadIdentity();

  unsigned int unEdges = 16;
  
  glTranslatef(fXOffset, fYOffset, -2.5);
  glRotatef((m_tfRobotPose.rotation.z * 180.0 * (unEdges / 4)) / 3.1415, 0, 0, 1);

  glBegin(GL_POLYGON);
  {
    glColor3f(1, 1, 1);
    float fDegreePerEdge = 360 / unEdges;
    float fDegreeStartAngle = 180;
    float fRadiansPerEdge = (fDegreePerEdge / 180.0) * 3.1415;
    float fRadiansStartAngle = (fDegreeStartAngle / 180.0) * 3.1415;
    
    for(unsigned int unEdge = 0; unEdge < unEdges; unEdge++) {
      float fXbiased = sin((float)unEdge * fRadiansPerEdge + fRadiansStartAngle);
      float fYbiased = cos((float)unEdge * fRadiansPerEdge + fRadiansStartAngle);
      float fXtexture = sin((float)unEdge * fRadiansPerEdge);
      float fYtexture = cos((float)unEdge * fRadiansPerEdge);

      glTexCoord2d((fXtexture + 0.5) * fQuadWidth, (fYtexture + 0.5) * fQuadHeight);
      glVertex2f(fQuadWidth / 2 * fXbiased, fQuadHeight / 2 * fYbiased);
    }
  }
  glEnd();

  m_mtxMapTexture.unlock();
}

void Map::setRobotPose(geometry_msgs::Transform tfRobotPose) {
  m_tfRobotPose = tfRobotPose;

  regenerateMapTexture();
}
