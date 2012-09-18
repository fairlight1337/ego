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

#include "Visualization.h"

Visualization::Visualization(ros::NodeHandle handleNode, string strCollisionMapTopic) {
  m_bInitialized = (glfwInit() == GL_TRUE ? true : false);

  if(m_bInitialized) {
    int nMajor, nMinor, nRev;
    glfwGetVersion(&nMajor, &nMinor, &nRev);
    ROS_INFO("Running GLFW version %d.%d (rev %d)", nMajor, nMinor, nRev);
    m_imgCameraFrame.Data = NULL;

    m_mapMap = new Map(handleNode, strCollisionMapTopic, 100, 100);
  } else {
    ROS_INFO("GLFW failed to initialize.");
  }
}

Visualization::~Visualization() {
  if(m_bInitialized) {
    glfwCloseWindow();
    glfwTerminate();
  }
}

bool Visualization::startVisualization() {
  int nWidth = 640, nHeight = 480;
  int nBitsPerComponent = 8, nDepthBits = 0, nStencilBits = 0;
  int nOpenGLMode = GLFW_WINDOW;

  if(glfwOpenWindow(nWidth, nHeight, nBitsPerComponent, nBitsPerComponent, nBitsPerComponent, nBitsPerComponent, nDepthBits, nStencilBits, nOpenGLMode)) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float fAspectRatio = ((float)nHeight) / ((float)nWidth);
    glFrustum(.5, -.5, -.5 * fAspectRatio, .5 * fAspectRatio, 1, 50);
    glMatrixMode(GL_MODELVIEW);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glEnable(GL_TEXTURE_2D);

    glGenTextures(1, &m_unTexture);

    return GL_TRUE;
  }

  return GL_FALSE;
}

void Visualization::setQuitCallback(GLFWwindowclosefun fncQuitCallback) {
  glfwSetWindowCloseCallback(fncQuitCallback);
}

void Visualization::setKeyboardInputCallback(GLFWkeyfun fncKeyboardInputCallback) {
  glfwSetKeyCallback(fncKeyboardInputCallback);
}

void Visualization::setCameraFrame(GLFWimage imgCameraFrame) {
  m_mtxCameraFrame.lock();

  if(m_imgCameraFrame.Data != NULL) {
    free(m_imgCameraFrame.Data);
  }

  m_imgCameraFrame = imgCameraFrame;
  m_imgCameraFrame.Data = (unsigned char*)malloc(imgCameraFrame.Width * imgCameraFrame.Height * m_imgCameraFrame.BytesPerPixel);
  memcpy(m_imgCameraFrame.Data, imgCameraFrame.Data, imgCameraFrame.Width * imgCameraFrame.Height * m_imgCameraFrame.BytesPerPixel);

  glBindTexture(GL_TEXTURE_2D, m_unTexture);
  gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, m_imgCameraFrame.Width, m_imgCameraFrame.Height, GL_RGB, GL_UNSIGNED_BYTE, m_imgCameraFrame.Data);

  m_mtxCameraFrame.unlock();
}

void Visualization::drawFrame() {
  drawCameraFrame();
  drawInterface();

  glfwSwapBuffers();
}

void Visualization::drawCameraFrame() {
  glLoadIdentity();

  m_mtxCameraFrame.lock();
  
  if(m_imgCameraFrame.Data != NULL) {
    float fQuadHeight = 2;
    float fQuadWidth = fQuadHeight * m_imgCameraFrame.Width / m_imgCameraFrame.Height;
    
    glBindTexture(GL_TEXTURE_2D, m_unTexture);
    
    glTranslatef(0, 0, -2.65);
    glBegin(GL_QUADS);
    {
      glColor3f(1, 1, 1);
      
      glVertex2f(fQuadWidth / 2, fQuadHeight / 2);
      glTexCoord2d(0, 1);
      glVertex2f(fQuadWidth / 2, -fQuadHeight / 2);
      glTexCoord2d(1, 1);
      glVertex2f(-fQuadWidth / 2, -fQuadHeight / 2);
      glTexCoord2d(1, 0);
      glVertex2f(-fQuadWidth / 2, fQuadHeight / 2);
      glTexCoord2d(0, 0);
    }
    glEnd();
  }

  m_mtxCameraFrame.unlock();
}

void Visualization::drawInterface() {
  drawMap();
}

void Visualization::drawMap() {
  m_mapMap->drawMap();
}
