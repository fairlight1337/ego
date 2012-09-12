#include "Visualization.h"

Visualization::Visualization() {
  m_bInitialized = (glfwInit() == GL_TRUE ? true : false);

  if(m_bInitialized) {
    int nMajor, nMinor, nRev;
    glfwGetVersion(&nMajor, &nMinor, &nRev);
    ROS_INFO("Running GLFW version %d.%d (rev %d)", nMajor, nMinor, nRev);
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
    return GL_TRUE;
  }

  return GL_FALSE;
}

void Visualization::setQuitCallback(GLFWwindowclosefun fncQuitCallback) {
  glfwSetWindowCloseCallback(fncQuitCallback);
}

void Visualization::drawFrame() {
  drawCameraFrame();
  drawInterface();

  glfwSwapBuffers();
}

void Visualization::drawCameraFrame() {
}

void Visualization::drawInterface() {
}
