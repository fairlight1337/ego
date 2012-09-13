#include "Visualization.h"

Visualization::Visualization() {
  m_bInitialized = (glfwInit() == GL_TRUE ? true : false);

  if(m_bInitialized) {
    int nMajor, nMinor, nRev;
    glfwGetVersion(&nMajor, &nMinor, &nRev);
    ROS_INFO("Running GLFW version %d.%d (rev %d)", nMajor, nMinor, nRev);
    m_imgCameraFrame.Data = NULL;
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
  
  m_mtxCameraFrame.unlock();
}

void Visualization::drawInterface() {
}
