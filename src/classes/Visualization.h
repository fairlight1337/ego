#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <ros/ros.h>
#include <GL/glfw.h>
#include <boost/thread.hpp>

class Visualization {
 private:
  bool m_bInitialized;
  boost::mutex m_mtxCameraFrame;
  GLFWimage m_imgCameraFrame;

 public:
  Visualization();
  ~Visualization();

  bool startVisualization();
  void setQuitCallback(GLFWwindowclosefun fncQuitCallback);
  
  void setCameraFrame(GLFWimage imgCameraFrame);

  void drawFrame();
  void drawCameraFrame();
  void drawInterface();
};

#endif /* __VISUALIZATION_H__ */
