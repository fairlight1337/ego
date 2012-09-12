#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <ros/ros.h>
#include <GL/glfw.h>

class Visualization {
 private:
  bool m_bInitialized;

 public:
  Visualization();
  ~Visualization();

  bool startVisualization();
  void setQuitCallback(GLFWwindowclosefun fncQuitCallback);

  void drawFrame();
  void drawCameraFrame();
  void drawInterface();
};

#endif /* __VISUALIZATION_H__ */
