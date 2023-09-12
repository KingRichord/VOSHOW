//
// Created by moi on 23-9-8.
//

#ifndef LISTENER_MYGL_H
#define LISTENER_MYGL_H
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <GL/glu.h>
#include "Eigen/Eigen"
#include "sophus/se3.hpp"



class Scene {
 public:
  Scene(int width = 800, int height = 600, const char* title = "3D Scene Example")
      : m_width(width), m_height(height), m_title(title) {}
  
  ~Scene() {}
  inline void draw_line(const float x1, const float y1, const float z1,
                        const float x2, const float y2, const float z2) const {
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
  }
  void drawHorizontalGrid();
 void run();

 private:
  // 窗口的宽高和标题
  int m_width;
  int m_height;
  const char* m_title;
  
  //
  double lastScrollValue;
  
  
  static bool firstMouse;
  static float yaw;
  static float pitch;
  static float lastX;
  static float lastY;
  
  
  
  void updateCameraView();
  void setCameraView(const Sophus::SE3f& cameraPose);
  // 相机参数
  static Sophus::SE3f m_cameraPose;
  
  // 键盘鼠标事件
  static bool m_isKeyPressed[4];
  static float mKeyscroll;

  
  // 渲染函数
  void render();
  
  // 键盘事件回调函数
  static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_W)
      m_isKeyPressed[0] = (action == GLFW_PRESS || action == GLFW_REPEAT);
    else if (key == GLFW_KEY_S)
      m_isKeyPressed[1] = (action == GLFW_PRESS || action == GLFW_REPEAT);
    else if (key == GLFW_KEY_A)
      m_isKeyPressed[2] = (action == GLFW_PRESS || action == GLFW_REPEAT);
    else if (key == GLFW_KEY_D)
      m_isKeyPressed[3] = (action == GLFW_PRESS || action == GLFW_REPEAT);
    }
    
  static void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
  {
    // float xpos = static_cast<float>(xposIn);
    // float ypos = static_cast<float>(yposIn);
    //
    // if (firstMouse)
    // {
    //   lastX = xpos;
    //   lastY = ypos;
    //   firstMouse = false;
    // }
    //
    // float xoffset = xpos - lastX;
    // float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    // lastX = xpos;
    // lastY = ypos;
    //
    // float sensitivity = 0.1f; // change this value to your liking
    // xoffset *= sensitivity;
    // yoffset *= sensitivity;
    //
    // yaw += xoffset;
    // pitch += yoffset;
    //
    // // make sure that when pitch is out of bounds, screen doesn't get flipped
    // if (pitch > 89.0f)
    //   pitch = 89.0f;
    // if (pitch < -89.0f)
    //   pitch = -89.0f;
  }
  
  static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
  {
    if (yoffset > 0)
      mKeyscroll += 1;
    else
      mKeyscroll -= 1;
  }
  
  
  static void mouseScrollEndCallback(GLFWwindow* window, int button, int action, int mods)
  {
    
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_REPEAT)
    {
      std::cout <<"test" <<std::endl;
    }
  }
  
  
};

#endif  //LISTENER_MYGL_H
