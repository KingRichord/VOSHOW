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
#define DOUBLE_CLICK_MIN_DT 0.02
#define DOUBLE_CLICK_MAX_DT 0.2

struct events_t {
  std::vector<unsigned int> characters;
  Eigen::Vector2f scroll_offset;
  bool double_click = false;
  Eigen::Vector2i double_click_position;
  double last_left_click_time = -std::numeric_limits<double>::max();
};
struct MouseStates {
  bool mouse_left;
  bool mouse_middle;
  bool mouse_right;
  bool mouse_double_click;
  bool control_left;
  bool control_right;
  bool shift_left;
  bool shift_right;
  Eigen::Vector2f mouse_normal_position;
  Eigen::Vector2f mouse_drag_position;
  Eigen::Vector2f scroll;
};
struct viewport_t {
  Eigen::Vector2i window_size;
  Eigen::Vector2i framebuffer_size;
  Eigen::Vector3f viewport_ypr = {-45, -42, 0};
  float viewport_distance = 15;
  Eigen::Vector3f world_xyz = {0, 0, 0};
  float scale = 1.0;
};

struct context_t {
  GLFWwindow *window;
  struct nk_context nuklear;
  struct nk_buffer commands;
};

class Scene {
 public:
  Scene(int width = 800, int height = 600, const char* title = "3D Scene Example")
      : m_width(width), m_height(height), m_title(title) {}
  
  ~Scene() {}
  
  static events_t events;
  context_t context;
  viewport_t viewport;
  MouseStates mouse_states;
  void process_events();
  void render_canvas();
  void activate_context();
  
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
  
  void updateCameraView();
  void setCameraView(const Sophus::SE3f& cameraPose);
  // 相机参数
  static Sophus::SE3f m_cameraPose;
  

  std::unique_ptr<Shader> position_shader;
  
  // 渲染函数
  void render();
  
  // 键盘事件回调函数
  static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
   
    }
    
  static void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
  {
  
  }
  
  static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
  {
    
    events.scroll_offset += Eigen::Vector2f((float)xoffset, (float)yoffset);
  }
  
  
  static void mouseScrollEndCallback(GLFWwindow* window, int button, int action, int mods)
  {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      if (action == GLFW_PRESS) {
        double current_button_time = glfwGetTime();
        double dt = current_button_time - events.last_left_click_time;
        if (dt > DOUBLE_CLICK_MIN_DT &&
            dt < DOUBLE_CLICK_MAX_DT) {
          double x, y;
          glfwGetCursorPos(window, &x, &y);
          events.double_click = true;
          events.double_click_position =
              Eigen::Vector2d(x, y).cast<int>();
          events.last_left_click_time =
              -std::numeric_limits<double>::max();
        } else {
          events.last_left_click_time = current_button_time;
        }
      }
    }
  }
  
  
};

#endif  //LISTENER_MYGL_H
