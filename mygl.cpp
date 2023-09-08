#include "mygl.h"
#define NK_IMPLEMENTATION
#include "nuklear.h"
Sophus::SE3f Scene::m_cameraPose = Sophus::SE3f();
events_t Scene::events = {
    {},        // characters 初始化为空的向量
    {0.0f, 0.0f}, // scroll_offset 初始化为 (0.0, 0.0)
    false,     // double_click 初始化为 false
    {0, 0},    // double_click_position 初始化为 (0, 0)
    -std::numeric_limits<double>::max() // last_left_click_time 初始化为负无穷大
};


void Scene::run()
{
  // 初始化GLFW
  if (!glfwInit())
  {
    return;
  }
  viewport.window_size = {m_width, m_height};
  memset(&context, 0, sizeof(context_t));
  // 创建窗口
  GLFWwindow* window = glfwCreateWindow(m_width, m_height, m_title, nullptr, nullptr);
  if (!window)
  {
    glfwTerminate();
    return;
  }
  context.window = window;
  activate_context();
  // 渲染循环
  while (!glfwWindowShouldClose(context.window))
  {
    process_events();
    render_canvas();
  }
  // 清理资源
  glfwTerminate();
}



void Scene::setCameraView(const Sophus::SE3f& cameraPose)
{
  // 获取相机位置和姿态
  Eigen::Matrix3f rotationMatrix = cameraPose.rotationMatrix();
  Eigen::Vector3f translation = cameraPose.translation();
  
  // 计算相机目标位置
  Eigen::Vector3f cameraTarget = rotationMatrix * Eigen::Vector3f(0.0f, 0.0f, 1.0f) + translation;

  // 计算相机上向量
  Eigen::Vector3f worldUpVector = rotationMatrix * Eigen::Vector3f(0.0f, 1.0f, 0.0f);

  // 设置相机视图矩阵
  gluLookAt(translation.x(), translation.y(), translation.z(),
      cameraTarget.x(), cameraTarget.y(), cameraTarget.z(),
      worldUpVector.x(), worldUpVector.y(), worldUpVector.z()
  );
}
void Scene::updateCameraView() {
  Eigen::Vector3f cameraPosition;
  cameraPosition = m_cameraPose.translation();
  //
  m_cameraPose.translation()= cameraPosition;
}
void Scene::process_events() {
  auto window = context.window;
  auto nuklear = &context.nuklear;
  
  nk_input_begin(nuklear);

  for (const auto &character : events.characters) {
    nk_input_unicode(nuklear, character);
  }
  events.characters.clear();

  bool shift_left =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
  bool shift_right =
      (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  bool control_left =
      (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);
  bool control_right =
      (glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);
  bool shift_down = shift_left || shift_right;
  bool control_down = control_left || control_right;
  nk_input_key(nuklear, NK_KEY_SHIFT, shift_down);
  nk_input_key(nuklear, NK_KEY_CTRL, control_down);

  if (control_down) {
    if (shift_down) {
      nk_input_key(nuklear, NK_KEY_TEXT_REDO,
                   glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS);
    } else {
      nk_input_key(nuklear, NK_KEY_COPY,
                   glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS);
      nk_input_key(nuklear, NK_KEY_PASTE,
                   glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS);
      nk_input_key(nuklear, NK_KEY_CUT,
                   glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS);
      nk_input_key(nuklear, NK_KEY_TEXT_UNDO,
                   glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS);
      nk_input_key(nuklear, NK_KEY_TEXT_WORD_LEFT,
                   glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
      nk_input_key(nuklear, NK_KEY_TEXT_WORD_RIGHT,
                   glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
    }
  } else {
    nk_input_key(nuklear, NK_KEY_LEFT,
                 glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
    nk_input_key(nuklear, NK_KEY_RIGHT,
                 glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
  }

  bool button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  bool button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  bool button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  double x, y;
  glfwGetCursorPos(window, &x, &y);
  nk_input_motion(nuklear, (int)x, (int)y);
  nk_input_button(nuklear, NK_BUTTON_LEFT, (int)x, (int)y,
                  (int)button_left);
  nk_input_button(nuklear, NK_BUTTON_MIDDLE, (int)x, (int)y,
                  (int)button_middle);
  nk_input_button(nuklear, NK_BUTTON_RIGHT, (int)x, (int)y,
                  (int)button_right);
  nk_input_button(nuklear, NK_BUTTON_DOUBLE,
                  events.double_click_position.x(),
                  events.double_click_position.y(), events.double_click);
  nk_input_scroll(nuklear, nk_vec2(events.scroll_offset.x(),
                                   events.scroll_offset.y()));

  nk_input_end(nuklear);

  if (!nk_item_is_any_active(nuklear)) {
    MouseStates &states = mouse_states;
    states.mouse_left = button_left;
    states.mouse_middle = button_middle;
    states.mouse_right = button_right;
    states.mouse_double_click = events.double_click;
    states.scroll = events.scroll_offset;
    if (!(states.mouse_left || states.mouse_middle ||
          states.mouse_right)) {
      states.mouse_normal_position = {x, y};
    }
    states.mouse_drag_position = {x, y};
    states.control_left = control_left;
    states.control_right = control_right;
    states.shift_left = shift_left;
    states.shift_right = shift_right;
    // if (!vis->mouse(states)) {
      static Eigen::Vector3f last_ypr;
      if (!(states.mouse_left || states.mouse_middle ||
            states.mouse_right)) {
        last_ypr = viewport.viewport_ypr;
      }
      Eigen::Vector2f drag =
          states.mouse_drag_position - states.mouse_normal_position;
      viewport.viewport_ypr.x() = last_ypr.x() - drag.x() / 10;
      viewport.viewport_ypr.y() = last_ypr.y() - drag.y() / 10;
      viewport.scale = std::clamp(
          viewport.scale * (1.0 + events.scroll_offset.y() / 600.0),
          1.0e-4, 1.0e4);
    // }
  }

  events.double_click = false;
  events.scroll_offset.setZero();
}
void Scene::render_canvas() {
  int w = viewport.framebuffer_size.x();
  int h = viewport.framebuffer_size.y();
  glViewport(0, 0, w, h);
  glClearColor(0.125, 0.125, 0.125, 1.0); // TODO
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(3);
  // 在此处添加您的渲染逻辑（绘制3D场景）
  drawHorizontalGrid();
  // 交换前后缓冲区
  glfwSwapBuffers(context.window);
}

void Scene::drawHorizontalGrid() {
	
  Eigen::Matrix4f origin;
  origin << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
  glPushMatrix();
  glMultTransposeMatrixf(origin.data());
	
  glLineWidth(1);
  std::array<float, 3> grid_{};
  grid_ = {{0.7f, 0.7f, 0.7f}};
  glColor3fv(grid_.data());
	
  glBegin(GL_LINES);
	
  constexpr float interval_ratio = 0.1;
  constexpr float grid_min = -100.0f * interval_ratio;
  constexpr float grid_max = 100.0f * interval_ratio;
	
  for (int x = -10; x <= 10; x += 1) {
      draw_line(x * 10.0f * interval_ratio, grid_min, 0, x * 10.0f * interval_ratio, grid_max, 0);
  }
  for (int y = -10; y <= 10; y += 1) {
      draw_line(grid_min, y * 10.0f * interval_ratio, 0, grid_max, y * 10.0f * interval_ratio, 0);
  }
  glEnd();
  glPopMatrix();
}
void Scene::activate_context() {
  glfwMakeContextCurrent(context.window);
  
  glfwGetWindowSize(context.window, &viewport.window_size.x(),
                    &viewport.window_size.y());
  
  glfwGetFramebufferSize(context.window, &viewport.framebuffer_size.x(),
                         &viewport.framebuffer_size.y());
  
  // 设置视口大小
  glViewport(0, 0, m_width, m_height);
  // 开启深度测试
  glEnable(GL_DEPTH_TEST);
  // 键盘回调函数
  glfwSetKeyCallback(context.window, keyCallback);
  // 滚动回调函数
  glfwSetScrollCallback(context.window, scrollCallback);
  // 鼠标移动回调函数
  glfwSetCursorPosCallback(context.window, mouse_callback);
  // 鼠标按键回调函数
  glfwSetMouseButtonCallback(context.window, mouseScrollEndCallback);
  
}
