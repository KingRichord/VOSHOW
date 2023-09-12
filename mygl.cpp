#include "mygl.h"

Sophus::SE3f Scene::m_cameraPose = Sophus::SE3f();
bool Scene::m_isKeyPressed[4] = { false, false, false, false };
float Scene::mKeyscroll = 0.;

bool Scene::firstMouse = true;
float Scene::yaw   = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float Scene::pitch =  0.0f;
float Scene::lastX =  800.0f / 2.0;
float Scene::lastY =  600.0 / 2.0;

void Scene::run()
{
  
  // 初始化GLFW
  if (!glfwInit())
  {
    return;
  }
  // 创建窗口
  GLFWwindow* window = glfwCreateWindow(m_width, m_height, m_title, nullptr, nullptr);
  if (!window)
  {
    glfwTerminate();
    return;
  }

  // 将当前上下文设为窗口的上下文
  glfwMakeContextCurrent(window);

  // 设置视口大小
  glViewport(0, 0, m_width, m_height);

  // 开启深度测试
  glEnable(GL_DEPTH_TEST);
  
  // 注册键盘事件回调函数
  glfwSetKeyCallback(window, keyCallback);
  // 注册鼠标滚轮事件回调函数
  glfwSetScrollCallback(window, scrollCallback);
  // 鼠标
  glfwSetCursorPosCallback(window, mouse_callback);
  
  glfwSetMouseButtonCallback(window, mouseScrollEndCallback);
  // 渲染循环
  while (!glfwWindowShouldClose(window))
  {
    render();

    // 处理事件
    glfwPollEvents();
  }

  // 清理资源
  glfwTerminate();
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

void Scene::render()
{
  // 设置清屏颜色为黑色
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  // 清空颜色缓冲和深度缓冲
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  // 设置视角和投影矩阵
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, (float)m_width / (float)m_height, 0.1f, 1000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  
  updateCameraView();
  setCameraView(m_cameraPose);

  // 在此处添加您的渲染逻辑（绘制3D场景）
  drawHorizontalGrid();
  // 交换前后缓冲
  glfwSwapBuffers(glfwGetCurrentContext());
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
  
  // WASD 按键控制
  if (m_isKeyPressed[0])  //w
    cameraPosition.x() -= 0.5f;
  if (m_isKeyPressed[1])  //S
    cameraPosition.x() += 0.5f;
  if (m_isKeyPressed[2])  //A
    cameraPosition.y() -= 0.5f;
  if (m_isKeyPressed[3])  //D
    cameraPosition.y() += 0.5f;
  // 滚轮控制
  if(lastScrollValue != mKeyscroll)
  {
    lastScrollValue = mKeyscroll;
    cameraPosition.z() +=  mKeyscroll;
    mKeyscroll =0;
  }
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
                    * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  m_cameraPose.setRotationMatrix(rotation_matrix);
  //
  m_cameraPose.translation()= cameraPosition;
}
