#include "vis.h"
#include <Eigen/Core>


VisTool::VisTool() {
	m_window_name2 ="plot";
	pangolin::CreateWindowAndBind(m_window_name, 1920, 1080);
	pangolin::CreateWindowAndBind(m_window_name2, 1920, 1080);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	pangolin::GetBoundWindow()->RemoveCurrent();
}

VisTool::~VisTool() {
	pangolin::FinishFrame();
	pangolin::GetBoundWindow()->RemoveCurrent();
}

void VisTool::run() {
	
	pangolin::BindToContext(m_window_name);
	pangolin::OpenGlRenderState s_cam(
			pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 10000),
			pangolin::ModelViewLookAt(-2, 0, -2, 0, 0, 0, pangolin::AxisZ));
	pangolin::Handler3D handler(s_cam);
	pangolin::View &d_cam = pangolin::CreateDisplay()
			.SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
			.SetHandler(&handler);
	
	pangolin::View &d_image = pangolin::Display("image")
			.SetBounds(0.7f, 1.0f, 0.001, 1. / 3, true);
	
	pangolin::GlTexture image_texture(image_width, image_height, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
	
	Eigen::Isometry3d xyz;
	xyz.setIdentity();
	
	while (!pangolin::ShouldQuit()) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam.Activate(s_cam);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);//background Color
		drawAxis(xyz);
		drawHorizontalGrid();
		drawCameraPose();
		DrawGpsPoints();
		drawPoints();
		drawMap();
		DrawOdomTrajectory();
		DrawTrajectory();
		drawPoseGraph();
		{
			std::lock_guard<std::mutex> lg(m_lock_Image);
			if (!image_.empty()) {
				image_texture.Upload(image_.data, GL_BGR, GL_UNSIGNED_BYTE);
				d_image.Activate();
				glColor3f(1.0, 1.0, 1.0);
				image_texture.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的
				image_texture.RenderToViewport();
			}
		}
		glFlush();
		pangolin::FinishFrame();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pangolin::GetBoundWindow()->RemoveCurrent();
}
void VisTool::run2() {
	
	pangolin::BindToContext(m_window_name2);
	pangolin::View& plot_display = pangolin::CreateDisplay().SetBounds(
			0.0, 0.4, 0, 1.0);
	
	plotter = new pangolin::Plotter(&log, 0.0, 100, -10.0, 10.0, 0.01f,0.01f);
	plot_display.AddDisplay(*plotter);
	
	plotter->AddMarker(pangolin::Marker::Vertical, 0, pangolin::Marker::Equal,
	                   pangolin::Colour::Red());
	float t = 0;
	std::vector<std::string> labels;
	labels.push_back(std::string("x(t)"));
	labels.push_back(std::string("y(t)"));
	labels.push_back(std::string("z(t)"));
	log.SetLabels(labels);
	const float tinc = 0.02f;
	
	
	// pangolin::DisplayBase().AddDisplay(plotter);
	while (!pangolin::ShouldQuit()) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		log.Log(sin(t),cos(t),sin(t)+cos(t));
		t += tinc;
		glFlush();
		pangolin::FinishFrame();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pangolin::GetBoundWindow()->RemoveCurrent();
}



void VisTool::add_traj_pose(Eigen::Isometry3d &pose) {
	std::lock_guard<std::mutex> lg(m_lock_Camera_Pose_current);
	m_poses.push_back(pose);
	if (m_poses.size() > 1000000) {
		m_poses.pop_front();
	}
}

void VisTool::updateImage(const cv::Mat &img) {
	std::lock_guard<std::mutex> lg(m_lock_Image);
	image_ = img.clone();
}

void VisTool::addGps(const Eigen::Vector3d &G_p_Gps) {
	std::lock_guard<std::mutex> lg(m_lock_Gps);
	gps_points_.push_back(G_p_Gps);
	if (gps_points_.size() > 1000000) {
		gps_points_.pop_front();
	}
}

void VisTool::add_odom_traj_pose(Eigen::Isometry3d &pose) {
	std::lock_guard<std::mutex> lg(m_lock_Camera_Pose_current);
	m_odom_poses.push_back(pose);
	if (m_odom_poses.size() > 1000000) {
		m_odom_poses.pop_front();
	}
};

void VisTool::add_camera_pose(std::vector<Eigen::Isometry3d> &T_map_cams) {
	std::lock_guard<std::mutex> lg(m_lock_Camera_Pose_current);
	m_T_map_current_cams.clear();
	m_T_map_current_cams.shrink_to_fit();
	m_T_map_current_cams = T_map_cams;
}


void VisTool::add_points(std::vector<Eigen::Vector3d> &points) {
	std::lock_guard<std::mutex> lg(m_lock_Points);
	m_points.clear();
	m_points = points;
}

void VisTool::add_map(std::vector<Eigen::Vector3d> &points) {
	std::lock_guard<std::mutex> lg(m_lock_Points);
	m_map.clear();
	m_map = points;
}
// 添加位姿图数据
void VisTool::add_posegraph(std::vector<Eigen::Isometry3d> &graphs) {
	std::lock_guard<std::mutex> lg(m_lock_PoseGraph);
	m_posegraphs.clear();
	m_posegraphs = graphs;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VisTool::drawAxis(Eigen::Isometry3d &pose) {
	Eigen::Vector3d Ow = pose.translation();
	Eigen::Vector3d Xw = pose * (1 * Eigen::Vector3d(1, 0, 0));
	Eigen::Vector3d Yw = pose * (1 * Eigen::Vector3d(0, 1, 0));
	Eigen::Vector3d Zw = pose * (1 * Eigen::Vector3d(0, 0, 1));
	glLineWidth(10);
	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);
	glVertex3d(Ow[0], Ow[1], Ow[2]);
	glVertex3d(Xw[0], Xw[1], Xw[2]);
	glColor3f(0.0, 1.0, 0.0);
	glVertex3d(Ow[0], Ow[1], Ow[2]);
	glVertex3d(Yw[0], Yw[1], Yw[2]);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3d(Ow[0], Ow[1], Ow[2]);
	glVertex3d(Zw[0], Zw[1], Zw[2]);
	glEnd();
}
void VisTool::drawPoseGraph()
{
	glLineWidth(5);
	m_lock_PoseGraph.lock();
	for (size_t i = 0; i < m_posegraphs.size(); i++) {
		if (i == 0)
			continue;
		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 0.0);
		auto p1 = m_posegraphs[i - 1], p2 = m_posegraphs[i];
		glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
		glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
		glEnd();
	}
	m_lock_PoseGraph.unlock();
}
void VisTool::drawCameraPose() {
	m_lock_Camera_Pose_current.lock();
	const float &w = 0.5;
	const float h = w * 0.75;
	const float z = w * 1.2;
	for (auto &m_T_map_current_cam: m_T_map_current_cams) {
		glPushMatrix();
		pangolin::OpenGlMatrix Twc_(m_T_map_current_cam.matrix());
		glMultMatrixd(Twc_.m);
		glLineWidth(m_CameraLineWidth);
		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(w, h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, h, z);
		glVertex3f(w, h, z);
		glVertex3f(w, -h, z);
		glVertex3f(-w, h, z);
		glVertex3f(-w, -h, z);
		glVertex3f(-w, h, z);
		glVertex3f(w, h, z);
		glVertex3f(-w, -h, z);
		glVertex3f(w, -h, z);
		glEnd();
		glPopMatrix();
	}
	m_lock_Camera_Pose_current.unlock();
}

void VisTool::drawPoints() {
	m_lock_Points.lock();
	glPointSize(3);
	if (!m_points.empty())
		for (auto const &pt: m_points) {
			glBegin(GL_POINTS);
			glColor3f(1., 0., 0.);                 // 修改颜色
			glVertex3f(pt.x(), pt.y(), pt.z()); // 设置定点坐标
			glEnd();                            // 结束
		}
	m_lock_Points.unlock();
	
}
void VisTool::drawMap()
{
	m_lock_Map.lock();
	glPointSize(3);
	if (!m_map.empty())
		for (auto const &pt: m_map) {
			glBegin(GL_POINTS);
			glColor3f(0., 1., 0.);                 // 修改颜色
			glVertex3f(pt.x(), pt.y(), pt.z()); // 设置定点坐标
			glEnd();                            // 结束
		}
	m_lock_Map.unlock();

	
}
void VisTool::DrawTrajectory() {
	// glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(3);
	m_lock_Camera_Pose_current.lock();
	// 画出连线
	for (size_t i = 0; i < m_poses.size(); i++) {
		// 画每个位姿的三个坐标轴
		Eigen::Vector3d Ow = m_poses[i].translation();
		Eigen::Vector3d Xw = m_poses[i] * (1 * Eigen::Vector3d(1, 0, 0));
		Eigen::Vector3d Yw = m_poses[i] * (1 * Eigen::Vector3d(0, 1, 0));
		Eigen::Vector3d Zw = m_poses[i] * (1 * Eigen::Vector3d(0, 0, 1));
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3d(Ow[0], Ow[1], Ow[2]);
		glVertex3d(Xw[0], Xw[1], Xw[2]);
		glColor3f(0.0, 1.0, 0.0);
		glVertex3d(Ow[0], Ow[1], Ow[2]);
		glVertex3d(Yw[0], Yw[1], Yw[2]);
		glColor3f(0.0, 0.0, 1.0);
		glVertex3d(Ow[0], Ow[1], Ow[2]);
		glVertex3d(Zw[0], Zw[1], Zw[2]);
		glEnd();
	}
	for (size_t i = 0; i < m_poses.size(); i++) {
		
		if (i == 0)
			continue;
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		auto p1 = m_poses[i - 1], p2 = m_poses[i];
		glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
		glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
		glEnd();
	}
	m_lock_Camera_Pose_current.unlock();
}

void VisTool::DrawOdomTrajectory() {
	// glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(3);
	// for (size_t i = 0; i < poses.size(); i++) {
	// 	// 画每个位姿的三个坐标轴
	// 	Eigen::Vector3d Ow = poses[i].translation();
	// 	Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
	// 	Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
	// 	Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
	// 	glBegin(GL_LINES);
	// 	glColor3f(1.0, 0.0, 0.0);
	// 	glVertex3d(Ow[0], Ow[1], Ow[2]);
	// 	glVertex3d(Xw[0], Xw[1], Xw[2]);
	// 	glColor3f(0.0, 1.0, 0.0);
	// 	glVertex3d(Ow[0], Ow[1], Ow[2]);
	// 	glVertex3d(Yw[0], Yw[1], Yw[2]);
	// 	glColor3f(0.0, 0.0, 1.0);
	// 	glVertex3d(Ow[0], Ow[1], Ow[2]);
	// 	glVertex3d(Zw[0], Zw[1], Zw[2]);
	// 	glEnd();
	// }
	m_lock_Camera_Pose_current.lock();
	// 画出连线
	for (size_t i = 0; i < m_odom_poses.size(); i++) {
		if (i == 0)
			continue;
		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 0.0);
		auto p1 = m_odom_poses[i - 1], p2 = m_odom_poses[i];
		glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
		glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
		glEnd();
	}
	m_lock_Camera_Pose_current.unlock();
}

void VisTool::drawHorizontalGrid() {
	
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

void VisTool::DrawGpsPoints() {
	std::lock_guard<std::mutex> lg(m_lock_Gps);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (const Eigen::Vector3d &pt: gps_points_) {
		glVertex3f(pt[0], pt[1], pt[2]);
	}
	glEnd();
}
// void VisTool::draw_plots() {
// 	plotter->ClearSeries();
// 	plotter->ClearMarkers();
//
// 	if (show_est_pos) {
// 		plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Red(), "position x", &vio_data_log);
// 		plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Green(), "position y", &vio_data_log);
// 		plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Blue(), "position z", &vio_data_log);
// 	}
//
// 	if (show_est_vel) {
// 		plotter->AddSeries("$0", "$1", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Red(), "velocity x", &vio_data_log);
// 		plotter->AddSeries("$0", "$2", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Green(), "velocity y", &vio_data_log);
// 		plotter->AddSeries("$0", "$3", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Blue(), "velocity z", &vio_data_log);
// 	}
//
// 	if (show_est_bg) {
// 		plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Red(), "gyro bias x", &vio_data_log);
// 		plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Green(), "gyro bias y", &vio_data_log);
// 		plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Blue(), "gyro bias z", &vio_data_log);
// 	}
//
// 	if (show_est_ba) {
// 		plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Red(), "accel bias x", &vio_data_log);
// 		plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Green(), "accel bias y",
// 		                   &vio_data_log);
// 		plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine,
// 		                   pangolin::Colour::Blue(), "accel bias z", &vio_data_log);
// 	}
//
// 	double t = vio_dataset->get_image_timestamps()[show_frame] * 1e-9;
// 	plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal,
// 	                   pangolin::Colour::White());
// }