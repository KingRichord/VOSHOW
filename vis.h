

#ifndef TAGSLAM_VIS_H
#define TAGSLAM_VIS_H

#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>
#include "Eigen/Geometry"
#include <opencv2/opencv.hpp>
#include <mutex>

class VisTool {
public:
	VisTool();
	
	~VisTool();
	
	void add_camera_pose(std::vector<Eigen::Isometry3d> &T_map_cams);
	
	void add_traj_pose(Eigen::Isometry3d &pose);
	
	void add_odom_traj_pose(Eigen::Isometry3d &pose);
	
	void add_points(std::vector<Eigen::Vector3d> &points);
	
	void add_map(std::vector<Eigen::Vector3d> &points);
	
	void add_posegraph(std::vector<Eigen::Isometry3d> &graphs);
	
	void addGps(const Eigen::Vector3d &G_p_Gps);
	
	void updateImage(const cv::Mat &img);
	
	void run();

private:
	inline void draw_line(const float x1, const float y1, const float z1,
	                      const float x2, const float y2, const float z2) const {
		glVertex3f(x1, y1, z1);
		glVertex3f(x2, y2, z2);
	}
	
	void DrawGpsPoints();
	
	void draw_camera_pose();
	
	void draw_pose_graph(); // 画位姿图
	
	void drawAxis(Eigen::Isometry3d &pose);
	
	void drawPoints();
	
	
	void drawMap();
	
	
	void DrawTrajectory();
	
	void DrawOdomTrajectory();
	
	void draw_horizontal_grid();
	
	std::string m_window_name;
	size_t m_CameraLineWidth{1};
	std::mutex m_lock_Map;
	std::mutex m_lock_Points;
	std::mutex m_lock_Camera_Pose_current;
	std::mutex m_lock_Gps;
	std::mutex m_lock_Image;
	std::mutex m_lock_PoseGraph;
	std::vector<Eigen::Isometry3d> m_posegraphs;
	std::vector<Eigen::Vector3d> m_points;
	std::vector<Eigen::Vector3d> m_map;
	std::deque<Eigen::Isometry3d> m_poses;
	std::deque<Eigen::Isometry3d> m_odom_poses;
	std::deque<Eigen::Vector3d> gps_points_;
	std::vector<Eigen::Isometry3d> m_T_map_current_cams;
	cv::Mat image_;
	int image_width = 640;
	int image_height = 480;
	
	pangolin::GlFont *text_font;
};


#endif //TAGSLAM_VIS_H
