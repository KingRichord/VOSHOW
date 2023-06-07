
#include "dispatch.h"
Dispatch::Dispatch() {
	converter_= make_unique<ImagemConverter>();
	render_loop_ = std::thread([this] { visTool_.run(); });
}
Dispatch::~Dispatch() {
	while (render_loop_.joinable())
		render_loop_.join();
}
void Dispatch::MsgDispatch(nlohmann::json &msg)
{
	if(msg.empty())
		return;
	if(msg["Type"].empty())return;
	int type = msg["Type"];
	//所有的数据类型都必须含有一个固定的类型
	switch (type) {
		case Image:
			ProcessImageMessages(msg);
			break;
		case Points:
			ProcessPointsMessages(msg);
			break;
        case KeyFrame:
	        ProcessKeyframeMessages(msg);
            break;
		case Pose:
			ProcessPoseMessages(msg);
			break;
		case POSE_GRAPH:
			ProcessPoseGraphMessages(msg);
			break;
		default:
			printf("unhandled message: %d\n", type);
			break;
	}
}
void Dispatch::ProcessKeyframeMessages(nlohmann::json &msg)
{
	if (msg["data"].empty()) return;
	if(msg["data"].size() % 7 == 0)
	{
		std::vector<Eigen::Isometry3d>  Keyposes;
		for (int i = 0; i < msg["data"].size()/7; i++) {
			Eigen::Isometry3d pose;
			pose.setIdentity();
			double x = msg["data"][i*7];
			double y = msg["data"][i*7+1];
			double z = msg["data"][i*7+2];
			
			double q_w  = msg["data"][i*7+6];
			double q_x  = msg["data"][i*7+3];
			double q_y  = msg["data"][i*7+4];
			double q_z  = msg["data"][i*7+5];
			Eigen::Vector3d t{
				x,
				y,
				z};
			Eigen::Quaterniond q{
					q_w,
					q_x,
					q_y,
					q_z
			};
			pose.translate(t);
			pose.rotate(q);
			Keyposes.emplace_back(pose);
		}
		visTool_.add_camera_pose(Keyposes);
	}
}
void Dispatch::ProcessPointsMessages(nlohmann::json &msg) {
	if (msg["data"].empty()) return;
	std::vector<Eigen::Vector3d>  points;
	if(msg["data"].size() % 3 == 0)
	{
		for (int i = 0; i < msg["data"].size()/3; i++) {
			Eigen::Vector3d point;
			point.setZero();
			Eigen::Vector3d t{
					msg["data"][i*3],
					msg["data"][i*3+1],
					msg["data"][i*3+2]};
			points.emplace_back(t);
		}
	}
	visTool_.add_points(points);
}

void Dispatch::ProcessImageMessages(nlohmann::json &msg) {
	if (msg["data"].empty()) return;
	cv::Mat foo = converter_->str2mat( msg["data"]).clone();
	resize(foo, foo, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
	visTool_.updateImage(foo);
}

void Dispatch::ProcessPoseMessages(nlohmann::json &msg) {
	if (msg["data"].empty()) return;
	if(msg["data"].size() % 7 == 0)
	{
		Eigen::Isometry3d pose;
		pose.setIdentity();
		double x = msg["data"][0];
		double y = msg["data"][1];
		double z = msg["data"][2];
		
		double q_w  = msg["data"][6];
		double q_x  = msg["data"][3];
		double q_y  = msg["data"][4];
		double q_z  = msg["data"][5];
		Eigen::Vector3d t{x,y,z};
		Eigen::Quaterniond q{q_w,q_x,q_y,q_z};
		pose.translate(t);
		pose.rotate(q);
		visTool_.add_traj_pose(pose);
	}
}

void Dispatch::ProcessPoseGraphMessages(nlohmann::json &msg) {
	if (msg["data"].empty()) return;
	if(msg["data"].size() % 7 == 0)
	{
		Eigen::Isometry3d pose;
		pose.setIdentity();
		double x = msg["data"][0];
		double y = msg["data"][1];
		double z = msg["data"][2];
		
		double q_w  = msg["data"][6];
		double q_x  = msg["data"][3];
		double q_y  = msg["data"][4];
		double q_z  = msg["data"][5];
		Eigen::Vector3d t{x,y,z};
		Eigen::Quaterniond q{q_w,q_x,q_y,q_z};
		pose.translate(t);
		pose.rotate(q);
		visTool_.add_traj_pose(pose);
	}
}
