#include "talker.h"
zmq::context_t ctx;
zmq::socket_t skt(ctx, ZMQ_PUB);
Talker::Talker()
{
	converter_= make_unique<ImagemConverter>();
}
Talker::~Talker() {
	skt.close();
	converter_ = nullptr;
}
void Talker::Init(const std::string &ip) {
	//"tcp://127.0.0.1:8001"
	skt.bind(ip);
}
void Talker::Pub(rbk::utils::json &metadata) {
	std::string json_str = metadata.dump();
	zmq::message_t query(json_str.length());
	memcpy(query.data(), (json_str.c_str()), (json_str.size()));
	skt.send(query,0);
}
void Talker::PubImage(cv::Mat &imput) {
	// resize(imput, imput, Size(imput.cols, imput.rows), 0, 0, INTER_LINEAR);
	if(imput.empty()) return;
	std::string string_data = converter_->mat2str(imput.clone());
	nlohmann::json metadata = {
			{"Type", 0},
			{"data",  string_data}};
	std::string json_str = metadata.dump();
	zmq::message_t query(json_str.length());
	memcpy(query.data(), (json_str.c_str()), (json_str.size()));
	skt.send(query,0);
}
void Talker::PubPoints(const vector<Eigen::Vector3d> &points) {
	if(points.empty())return;
	nlohmann::json json_points;
	json_points["Type"] = 1;
	for (auto const &ptr_measure : points) {
		json_points["data"].emplace_back(ptr_measure.x());
		json_points["data"].emplace_back(ptr_measure.y());
		json_points["data"].emplace_back(ptr_measure.z());
	}
	Pub(json_points);
}
void Talker::PubKeyFrame(const vector<Eigen::Isometry3d> &frames) {
	if(frames.empty())return;
	nlohmann::json json_data;
	json_data["Type"] = 2;
	for (auto const &ptr_measure : frames) {
		json_data["data"].emplace_back(ptr_measure.translation().x());
		json_data["data"].emplace_back(ptr_measure.translation().y());
		json_data["data"].emplace_back(ptr_measure.translation().z());
		
		Eigen::Quaterniond q(ptr_measure.rotation());
		q.normalized();
		json_data["data"].emplace_back(q.x());
		json_data["data"].emplace_back(q.y());
		json_data["data"].emplace_back(q.z());
		json_data["data"].emplace_back(q.w());
	}
	Pub(json_data);
}

void Talker::PubKeyAxis(const vector< Eigen::Isometry3d> &Axis) {

}

void Talker::PubPose(const Eigen::Isometry3d &Pose) {
	nlohmann::json json_data;
	json_data["Type"] = 4;
	json_data["data"].emplace_back(Pose.translation().x());
	json_data["data"].emplace_back(Pose.translation().y());
	json_data["data"].emplace_back(Pose.translation().z());
	
	Eigen::Quaterniond q(Pose.rotation());
	q.normalized();
	json_data["data"].emplace_back(q.x());
	json_data["data"].emplace_back(q.y());
	json_data["data"].emplace_back(q.z());
	json_data["data"].emplace_back(q.w());
	Pub(json_data);
}

