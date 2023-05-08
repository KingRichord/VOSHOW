#ifndef ROBOKIT_TALKER_H
#define ROBOKIT_TALKER_H
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>
#include "robokit/utils/json/json.h"
#include "ConvertImage.h"
class Talker {
public:
	Talker();
	~Talker();
	void Init(const std::string &ip);
	void PubKeyFrame(const vector< Eigen::Isometry3d> &frames);
	void PubPoints(const std::vector<Eigen::Vector3d>& points);
	void PubKeyAxis(const vector< Eigen::Isometry3d> &Axis);
	void PubPose(const Eigen::Isometry3d &Pose);
	void PubImage(cv::Mat &imput);
private:
	void Pub(rbk::utils::json &metadata);
	std::unique_ptr<ImagemConverter> converter_;
};
#endif
