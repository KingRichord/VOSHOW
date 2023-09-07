#ifndef ROBOKIT_RECORD_CORE_H
#define ROBOKIT_RECORD_CORE_H
#include <robokit/utils/logger/logger.h>
#include <fstream>
#include <iostream>
#include "vector"
#include <memory>
#include "boost/filesystem.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sys/stat.h>

using namespace std;
struct DataRecord
{
	std::string topic_name;
	std::string file_path;
	std::shared_ptr<ofstream> file;
};
struct ImgRecord
{
	std::string camera_name;
	std::string file_path;
};

class record_core {
public:
	record_core();
	~record_core();
	void stop_record();
	void open_record();
	void add_record_msg(const std::string& topic,const std::string& file_name, std::string data);
	void add_record_image(const std::string& cam_name,const std::string& timestamp,cv::Mat &img);
	void set_all_folder_location_path();
	void save_to_yaml(const std::string file_name,
                      const cv::Vec2i &cam_resolution,
                      const cv::Vec4d &cam_intrinsics,
                      const cv::Vec4d &cam_distortion_coeffs,
                      const cv::Mat  &T_body_cam);
private:
	void create_folder(std::string path);
	vector<DataRecord> record_manger_;
	vector<ImgRecord> record_img_manger_;
	string folder_location_path_;
	string cur_folder;
	std::mutex lock_;
	bool start_record_status = false;
};


#endif //ROBOKIT_RECORD_CORE_H
