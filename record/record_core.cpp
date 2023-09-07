
#include <Eigen/Geometry>
#include "record_core.h"
record_core::record_core() {
	if(!record_manger_.empty())
		record_manger_.clear();
}
record_core::~record_core() {
	LogWarn("delete  record_core ");
	stop_record();
	record_manger_.clear();
}
void record_core::set_all_folder_location_path()
{
	boost::filesystem::path old_cpath = boost::filesystem::current_path();
	// 首先创建总的文件夹，即RBKBAG文件夹
	folder_location_path_ = old_cpath.string() +"/rbkbag";
	LogInfo(" current_path " <<old_cpath);
	create_folder(folder_location_path_);
}
void record_core::add_record_msg(const std::string& topic,const std::string& file_name,std::string data)
{
	if( start_record_status == false ) return;
	bool find_this_topic = false;
	std::shared_ptr<ofstream> find_file;
	for (auto const&record: record_manger_) {
		if(record.topic_name == topic)
		{
			find_file = record.file;
			find_this_topic = true;
		}
	}
	if(find_this_topic)
	{
		std::lock_guard<std::mutex> lg(lock_);
		*find_file <<data;
	}
	else
	{
		auto ptr = std::make_shared<ofstream>();
		std::string all_path = folder_location_path_ +"/"+cur_folder+ "/" + file_name;
		LogInfo("add new path" << all_path)
		ptr->open(all_path, ios::binary | ios::app | ios::in | ios::out);
		*ptr << data;
		LogInfo("add new path save: "<<data);
		// 创建这个消息记录器
		DataRecord datarecord;
		datarecord.file = ptr;
		datarecord.topic_name =topic;
		datarecord.file_path = all_path;
		record_manger_.push_back(datarecord);
	}
}
void record_core::add_record_image(const std::string& cam_name,const std::string& timestamp,cv::Mat &img)
{
	if( start_record_status == false ) return;
	bool find_this_cam = false;
	std::string  file_path;
	for (auto const&record: record_img_manger_) {
		if(record.camera_name == cam_name)
		{
			file_path = record.file_path;
			find_this_cam = true;
		}
	}
	if(find_this_cam)
	{
		std::string file_name_save = file_path+"/"+timestamp + ".png";
		cv::imwrite( file_name_save, img);
	}
	else
	{
		string path = folder_location_path_+ "/"+ cur_folder + "/"+ cam_name;
		create_folder(path);
		std::string file_name_save = path +"/"+timestamp + ".png";
		cv::imwrite( file_name_save, img);
		// 创建这个消息记录器
		ImgRecord datarecord;
		datarecord.camera_name = cam_name;
		datarecord.file_path = path;
		record_img_manger_.push_back(datarecord);
	}
	LogInfo("save image "<<cam_name);
}
void record_core::save_to_yaml(const std::string file_name,
							   const cv::Vec2i &cam_resolution,
							   const cv::Vec4d &cam_intrinsics,
							   const cv::Vec4d &cam_distortion_coeffs,
							   const cv::Mat  &T_body_cam)
{
	LogInfo("save camera info yaml ")
	std::string all_path;
	all_path = folder_location_path_ + file_name;
	cv::FileStorage fwrite(all_path, cv::FileStorage::WRITE);
	//2.写入数据
	fwrite << "resolution_width " << cam_resolution[0];
	fwrite << "resolution_height " << cam_resolution[1];
	fwrite << "intrinsics " << cam_intrinsics;
	fwrite << "distortion_coeffs " << cam_distortion_coeffs;
	fwrite << "T_body_cam " << T_body_cam;
	//3.关闭文件
	fwrite.release();
}
void record_core::stop_record()
{
	start_record_status = false;
	if(record_manger_.empty()) return;
	for (auto const&record: record_manger_) {
		record.file->close();
	}
	record_manger_.clear();
	std::string cur = folder_location_path_ +"/"+ cur_folder;
	
}
void record_core::open_record()
{
	start_record_status = true;
	// 获取当前系统的时间戳,并根据时间创建文件夹名字
	time_t timep;
	struct tm *p;
	char name[256] = {0};
	time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
	p = localtime(&timep);//用localtime将秒数转化为struct tm结构体
	sprintf(name, "%d.%d.%d.%d.%d.%d",1900+p->tm_year,1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);//把格式化的时间写入字符数组中
	cur_folder = name;
	std::string cur = folder_location_path_ +"/"+ name;
	create_folder(cur);
	if(record_manger_.empty()) return;
	for (auto const&record: record_manger_) {
		record.file->open(record.file_path, ios::binary | ios::app | ios::in | ios::out);;
	}
}

void record_core::create_folder(std::string path)
{
	//首先检查文件夹是否存在
	if(boost::filesystem::exists(path))
	{
		LogInfo(path<<"is already exist")
	}
	else
	{
		//文件夹不存在，创建该文件夹；
		LogInfo(path<<"  create_directory")
		boost::filesystem::create_directory(path);
	}
}