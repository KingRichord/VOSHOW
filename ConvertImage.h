#include <opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>
#include <string>

using namespace std;
using namespace cv;

#ifndef CONVERTIMAGE_H_
#define CONVERTIMAGE_H_
class ImagemConverter {
public:
	ImagemConverter();
	cv::Mat str2mat(const string& imageBase64);
	string mat2str(const Mat& img);
	virtual ~ImagemConverter();
private:
	std::string base64_encode(uchar const* bytesToEncode, unsigned int inLen);
	std::string base64_decode(std::string const& encodedString);

};

#endif /* CONVERTIMAGE_H_ */
