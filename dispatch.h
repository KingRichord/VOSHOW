#ifndef IMGUI_DEMO_3D_DISPATCH_H
#define IMGUI_DEMO_3D_DISPATCH_H
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include "json.hpp"
#include "Eigen/Eigen"
#include "vis.h"
#include "ConvertImage.h"
class Dispatch {
public:
	Dispatch();
	~Dispatch();
	void MsgDispatch(nlohmann::json &msg);
private:
	enum MsgTypeIdx {
		Image = 0,
		Points = 1,
		KeyFrame = 2,
		Axis =3,
		Pose =4
		
	};
	void ProcessImageMessages(nlohmann::json &msg);
	void ProcessPointsMessages(nlohmann::json &msg);
    void ProcessKeyframeMessages(nlohmann::json &msg);
    void ProcessAxisMessages(nlohmann::json &msg);
	void ProcessPoseMessages(nlohmann::json &msg);
	
	VisTool visTool_;
	std::thread render_loop_;
	std::unique_ptr<ImagemConverter> converter_;
};


#endif //IMGUI_DEMO_3D_DISPATCH_H
