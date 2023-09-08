#include <iostream>
#include "zmq.hpp"
#include "json.hpp"
#include "dispatch.h"
#include "points.h"
#include "sophus/se3.hpp"
int main() {
        Points<Eigen::Vector3f> a;
	Dispatch dp;
	zmq::context_t context(1);
	zmq::socket_t subscriber(context, ZMQ_SUB);
	std::string TOPIC = "";
	subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length());
	subscriber.connect("tcp://127.0.0.1:5555");
	// subscriber.connect("tcp://192.168.192.5:8002");
	while (subscriber.connected())
	{
		zmq::message_t message;
                zmq::recv_result_t res;
		res = subscriber.recv(message,zmq::recv_flags::none);
		if (res.has_value())
		{
			auto reply_str = std::string(static_cast<char *>(message.data()), message.size());
			auto json_data = nlohmann::json::parse(reply_str);
			dp.MsgDispatch(json_data);
		}
	}
	subscriber.close();
	context.close();
	return 0;
}
