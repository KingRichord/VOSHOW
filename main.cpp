#include <thread>
#include <mutex>
#include "visualizer.h"
std::thread visualizer_thread_;
int main(int argc, char *argv[]) {
	Visualizer visualizer(1);
	visualizer.show();
	visualizer_thread_ = std::thread(&Visualizer::main, &visualizer);
	
	
    if (visualizer_thread_.joinable()) {
        visualizer_thread_.join();
    }
    return EXIT_SUCCESS;
}
