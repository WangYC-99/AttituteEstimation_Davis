#include "./include/camera.hpp"
#include <thread>
using namespace std;

int main() {
	// std::shared_ptr<camera> cam;
	// std::thread cam_t(&camera::function, &cam);
	// cv::waitKey(3000);
	// cam_t.join();
	camera cam;
	cam.function();
	return 0;
}