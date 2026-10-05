#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cmath>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace image_transport;
using std::placeholders::_1;

class StreamGZCam : public rclcpp::Node
{
public:
	StreamGZCam() : Node("coord_advertiser")
	{
		rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("coord_advertiser");
		image_transport::ImageTransport it(node);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", qos,
      		std::bind(&StreamGZCam::camera_callback, this, _1));

		pub_ = it.advertise("/camera/stream/image", 1);

		RCLCPP_INFO(this->get_logger(), "coord_advertiser node");
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	cv::Mat img_;
	 
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
	image_transport::Publisher pub_;

	sensor_msgs::msg::Image msg_;

	void camera_callback(sensor_msgs::msg::Image msg);
	void timer_callback();
};

/**
 * @brief Subscribe to prediction image
 * @param 
 */
void StreamGZCam::camera_callback(const sensor_msgs::msg::Image msg) {
	msg_ = msg;
	pub_.publish(msg_);

}

int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StreamGZCam>());

	rclcpp::shutdown();
	return 0;
}
