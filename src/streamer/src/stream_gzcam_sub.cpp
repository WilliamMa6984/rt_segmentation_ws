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

class StreamGZCamSub : public rclcpp::Node
{
public:
	StreamGZCamSub() : Node("stream_gzcam_sub")
	{
		cv::namedWindow("view");
		cv::startWindowThread();

		img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/decomp/image", 10);
		RCLCPP_INFO(this->get_logger(), "stream_gzcam_sub node");
	}

	void initialize()
	{
		rclcpp::Node::SharedPtr node = shared_from_this();
		it_ = std::make_unique<image_transport::ImageTransport>(
			shared_from_this()
		);

		// image_transport::TransportHints hints = image_transport::TransportHints("compressed");
		sub_ = it_->subscribe("/camera/stream/image", 1, std::bind(&StreamGZCamSub::stream_callback, this, _1));
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	cv::Mat img_;
	 
	image_transport::Subscriber sub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
	std::unique_ptr<image_transport::ImageTransport> it_;

	sensor_msgs::msg::Image msg_;

	void stream_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
	void timer_callback();
};

/**
 * @brief Subscribe to prediction image
 * @param 
 */
void StreamGZCamSub::stream_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
	try {
		RCLCPP_INFO_ONCE(
		    this->get_logger(),
		    "Received image: %ux%u, encoding=%s, step=%u",
		    msg->width,
		    msg->height,
		    msg->encoding.c_str(),
		    msg->step
		);
		
        	auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        	
		cv::imshow("view", cv_ptr->image);
		cv::waitKey(1);
	} catch (const cv_bridge::Exception & e) {
		auto logger = rclcpp::get_logger("stream_gzcam_sub");
		RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	img_pub_->publish(*msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<StreamGZCamSub>();
	node->initialize();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
