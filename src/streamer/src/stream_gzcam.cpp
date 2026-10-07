#include <chrono>
#include <rclcpp/rclcpp.hpp>
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
	StreamGZCam() : Node("stream_gzcam")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", qos,
      		std::bind(&StreamGZCam::camera_callback, this, _1));
	}

	void initialize()
	{
		rclcpp::Node::SharedPtr node = shared_from_this();
		image_transport::ImageTransport it(node);

		pub_ = it.advertise("/camera/stream/image", 1);

		RCLCPP_INFO(this->get_logger(), "stream_gzcam node");
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	cv::Mat img_;
	 
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
	
	image_transport::Publisher pub_;

	sensor_msgs::msg::Image msg_;

	void camera_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
};

/**
 * @brief Subscribe to prediction image
 * @param 
 */
void StreamGZCam::camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {

    try {
        // Convert ROS Image -> OpenCV image.
        // "passthrough" preserves the original encoding.
        cv::Mat img = cv_bridge::toCvShare(msg, "rgb8")->image;

		// Reshape to 572, 572
		// Source - https://stackoverflow.com/a/61942452
		// Posted by Juan Esteban Fonseca, modified by community. See post 'Timeline' for change history
		// Retrieved 2026-05-17, License - CC BY-SA 4.0
        // Crop a centered 960x960 region.
        const int crop_w = 960;
        const int crop_h = 960;

        const int x = (img.cols - crop_w) / 2;
        const int y = (img.rows - crop_h) / 2;

        // Make sure the input image is large enough.
        if (x < 0 || y < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image is too small for 960x960 crop: %dx%d",
                img.cols, img.rows);
            return;
        }

        cv::Mat cropped = img(cv::Rect(x, y, crop_w, crop_h));

        // Resize 960x960 -> 572x572.
        cv::Mat resized;
        cv::resize(
            cropped,
            resized,
            cv::Size(572, 572),
            0.0,
            0.0,
            cv::INTER_LINEAR);

        // Convert OpenCV image -> ROS Image.
        auto output_msg =
            cv_bridge::CvImage(
                msg->header,
                "rgb8",
                resized).toImageMsg();

        pub_.publish(*output_msg);

    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Published");
}

int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<StreamGZCam>();
	node->initialize();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
