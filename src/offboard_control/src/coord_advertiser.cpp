#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "parameters.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using std::placeholders::_1;

cv::Mat rotate_image(const cv::Mat& image, double angle);

class CoordAdvertiser : public rclcpp::Node
{
public:
	CoordAdvertiser() : Node("coord_advertiser")
	{

		occupancy_grid_publisher_ = this->create_publisher<OccupancyGrid>("/moss_occ_grid", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_global_pos_subscriber_ = this->create_subscription<VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos,
      		std::bind(&CoordAdvertiser::global_position_callback, this, _1));
		vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
      		std::bind(&CoordAdvertiser::attitude_callback, this, _1));
		lidar_subscriber_ = this->create_subscription<LaserScan>("/lidar", qos,
      		std::bind(&CoordAdvertiser::lidar_callback, this, _1));
		prediction_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/predictor/image_100", qos,
      		std::bind(&CoordAdvertiser::predictor_callback, this, _1));

		auto timer_callback = [this]()->void {
			auto occuGrid = OccupancyGrid();

			occuGrid.header.stamp = rclcpp::Clock().now();
			occuGrid.header.frame_id = "map";

			occuGrid.info.resolution = 1.7/DETECTION_SZ;
			// occuGrid.info.resolution = 1;

			occuGrid.info.width = DETECTION_SZ;
			occuGrid.info.height = DETECTION_SZ;

			occuGrid.info.origin.position.x = 0.0;
			occuGrid.info.origin.position.y = 0.0;
			occuGrid.info.origin.position.z = 0.0;
			occuGrid.info.origin.orientation.x = 0.0;
			occuGrid.info.origin.orientation.y = 0.0;
			occuGrid.info.origin.orientation.z = 0.0;
			occuGrid.info.origin.orientation.w = 0.0;

			std::copy(&predict_img100_data[0], &predict_img100_data[DETECTION_SZ*DETECTION_SZ], back_inserter(occuGrid.data));

			this->occupancy_grid_publisher_->publish(occuGrid);
			
			std::cout << "Pos (lat lon alt): " +
			std::to_string(lat) + " " + 
			std::to_string(lon) + " " + 
			std::to_string(alt) + "\n" << std::endl;

			std::cout << "RPY: " +
			std::to_string(roll) + " " + 
			std::to_string(pitch) + " " + 
			std::to_string(yaw) + "\n" << std::endl;

			std::cout << "Dist to gnd: " +
			std::to_string(lidarDist) + "\n" << std::endl;
		};
		timer_ = this->create_wall_timer(1000ms, timer_callback);

		RCLCPP_INFO(this->get_logger(), "coord_advertiser node");
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_publisher_;

	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_pos_subscriber_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
	rclcpp::Subscription<LaserScan>::SharedPtr lidar_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr prediction_subscriber_;

	float lat = 0.0f;
	float lon = 0.0f;
	float alt = 0.0f;

	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;

	float lidarDist = 0.0f;

	uint8_t predict_img100_data[DETECTION_SZ*DETECTION_SZ]; // 100x100 image

	void global_position_callback(VehicleGlobalPosition msg);
	void attitude_callback(VehicleAttitude msg);
	void lidar_callback(LaserScan msg);
	void predictor_callback(sensor_msgs::msg::Image msg);
};

/**
 * @brief Subscribe vehicle global position
 * @param 
 */
void CoordAdvertiser::global_position_callback(const VehicleGlobalPosition msg)
{
	alt = msg.alt;
	lon = msg.lon;
	lat = msg.lat;
}

/**
 * @brief Subscribe vehicle attitude
 * @param 
 */
void CoordAdvertiser::attitude_callback(const VehicleAttitude msg)
{
	// Quaternion: w x y z
	tf2::Quaternion q(msg.q[1], msg.q[2], msg.q[3], msg.q[0]);
	tf2::Matrix3x3 m(q);

	m.getRPY(roll, pitch, yaw, 1);
}

/**
 * @brief Subscribe lidar sensor
 * @param 
 */
void CoordAdvertiser::lidar_callback(const LaserScan msg)
{
	std::vector<float> ranges = msg.ranges;

	lidarDist = ranges.front();
}

/**
 * @brief Subscribe to prediction image
 * @param 
 */
void CoordAdvertiser::predictor_callback(const sensor_msgs::msg::Image msg) {
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat cv_img;
	cv::Mat temp;
	cv::Mat cv_img_rot;
	cv::MatIterator_<uint8_t> it, end;
	int matArray_i;

	// Ignore if pitch too high
	if (pitch > 0.087) {
		return;
	}

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e) {
		RCLCPP_INFO(this->get_logger(), "cv_bridge exception");
		return;
	}

	// Source - https://stackoverflow.com/a/65835875
	// Posted by stateMachine, modified by community. See post 'Timeline' for change history
	// Retrieved 2026-07-26, License - CC BY-SA 4.0
	cv::transpose(cv_ptr->image, temp);
	cv::flip(temp, cv_img, -1);
	cv_img_rot = rotate_image(cv_img, yaw*180.0/CV_PI);

	matArray_i = 0;
	for ( it = cv_img_rot.begin<uint8_t>(), end = cv_img_rot.end<uint8_t>(); it != end; ++it ) {
		if (matArray_i >= DETECTION_SZ*DETECTION_SZ) {
			RCLCPP_INFO(this->get_logger(), "predictor_callback exception: matArray_i exceeds array index");
		}
		predict_img100_data[matArray_i] = *it;
		matArray_i++;
	}

	// std::cout << "c: " +
	// std::to_string(predict_img100_data[0]) + "\n" << std::endl;
}

// Source - https://stackoverflow.com/a/9042907
// Posted by Alex Rodrigues, modified by community. See post 'Timeline' for change history
// Retrieved 2026-07-29, License - CC BY-SA 4.0
//
// Function to rotate image about its centre
cv::Mat rotate_image(const cv::Mat& image, double angle) {
    // image.cols is width, image.rows is height
    cv::Point2f image_center(image.cols / 2.0f, image.rows / 2.0f);
    
    // Get the 2x3 rotation matrix
    cv::Mat rot_mat = cv::getRotationMatrix2D(image_center, angle, 1.0);
    
    // Perform the affine transformation
    cv::Mat result;
    cv::warpAffine(image, result, rot_mat, cv::Size(image.cols, image.rows), cv::INTER_LINEAR);
    
    return result;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CoordAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
