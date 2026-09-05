#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cmath>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "parameters.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace nav_msgs::msg;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using std::placeholders::_1;

class CoordAdvertiser : public rclcpp::Node
{
public:
	CoordAdvertiser() : Node("coord_advertiser")
	{

		occupancy_grid_publisher_ = this->create_publisher<OccupancyGrid>("/moss_occ_grid", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		lidar_subscriber_ = this->create_subscription<LaserScan>("/lidar", qos,
      		std::bind(&CoordAdvertiser::lidar_callback, this, _1));
		prediction_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/predictor/image_100", qos,
      		std::bind(&CoordAdvertiser::predictor_callback, this, _1));
		pose_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/predictor/robot_pose", qos,
      		std::bind(&CoordAdvertiser::pose_callback, this, _1));

		std::fill(std::begin(detection_map), std::end(detection_map), 0);

		auto timer_callback = [this]()->void {
			auto occuGrid = OccupancyGrid();
			cv::MatIterator_<uint8_t> it, end;
			int matArray_i;

			occuGrid.header.stamp = rclcpp::Clock().now();
			occuGrid.header.frame_id = "map";

			occuGrid.info.resolution = MAP_RESOLUTION;
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

			matArray_i = 0;
			for ( it = detection_map_img.begin<uint8_t>(), end = detection_map_img.end<uint8_t>(); it != end; ++it ) {
				if (matArray_i >= DETECTION_SZ*DETECTION_SZ) {
					RCLCPP_INFO(this->get_logger(), "predictor_callback exception: matArray_i exceeds array index");
					std::cout << "i: " + std::to_string(matArray_i) + "\n" + "x: " + std::to_string(detection_map_img.cols) + "\n" + "y: " + std::to_string(detection_map_img.rows) + "\n" << std::endl;
					break;
				}
				detection_map[matArray_i] = *it;
				matArray_i++;
			}
			std::copy(&detection_map[0], &detection_map[DETECTION_SZ*DETECTION_SZ], back_inserter(occuGrid.data));

			this->occupancy_grid_publisher_->publish(occuGrid);
			
			std::cout << "Pos (NED): " +
			std::to_string(north) + " " + 
			std::to_string(east) + " " + 
			std::to_string(down) + "\n" << std::endl;

			std::cout << "RPY: " +
			std::to_string(roll) + " " + 
			std::to_string(pitch) + " " + 
			std::to_string(yaw) + "\n" << std::endl;

			std::cout << "Dist to gnd: " +
			std::to_string(lidarDist) + "\n" << std::endl;
		};
		timer_ = this->create_wall_timer(200ms, timer_callback);

		RCLCPP_INFO(this->get_logger(), "coord_advertiser node");
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_publisher_;

	rclcpp::Subscription<LaserScan>::SharedPtr lidar_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr prediction_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_subscriber_;

	float north = 0.0f;
	float east = 0.0f;
	float down = 0.0f;

	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;

	float lidarDist = 0.0f;

	uint8_t detection_map[DETECTION_SZ*DETECTION_SZ]; // detection map
	cv::Mat detection_map_img = cv::Mat::zeros(cv::Size(DETECTION_SZ, DETECTION_SZ),CV_8UC1);

	void lidar_callback(LaserScan msg);
	void predictor_callback(sensor_msgs::msg::Image msg);
	void pose_callback(std_msgs::msg::Float32MultiArray msg);

	// std::tuple<cv::Mat, cv::Mat> rotate_image
	cv::Mat rotate_image(const cv::Mat& image, double angle, double dist_from_gnd, double tx, double ty);
};

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
 * @brief Subscribe to pose during position
 * @param 
 */
void CoordAdvertiser::pose_callback(const std_msgs::msg::Float32MultiArray msg) {
	north = msg.data[0];
	east = msg.data[1];
	down = msg.data[2];
	roll = msg.data[3];
	pitch = msg.data[4];
	yaw = msg.data[5];
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
	// cv::Mat mask;
	// cv::Mat mask_rot;
	// cv::Mat blended;

	// Ignore if roll or pitch too high
	if (std::abs(pitch) > 0.13 || std::abs(roll) > 0.13) { // 5 deg
		// return empty detection
		// std::fill(std::begin(detection_map), std::end(detection_map), 0);
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
	cv::flip(cv_ptr->image, cv_img, 0);
	cv_img_rot = rotate_image(cv_img, yaw*180.0/CV_PI, lidarDist, east, north);
	cv::rotate(cv_img_rot, detection_map_img, cv::ROTATE_90_COUNTERCLOCKWISE);

	// std::tie(temp, mask) = rotate_image(cv_img, yaw*180.0/CV_PI, lidarDist, east, north);
	// cv::rotate(temp, cv_img, cv::ROTATE_90_COUNTERCLOCKWISE);
	// cv::rotate(mask, mask_rot, cv::ROTATE_90_COUNTERCLOCKWISE);

	// // 1. Calculate the element-wise average of both images
	// cv::add(cv_img*0.25, detection_map_img*0.75, cv_img, mask_rot);
	// // 2. Copy the blended pixels into the destination ONLY within the mask region
    // cv_img.copyTo(detection_map_img, mask_rot);
	// // detection_map_img = mask;
	// // cv_img_rot = 

	// // Get maximum
	// // cv::max(detection_map_img, cv_img_rot, detection_map_img);
}

// Source - https://stackoverflow.com/a/9042907
// Posted by Alex Rodrigues, modified by community. See post 'Timeline' for change history
// Retrieved 2026-07-29, License - CC BY-SA 4.0
//
// Function to rotate image about its centre
cv::Mat CoordAdvertiser::rotate_image(const cv::Mat& image, double angle, double dist_from_gnd, double tx, double ty) {
    // image.cols is width, image.rows is height
    cv::Point2f image_center(image.cols / 2.0f, image.rows / 2.0f);
	cv::Size result_sz = cv::Size(DETECTION_SZ, DETECTION_SZ);

	// Focal lengths
    int w = image.cols; // image_width_in_pixels
    double fov = 1.74; // field_of_view_in_rad
    double f = (w * 0.5) / std::tan(fov * 0.5); // focal_length_in_pixels

    double th = std::atan2(w,f);

    double wp = dist_from_gnd*std::tan(th); // width' m

    double scale_factor = wp / w; // Only for square ratio images

    // Currently 1px:1m ratio
    // Make ratio 1px:X m
    scale_factor = scale_factor / MAP_RESOLUTION;
    
    // Get the 2x3 rotation matrix
    cv::Mat rot_mat = cv::getRotationMatrix2D(image_center, angle, scale_factor);

    // Perform the affine transformation
    cv::Mat img_rot;
    cv::warpAffine(image, img_rot, rot_mat, cv::Size(image.cols, image.rows), cv::INTER_LINEAR); // TODO: image.cols*scale_int for adjustable array size
    
	// // 3. Create a solid white mask of the source image to track its transformation
    // cv::Mat mask_src = cv::Mat::ones(image.size(), CV_8UC1) * 255;
    // cv::Mat mask_rot;
    // cv::warpAffine(mask_src, mask_rot, rot_mat, cv::Size(image.cols, image.rows), cv::INTER_NEAREST);

	// Translate image
	// TODO: move to origin->from global start coords
	double origin_x = 0;
	double origin_y = 0;
    cv::Mat trans_mat = (cv::Mat_<float>(2,3) << 1, 0, tx/MAP_RESOLUTION+origin_x, 0, 1, ty/MAP_RESOLUTION+origin_y);
    cv::Mat result;
	cv::warpAffine(img_rot, result, trans_mat, result_sz, cv::INTER_LINEAR);

    // // Apply translation
    // // 5. Apply translation to BOTH the rotated image and its mask
    // cv::Mat final_img;
    // cv::Mat final_mask;
    // cv::warpAffine(img_rot, final_img, trans_mat, result_sz, cv::INTER_LINEAR);
    // cv::warpAffine(mask_rot, final_mask, trans_mat, result_sz, cv::INTER_LINEAR);
	
    // return {final_img, final_mask};
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
