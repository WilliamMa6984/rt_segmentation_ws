#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class CoordAdvertiser : public rclcpp::Node
{
public:
	CoordAdvertiser() : Node("coord_advertiser")
	{

		publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("/fmu/in/img2coord", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_global_pos_subscriber_ = this->create_subscription<VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos,
      		std::bind(&CoordAdvertiser::global_position_callback, this, _1));
		vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
      		std::bind(&CoordAdvertiser::attitude_callback, this, _1));

		auto timer_callback = [this]()->void {
			auto debug_vect = px4_msgs::msg::DebugVect();
			this->publisher_->publish(debug_vect);
			
			std::cout << "Global Pos (lat lon alt): " +
			std::to_string(lat) + " " + 
			std::to_string(lon) + " " + 
			std::to_string(alt) + "\n" << std::endl;

			std::cout << "RPY: " +
			std::to_string(roll) + " " + 
			std::to_string(pitch) + " " + 
			std::to_string(yaw) + "\n" << std::endl;
		};
		timer_ = this->create_wall_timer(1000ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;

	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_pos_subscriber_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;

	float lat = 0.0f;
	float lon = 0.0f;
	float alt = 0.0f;

	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;

	void global_position_callback(VehicleGlobalPosition msg);
	void attitude_callback(VehicleAttitude msg);
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
	tf2::Quaternion q(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
	tf2::Matrix3x3 m(q);

	m.getRPY(roll, pitch, yaw, 1);
}

// /**
//  * Evandro Bernardes, Stéphane Viollet. Quaternion to Euler angles conversion: a direct, general and com-
//  * putationally efficient method. PLoS ONE, 2022, 17 (11), pp.e0276302. ⟨10.1371/journal.pone.0276302⟩. ⟨hal-
//  * 03848730⟩
//  * @brief Quaternion to Euler
//  */
// std::vector<float> CoordAdvertiser::q2euler(const std::array<float, 4>& q)
// {
// 	Eigen::Quaternion
// }

int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CoordAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
