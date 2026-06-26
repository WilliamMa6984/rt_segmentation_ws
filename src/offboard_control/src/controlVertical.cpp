/**
 * @brief Offboard control adapted from PX4 Offboard control example (Mickey Cowden <info@cowden.tech>, Nuno Marques <nuno.marques@dronesolutions.io>)
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <math.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class VerticalControl : public rclcpp::Node
{
public:
	VerticalControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		goto_setpoint_publisher_ = this->create_publisher<GotoSetpoint>("/fmu/in/goto_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		vehicle_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos,
      		std::bind(&VerticalControl::position_callback, this, _1));
		vehicle_stat_subscriber_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status_v4", qos,
      		[this](const VehicleStatus::UniquePtr msg) {
				// Sim already armed
				if (msg->arming_state == 2) {
					armed = true;
				}
				// Sim started -> continue
				if (msg->timestamp > 1000000 && msg->arming_state == 1) {
					// Change to Offboard mode (2 seconds after system start)
					publish_offboard_control_mode();
					publish_goto_setpoint(target_x, target_y, target_z);
					
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

					// Arm the vehicle
					this->arm();
					
					armed = true;
				}
			}
		);
		
		auto timer_callback = [this]() -> void {
			if (armed == true) {
				// offboard_control_mode needs to be paired with goto_setpoint
				publish_offboard_control_mode();
				publish_goto_setpoint(target_x, target_y, target_z);
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();


private:
	bool armed = false;
	
	float target_x = 0.0f;
	float target_y = 0.0f;
	const float target_z = -10.0; // Constant

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<GotoSetpoint>::SharedPtr goto_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_stat_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_goto_setpoint(float x, float y, float z);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void position_callback(VehicleLocalPosition msg);
	float magnitude(float x, float y, float z);
	template <typename T> int sgn(T val);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void VerticalControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void VerticalControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void VerticalControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void VerticalControl::publish_goto_setpoint(float x, float y, float z)
{
	GotoSetpoint msg{};
	msg.position = {x, y, z};
	msg.flag_control_heading = false;
	msg.heading = 0.0f;
	msg.flag_set_max_horizontal_speed = true;
	msg.max_horizontal_speed = 5.0f;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	goto_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void VerticalControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Subscribe vehicle position
 * @param 
 */
void VerticalControl::position_callback(const VehicleLocalPosition msg)
{
	const float x_step_size = 10.0f;
	const float y_step_size = 15.0f;
	float x = 0.0f;
	float y = 0.0f;
	static bool axis_flag = false;
	static bool posneg_flag = false;

	float dist = std::abs(VerticalControl::magnitude(msg.x-target_x, msg.y-target_y, msg.z-target_z));
	

	if (dist <= 0.2)
	{
		x = round(msg.x);

		if (axis_flag)
		{
			target_x = float(x+x_step_size);
		}
		else 
		{
			if (posneg_flag) {
				target_y = float(-y_step_size);
			} else {
				target_y = float(y_step_size);
			}
			posneg_flag = !posneg_flag;
		}
		axis_flag = !axis_flag;

		RCLCPP_INFO(this->get_logger(), "=========");
		std::cout << "Out Target: " +
		std::to_string(target_x) + " " + 
		std::to_string(target_y) + "\n" << std::endl;
	}
}

// Source - https://stackoverflow.com/a/4609795
// Posted by user79758, modified by community. See post 'Timeline' for change history
// Retrieved 2026-04-26, License - CC BY-SA 4.0
template <typename T> int VerticalControl::sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


float VerticalControl::magnitude(float x, float y, float z)
{
	return std::sqrt(x*x + y*y + z*z);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VerticalControl>());

	rclcpp::shutdown();
	return 0;
}
