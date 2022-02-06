#ifndef __DRONE_HPP__
#define __DRONE_HPP__

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Drone : public rclcpp::Node
{
public:
	Drone();

private:

	enum class FlightState
	{ sDisarmed, sArmed, sTakingOff, sReturningToLaunch, sLanding };

	enum class FlightMode
	{ mOffboard, mTakeOff, mLand, mReturnToLaunch };

    void arm();
	void disarm();
    void setFlightMode(FlightMode mode);

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint2(float x, float y, float z, float yaw);
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	void flight_mode_timer_callback();

	rclcpp::TimerBase::SharedPtr timer_;
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

};

Drone::Drone() : Node("drone")
{
	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

	// get common timestamp
	timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10, [this](const px4_msgs::msg::Timesync::UniquePtr msg) {timestamp_.store(msg->timestamp);});

	offboard_setpoint_counter_ = 0;

	timer_ = this->create_wall_timer(100ms, std::bind(&Drone::flight_mode_timer_callback, this));
}


void Drone::flight_mode_timer_callback()
{
	if (offboard_setpoint_counter_ == 10)
	{
		this->setFlightMode(FlightMode::mOffboard);
	}

	if (offboard_setpoint_counter_ == 20)
	{
		this->arm();
	}

	if (offboard_setpoint_counter_ < 200)
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint2(0.0, 0.0, -5.0, -1.6);
	}

	if (offboard_setpoint_counter_ < 300 && offboard_setpoint_counter_ > 200)
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint2(20, 0.0, -5.0, 0);
	}

	if (offboard_setpoint_counter_ < 400 && offboard_setpoint_counter_ > 300) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint2(20, 20, -5.0, 1.6);
	}

	if (offboard_setpoint_counter_ < 500 && offboard_setpoint_counter_ > 400) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint2(0, 20, -5.0, 3.14);
	}

	if (offboard_setpoint_counter_ == 500)
	{
		this->setFlightMode(FlightMode::mLand);
	}

	offboard_setpoint_counter_++;
}



void Drone::setFlightMode(FlightMode mode)
{
	switch (mode)
	{
		case FlightMode::mOffboard:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			RCLCPP_INFO(this->get_logger(), "Offboard flight mode set");
			break;

		case FlightMode::mTakeOff:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 5.5, 1.5);
			RCLCPP_INFO(this->get_logger(), "TakeOff flight mode set");
			break;
			
		case FlightMode::mLand:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
			RCLCPP_INFO(this->get_logger(), "Land flight mode set");
			break;
			
		case FlightMode::mReturnToLaunch:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
			RCLCPP_INFO(this->get_logger(), "Return to Launch flight mode set");
			break;
			
		default:
			RCLCPP_INFO(this->get_logger(), "No flight mode set");
	}

}

void Drone::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void Drone::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void Drone::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

void Drone::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = 0.0;
	msg.y = 0.0;
	msg.z = -5.0;
	msg.yaw = 0; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}

void Drone::publish_trajectory_setpoint2(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = x;
	msg.y = y;
	msg.z = z;
	msg.yaw = yaw; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}

void Drone::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

#endif /*__DRONE_HPP__*/