#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <math.h>

float X;
float Y;	

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
class setpoint : public rclcpp::Node {
public:
    setpoint() : Node("setpoint") {
     
	    offboard_control_mode_publisher_ =
		    this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
	    trajectory_setpoint_publisher_ =
		    this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
	    vehicle_command_publisher_ =
		    this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

	    // get common timestamp
	    timesync_sub_ =
		    this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
			    [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
				    timestamp_.store(msg->timestamp);
			    });

	    offboard_setpoint_counter_ = 0;

		subscription_ =
			this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
				"/fmu/vehicle_local_position/out",
				10,
				[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
					X = msg->x;
					Y = msg->y;
					float Z = msg->z;
					if(!start_trj && (p0_x + 1.0 > X && p0_x - 1.0 < X)&&(p0_y + 1.0 > Y && p0_y - 1.0 < Y)&&(p0_z + 1.0 > Z && p0_z - 1.0 < Z)){
						start_trj = true;
						std::cout << "start trj!" << std::endl;
					}
				}
			);


	    auto sendCommands = [this]() -> void {
		    if (offboard_setpoint_counter_ == 10) {
			    // Change to Offboard mode after 10 setpoints
			    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			    // Arm the vehicle
			    this->arm();

		    }
			// the spiral, in polar coordinates (theta, rho), is given by
			// theta = theta_0 + omega*t
			// rho = rho_0 + K*theta
			float theta = theta_0 + omega * 0.1 * discrete_time_index;
			float rho = rho_0 + K * theta;
			
			// from polar to cartesian coordinates
			des_x = rho * cos(theta);
			des_y = rho * sin(theta);

			// velocity computation
			float dot_rho = K*omega;
			dot_des_x = dot_rho*cos(theta) - rho*sin(theta)*omega;
			dot_des_y = dot_rho*sin(theta) + rho*cos(theta)*omega;
			// desired heading direction
			gamma = atan2(dot_des_y, dot_des_x);

        	// offboard_control_mode needs to be paired with trajectory_setpoint
		    publish_offboard_control_mode();
		    publish_trajectory_setpoint();

       		     // stop the counter after reaching 11
		    if (offboard_setpoint_counter_ < 11) {
			    offboard_setpoint_counter_++;
		    }
			if (start_trj){
				discrete_time_index++;
			}
	    };
	    commandTimer = this->create_wall_timer(100ms, sendCommands);
    }

    void arm() const;
    void disarm() const;
    void topic_callback() const;
private:
    
	bool start_trj = false;

	const float omega = 0.3; 	// angular speed of the POLAR trajectory
	const float K = 2;			// [m] gain that regulates the spiral pitch

	
	const float rho_0 = 2;
	const float theta_0 = 0;
	const float p0_z = -5.0;
	float p0_x = rho_0*cos(theta_0);
	float p0_y = rho_0*sin(theta_0);
	float des_x = p0_x, des_y = p0_y, des_z = p0_z;
	float dot_des_x = 0.0, dot_des_y = 0.0;
	float gamma = M_PI_4;
    
	uint32_t discrete_time_index = 0;

    rclcpp::TimerBase::SharedPtr commandTimer;
    rclcpp::TimerBase::SharedPtr waypointTimer;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    //
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
    //
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint() const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
			         float param2 = 0.0) const;
};

void setpoint::arm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void setpoint::disarm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void setpoint::publish_offboard_control_mode() const {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
}	

void setpoint::publish_trajectory_setpoint() const {
    TrajectorySetpoint msg{};
    msg.timestamp = timestamp_.load();
    msg.position = {des_x, des_y, des_z};
    msg.velocity = {dot_des_x, dot_des_y, 0.0};
    msg.yaw = gamma;		//-3.14; // [-PI:PI]
    trajectory_setpoint_publisher_->publish(msg);
}

void setpoint::publish_vehicle_command(uint16_t command, float param1,
				          float param2) const {
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

int main(int argc, char* argv[]) {
    std::cout << "Starting setpoint node..." << std::endl;
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<setpoint>());

    rclcpp::shutdown();
    return 0;
}