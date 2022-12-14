/**
 * @file offb_coords.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double x_current = 0;
double y_current = 0;
double z_current = 0;
std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

void cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    z_current = msg->pose.position.z;
    ROS_INFO("X: ");
    ROS_INFO_STREAM(x_current);
    ROS_INFO("Y: ");
    ROS_INFO_STREAM(y_current);
    ROS_INFO("Z: ");
    ROS_INFO_STREAM(z_current);
    ROS_INFO(" ");
    pose.push_back(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_coords");
    ros::NodeHandle nh;

    //ros::init(argc, argv, "sub");
    ros::NodeHandle nk;
    ros::Subscriber sub = nk.subscribe("mavros/local_position/pose", 1000, cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;


    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 1;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 0.75;
    pose2.pose.position.y = 0;
    pose2.pose.position.z = 1;

    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 1;
    pose3.pose.position.y = 0.75;
    pose3.pose.position.z = 1;

    geometry_msgs::PoseStamped pose4;
    pose4.pose.position.x = -0.5;
    pose4.pose.position.y = 0.75;
    pose4.pose.position.z = 1;

    geometry_msgs::PoseStamped pose5;
    pose5.pose.position.x = 0;
    pose5.pose.position.y = 0;
    pose5.pose.position.z = 1;

    geometry_msgs::PoseStamped pose6;
    pose6.pose.position.x = 0;
    pose6.pose.position.y = 0;
    pose6.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int order = 0;
    ros::Time fly_time = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent && order == 0){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) && order == 0){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                fly_time = ros::Time::now();
                order++;
                ROS_INFO("Going to Pose 1");
            }
        }

        if(current_state.mode == "OFFBOARD" && current_state.armed && order == 1) {
            pose = pose1;
            if((ros::Time::now() - fly_time > ros::Duration(6.0)) && z_current > (pose1.pose.position.z - 0.1)) { 
                order++;
                ROS_INFO("Going to Pose 2");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 2) {
            pose = pose2;
            if(ros::Time::now() - fly_time > ros::Duration(12.0)) { 
                order++;
                ROS_INFO("Going to Pose 3");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 3) {
            pose = pose3;
            if(ros::Time::now() - fly_time > ros::Duration(18.0)) { 
                order++;
                ROS_INFO("Going to Pose 4");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 4) {
            pose = pose4;
            if(ros::Time::now() - fly_time > ros::Duration(24.0)) { 
                order++;
                ROS_INFO("Going to Pose 5");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 5) {
            pose = pose5;
            if(ros::Time::now() - fly_time > ros::Duration(30.0)) { 
                order++;
                ROS_INFO("Going to Pose 6");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 6) {
            pose = pose6;
            if(ros::Time::now() - fly_time > ros::Duration(33.0)) { 
                order++;
                offb_set_mode.request.custom_mode = "AUTO.LAND"; 
                set_mode_client.call(offb_set_mode);
                ROS_INFO("LANDING");
                
            }
        }

        

        local_pos_pub.publish(pose);

        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
