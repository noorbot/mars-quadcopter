/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "smooth_coords");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // ros::Subscriber local_position_pose=nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/local_position/pose", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;

    // double poseArray[3][9] = {
    //     {0, 0, 0},
    //     {0, 0, 1.25},
    //     {0, 0.2, 1.25},
    //     {0, 0.4, 1.25},
    //     {0, 0.6, 1.25},
    //     {0, 0.8, 1.25},
    //     {0, 1.0, 1.25},
    //     {0, 1.2, 1.25},
    //     {0, 1.2, 0},        
    // }


    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 1.25;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 1;
    pose2.pose.position.z = 1.25;

    geometry_msgs::PoseStamped pose3;
    pose3 = pose2;

    geometry_msgs::PoseStamped pose4;
    pose4.pose.position.x = 0;
    pose4.pose.position.y = 0;
    pose4.pose.position.z = 1.25;

    geometry_msgs::PoseStamped pose5;
    pose5.pose.position.x = 0;
    pose5.pose.position.y = 0;
    pose5.pose.position.z = 0;

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
                    last_request = ros::Time::now();
                    fly_time = ros::Time::now();
                    order++;
                    ROS_INFO("Rising up!");
                }
            }
        }

        if(current_state.mode == "OFFBOARD" && current_state.armed && order == 1) {
            pose = pose1;
            if(ros::Time::now() - fly_time > ros::Duration(14.0)) { 
                order++;
                ROS_INFO("Going to Pose 2");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 2) {
            pose = pose2;
            if(ros::Time::now() - fly_time > ros::Duration(20.0)) { 
                order++;
                ROS_INFO("now lets go slowwww");
            }
        }
        
        // trying something new here
        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 3) {
            for(int i=0; i<10; i++){
                pose3.pose.position.y = pose3.pose.position.y-0.2;
                pose=pose3;
                std::cout << pose3.pose.position;
                std::cout << "---"
                local_pos_pub.publish(pose);//this should be here right?
                sleep(1.0);
            }
            if(ros::Time::now() - fly_time > ros::Duration(30.0)) { 
                order++;
                ROS_INFO("Going to Pose 4");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 4) {
            pose = pose4;
            if(ros::Time::now() - fly_time > ros::Duration(36.0)) { 
                order++;
                ROS_INFO("Going to Land");
            }
        }

        else if(current_state.mode == "OFFBOARD" && current_state.armed && order == 5) {
            pose = pose5;
            if(ros::Time::now() - fly_time > ros::Duration(39.0)) { 
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


