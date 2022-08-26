#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

#flag = False    # flag to only send return goal once
markerGoal = PoseStamped()    # PoseStamped object to be published as the goal

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


# callback function for /tb3_init_pose subscription 
def locate_callback_1(data):
    if (data != None): #and flag == True):
        global markerGoal

        # populate PoseStamped object with the data received
        markerGoal.pose.position.x = data.pose.position.x
        markerGoal.pose.position.y = data.pose.position.y
        markerGoal.pose.position.z = 1.0
        markerGoal.pose.orientation.w = 1.0 

def go_to_marker():
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # subscribe to 
    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        pose = markerGoal
        local_pos_pub.publish(pose)


        rate.sleep()

if __name__ == '__main__':
    try:
        go_to_marker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass