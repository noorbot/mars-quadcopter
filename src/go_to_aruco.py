#!/usr/bin/env python3
from io import StringIO
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from visualization_msgs.msg import Marker


flag = False    # flag to only send return goal once
markerGoal = PoseStamped()    # PoseStamped object to be published as the goal


# callback function for /tb3_init_pose subscription 
def locate_callback_0(data):
    if (data != None and flag == True):
        global markerGoal

        # populate PoseStamped object with the data received
        markerGoal.pose.position.x = data.pose.position.x
        markerGoal.pose.position.y = data.pose.position.y
        markerGoal.pose.position.z = 1.0
        markerGoal.pose.orientation.w = 1.0                               #WHAT DO WE MAKE ORIENTATION? nEED TO GO HEAD ON


# callback function for /tb3_init_pose subscription 
def locate_callback_1(data):
    if (data != None and flag == True):
        global markerGoal

        # populate PoseStamped object with the data received
        markerGoal.pose.position.x = data.pose.position.x
        markerGoal.pose.position.y = data.pose.position.y
        markerGoal.pose.position.z = 0.0
        markerGoal.pose.orientation.w = 1.0                         

def go_to_marker():
    rospy.loginfo('GOING TO ARUCO')

    # initialize node
    rospy.init_node('go_to_aruco', anonymous=True)

    # subscribe to 
    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1)


    # Publisher obect to publish goal to return to origin
    publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=50)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if (flag == True):  # Check condition for explore_lite completion
            goal = markerGoal

            # publish the PoseStamped object goal
            publisher.publish(goal)
            rospy.sleep(50)
            
            # set flag to false to prevent repeated sending of the goal
            global flag
            flag = False

        rate.sleep()

if __name__ == '__main__':
    try:
        go_to_marker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass