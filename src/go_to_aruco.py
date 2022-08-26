#!/usr/bin/env python3
from io import StringIO
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

#flag = True   # flag to only send return goal once
markerGoal = PoseStamped()    # PoseStamped object to be published as the goal
markerGoal_Down = PoseStamped()    # PoseStamped object to be published as the goal

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

rospy.loginfo('we are in the top of the code')

# callback function for the location of the aruco marker
def locate_callback_1(data):
    if (data != None): # and flag == True):
        global markerGoal

        # populate PoseStamped object with the data received
        markerGoal.pose.position.x = data.pose.position.x
        markerGoal.pose.position.y = data.pose.position.y
        markerGoal.pose.position.z = 1.0
        markerGoal.pose.orientation.w = 1.0   

        global markerGoal_Down

        # populate PoseStamped object with the data received
        markerGoal_Down.pose.position.x = data.pose.position.x
        markerGoal_Down.pose.position.y = data.pose.position.y
        markerGoal_Down.pose.position.z = 0.0
        markerGoal_Down.pose.orientation.w = 1.0                        

def go_to_marker():
    rospy.loginfo('GOING TO ARUCO')

    # initialize node
    rospy.init_node('go_to_aruco', anonymous=True)

    # subscribe to 
    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1)

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Publisher obect to publish goal to return to origin
    publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=50)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #if (flag == True):  # Check condition for explore_lite completion
        goal = markerGoal

        # go to marker and hover above
        publisher.publish(goal)

        rospy.sleep(50)

        # go down in z
        goal = markerGoal_Down
        publisher.publish(goal)
        rospy.sleep(50)

        #land
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'AUTO.LAND'
        set_mode_client.call(offb_set_mode).mode_sent 

            
            # set flag to false to prevent repeated sending of the goal
            #global flag
            #flag = False

        rate.sleep()

if __name__ == '__main__':
    try:
        go_to_marker()
        rospy.loginfo('we are in main')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('njsdfkhsdkjf')
        pass