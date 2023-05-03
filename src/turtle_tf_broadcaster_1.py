#! /usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


arucoPose = PoseStamped()    # PoseStamped object to be published as the aruco position

# callback function for the location of the aruco marker
def locate_callback_1(data):
    if (data != None): # and flag == True):
        global arucoPose

        # populate PoseStamped object with the data received
        arucoPose.pose.position.x = data.pose.position.x
        arucoPose.pose.position.y = data.pose.position.y - 0.10
        arucoPose.pose.position.z = 0.0
        arucoPose.pose.orientation.x = data.pose.orientation.x   #may need to change to match aruco orientation
        arucoPose.pose.orientation.y = data.pose.orientation.y   #may need to change to match aruco orientation
        arucoPose.pose.orientation.z = data.pose.orientation.z   #may need to change to match aruco orientation
        arucoPose.pose.orientation.w = data.pose.orientation.w   #may need to change to match aruco orientation
                     

def turtle_tf_broadcaster():

    # initialize node
    rospy.init_node('turtle_tf_broadcaster_1')

    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1)

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((arucoPose.pose.position.x, arucoPose.pose.position.y, 0.0),
                         (0.0, 0.0, arucoPose.pose.orientation.z, arucoPose.pose.orientation.w),
                         rospy.Time.now(),
                         "robot_2/map",
                         "map")
        rate.sleep()


if __name__ == '__main__':
    try:
        turtle_tf_broadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("startup error")
        pass