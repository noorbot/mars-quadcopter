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
        arucoPose.pose.position.y = data.pose.position.y
        arucoPose.pose.position.z = 0.0
        arucoPose.pose.orientation.w = 1.0   #may need to change to match aruco orientation
                     

def turtle_tf_broadcaster():

    # initialize node
    rospy.init_node('turtle_tf_broadcaster')

    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1)

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((arucoPose.pose.position.x, arucoPose.pose.position.y, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "robot_1/map",
                         "map")
        rate.sleep()


if __name__ == '__main__':
    try:
        turtle_tf_broadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("startup error")
        pass