#! /usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from pyquaternion import Quaternion



arucoPose = PoseStamped()    # PoseStamped object to be published as the aruco position

# callback function for the location of the aruco marker
def locate_callback_1(data):
    if (data != None): # and flag == True):
        global arucoPose

        # populate PoseStamped object with the data received
        arucoPose.pose.position.x = data.pose.position.x
        arucoPose.pose.position.y = data.pose.position.y - 0.10  #10 cm offset from aruco marker to UGV centre
        arucoPose.pose.position.z = 0.0 # z position does not matter
        arucoPose.pose.orientation.x = data.pose.orientation.x 
        arucoPose.pose.orientation.y = data.pose.orientation.y
        arucoPose.pose.orientation.z = data.pose.orientation.z
        arucoPose.pose.orientation.w = data.pose.orientation.w
                     

def turtle_tf_broadcaster():

    # initialize node
    rospy.init_node('turtle_tf_broadcaster_3')

    rospy.Subscriber('visulization_marker/ArUco_Location_3', Marker, locate_callback_1)

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        # Quaternion(w, x, y, z)
        my_quat=Quaternion(arucoPose.pose.orientation.w, 0, 0, arucoPose.pose.orientation.z)
        norm_quat = my_quat.normalised

        if norm_quat == Quaternion(0,0,0,0):
            norm_quat = Quaternion(1,0,0,0)

        br.sendTransform((arucoPose.pose.position.x, arucoPose.pose.position.y, 0.0),
                         (norm_quat.x, norm_quat.y, norm_quat.z, norm_quat.w),
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