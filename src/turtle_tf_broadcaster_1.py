#! /usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion


arucoPose = PoseStamped()    # PoseStamped object to be published as the aruco position
do_once = True               # variable to ensure the map merging parameters are only set once
aruco_found = False          # variable to ensure the map merging parameters are set only after the aruco marker is found

# callback function for the location of the aruco marker
def locate_callback_1(data):
    if (data != None): # and flag == True):
        global arucoPose
        global aruco_found

        # populate PoseStamped object with the data received
        arucoPose.pose.position.x = data.pose.position.x - 0.10 #10 cm offset from aruco marker to UGV centre
        arucoPose.pose.position.y = data.pose.position.y   
        arucoPose.pose.position.z = 0.0 # z position does not matter
        arucoPose.pose.orientation.x = data.pose.orientation.x 
        arucoPose.pose.orientation.y = data.pose.orientation.y
        arucoPose.pose.orientation.z = data.pose.orientation.z
        arucoPose.pose.orientation.w = data.pose.orientation.w

        aruco_found = True # set true once aruco has been found and callback activated
                     

def turtle_tf_broadcaster():

    # initialize node
    rospy.init_node('turtle_tf_broadcaster_1')

    rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1)

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        # Quaternion(w, x, y, z)    MAY WANT TO LEAVE OUT X AND Y
        # it is necessary to normalize the quaternion before sending the transform
        my_quat=Quaternion(arucoPose.pose.orientation.w, arucoPose.pose.orientation.x, arucoPose.pose.orientation.y, arucoPose.pose.orientation.z)
        norm_quat = my_quat.normalised
        if norm_quat == Quaternion(0,0,0,0): # before the aruco is found, this quaternion throws an error. Set to a default value
            norm_quat = Quaternion(1,0,0,0)

        # set the parameters for map merging initial poses (x, y, z, yaw)
        global do_once, aruco_found    # only do once, and wehen aruco is found
        if aruco_found == True and do_once == True:
            rospy.set_param('/robot_2/map_merge/init_pose_x', arucoPose.pose.position.x)
            rospy.set_param('/robot_2/map_merge/init_pose_y', arucoPose.pose.position.y)
            rospy.set_param('/robot_2/map_merge/init_pose_z', arucoPose.pose.position.z)
            (roll, pitch, yaw) = euler_from_quaternion([norm_quat.x, norm_quat.y,norm_quat.z, norm_quat.w]) # convert quaternion to RPY
            rospy.set_param('/robot_2/map_merge/init_pose_yaw' , yaw)
            rospy.loginfo("PARAMS SET: " + str(arucoPose.pose.position.x) + " " + str(arucoPose.pose.position.y) + " " + str(arucoPose.pose.position.z) + " " + str(yaw))
            do_once = False

        # publish transform from map to robot_2/map
        br.sendTransform((arucoPose.pose.position.x, arucoPose.pose.position.y, 0.0),
                         (norm_quat.x, norm_quat.y, norm_quat.z, norm_quat.w),
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