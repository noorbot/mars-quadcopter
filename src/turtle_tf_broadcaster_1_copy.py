#! /usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion
import math


ttbPose = PoseStamped()    # PoseStamped object to be published as the aruco position
do_once = True               # variable to ensure the map merging parameters are only set once
aruco_found = False          # variable to ensure the map merging parameters are set only after the aruco marker is found

# callback function for the location of the aruco marker
def calc_ttb_pose(trans, rot):
    if (trans != None): 
        global ttbPose
        global aruco_found
        aruco = PoseStamped()

        aruco.pose.position.x = trans[0]
        aruco.pose.position.y = trans[1]
        aruco.pose.position.z = trans[2]
        aruco.pose.orientation.x = rot[0]
        aruco.pose.orientation.y = rot[1]
        aruco.pose.orientation.z = rot[2]
        aruco.pose.orientation.w = rot[3]

        # populate PoseStamped object with the data received
        (aruco_roll, aruco_pitch, aruco_yaw) = euler_from_quaternion([aruco.pose.orientation.x, aruco.pose.orientation.y, aruco.pose.orientation.z, aruco.pose.orientation.w]) # convert quaternion to RPY
        ttbPose.pose.position.x = aruco.pose.position.x + 0.10*math.cos(aruco_yaw) # x position account for 10 cm offset from aruco marker to UGV centre
        ttbPose.pose.position.y = aruco.pose.position.y + 0.10*math.sin(aruco_yaw) # y position account for 10 cm offset from aruco marker to UGV centre
        ttbPose.pose.position.z = 0.0 # z position does not matter
        ttbPose.pose.orientation.x = aruco.pose.orientation.x 
        ttbPose.pose.orientation.y = aruco.pose.orientation.y
        ttbPose.pose.orientation.z = aruco.pose.orientation.z
        ttbPose.pose.orientation.w = aruco.pose.orientation.w

        aruco_found = True # set true once aruco has been found and callback activated

        return ttbPose
                     

def turtle_tf_broadcaster():

    # initialize node
    rospy.init_node('turtle_tf_broadcaster_1')

    global ttbPose

    #rospy.Subscriber('visulization_marker/ArUco_Location_1', Marker, locate_callback_1) NO LONGER

    # create the TransformListener object   NEW
    listener = tf.TransformListener()

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        # lookup transform between map and fiducial NEW
        try:
            # lookup transform between map and fiducial
            (trans,rot) = listener.lookupTransform('/map', '/fiducial_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        ttbPose = calc_ttb_pose(trans, rot)

        # Quaternion(w, x, y, z)    MAY WANT TO LEAVE OUT X AND Y
        # it is necessary to normalize the quaternion before sending the transform
        my_quat=Quaternion(ttbPose.pose.orientation.w, 0, 0, ttbPose.pose.orientation.z)
        norm_quat = my_quat.normalised
        if norm_quat == Quaternion(0,0,0,0): # before the aruco is found, this quaternion throws an error. Set to a default value
            norm_quat = Quaternion(1,0,0,0)

        # set the parameters for map merging initial poses (x, y, z, yaw)
        global do_once, aruco_found    # only do once, and wehen aruco is found
        if aruco_found == True and do_once == True:
            rospy.set_param('/robot_2/map_merge/init_pose_x', ttbPose.pose.position.x)
            rospy.set_param('/robot_2/map_merge/init_pose_y', ttbPose.pose.position.y)
            rospy.set_param('/robot_2/map_merge/init_pose_z', ttbPose.pose.position.z)
            rospy.set_param('/robot_2/map_merge/init_pose_yaw' , 0.0) #aruco_yaw)   Set yaw to zero always, as with simulation. Taken care of already 
            rospy.loginfo("PARAMS SET robot 2: x:" + str(ttbPose.pose.position.x) + "  y:" + str(ttbPose.pose.position.y) + "  z:" + str(ttbPose.pose.position.z) + "  yaw: 0.0")
            do_once = False

        # publish transform from map to robot_2/map
        br.sendTransform((ttbPose.pose.position.x, ttbPose.pose.position.y, 0.0),
                         (norm_quat.x, norm_quat.y, norm_quat.z, norm_quat.w),
                         rospy.Time.now(),
                         "robot_2/odom",
                         "robot_2_noor/odom")
        rate.sleep()


if __name__ == '__main__':
    try:
        turtle_tf_broadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("startup error")
        pass