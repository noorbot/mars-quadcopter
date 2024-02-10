#! /usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion
import math
import pandas as pd
import time

avgArucoPose = PoseStamped()
ttbPose = PoseStamped()    # PoseStamped object to be published as the aruco position
do_once = True               # variable to ensure the map merging parameters are only set once


aruco_df = pd.DataFrame(columns = ['x', 'y', 'z', 'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w' ])

conditionMet = False
firstReading = True
old_value = [0,0,0]
start = time.process_time()

# callback function for the location of the aruco marker
def listen_aruco_pose(trans, rot):
    if (trans != None): 

        global aruco_df
        global avgArucoPose
        global conditionMet
        global firstReading
        global old_value
        global start
        
        if (firstReading):
            print(old_value)
            start = time.process_time()
            firstReading = False
            print("FIRST READING")

        new_value = trans
        if (new_value != old_value):

            list = [trans[0], 
                    trans[1],
                    trans[2],
                    rot[0],
                    rot[1],
                    rot[2],
                    rot[3]
            ]
        
            aruco_df.loc[len(aruco_df)] = list
            print((time.process_time() - start))
        

        old_value = new_value

        if (len(aruco_df.index) > 10 and (time.process_time() - start) > 0.3):  # if we have more than 10 aruco readings and over 5 seconds has passed since the first one
            print(aruco_df)
            conditionMet = True

            avgArucoPose.pose.position.x = aruco_df.loc[:, 'x'].mean()
            avgArucoPose.pose.position.y = aruco_df.loc[:, 'x'].mean()
            avgArucoPose.pose.position.z = aruco_df.loc[:, 'x'].mean()
            avgArucoPose.pose.orientation.x = aruco_df.loc[:, 'orientation.x'].mean()
            avgArucoPose.pose.orientation.y = aruco_df.loc[:, 'orientation.y'].mean()
            avgArucoPose.pose.orientation.z = aruco_df.loc[:, 'orientation.z'].mean()
            avgArucoPose.pose.orientation.w = aruco_df.loc[:, 'orientation.w'].mean()


    return avgArucoPose, conditionMet

# callback function for the location of the aruco marker
def calc_ttb_pose(avgArucoPose):

    global ttbPose

    # populate PoseStamped object with the data received
    (aruco_roll, aruco_pitch, aruco_yaw) = euler_from_quaternion([avgArucoPose.pose.orientation.x, avgArucoPose.pose.orientation.y, avgArucoPose.pose.orientation.z, avgArucoPose.pose.orientation.w]) # convert quaternion to RPY
    ttbPose.pose.position.x = avgArucoPose.pose.position.x + 0.10*math.cos(aruco_yaw) # x position account for 10 cm offset from aruco marker to UGV centre
    ttbPose.pose.position.y = avgArucoPose.pose.position.y + 0.10*math.sin(aruco_yaw) # y position account for 10 cm offset from aruco marker to UGV centre
    ttbPose.pose.position.z = 0.0 # z position does not matter
    ttbPose.pose.orientation.x = avgArucoPose.pose.orientation.x 
    ttbPose.pose.orientation.y = avgArucoPose.pose.orientation.y
    ttbPose.pose.orientation.z = avgArucoPose.pose.orientation.z
    ttbPose.pose.orientation.w = avgArucoPose.pose.orientation.w

    return ttbPose
                     

def turtle_tf_broadcaster():

    # initialize node
    rospy.init_node('turtle_tf_broadcaster_1')

    global ttbPose
    global conditionMet

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        if(conditionMet == False):
            # lookup transform between map and fiducial NEW
            try:
                # lookup transform between map and fiducial
                (trans,rot) = listener.lookupTransform('/map', '/fiducial_1', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            avgArucoPose, conditionMet = listen_aruco_pose(trans, rot)

        if(conditionMet == True):

            

            
            global do_once  # only do once

            # set the parameters for map merging initial poses (x, y, z, yaw)
            if do_once == True:

                ttbPose = calc_ttb_pose(avgArucoPose)

                # Quaternion(w, x, y, z)    MAY WANT TO LEAVE OUT X AND Y
                # it is necessary to normalize the quaternion before sending the transform
                my_quat=Quaternion(ttbPose.pose.orientation.w, 0, 0, ttbPose.pose.orientation.z)
                norm_quat = my_quat.normalised
                if norm_quat == Quaternion(0,0,0,0): # before the aruco is found, this quaternion throws an error. Set to a default value
                    norm_quat = Quaternion(1,0,0,0)

                rospy.set_param('/robot_2/map_merge/init_pose_x', ttbPose.pose.position.x.item()) # the .item() is needed to convert from numpy float to native python gloat to be accepted
                rospy.set_param('/robot_2/map_merge/init_pose_y', ttbPose.pose.position.y.item())
                rospy.set_param('/robot_2/map_merge/init_pose_z', 0.0)
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