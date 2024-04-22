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
import csv

avgArucoPose = PoseStamped()
ttbPose = PoseStamped()    # PoseStamped object to be published as the UGV position
do_once = True               # variable to ensure the map merging parameters are only set once


aruco_df = pd.DataFrame(columns = ['x', 'y', 'z', 'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w' ]) # dataframe to hold all aruco readings

conditionMet = False    # whether we have sufficient aruco readings
firstReading = True
old_value = [0,0,0]
start = rospy.Time(5.0)

# callback function for the location of the aruco marker
def listen_aruco_pose(trans, rot):
    if (trans != None): 

        global aruco_df
        global avgArucoPose
        global conditionMet
        global firstReading
        global old_value
        global start
        
        if (firstReading):    # first time through, set start time to now
            start = rospy.get_rostime()
            firstReading = False

        new_value = trans
        if (new_value != old_value):  # if the aruco position reading is new, made a new row with the pose information

            row = [trans[0], 
                    trans[1],
                    trans[2],
                    rot[0],
                    rot[1],
                    rot[2],
                    rot[3]
            ]
        
            aruco_df.loc[len(aruco_df)] = row   # add row to aruco_df dataframe

        old_value = new_value  # update

        if (len(aruco_df.index) > 10 and (rospy.get_rostime() - start) > rospy.Duration(5)):  # if we have more than 10 aruco readings and over 5 seconds has passed since the first one
            #print(aruco_df)
            conditionMet = True    # update variable to say, we have enough readings and now we can take the averages and then publish them

            avgArucoPose.pose.position.x = aruco_df.loc[:, 'x'].mean()   # take the mean of each column of aurco_df adn save to avgArucoPose variable
            avgArucoPose.pose.position.y = aruco_df.loc[:, 'y'].mean()
            avgArucoPose.pose.position.z = aruco_df.loc[:, 'z'].mean()
            avgArucoPose.pose.orientation.x = aruco_df.loc[:, 'orientation.x'].mean()
            avgArucoPose.pose.orientation.y = aruco_df.loc[:, 'orientation.y'].mean()
            avgArucoPose.pose.orientation.z = aruco_df.loc[:, 'orientation.z'].mean()
            avgArucoPose.pose.orientation.w = aruco_df.loc[:, 'orientation.w'].mean()


    return avgArucoPose, conditionMet

# function to calculate the UGV pose
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
    rospy.init_node('turtle_tf_broadcaster_3')

    global ttbPose
    global conditionMet

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        if(conditionMet == False):    # if we don't yet have enough aruco readings
            try:
                # lookup transform between map and fiducial
                (trans,rot) = listener.lookupTransform('/map', '/fiducial_3', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            avgArucoPose, conditionMet = listen_aruco_pose(trans, rot)  # call function to save the aruco pose information


        if(conditionMet == True):       # if we DO have enough aruco readings

            global do_once  # only do once

            # set the parameters for map merging initial poses (x, y, z, yaw)
            if do_once == True:

                ttbPose = calc_ttb_pose(avgArucoPose)  # call function to calculate UGV pose

                # Quaternion(w, x, y, z)    MAY WANT TO LEAVE OUT X AND Y
                # it is necessary to normalize the quaternion before sending the transform
                my_quat=Quaternion(ttbPose.pose.orientation.w, 0, 0, ttbPose.pose.orientation.z)
                norm_quat = my_quat.normalised
                if norm_quat == Quaternion(0,0,0,0): # before the aruco is found, this quaternion throws an error. Set to a default value
                    norm_quat = Quaternion(1,0,0,0)

                # update ROS parameters regarding UGV starting pose used for map_merging
                rospy.set_param('/robot_1/map_merge/init_pose_x', ttbPose.pose.position.x.item()) # the .item() is needed to convert from numpy float to native python gloat to be accepted
                rospy.set_param('/robot_1/map_merge/init_pose_y', ttbPose.pose.position.y.item())
                rospy.set_param('/robot_1/map_merge/init_pose_z', 0.0)
                rospy.set_param('/robot_1/map_merge/init_pose_yaw' , 0.0) #aruco_yaw)   Set yaw to zero always, as with simulation. Taken care of already by broadcaster
                rospy.loginfo("PARAMS SET robot 1: x:" + str(ttbPose.pose.position.x) + "  y:" + str(ttbPose.pose.position.y) + "  z:" + str(ttbPose.pose.position.z) + "  yaw: 0.0")
                
                # write data to csv
                # file = 'Feb15_aruco_heights_3.csv'
                # trial_num = 2.5
                # with open(file, 'a') as f: #prints header in csv
                #                 write = csv.writer(f)
                #                 data= [trial_num, ttbPose.pose.position.x, ttbPose.pose.position.y, norm_quat.w]
                #                 write.writerow(data)

                do_once = False

            # publish transform for the UGV starting pose [from robot_x_noor/odom (middle step tf which is at same loc as /map) to robot_x/odom]
            br.sendTransform((ttbPose.pose.position.x, ttbPose.pose.position.y, 0.0),
                            (norm_quat.x, norm_quat.y, norm_quat.z, norm_quat.w),
                            rospy.Time.now(),
                            "robot_1/odom",         
                            "robot_1_noor/odom")
        rate.sleep()


if __name__ == '__main__':
    try:
        turtle_tf_broadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("startup error")
        pass