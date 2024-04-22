#!/usr/bin/env python3

import rospy
import tf
from visualization_msgs.msg import Marker

do_once = False

# function to create a Marker object from the trans and rot array populated by the tf listener
def create_marker(trans, rot):

    global do_once # added this variable Feb 7 2024 so that the aruco is only read once. this helps with rosbagging

    # create the Marker object to be returned

    marker = Marker()

    # populate with marker parameters
    #################################
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 3  # fiducial id

    # Set the scale of the marker
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = trans[0]
    marker.pose.position.y = trans[1]
    marker.pose.position.z = trans[2]
    marker.pose.orientation.x = rot[0]
    marker.pose.orientation.y = rot[1]
    marker.pose.orientation.z = rot[2]
    marker.pose.orientation.w = rot[3]

    do_once = True  # read aruco once, that's all we need
    
    # return the Marker object
    return marker 

def locate_marker():
    # initialize the node
    rospy.init_node('locate_marker_3', anonymous=True)

    # create the TransformListener object
    listener = tf.TransformListener()


    rate = rospy.Rate(10)       # 10 Hz refresh rate

    global do_once

    while not rospy.is_shutdown():
        try:
            # lookup transform between map and fiducial
            if(do_once == False):  # only read aruco if havent got it before
                (trans,rot) = listener.lookupTransform('/map', '/fiducial_3', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # create the publisher object to publish the Marker object to rviz
        publisher = rospy.Publisher('visulization_marker/ArUco_Location_3', Marker, queue_size=50)


        # create the Marker object
        marker = create_marker(trans, rot)

        # publish the Marker object
        publisher.publish(marker)


        rate.sleep()

if __name__ == '__main__':
    try:
        locate_marker()
        rospy.loginfo('test')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
 