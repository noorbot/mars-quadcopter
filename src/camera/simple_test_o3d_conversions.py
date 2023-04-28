#! /usr/bin/env python3
import rospy
import open3d_conversions
from sensor_msgs.msg import PointCloud2

rospy.init_node('open3d_conversions_example')

current_cloud = None

def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    print("reading pointcloud...")
    current_cloud = pointcloud2_msg

rate = rospy.Rate(1)

listener = rospy.Subscriber('/camera/depth/color/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher = rospy.Publisher('~processed_point_cloud', PointCloud2, queue_size=1)

while not rospy.is_shutdown():
    if current_cloud is None:
        continue

    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    # do open3d things
    # ...

    ros_cloud = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
        
    current_cloud = None

    publisher.publish(ros_cloud)
    rate.sleep()
