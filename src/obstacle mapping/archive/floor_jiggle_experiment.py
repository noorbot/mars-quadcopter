# january 26, 2024
# this is stared as a copy from my_plane_segmentation.py
# the goal is to the the wavering of the points that make up the floor - even in just one instant

#! /usr/bin/env python3
import rospy
import open3d as o3d
import open3d_conversions
from sensor_msgs.msg import PointCloud2
import time
import copy
import numpy as np

rospy.init_node('my_plane_segmentation')

current_cloud = None

# CALLBACK FUNCTION TO READ POINTCLOUD DATA FROM SUBSCRIPTION
def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(1)

listener = rospy.Subscriber('/camera/depth/color/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher1 = rospy.Publisher('output_cloud', PointCloud2, queue_size=1)

while not rospy.is_shutdown():
    if current_cloud is None:
        continue

    # CONVERT POINTCLOUD MSG TO O3D DATATYPE
    start = time.process_time()
    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    # VOXEL DOWNSAMPLING (FOR SPEED)
    o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size=0.01)

# NEW STUFF HERE FOR FLOOR JIGGLE
    # for one frame, print the range of max-min in z (m)
    output_cloud = copy.deepcopy(o3d_cloud)
    output_points = np.asarray(output_cloud.points)
    max_point = np.max(output_points[:, 2])
    min_point = np.min(output_points[:, 2])
    print(max_point - min_point)
    #print(min_point)

    # # print z-value of a chosen pixel
    # static_point = np.array([0, 0, 0.15])
    # pcd_tree = o3d.geometry.KDTreeFlann(output_cloud)  # build KDTree from outlier_cloud
    # [k, idx, _] = pcd_tree.search_knn_vector_xd(static_point, 1) # use knn to find 1 nearest point to ttb location

    


    # CONVERT O3D DATA BACK TO POINTCLOUD MSG TYPE - SEE TIME THIS TAKES (MOST TIME CONSUMING)
    start3 = time.process_time()
    ros_output_cloud = open3d_conversions.to_msg(output_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)

 
    publisher1.publish(ros_output_cloud)

    current_cloud = None
    rate.sleep()