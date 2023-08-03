#! /usr/bin/env python3
import rospy
import open3d as o3d
import open3d_conversions
from sensor_msgs.msg import PointCloud2
import time
import numpy as np
import tf
#import ros_numpy
from scipy.spatial.transform import Rotation as R
import copy
import pandas as pd
import matplotlib.pyplot as plt
import sys
np.set_printoptions(threshold=sys.maxsize)


rospy.init_node('my_plane_segmentation')

current_cloud = None

# my function to create the transformation matrix from /map to /camera_depth_optical_frame
def convert_to_transfromation_matrix(trans, rot):
    r = R.from_quat(rot)        # convert quaternion to rotation matrix
    T = np.eye(4)               # initialize transformation matrix T
    T[0:3,0:3] = r.as_matrix()  # set rotation matrix elements
    T[0:3,3] = trans            # set translation vector elements
    return T

# my function to ignore the pointcloud points in the location of the turtlebots
def ignore_ttb_points(points_global):
    # ignore points around ttb location (20cm radius)
    # lets say we have a ttb at ttb_x, ttb_y
    ttb1_x = trans_ttb1[0]
    ttb1_y = trans_ttb1[1]
    center1 = np.array([ttb1_x, ttb1_y, 0.1])
    radius1 = 0.25
    distances1 = np.linalg.norm(points_global - center1, axis=1)
    points_global = points_global[distances1 >= radius1]
    cloud_global.points = o3d.utility.Vector3dVector(points_global) # ahjkh this line is causing issues.... can't display this pcd

    ttb2_x = trans_ttb2[0]
    ttb2_y = trans_ttb2[1]
    center2 = np.array([ttb2_x, ttb2_y, 0.1])
    radius2 = 0.25
    distances2 = np.linalg.norm(points_global - center2, axis=1)
    points_global = points_global[distances2 >= radius2]
    cloud_global.points = o3d.utility.Vector3dVector(points_global) # ahjkh this line is causing issues.... can't display this pcd

    return cloud_global

# my function to clean up the obstacle cloud to leave only useful points
def clean_pointcloud(outlier_cloud):
    # DBSAN CLUSTERING
    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.03, min_points=10, print_progress=False))
    if len(np.unique(labels)) > 1:  # if robot is on floor the camera will detect zero clusters and the below code should not be executed
        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # create pandas Dataframe with all outlier_cloud points including their DBSCAN clustering labels
        outlier_cloud_labels = pd.DataFrame(outlier_cloud.points, columns=['x','y', 'z'])
        outlier_cloud_labels['label'] = labels
        # remove all points with label -1 (noise)
        index_1 = outlier_cloud_labels[outlier_cloud_labels['label']==-1]
        points_to_remove = index_1.index.to_list()
        # remove points with fewer than 50 members in the cluster, or only have members within the 'sandwich'
        for cluster in range(max_label):
            curr_label = outlier_cloud_labels[outlier_cloud_labels['label']==cluster]
            if(len(curr_label) < 100) or not((abs(curr_label['z'].min()) > 0.05) or (abs(curr_label['z'].max()) > 0.05)): # remove points that a) are in a cluster with fewer than 100 members or b) are in a cluster with extreme z heights lower than 3cm from the ground.
                points_to_remove.extend(curr_label.index.to_list())

        print('points to remove length: ' + str(len(points_to_remove)))
        # remove points from outlier cloud
        outlier_cloud = outlier_cloud.select_by_index(points_to_remove, invert=True)
        return outlier_cloud
    
def plane_segmentation(cloud_global):
    plane_model, inliers = cloud_global.segment_plane(distance_threshold=0.03,
                                             ransac_n=3,
                                             num_iterations=100)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    #print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = cloud_global.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = cloud_global.select_by_index(inliers, invert=True)
    return outlier_cloud, inlier_cloud

# CALLBACK FUNCTION TO READ POINTCLOUD DATA FROM SUBSCRIPTION
def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(1)

listener_pcd = rospy.Subscriber('/camera/depth/color/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher1 = rospy.Publisher('inlier_cloud', PointCloud2, queue_size=1)
publisher2 = rospy.Publisher('outlier_cloud', PointCloud2, queue_size=1)

# create the TransformListener object
tf_listener_cam = tf.TransformListener()
tf_listener_ttb1 = tf.TransformListener()
tf_listener_ttb2 = tf.TransformListener()


while not rospy.is_shutdown():
    if current_cloud is None:
        continue

    try:
        # lookup transform between map and camera_depth_optical_frame
        (trans,rot) = tf_listener_cam.lookupTransform('/map', '/camera_depth_optical_frame', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    try:
        # lookup transform between map and robot_1/base_footprint
        (trans_ttb1,rot_ttb1) = tf_listener_ttb1.lookupTransform('/map', '/robot_1/base_footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("No tf data for robot_1. Set to x=100 y=0 for point removal(out of the way)")
        (trans_ttb1,rot_ttb1) = ([100.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        pass

    try:
        # lookup transform between map and robot_1/base_footprint
        (trans_ttb2,rot_ttb2) = tf_listener_ttb1.lookupTransform('/map', '/robot_2/base_footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("No tf data for robot_2. Set to x=100 y=0 for point removal (out of the way)")
        (trans_ttb2,rot_ttb2) = ([100.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        pass


    # CONVERT POINTCLOUD MSG TO O3D DATATYPE
    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    # VOXEL DOWNSAMPLING (FOR SPEED)
    o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size=0.01)

    points = np.asarray(o3d_cloud.points)  # points are in the camera_depth_optical_frame

    # transform cloud to be global - apply transform from map to camera_depth_optical_frame
    T = convert_to_transfromation_matrix(trans, rot)  # convert the transform to a transformation matrix
    cloud_global = copy.deepcopy(o3d_cloud).transform(T)
    points_global = np.asarray(cloud_global.points)

    # ONLY CONSIDER POINTS THAT ARE UNDER 20 CM IN Z (BELOW TTB LIDAR)
    points_global = points_global[points_global[:, 2] < 0.2]
    cloud_global = cloud_global.select_by_index(np.where(points_global)[0])

    # CALL FUNCTION TO REMOVE TTB POINTS
    cloud_global = ignore_ttb_points(points_global)

    # PLANE SEGMENTATION
    outlier_cloud, inlier_cloud = plane_segmentation(cloud_global)

    # CALL FUNCTION TO CLEAN UP OBSTACLE POINTCLOUD
    outlier_cloud = clean_pointcloud(outlier_cloud)

    # transform clouds back for visualization purposes
    outlier_cloud_vis = copy.deepcopy(outlier_cloud).transform(np.linalg.inv(T))
    inlier_cloud_vis = copy.deepcopy(inlier_cloud).transform(np.linalg.inv(T))

    # CONVERT O3D DATA BACK TO POINTCLOUD MSG TYPE - SEE TIME THIS TAKES (MOST TIME CONSUMING)
    ros_inlier_cloud = open3d_conversions.to_msg(inlier_cloud_vis, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    ros_outlier_cloud = open3d_conversions.to_msg(outlier_cloud_vis, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)

 
    publisher1.publish(ros_inlier_cloud)
    publisher2.publish(ros_outlier_cloud)

    print("-------------------------")
    current_cloud = None
    rate.sleep()