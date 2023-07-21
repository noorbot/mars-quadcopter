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

rospy.init_node('my_plane_segmentation')

current_cloud = None


# my custom function to create the transformation matrix from /map to /camera_depth_optical_frame
def convert_to_transfromation_matrix(trans, rot):
    r = R.from_quat(rot)        # convert quaternion to rotation matrix
    T = np.eye(4)               # initialize transformation matrix T
    T[0:3,0:3] = r.as_matrix()  # set rotation matrix elements
    T[0:3,3] = trans            # set translation vector elements
    return T

# CALLBACK FUNCTION TO READ POINTCLOUD DATA FROM SUBSCRIPTION
def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(1)

listener = rospy.Subscriber('/camera/depth/color/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher1 = rospy.Publisher('inlier_cloud', PointCloud2, queue_size=1)
publisher2 = rospy.Publisher('outlier_cloud', PointCloud2, queue_size=1)

# create the TransformListener object
tf_listener = tf.TransformListener()


while not rospy.is_shutdown():
    if current_cloud is None:
        continue

    try:
        # lookup transform between map and camera_depth_optical_frame
        (trans,rot) = tf_listener.lookupTransform('/map', '/camera_depth_optical_frame', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue


    # CONVERT POINTCLOUD MSG TO O3D DATATYPE
    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    # tested idea of converting the Pointcloud2 msg to points array - same result as o3d array
    # points_OG = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(current_cloud)   
    # print("points OG")
    # print(points_OG)


    # VOXEL DOWNSAMPLING (FOR SPEED)
    o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size=0.01)

    points = np.asarray(o3d_cloud.points)  # points are in the camera_depth_optical_frame

    # transform points array to be global - apply transform from map to camera_depth_optical_frame
    T = convert_to_transfromation_matrix(trans, rot)  # convert the transform to a transformation matrix
    cloud_global = o3d_cloud.transform(T)
    points_global = np.asarray(cloud_global.points)
    print("points global")
    print(points)

    cloud_global = cloud_global.select_by_index(np.where(points[:, 2] < 0.2)[0]) # only consider points that are under 20 cm in z (below ttb lidar)


    # transform cloud back for visualization purposes
    cloud_vis = copy.deepcopy(cloud_global).transform(np.linalg.inv(T))

    # convert pointcloud back to msg type
    #ros_cloud1 = open3d_conversions.to_msg(cloud_global, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    # ros_cloud2 = open3d_conversions.to_msg(cloud_vis, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)

    # publish processed pointcloud(s)
    #publisher1.publish(ros_cloud1)
    # publisher2.publish(ros_cloud2)



    # PLANE SEGMENTATION

    plane_model, inliers = cloud_global.segment_plane(distance_threshold=0.03,
                                             ransac_n=3,
                                             num_iterations=100)
    [a, b, c, d] = plane_model
    #print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    #print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = cloud_global.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = cloud_global.select_by_index(inliers, invert=True)
    # o3d.visualization.draw([inlier_cloud, outlier_cloud])


    obstacle_cloud = copy.deepcopy(outlier_cloud)

    # # ignore points around ttb location (20cm radius)
    # # lets say we have a ttb at ttb_x, ttb_y
    # ttb_x = 1
    # ttb_y = 1
    # ttb_points = points_global
    # np.delete(ttb_points, np.all((ttb_x-0.2) < ttb_points[0] and ttb_points < (ttb_x+0.2) and (ttb_y-0.2) < ttb_points[1] and ttb_points[1] < (ttb_y+0.2)))
    # cloud_global = cloud_global.select_by_index(ttb_points[0])
    # # for row in ttb_points:
    # #     distance = np.sqrt(np.square(row[0] - ttb_x) + (row[1] - ttb_y))
    # #     if distance < 0.2:
    # #         ttb_points = np.delete(ttb_points, row)
    # # calculate distance from point (x y) to (ttbx, y). if its <0.2m then remove this point

    obstacle_cloud = cloud_global
    # CONVERT O3D DATA BACK TO POINTCLOUD MSG TYPE - SEE TIME THIS TAKES (MOST TIME CONSUMING)
    ros_inlier_cloud = open3d_conversions.to_msg(inlier_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    ros_outlier_cloud = open3d_conversions.to_msg(obstacle_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    
 
    publisher1.publish(ros_inlier_cloud)
    publisher2.publish(ros_outlier_cloud)

    print("-------------------------")
    current_cloud = None
    rate.sleep()