#! /usr/bin/env python3
import rospy
import open3d as o3d
import open3d_conversions
from sensor_msgs.msg import PointCloud2
import time

rospy.init_node('my_plane_segmentation')

current_cloud = None

# CALLBACK FUNCTION TO READ POINTCLOUD DATA FROM SUBSCRIPTION
def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(1)

listener = rospy.Subscriber('/camera/depth/color/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher1 = rospy.Publisher('inlier_cloud', PointCloud2, queue_size=1)
publisher2 = rospy.Publisher('outlier_cloud', PointCloud2, queue_size=1)

while not rospy.is_shutdown():
    if current_cloud is None:
        continue

    # CONVERT POINTCLOUD MSG TO O3D DATATYPE -  SEE TIME THIS TAKES
    start = time.process_time()
    o3d_cloud = open3d_conversions.from_msg(current_cloud)
    print("TIME convert to open3d: ")
    print(time.process_time() - start)

    # VOXEL DOWNSAMPLING (FOR SPEED)
    o3d_cloud = o3d_cloud.voxel_down_sample(voxel_size=0.01)


    #print("Radius oulier removal")
    #cl, ind = o3d_cloud.remove_radius_outlier(nb_points=16, radius=0.02)
    # display_inlier_outlier(voxel_down_pcd, ind)

    # PLANE SEGMENTATION - SEE TIME THIS TAKES
    start2 = time.process_time()
    plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.03,
                                             ransac_n=3,
                                             num_iterations=100)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = o3d_cloud.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = o3d_cloud.select_by_index(inliers, invert=True)
    # o3d.visualization.draw([inlier_cloud, outlier_cloud])
    print("TIME perform plane segmentation: ")
    print(time.process_time() - start2)

    # CONVERT O3D DATA BACK TO POINTCLOUD MSG TYPE - SEE TIME THIS TAKES (MOST TIME CONSUMINF)
    start3 = time.process_time()
    ros_inlier_cloud = open3d_conversions.to_msg(inlier_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    ros_outlier_cloud = open3d_conversions.to_msg(outlier_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    print("TIME convert back to msg: ")
    print(time.process_time() - start3)
    print("-------------------------")
 
    publisher1.publish(ros_inlier_cloud)
    publisher2.publish(ros_outlier_cloud)

    current_cloud = None
    rate.sleep()