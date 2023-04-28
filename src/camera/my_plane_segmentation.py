#! /usr/bin/env python3
import rospy
import open3d as o3d
import open3d_conversions
from sensor_msgs.msg import PointCloud2

rospy.init_node('my_plane_segmentation')

current_cloud = None

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

    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    # do open3d things
    # ...
    plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.04,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = o3d_cloud.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = o3d_cloud.select_by_index(inliers, invert=True)
    # o3d.visualization.draw([inlier_cloud, outlier_cloud])

    ros_inlier_cloud = open3d_conversions.to_msg(inlier_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    ros_outlier_cloud = open3d_conversions.to_msg(outlier_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
 
    publisher1.publish(ros_inlier_cloud)
    publisher2.publish(ros_outlier_cloud)

    current_cloud = None
    rate.sleep()