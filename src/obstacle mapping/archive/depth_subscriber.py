#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import open3d_conversions

vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)
pcd = o3d.geometry.PointCloud()
vis.add_geometry(inlier_cloud, outlier_cloud)

def pcl_callback(data):
    global inlier_cloud, outlier_cloud
    rospy.info("Recieved pointcloud with seqience number " + data.header.seq)

    open3d_conversions.rosToOpen3d(data, pcd)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.03,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    

rospy.init_node('depth_subscriber', anonymous=True)
sub = rospy.Subscriber('camera/depth/color/points', PointCloud2, pcl_callback)
pub_inlier = rospy.Publisher('inlier_cloud', PointCloud2, queue_size=10)
pub_outlier = rospy.Publisher('outlier_cloud', PointCloud2, queue_size=10)

while not rospy.is_shutdown():
    vis.update_geometry()
    keep_running = vis.poll_events()
    vis.update_renderer(inlier_cloud, outlier_cloud)
rospy.spin()