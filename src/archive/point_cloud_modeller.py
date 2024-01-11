#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl
import std_msgs.msg


pcl_list = []


def pcl_callback(data):
    new_pcl = []
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    # new_pcl.header = header
    for p in pcl.read_points(data, field_names=('x', 'y', 'z'), skip_nans=True):
        p_int = (p[0], round(p[1]), round(p[2]))
        if p_int not in pcl_list:
            pcl_list.append(p_int)


rospy.init_node('point_cloud_modeller', anonymous=True)
sub = rospy.Subscriber('camera/depth/color/points', PointCloud2, pcl_callback)
pub = rospy.Publisher('env_model', PointCloud2, queue_size=10)
while not rospy.is_shutdown():
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pub.publish(
        pcl.create_cloud_xyz32(
            header,
            pcl_list
        )
    )
rospy.spin()