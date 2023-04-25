import open3d as o3d
import numpy as np

print("Load a ply point cloud, print it, and render it")
sample_ply_data = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud("src/camera/pics/first pic.ply")
o3d.visualization.draw_geometries([pcd])