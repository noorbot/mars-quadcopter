import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R

def convert_to_transfromation_matrix(trans, rot):
    r = R.from_rotvec(rot)        # convert RPY to rotation matrix
    T = np.eye(4)               # initialize transformation matrix T
    T[0:3,0:3] = r.as_matrix()  # set rotation matrix elements
    T[0:3,3] = trans            # set translation vector elements
    return T

if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("camera/pics/ttb.ply")

    T = convert_to_transfromation_matrix([0,0,-1.3], [3.14,0,0])
    #T = np.array([(3.14,0,0,0),(0,1,0,0),(0,0,1.57,0),(0,0,0,1)])
    pcd_trans = pcd.transform(T)


    # voxel downsamploing
    pcd = pcd.voxel_down_sample(voxel_size=0.01)

    points = np.asarray(pcd.points)
    # points = points[0:5000]
    print(points.shape)
    pcd_sel = pcd_trans#.select_by_index(np.where(points[:, 2] < -1)[0])

    #o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([pcd_sel])


    # ground plane segmentation
    plane_model, inliers = pcd_sel.segment_plane(distance_threshold=0.03,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")

    inlier_cloud = pcd_sel.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd_sel.select_by_index(inliers, invert=True)
   
   
    # visualize output
    o3d.visualization.draw([inlier_cloud, outlier_cloud])