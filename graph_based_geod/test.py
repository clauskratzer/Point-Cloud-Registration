import numpy as np
import pandas as pd
import ggd 

import matplotlib.pyplot as plt
import open3d as o3d


def o3d_pc(np_pc):
    vpc = o3d.utility.Vector3dVector(np_pc)
    pc = o3d.geometry.PointCloud(vpc)
    return pc

def point_cloud_bbox(pc):
    p_min = pc.min(axis=0)
    p_max = pc.max(axis=0)
    print("Bounding box: ", p_min, p_max)
    print("Range:", p_max - p_min)
    return np.abs(p_max - p_min)

def diffusion_distance(alpha, pathlength, eu_dist, inds):
    # 1. Compute transition probability matrix
    p_matrix = np.exp(-eu_dist**2 / alpha)
    np.fill_diagonal(p_matrix, 0)

    # Scaling
    p_matrix = p_matrix/p_matrix.sum(axis=-1)

    # 2. Compute powers of p
    t_matrix = p_matrix**pathlength

    # 3. Compute diffusion distance matrix
    diffusion_distance = np.sqrt(
        np.sum((t_matrix[None, inds, :] - t_matrix[:, None, :])**2, axis=-1)
    )

    return diffusion_distance


def kde_plot(source, target, sample_rate, title, ax):
    # N = source.shape[0]
    # idx = np.random.choice(range(N), int(sample_rate * N))
    data = pd.DataFrame({"source": list(source[0, :].flatten()),
                         "target": list(target[0, :].flatten())})
    data.plot.kde(title=title, ax=ax)
    ax.set_xlabel("distance")


def perturb(pc, voxel_size):
    return np.vstack([pc, pc + np.random.randn(*pc.shape) * voxel_size * 0.1])


if __name__ == "__main__":

    # read data
    steve_info = np.load("./cam2_0003_cam1_0009.npz")
    s_pc = steve_info['s_pc']  # load the source point cloud
    t_pc = steve_info['t_pc']  # load the target point cloud
    rot = steve_info['rot']
    trans = steve_info['trans']
    matching = steve_info['correspondences']

    # re-collect data to make pairs to be index-aligned
    s_pc = s_pc[matching[:, 0]]
    t_pc = t_pc[matching[:, 1]]
    s_pc = (s_pc)@rot + trans.flatten()
    N = s_pc.shape[0]
    # idx = np.random.choice(range(N), int(0.3 * N))
    # s_pc = s_pc[idx, :]
    # t_pc = t_pc[idx, :]

    # eulidean distances
    eucl = np.sqrt(
        np.sum((s_pc[None, 5900, :] - s_pc[:, None, :])**2, axis=-1)
    )

    patch_rate = float(input("Percentage of the Point Cloud to calculate: "))

    print("Patching")
    # extract part of the point cloud
    s_ran = point_cloud_bbox(s_pc)
    t_ran = point_cloud_bbox(t_pc)
    s_rad = s_ran * patch_rate
    t_rad = t_ran * patch_rate
    print("patch radius: ", s_rad)
    print("patch radius: ", t_rad)
    mask = (eucl <= s_rad.max()).flatten()
    s_patch = s_pc[mask]
    t_patch = t_pc[mask]

    # s_o3d = o3d_pc(s_patch)
    # t_o3d = o3d_pc(t_patch)
    # o3d.visualization.draw_geometries([s_o3d, t_o3d])
    # computation and visualization

    print("Patching is done")
    # _, _, s_patch, t_patch = build_C_data()

    n = s_patch.shape[0]
    node_inds = np.random.choice(range(n), n//50)
    node_inds = np.sort(node_inds)

    radius = float(input("radius: "))
    print(n)
    s_dist = ggd.np_graph_geo_dist(s_patch, node_inds, radius)
    t_dist = ggd.np_graph_geo_dist(t_patch, node_inds, radius)
    

    
    print("calculating Euclidean distances ...")
    s_eu_dist = np.sqrt(
        np.sum((s_patch[None, :, :] - s_patch[:, None, :])**2, axis=-1)
    )
    t_eu_dist = np.sqrt(
        np.sum((t_patch[None, :, :] - t_patch[:, None, :])**2, axis=-1)
    )
    print("Euclidean distances is calculated : )\n")
    
    s_eu_dist_patch = s_eu_dist[:, node_inds]
    t_eu_dist_patch = t_eu_dist[:, node_inds]


    # diffusion distances
    path_len = int(input("input the path length for diffusion distance: "))
    alpha = float(input("alpha: "))

    print("calculating diffusion distances ...")
    s_diff_dist = diffusion_distance(alpha, path_len, s_eu_dist, node_inds)
    t_diff_dist = diffusion_distance(alpha, path_len, t_eu_dist, node_inds)

    # print("diffusion distances are calculated\n")


    print("Plotting...\n")
    kde_rate = float(input("Input a sampling rate for KDE (0 to 1): "))
    _, axes = plt.subplots(1, 2, figsize=(25, 5))

    kde_plot(s_eu_dist_patch,
             t_eu_dist_patch,
             kde_rate,
             "Euclidean distance distributions",
             ax=axes[0])
    kde_plot(s_dist,
             t_dist,
             kde_rate,
             "Geodesic distance distributions",
             ax=axes[1])
    kde_plot(s_diff_dist,
             t_diff_dist,
             kde_rate,
             "Diffusion distance distributions",
             ax=axes[2])

    plt.show()
