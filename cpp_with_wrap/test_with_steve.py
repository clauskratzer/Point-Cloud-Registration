import numpy as np
import pandas as pd
from vgd import np_voxel_size_geo_dist, np_num_voxel_geo_dist

import matplotlib.pyplot as plt
from numpy.linalg import matrix_power

from utils import build_C_data

def point_cloud_bbox(pc):
    p_min = pc.min(axis=0)
    p_max = pc.max(axis=0)
    print("Bounding box: ", p_min, p_max)
    print("Range:", p_max - p_min)
    return np.abs(p_max - p_min)


def pairwise_Euclidean_distances(point_cloud):
    cols, rows = np.shape(point_cloud)
    Eucl_dist = np.empty((cols, cols))
    for i in range(cols):
        for j in range(cols):
            Eucl_dist[i, j] = np.linalg.norm(point_cloud[i] - point_cloud[j])
    return Eucl_dist


def diffusion_distance(point_cloud, alpha, pathlength, eu_dist, inds):
    # Just for now
    Euclidean_distances = eu_dist

    # 1. Compute transition probability matrix
    p_matrix = np.exp(- Euclidean_distances**2 / alpha)
    np.fill_diagonal(p_matrix, 0)

    # Scaling
    p_matrix = p_matrix/p_matrix.sum(axis=-1)

    # 2. Compute powers of p
    t_matrix = matrix_power(p_matrix, pathlength)

    # 3. Compute diffusion distance matrix
    n = np.shape(t_matrix)[0]
    diffusion_distance = np.sqrt(
        np.sum((t_matrix[None, inds, :] - t_matrix[:, None, :])**2, axis=-1)
    )

    return diffusion_distance


def kde_plot(source, target, sample_rate, title, ax):
    N = source.shape[0]
    idx = np.random.choice(range(N), int(sample_rate * N))
    data = pd.DataFrame({"source": list(source[0, :].flatten()),
                         "target": list(target[0, :].flatten())})
    data.plot.kde(title=title, ax=ax)


def perturb(pc, voxel_size):
    return np.vstack([pc, pc + np.random.randn(*pc.shape) * voxel_size * 0.5])


if __name__ == "__main__":

    # read data
    steve_info = np.load("./cam2_0003_cam1_0009.npz")
    s_pc = steve_info['s_pc']  # load the source point cloud
    t_pc = steve_info['t_pc']  # load the target point cloud
    matching = steve_info['correspondences']

    # re-collect data to make pairs to be index-aligned
    s_pc = s_pc[matching[:, 0]]
    t_pc = t_pc[matching[:, 1]]

    # eulidean distances
    eucl = np.sqrt(
        np.sum((s_pc[None, 8900, :] - s_pc[:, None, :])**2, axis=-1)
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

    # # computation and visualization

    print("Patching is done")
    # _, _, s_patch, t_patch = build_C_data()

    n = s_patch.shape[0]
    node_inds = np.random.choice(range(n), n//50)

    while True:
        use_voxel_count = int(input("Use voxel number [type \"1\"]"
                                    + "or voxel size [type \"2\"]? "))
        if use_voxel_count == 1:
            voxel_count = int(input("Voxel Count: "))
            s_voxel_size = s_ran.max() / (voxel_count - 1)
            t_voxel_size = t_ran.max() / (voxel_count - 1)
            s_patch_temp = perturb(s_patch, s_voxel_size)
            t_patch_temp = perturb(t_patch, t_voxel_size)
            s_dist = np_num_voxel_geo_dist(s_patch_temp, node_inds, voxel_count)
            t_dist = np_num_voxel_geo_dist(t_patch_temp, node_inds, voxel_count)
            print(s_dist.shape)
            break

        elif use_voxel_count == 2:
            voxel_size = float(input("Voxel Size: "))
            s_patch_temp = perturb(s_patch, voxel_size)
            t_patch_temp = perturb(t_patch, voxel_size)
            s_dist = np_voxel_size_geo_dist(s_patch_temp, node_inds, voxel_size)
            t_dist = np_voxel_size_geo_dist(t_patch_temp, node_inds, voxel_size)
            print(s_dist.shape)

            break
        else:
            continue

    
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
    s_diff_dist = diffusion_distance(s_patch, alpha, path_len, s_eu_dist, node_inds)
    t_diff_dist = diffusion_distance(t_patch, alpha, path_len, t_eu_dist, node_inds)

    print("diffusion distances are calculated\n")


    print("Plotting...\n")
    kde_rate = float(input("Input a sampling rate for KDE (0 to 1): "))
    _, axes = plt.subplots(1, 3, figsize=(25, 5))

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
