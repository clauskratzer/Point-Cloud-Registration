import numpy as np
import pandas as pd
from vgd import np_voxel_size_geo_dist, np_num_voxel_geo_dist

import matplotlib.pyplot as plt


def point_cloud_bbox(pc):
    p_min = pc.min(axis=0)
    p_max = pc.max(axis=0)
    print("Bounding box: ", p_min, p_max)
    print("Range:", p_max - p_min)
    return np.abs(p_max - p_min)


def kde_plot(source, target, sample_rate, title, ax):
    N = source.shape[0]
    idx = np.random.choice(range(N), int(sample_rate * N))
    data = pd.DataFrame({"source": list(source[idx, :].flatten()),
                         "target": list(target[idx, :].flatten())})
    data.plot.kde(title=title, ax=ax)


def perturb(pc, voxel_size):
    return np.vstack([pc, pc + np.random.randn(*pc.shape) * voxel_size * 0.5])


if __name__ == "__main__":

    # read data
    steve_info = np.load("./cam2_0003_cam1_0009.npz")
    s_pc = steve_info['s_pc']  # load the source point cloud
    t_pc = steve_info['t_pc']  # load the target point cloud
    trans = steve_info['trans'].flatten()
    rot = steve_info['rot']
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

    # computation and visualization
    n = s_patch.shape[0]
    node_inds = np.random.choice(range(n), n // 50)

    print("Patching is done")

    kde_rate = float(input("Input a sampling rate for KDE (0 to 1): "))
    _, axes = plt.subplots(1, 2, figsize=(15, 5))
    while True:
        use_voxel_count = int(input("Use voxel number [type \"1\"]"
                                    + "or voxel size [type \"2\"]? "))
        if use_voxel_count == 1:
            voxel_count = int(input("Voxel Count: "))
            s_voxel_size = s_ran.max() / (voxel_count - 1)
            t_voxel_size = t_ran.max() / (voxel_count - 1)
            s_patch = perturb(s_patch, s_voxel_size)
            t_patch = perturb(t_patch, t_voxel_size)
            s_dist = np_num_voxel_geo_dist(s_patch, node_inds, voxel_count)
            t_dist = np_num_voxel_geo_dist(t_patch, node_inds, voxel_count)
            print(s_dist.shape)
            kde_plot(s_dist,
                     t_dist,
                     kde_rate,
                     "Geodesic distance distributions",
                     ax=axes[1])
            break

        elif use_voxel_count == 2:
            voxel_size = float(input("Voxel Size: "))
            s_patch = perturb(s_patch, voxel_size)
            t_patch = perturb(t_patch, voxel_size)
            s_dist = np_voxel_size_geo_dist(s_patch, node_inds, voxel_size)
            t_dist = np_voxel_size_geo_dist(t_patch, node_inds, voxel_size)
            print(s_dist.shape)
            kde_plot(s_dist,
                     t_dist,
                     kde_rate,
                     "Geodesic distance distributions",
                     ax=axes[1])
            break
        else:
            continue

    s_eu_dist = np.sqrt(
        np.sum((s_patch[None, node_inds, :] - s_patch[:, None, :])**2, axis=-1)
    )
    t_eu_dist = np.sqrt(
        np.sum((t_patch[None, node_inds, :] - t_patch[:, None, :])**2, axis=-1)
    )
    print(s_eu_dist.shape)

    kde_plot(s_eu_dist,
             t_eu_dist,
             kde_rate,
             "Euclidean distance distributions",
             ax=axes[0])
    plt.show()
