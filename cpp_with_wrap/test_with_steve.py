import numpy as np
# import cppimport
# vgd = cppimport.imp('wrap')
import pandas as pd
from vgd import np_voxel_size_geo_dist

import matplotlib.pyplot as plt


# test_pc = np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.],[3.,0.,0.],[3.,1.,0.],[3.,2.,0.],[3.,3.,0.],[0.,1.,0.],[0.,2.,0.],[0.,3.,0.],[0.,0.,1.],[1.,0.,1.],[2.,0.,1.],[3.,0.,1.],[3.,1.,1.],[3.,2.,1.],[3.,3.,1.],[0.,1.,1.],[0.,2.,1.],[0.,3.,1.]])

def point_cloud_bbox(pc):
    p_min = pc.min(axis=0)
    p_max = pc.max(axis=0)
    print("Bounding box: ", p_min, p_max)
    print("Range:", p_max-p_min)
    return np.abs(p_max-p_min)

def kde_plot(source, target, sample_rate, title, ax):
    N = source.shape[0]
    idx = np.random.choice(range(N), int(sample_rate*N))
    data = pd.DataFrame({ "source":list(source.flatten()[idx]), "target":list(target.flatten()[idx]) })
    data.plot.kde(title=title, ax=ax)

if __name__ == "__main__":
    patch_rate = float(input("Percentage of the Point Cloud to calculate: "))
    voxel_size = float(input("Voxel Size: "))
    kde_rate = float(input("Sample_rate for Visualization: "))

    # read data
    steve_info = np.load("./cam2_0003_cam1_0009.npz")
    s_pc = steve_info['s_pc'] # load the source point cloud 
    t_pc = steve_info['t_pc'] # load the target point cloud
    trans = steve_info['trans'].flatten()
    rot = steve_info['rot']
    matching = steve_info['correspondences']

    # re-collect data to make pairs to be index-aligned 
    s_pc = s_pc[matching[:,0]]
    t_pc = t_pc[matching[:,1]]
    t_pc = (t_pc - 0.4*trans)@rot


    # eulidean distances
    eucl = np.sqrt(np.sum((s_pc[None,8900,:] - s_pc[:,None,:])**2, axis=-1))

    # extract part of the point cloud
    s_rad = point_cloud_bbox(s_pc)*patch_rate
    t_rad = point_cloud_bbox(t_pc)*patch_rate
    print("patch radius: ",s_rad)
    print("patch radius: ",t_rad)
    mask = (eucl<=s_rad.max()).flatten()
    s_patch=s_pc[mask]
    t_patch=t_pc[mask]

    # computation and visualization
    _, axes = plt.subplots(1, 2, figsize=(15, 5))
    s_eu_dist = np.sqrt(np.sum((s_patch[None,:,:] - s_patch[:,None,:])**2, axis=-1))
    t_eu_dist = np.sqrt(np.sum((t_patch[None,:,:] - t_patch[:,None,:])**2, axis=-1))
    kde_plot(s_eu_dist, t_eu_dist, kde_rate, "Euclidean distance distributions", ax=axes[0])

    s_dist = np_voxel_size_geo_dist(s_patch, 0.03)
    t_dist = np_voxel_size_geo_dist(t_patch, 0.03)
    kde_plot(s_dist, t_dist, kde_rate, "Geodesic distance distributions", ax=axes[1])
    plt.show()

