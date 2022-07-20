import numpy as np
import plotly.graph_objs as go
from itertools import combinations
import open3d as o3d
import tqdm.autonotebook as tqdm
import pandas as pd


def build_C_data():
    test_idc = np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3,1,0],[3,2,0],[3,3,0],[0,1,0],[0,2,0],[0,3,0],[0,0,1],[1,0,1],[2,0,1],[3,0,1],[3,1,1],[3,2,1],[3,3,1],[0,1,1],[0,2,1],[0,3,1]])
    
    test_idc_deformed = np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3.3,1,0],[3.6,2,0],[4.0,3,0],[-0.3,1,0],[-0.6,2,0],[-1,3,0],[0,0,1],[1,0,1],[2,0,1],[3,0,1],[3.3,1,1],[3.6,2,1],[4.0,3,1],[-0.3,1,1],[-0.6,2,1],[-1,3,1]])
    
    pert_pc = np.random.randn(*test_idc.shape)*0.1 + test_idc
    pert_pc2 = np.random.randn(*test_idc.shape)*0.1 + test_idc
    pert_pc_combine = np.vstack([pert_pc, pert_pc2])
    
    pert_pc = np.random.randn(*test_idc_deformed.shape)*0.1 + test_idc_deformed
    pert_pc2 = np.random.randn(*test_idc_deformed.shape)*0.1 + test_idc_deformed
    pert_pc_combine_deformed = np.vstack([pert_pc, pert_pc2])
    
    return test_idc, test_idc_deformed, pert_pc_combine, pert_pc_combine_deformed


def voxelize_pc(pc, size):
    pcd = o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(pc)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=size)
    return voxel_grid

def visualize_point_cloud(*point_cloud):
    vis_data = []
    
    N = len(point_cloud)
#     M = len(color)
#     if M < N: 
#         color = color + [color[-1] for i in range(N-M)]
    for p in point_cloud:
        x, y, z = zip(*p)
        vis_data.append(go.Scatter3d(x=x, y=y, z=z, mode='markers', marker=dict(size=4)))

    fig = go.Figure(data=vis_data)
    fig.update_layout(
        width=500,
        height=500,
        autosize=False,
        scene=dict(
            aspectratio = dict( x=1, y=1, z=0.5 )
        ),
        showlegend=False,
        margin=dict(l=0, r=0, t=0, b=0),
    )
    fig.show()

def visualize_voxelization(point_cloud, voxel_grid, vidc=None):
    if vidc == None:
        voxels = voxel_grid.get_voxels()
        vidc = np.zeros((len(voxels),3))
        for i, vx in enumerate(voxels):
            vidc[i, :] = vx.grid_index
            
    cube_edges = set()
    for ind in vidc:
        bbp = np.asanyarray(voxel_grid.get_voxel_bounding_points(ind))
        for s, e in combinations(bbp, 2):
            if np.abs(np.sum(np.abs(s-e)) - voxel_grid.voxel_size) < 0.00001 :
                cube_edges.add(((tuple(s), tuple(e))))

    vis_data = []
    for s, e in cube_edges:
        xx, yy, zz = list(zip(s,e))
        vis_data.append(
            go.Scatter3d(x=xx, y=yy, z=zz, mode='lines', 
                         marker=dict(color='rgba(0,0,255,0.3)')
                        )
        )
    
    rx, ry, rz = point_cloud.max(axis=0) - point_cloud.min(axis=0)
    ry = ry/rx
    rz = rz/rx
    
    
    x, y, z = zip(*point_cloud)
    vis_data.append(go.Scatter3d(x=x, y=y, z=z, mode='markers', marker=dict(size=4, color='red')))

    fig = go.Figure(data=vis_data)
    fig.update_layout(
        width=500,
        height=500,
        autosize=False,
        scene=dict(
            aspectratio = dict( x=1, y=ry, z=rz )
        ),
        showlegend=False,
        margin=dict(l=0, r=0, t=0, b=0),
        hovermode=False
    )
    fig.show()

       
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

def load_steve_patch(center=8900, patch_rate=0.4):
    steve_info = np.load("./Steve_Moonwalk/cam2_0003_cam1_0009.npz")
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
    
    return s_patch, t_patch