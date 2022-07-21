import numpy as np
import open3d as o3d
import torch

data=torch.load('0.pth')
dat_keys=data.keys()
print(list(dat_keys))


lineset=o3d.geometry.LineSet()
src_corr=data["src_corr_pts"]
tgt_corr=data["tgt_corr_pts"]
n=src_corr.size()[0]

print(n)
for i in range(n):
	tgt_corr[i,:]=tgt_corr[i,:]+torch.tensor([1,1,1])


points=np.concatenate([src_corr, tgt_corr])
lineset.points=o3d.utility.Vector3dVector(points)

lines=[];
for i in range(20):
	lines.append([i,n+i])

lineset.lines=o3d.utility.Vector2iVector(lines)

pcd1=o3d.geometry.PointCloud()

pcd1.points=o3d.utility.Vector3dVector(data['src_pcd'])

pcd2=o3d.geometry.PointCloud()

pcd2.points=o3d.utility.Vector3dVector(data['tgt_pcd'])

pcd2.translate([1,1,1])


o3d.visualization.draw_geometries([pcd1,pcd2,lineset])

