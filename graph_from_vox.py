import open3d as o3d
import numpy as np
import math
#edges class
class edge:

    def __init__(self,svert,evert,l):
        self.svert=svert
        self.evert=evert
        self.l=l
    
class edges_of_vertex:
	def __init__(self,vertex,edgelist):
		self.vertex=vertex
		self.edgelist=edgelist
#load
#data=np.load('cam1_0001_cam2_0004.npz')
#pcd=o3d.geometry.PointCloud()
#pcd.points=o3d.utility.Vector3dVector(data['s_pc'])

#test
testcloud=np.array([[0,0,0],[0,0,1],[0,0,2],[0,1,1],[0,0,3]])
print(testcloud)
pcd=o3d.geometry.PointCloud()
pcd.points=o3d.utility.Vector3dVector(testcloud)




#create voxelization
size=1
voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=size)

#add edges
voxels=voxel_grid.get_voxels()
print(voxels)
edges=list()
for v in voxels:
	e=edges_of_vertex(v,list())

    c1=voxel_grid.get_voxel_center_coordinate(v.grid_index)
    for w in voxels:
        c2=voxel_grid.get_voxel_center_coordinate(w.grid_index)
        if np.linalg.norm(c1-c2)<=math.sqrt(3)*size and np.linalg.norm(c1-c2)!=0:
           eg=edge(v,w,np.linalg.norm(c1-c2))
           e.edgelist.append(eg)
    edges.append(e)       
           
          

