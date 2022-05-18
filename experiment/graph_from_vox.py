import open3d as o3d
import numpy as np
import math
import matplotlib.pyplot as plt

#edges classes
class edge:
    def __init__(self,svert,evert,l):
        self.svert=svert
        self.evert=evert
        self.l=l
    
class graph_entry:
	def __init__(self,vertex,edgelist):
		self.vertex=vertex
		self.edgelist=edgelist

#load
#data=np.load('cam1_0001_cam2_0004.npz')
#pcd=o3d.geometry.PointCloud()
#pcd.points=o3d.utility.Vector3dVector(data['s_pc'])

#test
testcloud=np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3,1,0],[3,2,0],[3,3,0],[0,1,0],[0,2,0],[0,3,0],[0,0,1],[1,0,1],[2,0,1],[3,0,1],[3,1,1],[3,2,1],[3,3,1],[0,1,1],[0,2,1],[0,3,1]])
pcd=o3d.geometry.PointCloud()
pcd.points=o3d.utility.Vector3dVector(testcloud)




#create voxelization
size=0.05
voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=size)


#add edges
voxels=voxel_grid.get_voxels()
graph=list()
for v in voxels:
	e=graph_entry(v,list())
	c1=voxel_grid.get_voxel_center_coordinate(v.grid_index)
	for w in voxels:
		c2=voxel_grid.get_voxel_center_coordinate(w.grid_index)
		if np.linalg.norm(c1-c2)<=math.sqrt(3)*size and np.linalg.norm(c1-c2)!=0:
			eg=edge(v,w,np.linalg.norm(c1-c2))
			e.edgelist.append(eg)
	graph.append(e)       
           

#visualize
fig = plt.figure()
ax = plt.axes(projection='3d')
for e in graph:
	c1=voxel_grid.get_voxel_center_coordinate(e.vertex.grid_index)
	ax.scatter(c1[0],c1[1],c1[2],marker=',')
	for e2 in e.edgelist:
		c2=voxel_grid.get_voxel_center_coordinate(e2.evert.grid_index)
		ax.plot3D([c1[0],c2[0]],[c1[1],c2[1]],[c1[2],c2[2]])






plt.show()
