import open3d as o3d
import numpy as np
import math

import sys
import matplotlib.pyplot as plt


from graph import Vertex, Edge, Dijkstra

#edges class
   
def voxels2graph(grid, size):
    #add edges
    voxels=grid.get_voxels()
    vertices=[ Vertex(tuple(vx.grid_index),list()) for vx in voxels ]
    for v1 in vertices:
        c1=grid.get_voxel_center_coordinate(np.array(v1.vid))
        for v2 in vertices:
            c2=grid.get_voxel_center_coordinate(np.array(v2.vid))
            if np.linalg.norm(c1-c2)<=math.sqrt(3)*size and np.linalg.norm(c1-c2)!=0:
                edge=Edge(v1,v2,np.linalg.norm(c1-c2)) # length
                v1.edgelist.append(edge)
    return vertices

def trim_dijkstra_output(samplepoints, di_output):
    ditrimmed={}
    
    for vert in di_output.keys():
        if (vert in samplepoints):
            ditrimmed[vert]=di_output.get(vert)
    return ditrimmed

def averageDistanceMeasure(graph,samplepoints, printdistances=False):
    alpha, beta, gamma = 0.4, 0.2, 0.4 #convex parameters
    di = Dijkstra(graph)
    n= len(di.vertices)
    averageDistance={}
    for v in graph: 
        distances=di.dijkstra(v, False)
        distances_trimmed=trim_dijkstra_output(samplepoints, distances)
        eccentricity = max(distances_trimmed.values())
        centricity = 1/n*(sum(distances_trimmed.values()))
        if printdistances: 
            print("Maximum distance:",eccentricity)
            print("Average distance to all other voxels:",centricity, "\n")

        avDistToAllOther =[]
        for vert in distances_trimmed.keys():
            # TODO: should starting point be skipped?
            # if distances.get(vert)==0:
            #     continue
            avDistToAllOther.append(alpha*distances_trimmed.get(vert) + beta*eccentricity + gamma*centricity)
            if printdistances:
                print("average distance from vertex ", v.vid, " to vertex ", vert, "\n")
                print(avDistToAllOther[-1], "\n")  

        averageDistance[v.vid] = avDistToAllOther         

    return averageDistance


#Required input of geodesic distance: (lists of n points (sparsly sampled point cloud), list of m neighbouring points (whole point cloud))
#output of geodesic distance: [m, n] matrix
def calcPointPairDistances(datapoints, neighbourPoints):
    m,n=len(neighbourPoints), len(datapoints)
    distances=np.zeros(shape=(m,n), dtype=float)
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points=o3d.utility.Vector3dVector(neighbourPoints)
    size=1
    voxel_grid1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1,voxel_size=size)
    graph = voxels2graph(voxel_grid1, size)

    samplepoints_as_vertices= points_to_vertices(datapoints, size)
    averageDistances = averageDistanceMeasure(graph,samplepoints_as_vertices,False)
    i=0
    # insert the distances into matrix
    for k,v in averageDistances.items():
        distances[i,:]=v
        i=i+1

    return distances

def points_to_vertices(points, size):
    #create also vertices for the samplepoints
    pcd = o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(points)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=size)
    graph = voxels2graph(voxel_grid, size)
    di = Dijkstra(graph)
    points_as_vertices=[]
    for v in di.vertices:
        points_as_vertices.append(v.vid)
    return points_as_vertices






if __name__ == "__main__":
    #---- Test Case 1 for printing ----#
    # testcloud1 = np.array([[0,0,0],[0,0,1],[0,0,2],[0,1,1],[0,0,3]])

    # pcd1 = o3d.geometry.PointCloud()
    # pcd1.points=o3d.utility.Vector3dVector(testcloud1)
    # size=1
    # voxel_grid1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1,voxel_size=size)
    # graph1 = voxels2graph(voxel_grid1, size)

    # averageDistances = averageDistanceMeasure(graph1, True)

    # print("----------------------",averageDistances.values())


    #---- Test Case 2 for visualization ----#
    points = np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3,1,0],[3,2,0],[3,3,0],[0,1,0],[0,2,0],[0,3,0],[0,0,1],[1,0,1],[2,0,1],[3,0,1],[3,1,1],[3,2,1],[3,3,1],[0,1,1],[0,2,1],[0,3,1]])
    samplepoints = np.array([[0,0,0],[1,0,0],[2,0,0],[3,1,0],[3,2,0],[3,3,0],[0,1,0],[0,3,0],[0,0,1],[1,0,1],[2,0,1],[3,0,1],[3,1,1],[3,3,1],[0,1,1],[0,2,1]])
 
    distances=calcPointPairDistances(samplepoints, points)
    print(distances)
    print(distances.shape) #should be 20 times 16



  