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
    # print(voxels)
    vertices=[ Vertex(tuple(vx.grid_index),list()) for vx in voxels ]
    for v1 in vertices:
        c1=grid.get_voxel_center_coordinate(np.array(v1.vid))
        for v2 in vertices:
            c2=grid.get_voxel_center_coordinate(np.array(v2.vid))
            if np.linalg.norm(c1-c2)<=math.sqrt(3)*size and np.linalg.norm(c1-c2)!=0:
                edge=Edge(v1,v2,np.linalg.norm(c1-c2)) # length
                v1.edgelist.append(edge)
    return vertices

def averageDistanceMeasure(graph, printdistances=False):
    alpha, beta, gamma = 0.4, 0.2, 0.4 #convex parameters
    di = Dijkstra(graph)
    n= len(di.vertices)
    averageDistance={}
    for v in graph: 
        #distances=di.dijkstra(v, printdistances)
        distances=di.dijkstra(v, False)

        print(distances)
        print("________________________________________________________________________________________________________________")
        eccentricity = max(distances.values())
        centricity = 1/n*(sum(distances.values()))
        if printdistances: 
            print("Maximum distance:",eccentricity)
            print("Average distance to all other voxels:",centricity, "\n")

        avDistToAllOther =[]
        for vert in distances.keys():
            if distances.get(vert)==0:
                continue
            avDistToAllOther.append(alpha*distances.get(vert) + beta*eccentricity + gamma*centricity)
            if printdistances:
                print("average distance from vertex ", v.vid, " to vertex ", vert, "\n")
                print(avDistToAllOther[-1], "\n")  

        averageDistance[v.vid] = avDistToAllOther         

    return averageDistance


#Required input of geodesic distance: (lists of n points, list of m neighbouring points)
#output of geodesic distance: [n, number of neighbour points] object
#calculate geodesic dist: from all sampeled points to all sampeled points and from all sampeled points to all points: [m, m+n] m…. number of sampeled points, n number of all points.'
def calcPointPairDistances(datapoints, neighbourPoints):
    m,n=len(neighbourPoints), len(datapoints)
    print(m, n)
   # distances=np.zeros(shape=(m,m+n), dtype=float)
    distances=np.zeros(shape=(n,m), dtype=float)
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points=o3d.utility.Vector3dVector(datapoints)
    size=1
    voxel_grid1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1,voxel_size=size)
    graph = voxels2graph(voxel_grid1, size)

    averageDistances = averageDistanceMeasure(graph, True)
    i=0
    for k,v in averageDistances.items():
        distances[i,:]=v
        i=i+1

    return distances

    






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
    samplepoints = np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[3,1,0],[3,2,0],[3,3,0],[0,1,0],[0,2,0],[0,3,0],[0,0,1],[1,0,1],[2,0,1],[3,0,1],[3,1,1],[3,2,1],[3,3,1],[0,1,1],[0,2,1]])
    

    distances=calcPointPairDistances(points, samplepoints)
    print(distances)
    print(distances.shape) #should be



  