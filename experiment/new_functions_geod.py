#Required input of geodesic distance: (lists of n points (sparsly sampled point cloud), list of m neighbouring points (whole point cloud))
#output of geodesic distance: [n m] matrix
def calcPointPairDistances(datapoints, neighbourPoints):
    m,n=len(neighbourPoints), len(datapoints)
    distances=np.zeros(shape=(n,m), dtype=float)
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points=o3d.utility.Vector3dVector(neighbourPoints)
    size=1
    voxel_grid1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1,voxel_size=size)
    graph = voxels2graph(voxel_grid1, size)
    di=Dijkstra(graph)
    

    for i in range(n): 
        sp_as_vert=point_to_vertex(datapoints[i,:],voxel_grid1, graph, size)
        distances_dict=di.dijkstra(sp_as_vert)
    
        for j in range(m): 
            neighborpoint_as_vertex=point_to_vertex(neighbourPoints[j,:],voxel_grid1, graph, size)
            distances[i,j]=distances_dict[neighborpoint_as_vertex.vid]
    
    return distances

def averageDistanceMeasure(graph,samplepoints, printdistances=False):
    alpha, beta, gamma = 0.4, 0.2, 0.4 #convex parameters
    di = Dijkstra(graph)
    n= len(di.vertices)
    averageDistance={}
    for v in samplepoints: 
        distances=di.dijkstra(v, False)
        eccentricity = max(distances.values())
        centricity = 1/n*(sum(distances.values()))
        if printdistances: 
            print("Maximum distance:",eccentricity)
            print("Average distance to all other voxels:",centricity, "\n")

        avDistToAllOther ={}
        for vert in distances.keys():
            # TODO: should starting point be skipped?
            # if distances.get(vert)==0:
            #     continue
            avDistToAllOther[vert]=alpha*distances.get(vert) + beta*eccentricity + gamma*centricity
            if printdistances:
                print("average distance from vertex ", v.vid, " to vertex ", vert, "\n")
                print(avDistToAllOther[-1], "\n")  

        averageDistance[v.vid] = avDistToAllOther         

    return averageDistance

def point_to_vertex(point,voxel_grid, graph, size):
    #create also vertices for the samplepoints
    
    point_ind=voxel_grid.get_voxel(point)
    for v in graph:
        if (v.vid==point_ind).all():
            return v