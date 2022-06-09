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
    
    #Average distance measure
        alpha, beta, gamma = 0.4, 0.2, 0.4 #convex parameters

        averageDistance={}
        
        eccentricity = max(distances_dict.values())
        centricity = 1/n*(sum(distances_dict.values()))
        
        avDistToAllOther ={}
        for vert in distances_dict.keys():
            # TODO: should starting point be skipped?
            # if distances.get(vert)==0:
            #     continue
            avDistToAllOther[vert]=alpha*distances_dict.get(vert) + beta*eccentricity + gamma*centricity

        for j in range(m): 
            neighborpoint_as_vertex=point_to_vertex(neighbourPoints[j,:],voxel_grid1, graph, size)
            distances[i,j]=avDistToAllOther[neighborpoint_as_vertex.vid]
    
    return distances



def point_to_vertex(point,voxel_grid, graph, size):
    #create also vertices for the samplepoints
    
    point_ind=voxel_grid.get_voxel(point)
    for v in graph:
        if (v.vid==point_ind).all():
            return v