import numpy as np
import math
from heapq import heappush, heappop
from numba import njit, jit, float32, float64, int32, typeof
from numba.typed import List

POSSIBLE_WEIGHTS = [0., 1., np.sqrt(2), np.sqrt(3)]



def vidc2graph(vidc):
    """
    Create graph from voxel centers 
    Input : voxel_centers are numpy array of size (3, )
    Output: graph, looks like
    {
      tuple(A): [ ( index_of_normalized_distance, tuple(B) ), ... ], 
      tuple(B): [ ... ],
      ...
     }
    """
    graph = []
    for i in range(len(vidc)): 
        vertex_edges = []
        for j in range(len(vidc)):
            diff = np.abs(vidc[i] - vidc[j])
            if np.max(diff) > 1: continue # skip if the inf norm bigger than 1 * voxel_size
            else:
                idx_prepared = (np.sum(diff)).astype(int).item() # the square of the eucl. dist. = 1 norm = index
                vertex_edges.append((POSSIBLE_WEIGHTS[idx_prepared], float(j)))
        graph.append(vertex_edges)
    return graph


@njit(locals=dict(edge=(float64, float64)))
def dijkstra(graph, s): #, report=False):
    if s > len(graph) - 1:
        return None

    distances, visited, hq = [], [], [] 

    for i, v in enumerate(graph): 
        distances.append(9999999.)
        visited.append(False)

    distances[s], visited[s] = 0., False # initialize the distance for the start vertex, and the visibility for the start vertex. 
    heappush(hq, (0., float(s)))

    while hq:
        l = heappop(hq)

        visited[int(l[1])] = True

        for edge in graph[int(l[1])]: # edge ~ (dist, vert)
            
            if (not visited[int(edge[1])]) and (edge[0] + l[0] < distances[int(edge[1])]):
                distances[int(edge[1])] = edge[0] + l[0] 
                heappush(hq, (distances[int(edge[1])], edge[0]))
#         if report:
#             print(f"Starting at: {s}...", 
#                   ''.join(['' if val==0 else f"\nthe s.d. to {key} = {val:.4f}" for key, val in distances.items()]), 
#                   "\n")
    return distances



vidc=np.array([[1,2,3,4,5,6,7,8,9,10],[10,9,8,7,6,5,4,3,2,1],[4,6,2,7,9,2,1,5,6,3]])
print(vidc)
test=vidc2graph(vidc)
print(test)
print(typeof(test))
test=dijkstra(test,0)