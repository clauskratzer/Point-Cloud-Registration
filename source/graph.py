import numpy as np
import math
from heapq import heappush, heappop



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
                vertex_edges.append((POSSIBLE_WEIGHTS[idx_prepared], j))
        graph.append(vertex_edges)
    return graph




class Dijkstra:
    def __init__(self, graph):
        self.graph = graph

    
    def dijkstra(self, s): #, report=False):
        if s > len(self.graph) - 1:
            return None

        distances, visited, hq = [], [], [] 

        for i, v in enumerate(self.graph): 
            distances.append(math.inf)
            visited.append(False)

        distances[s], visited[s] = 0, False # initialize the distance for the start vertex, and the visibility for the start vertex. 
        heappush(hq, (0, s))

        while hq:
            (d, v) = heappop(hq)
            visited[v] = True

            for edge in self.graph[v]: # edge ~ (dist, vert)
                dist2end, end_v = edge
                if (not visited[end_v]) and (dist2end + d < distances[end_v]):
                    distances[end_v] = dist2end + d 
                    heappush(hq, (distances[end_v], end_v))
#         if report:
#             print(f"Starting at: {s}...", 
#                   ''.join(['' if val==0 else f"\nthe s.d. to {key} = {val:.4f}" for key, val in distances.items()]), 
#                   "\n")
        return distances