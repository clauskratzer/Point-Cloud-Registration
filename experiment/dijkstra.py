from heapq import heappush, heappop

class Dijkstra:
    def __init__(self, vertices):
        self.edges = vertices
        self.n = len(vertices)

    def dijkstra(self, start):
        dis, vis, hq = {}, {}, []

        for node in self.edges: # todo 
            dis[node] = 99999999
            vis[node] = False

        dis[start], vis[start] = 0, True 
        heappush(hq, (0, start))

        while hq:
            (d, node) = heappop(hq)
            vis[node] = True

            for n, weight in self.adj[node].items(): 
                if (not vis[n]) and (d + weight < dis[n]):
                    dis[n] = d + weight 
                    heappush(hq, (dis[n], n))

        return dis


G = {'s':{'u':10, 'x':5},
     'u':{'v':1, 'x':2},
     'v':{'y':4},
     'x':{'u':3, 'v':9, 'y':2},
     'y':{'s':7, 'v':6}}


if __name__ == '__main__':
    d = Dijkstra(G)
    print(d.dijkstra('s'))
    print(d.dijkstra('u'))
    print(d.dijkstra('x'))
