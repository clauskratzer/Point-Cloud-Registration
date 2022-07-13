// Program to find Dijkstra's shortest path using
// priority_queue in STL
#include<bits/stdc++.h>
#include <vector>
#include <queue>
#include<list>
using namespace std;
# define INF 0x3f3f3f3f
 



vector<float> shortestPath(int src, vector<vector<pair<float,int>>> graph)
{
    // Create a priority queue to store vertices that
    // are being preprocessed. This is weird syntax in C++.
    // Refer below link for details of this syntax
    // https://www.geeksforgeeks.org/implement-min-heap-using-stl/
    priority_queue< pair<float,int>, vector <pair<float,int>> , greater<pair<float,int>> > pq;
    int V=graph.size();
 
    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    vector<float> dist(V, INF);
 
    // Insert source itself in priority queue and initialize
    // its distance as 0.
    pq.push(make_pair(0.0, src));
    dist[src] = 0.0;
 
    /* Looping till priority queue becomes empty (or all
    distances are not finalized) */
    while (!pq.empty())
    {
        // The first vertex in pair is the minimum distance
        // vertex, extract it from priority queue.
        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = pq.top().second;
        pq.pop();
 
        // 'i' is used to get all adjacent vertices of a vertex
        
        int s = graph[u].size();
        for (int i = 0; i < s ; ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = graph[u][i].second;
            float weight = graph[u][i].first;
 
            // If there is shorter path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                // Updating distance of v
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
            }
        }

    }
 
   

    return dist;
}

int main()
{
	

}