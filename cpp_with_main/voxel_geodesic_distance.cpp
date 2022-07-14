#include "voxel_geodesic_distance.h"
#include <queue>
# define INF 0x3f3f3f3f

const int DELTAS[26][3] = {    {-1,-1,-1},{ 0,-1,-1},{ 1,-1,-1},
                               {-1,-1, 0},{ 0,-1, 0},{ 1,-1, 0},
                               {-1,-1, 1},{ 0,-1, 1},{ 1,-1, 1},
                               {-1, 0,-1},{ 0, 0,-1},{ 1, 0,-1},
                               {-1, 0, 0},/*myself*/ { 1, 0, 0},
                               {-1, 0, 1},{ 0, 0, 1},{ 1, 0, 1},
                               {-1, 1,-1},{ 0, 1,-1},{ 1, 1,-1},
                               {-1, 1, 0},{ 0, 1, 0},{ 1, 1, 0},
                               {-1, 1, 1},{ 0, 1, 1},{ 1, 1, 1}
                            };

float WEIGHTS[4] = {0.0, 1.0, float(sqrt(2)), float(sqrt(3))};

pair<PointXYZ, PointXYZ> bbox(vector<PointXYZ> points) {
    float minx = 1e9, maxx = -1e9;
    float miny = 1e9, maxy = -1e9;
    float minz = 1e9, maxz = -1e9;

    for (int i = 0; i < points.size(); i++) {
        minx = min(minx, points[i].x);
        miny = min(miny, points[i].y);
        minz = min(minz, points[i].z);
        maxx = max(maxx, points[i].x);
        maxy = max(maxy, points[i].y);
        maxz = max(maxz, points[i].z);
    }
    PointXYZ bbox0 = PointXYZ(minx, miny, minz);
    PointXYZ bbox1 = PointXYZ(maxx, maxy, maxz);
    return make_pair(bbox0, bbox1);
}

vector<float> heap_shortest_path(int src, vector<vector<pair<float,int>>> graph)
{
    // Create a priority queue to store vertices that
    // are being preprocessed. This is weird syntax in C++.
    // Refer below link for details of this syntax
    // https://www.geeksforgeeks.org/implement-min-heap-using-stl/
    priority_queue< pair<float,int>, vector <pair<float,int>> , greater<pair<float,int>> > pq;
    int V = graph.size();
 
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


vector<vector<float>> voxel_geo_distance(vector<PointXYZ> points,
                                         vector<PointXYZ> nodes,
                                         float voxel_size) {

    // ==================
    //  Step 1 : Voxelize
    // ==================
    // [x] TODO: number of voxels in x, y, z directions
    auto coords = bbox(points);
    PointXYZ min_coord = coords.first;
    PointXYZ max_coord = coords.second;

    int n_i = floor( (max_coord.x - min_coord.x) / voxel_size ) + 1; 
    int n_j = floor( (max_coord.y - min_coord.y) / voxel_size ) + 1;  
    int n_k = floor( (max_coord.z - min_coord.z) / voxel_size ) + 1; 

    // [x] TODO: this translates points to ONE voxel in grid
    vector<int> idx_map; 
    // vector<vector<int,3>> grid_idc;


    unordered_map<int, int> acc_to_vertex;
    
    int vertex_idx = 0;
    for (int i=0; i<points.size(); i++) {
        float x = points[i].x;
        float y = points[i].y;
        float z = points[i].z;

        int grid_i = floor( (x - min_coord.x) / voxel_size );
        int grid_j = floor( (y - min_coord.y) / voxel_size );
        int grid_k = floor( (z - min_coord.z) / voxel_size );

        int acc_idx = grid_i + grid_j*n_i + grid_k*n_i*n_j;
        if ( acc_to_vertex.find(acc_idx) == acc_to_vertex.end()) { // this means non-existence 
            acc_to_vertex[acc_idx] = vertex_idx;
            vertex_idx++; 
        }

        idx_map.push_back(acc_to_vertex[acc_idx]);
        
    }
    // cout << idx_map.size() << endl;
    // cout << "Step 1 sir!" << endl;
    // ========================================
    // Step 2: Construct graph from kept voxels
    // ========================================
    vector<vector<pair<float,int>>> graph;
    graph.resize( acc_to_vertex.size(), vector<pair<float, int> >() );

    for (auto& it : acc_to_vertex) {
        // !! same names
        int acc_idx = it.first;
        int vertex_idx = it.second;

        // [ ] TODO: this is a little dirty 
        int a_i = acc_idx%n_i;
        int a_j = (acc_idx/n_i)%n_j;
        int a_k = acc_idx/(n_i*n_j);
        // End TODO: 

        // try to visit neighbors
        for (int d=0; d<26; d++) { 
            int di = DELTAS[d][0];
            int dj = DELTAS[d][1];
            int dk = DELTAS[d][2];

            // [ ] TODO: this is a little dirty (stupid)
            if (a_i + di < 0    || a_j + dj < 0    || a_k + dk < 0  || 
                a_i + di >= n_i || a_j + dj >= n_j || a_k + dk >= n_k) continue;
            // End TODO: 

            int offset_acc_idx = acc_idx + di + dj*n_i + dk*n_i*n_j; 

            if ( acc_to_vertex.find(offset_acc_idx) != acc_to_vertex.end() ) {
                cout << di << dj << dk << endl;
                int w = abs(di) + abs(dj) + abs(dk);
                int offset_vertex_idx = acc_to_vertex[offset_acc_idx];
                graph[vertex_idx].push_back( make_pair(WEIGHTS[w], offset_vertex_idx) );

                // // test:
                // int b_i = offset_acc_idx%n_i;
                // int b_j = (offset_acc_idx/n_i)%n_j;
                // int b_k = offset_acc_idx/(n_i*n_j);
                // cout << acc_idx << ' '<< a_i << a_j << a_k << " connects to " << offset_acc_idx << ' ' << b_i << b_j << b_k << " with " << WEIGHTS[w] << endl;
                // // end test
            }
        }
    }


    vector<vector<float>> vertex_geo_dist_mat; 
    for (int i=0; i<graph.size(); i++) {
        vector<float> dijk_info = heap_shortest_path(i, graph);

    // cout << "Step 2 sir!" << endl;
    // ========================================
    // Step 3: eccentricity & centricity
	// ========================================
	
	float alpha=0.4;
	float beta=0.2;
	float gamma=0.4;
	int maxElementIndex = std::max_element(dijk_info.begin(),dijk_info.end()) - dijk_info.begin();
	float maxElement = *std::max_element(dijk_info.begin(), dijk_info.end());
	float eccentricity = maxElement;
	
	float centricity = std::accumulate(dijk_info.begin(), dijk_info.end(), 0.0) / dijk_info.size();
	
	for(int j=0; j<dijk_info.size(); j++){
	dijk_info[j]= alpha*dijk_info[j] + beta*eccentricity+ gamma*centricity;
	}
		
	vertex_geo_dist_mat.push_back(dijk_info);
	}
	
	
    // ========================================
    // Step 4: Return
    // ========================================
    vector<vector<float>> points_geo_dist_mat; 
    points_geo_dist_mat.resize( points.size(), vector<float>() );
    for (int i=0; i<points.size(); i++) 
        for (int j=0; j<points.size(); j++) 
        {
            int v_i = idx_map[i];
            int v_j = idx_map[j];
            points_geo_dist_mat[i].push_back( vertex_geo_dist_mat[v_i][v_j] * voxel_size );
        }

    return points_geo_dist_mat;

}
