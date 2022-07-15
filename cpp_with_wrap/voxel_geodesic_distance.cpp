#include "voxel_geodesic_distance.h"
#define INF 0x3f3f3f3f

int DELTAS[26][3] = {    {-1,-1,-1},{ 0,-1,-1},{ 1,-1,-1},
                         {-1,-1, 0},{ 0,-1, 0},{ 1,-1, 0},
                         {-1,-1, 1},{ 0,-1, 1},{ 1,-1, 1},
                         {-1, 0,-1},{ 0, 0,-1},{ 1, 0,-1},
                         {-1, 0, 0},/*myself*/ { 1, 0, 0},
                         {-1, 0, 1},{ 0, 0, 1},{ 1, 0, 1},
                         {-1, 1,-1},{ 0, 1,-1},{ 1, 1,-1},
                         {-1, 1, 0},{ 0, 1, 0},{ 1, 1, 0},
                         {-1, 1, 1},{ 0, 1, 1},{ 1, 1, 1}
                    };

double WEIGHTS[4] = {0.0, 1.0, double(sqrt(2)), double(sqrt(3))};


pair<vector<double>, vector<double>> bbox(double* points, int N) {
    double minx = 1e9, maxx = -1e9;
    double miny = 1e9, maxy = -1e9;
    double minz = 1e9, maxz = -1e9;

    for (int i = 0; i < N; i++) {
        minx = min(minx, points[i*3 + 0]);
        miny = min(miny, points[i*3 + 1]);
        minz = min(minz, points[i*3 + 2]);
        maxx = max(maxx, points[i*3 + 0]);
        maxy = max(maxy, points[i*3 + 1]);
        maxz = max(maxz, points[i*3 + 2]);
    }
    vector<double> bbox0 { minx, miny, minz };
    vector<double> bbox1 { maxx, maxy, maxz };
    return make_pair(bbox0, bbox1);
}


vector<double> heap_shortest_path(int src, vector<vector<pair<double,int>>> graph) {
    priority_queue< pair<double,int>, vector <pair<double,int>> , greater<pair<double,int>> > pq;
    int V = graph.size();

    vector<double> dist(V, INF);
    pq.push(make_pair(0.0, src));
    dist[src] = 0.0;

    while (!pq.empty()) {

        int u = pq.top().second;
        pq.pop();

        int s = graph[u].size();
        for (int i = 0; i < s ; ++i) {
            int v = graph[u][i].second;
            double weight = graph[u][i].first;
 
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
            }
        }
        
    }

    return dist;
}



void voxel_geo_distance(double* points, double* nodes, int N, int M, double voxel_size, double* dist_mat) {
    // ==================
    //  Step 1 : Voxelize
    // ==================
    auto coords = bbox(points, N);
    auto min_coord = coords.first;
    auto max_coord = coords.second;

    // number of voxels in x, y, z directions
    int n_i = floor( (max_coord[0] - min_coord[0]) / voxel_size ) + 1; 
    int n_j = floor( (max_coord[1] - min_coord[1]) / voxel_size ) + 1;  
    int n_k = floor( (max_coord[2] - min_coord[2]) / voxel_size ) + 1; 

    // this translates points to ONE voxel in grid
    vector<int> idx_map; 
    unordered_map<int, int> acc_to_vertex;

    int vertex_idx = 0; // accumulate index <= N
    for (int i=0; i<N; i++) {
        double x = points[i*3 + 0];
        double y = points[i*3 + 1];
        double z = points[i*3 + 2];

        int grid_i = floor( (x - min_coord[0]) / voxel_size );
        int grid_j = floor( (y - min_coord[1]) / voxel_size );
        int grid_k = floor( (z - min_coord[2]) / voxel_size );

        int acc_idx = grid_i + grid_j*n_i + grid_k*n_i*n_j;
        if ( acc_to_vertex.find(acc_idx) == acc_to_vertex.end()) { // this means non-existence 
            acc_to_vertex[acc_idx] = vertex_idx;
            vertex_idx++; 
        }

        idx_map.push_back(acc_to_vertex[acc_idx]);
        
    }

    // ========================================
    // Step 2: Construct graph from kept voxels
    // ========================================
    vector<vector<pair<double,int>>> graph;
    graph.resize( acc_to_vertex.size(), vector<pair<double, int> >() );

    for (pair<int, int> it : acc_to_vertex) {
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
                int w = abs(di) + abs(dj) + abs(dk);
                int offset_vertex_idx = acc_to_vertex[offset_acc_idx];
                
                graph[vertex_idx].push_back( make_pair(WEIGHTS[w], offset_vertex_idx) );

            }
        }
    }


    // ===============================================
    // Step 3: dijkstra and eccentricity & centricity
    // ===============================================
    vector<vector<double>> vertex_geo_dist_mat; 
    for (size_t i=0; i<graph.size(); i++) {
        vector<double> dijk_info = heap_shortest_path(i, graph);

        double alpha=0.4;
        double beta=0.2;
        double gamma=0.4;
        // int maxElementIndex = std::max_element(dijk_info.begin(),dijk_info.end()) - dijk_info.begin();
        double maxElement = *std::max_element(dijk_info.begin(), dijk_info.end());
        double eccentricity = maxElement;
        
        double centricity = std::accumulate(dijk_info.begin(), dijk_info.end(), 0.0) / dijk_info.size();
        
        for(size_t j=0; j<dijk_info.size(); j++) {
           dijk_info[j]= alpha*dijk_info[j] + beta*eccentricity+ gamma*centricity;
        }
            
        vertex_geo_dist_mat.push_back(dijk_info);
    }
    
    // ========================================
    // Step 4: *Return
    // ========================================
    // return points_geo_dist_mat;
    for (int i=0; i<N; i++) 
        for (int j=0; j<N; j++) 
        {
            int v_i = idx_map[i];
            int v_j = idx_map[j];
            dist_mat[i*N + j] = vertex_geo_dist_mat[v_i][v_j] * voxel_size;
        }
    

}

