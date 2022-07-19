#include "voxel_geodesic_distance.h"
#define INF 0x3f3f3f3f
#define NULL_SIZE 0.0
//#define NULL_N -1

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


double voxel_info(double* points, int N, 
                  double* max_coord, double* min_coord,
                  double voxel_size, int n_voxels) {
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

    max_coord[0] = maxx;
    max_coord[1] = maxy;
    max_coord[2] = maxz;
    min_coord[0] = minx;
    min_coord[1] = miny;
    min_coord[2] = minz;

    if (NULL_SIZE==voxel_size)
        return max({maxx-minx, maxy-miny, maxz-minz})/(n_voxels-1); 
    else
        return voxel_size;
}


vector<double> heap_shortest_path(int src, vector<vector<pair<double,int>>> graph) {
    priority_queue< pair<double,int>, vector <pair<double,int>> , greater<pair<double,int>> > pq;

    int V = graph.size();
    vector<bool> visited(V, false);
    vector<double> dist(V, INF);
    pq.push(make_pair(0.0, src));
    dist[src] = 0.0;

    while (!pq.empty()) {

        int u = pq.top().second;
        visited[u] = true;
        pq.pop();

        int s = graph[u].size();
        for (int i = 0; i < s ; ++i) {
            int v = graph[u][i].second;
            if (visited[v]) continue;

            double weight = graph[u][i].first;
            if ( dist[v] > dist[u] + weight ) {
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
            }
        }
        
    }

    return dist;
}



void voxel_geo_distance(double* points, int N,
                        int* node_inds, int M, 
                        double* dist_mat, 
                        double size_of_voxel, int n_voxels) {
    // ==================
    //  Step 1 : Voxelize
    // ==================
    double max_coord[3], min_coord[3];
    auto voxel_size = voxel_info(points, N, max_coord, min_coord, size_of_voxel, n_voxels);

    // number of voxels in x, y, z directions, [ ] todo: we can also fix them to a constant (i.e., a cube)
    int n_i = floor( (max_coord[0] - min_coord[0]) / voxel_size ) + 1; 
    int n_j = floor( (max_coord[1] - min_coord[1]) / voxel_size ) + 1;  
    int n_k = floor( (max_coord[2] - min_coord[2]) / voxel_size ) + 1; 

    // cout << n_i << ' ' << n_j << ' ' << n_k << endl;

    // this translates points to ONE voxel in grid
    vector<int> idx_map; 

    // cannot be replaced by list because the construction is by ordered
    unordered_map<int, int> acc_to_vertex;

    vector<bool> voi; // whether a vertex is in node

    int vertex_idx = 0; // accumulate index <= N
    int cur_node = 0;
    for (int i=0; i<N; i++) {
        double x = points[i*3 + 0];
        double y = points[i*3 + 1];
        double z = points[i*3 + 2];

        int grid_i = floor( (x - min_coord[0]) / voxel_size );
        int grid_j = floor( (y - min_coord[1]) / voxel_size );
        int grid_k = floor( (z - min_coord[2]) / voxel_size );

        int acc_idx = grid_i + grid_j*n_i + grid_k*n_i*n_j;

        // cout << grid_i << ' ' << grid_j << ' ' << grid_k << ' ' << endl;
        // cout << "acc_idx" << acc_idx << endl;

        // increment vertex index if vertex does not exist
        if ( acc_to_vertex.count(acc_idx) == 0 ) { 
            acc_to_vertex[acc_idx] = vertex_idx;
            voi.push_back(false);
            vertex_idx++; 
        }


        // increment node index if a node shows up
        if ( i == node_inds[cur_node] ) {
            voi[ acc_to_vertex[acc_idx] ] = true;
            cur_node++;
        }

        idx_map.push_back(acc_to_vertex[acc_idx]);
        
    }

    // cout << vertex_idx << ' ' << voi.size() << endl;

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

            if ( acc_to_vertex.count(offset_acc_idx) > 0 ) {
                int w = abs(di) + abs(dj) + abs(dk);
                int offset_vertex_idx = acc_to_vertex[offset_acc_idx];
                
                graph[vertex_idx].push_back( make_pair(WEIGHTS[w], offset_vertex_idx) );

            }
        }
    }

    // ===============================================
    // Step 3: dijkstra and eccentricity & centricity
    // ===============================================

    vector<vector<double>> vertex_geo_dist_mat; // size should be nr_vert_nodes * nr_vert_points
    unordered_map<int, int> real_row;
    for (size_t i=0; i<graph.size(); i++) {

        if ( voi[i] == false ) 
            continue;

        real_row[i] = vertex_geo_dist_mat.size();
        vector<double> dijk_info = heap_shortest_path(i, graph);
        
        // Replace INF [ ] TODO: should we? 
        for (size_t k=0; k<dijk_info.size(); k++) 
            if (dijk_info[k] >= 600) 
                dijk_info[k] = 0;

        double alpha=0.4,  beta=0.2, gamma=0.4;
        double eccentricity = *std::max_element(dijk_info.begin(), dijk_info.end());
        double centricity = std::accumulate(dijk_info.begin(), dijk_info.end(), 0.0) / dijk_info.size();
        
        for(size_t j=0; j<dijk_info.size(); j++) 
           dijk_info[j]= alpha*dijk_info[j] + beta*eccentricity + gamma*centricity;
        
        
        // Replace -1 back to some value
        for (size_t k=0; k<dijk_info.size(); k++) 
            if (dijk_info[k] < 0) 
                dijk_info[k] = 600;


        vertex_geo_dist_mat.push_back(dijk_info);
    }


    // ========================================
    // Step 4: *Return
    // ========================================
    // return points_geo_dist_mat;
    for (int i=0; i<N; i++) 
        for (int j=0; j<M; j++) 
        {
            int v_i = idx_map[i];
            int v_j = real_row[ idx_map[ node_inds[j] ] ]; // I know this is ugly :(
            dist_mat[i*M + j] = vertex_geo_dist_mat[v_j][v_i] * voxel_size;
        }
}

