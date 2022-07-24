#include "pure_geod.h"
#define INF 0x3f3f3f3f
//#define NULL_N -1

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


void pure_geod(double* points, int N, int* node_inds, int M, double* dist_mat, float radius) {
    // ===========================
    //  Step 1 : Pairwise distance 
    // ===========================
    double sq_radius = radius*radius;
    vector<vector<double>> sq_eucl_dist;
    sq_eucl_dist.resize(N, vector<double>(N, -1));
    for (int i=0; i<N; i++) {
        double x1 = points[i*3 + 0];
        double y1 = points[i*3 + 1];
        double z1 = points[i*3 + 2];
        for (int j=i; j<N; j++) {
            double x2 = points[j*3 + 0];
            double y2 = points[j*3 + 1];
            double z2 = points[j*3 + 2];
            double dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
            double sq_dist = dx*dx + dy*dy + dz*dz;
            sq_eucl_dist[i][j] = sq_dist;
            sq_eucl_dist[j][i] = sq_dist;
        }
    }

    // cout << "sir" << endl;
    // ========================================
    // Step 2: Construct graph with 2 norm
    // ========================================
    vector<vector<pair<double,int>>> graph;
    graph.resize( N, vector<pair<double, int> >() );

    for (int i=0; i<N; i++)
        for (int j=i; j<N; j++) 
            if (sq_eucl_dist[i][j] <= sq_radius) {
                graph[i].emplace_back( make_pair(sqrt(sq_eucl_dist[i][j]), j));
            }

    // cout << "sir" << endl;
    // ========================================
    // Step 3: Handle disconnected component
    // ========================================
    // BFS to label group of points
    stack<int> pending_vertices;
    vector<int> labels(N, -1);
    int cur_label = 0;
    
    for (int i=0; i<N; i++) {
        if ( labels[i] < 0 ) {
            pending_vertices.push(i);
            labels[i] = cur_label;
            vector<bool> visited(N, false);
            while ( !pending_vertices.empty() ) {
                int cur_vtx = pending_vertices.top();
                pending_vertices.pop();
                if (visited[cur_vtx] == false) {
                    for (auto edge: graph[cur_vtx]) {
                        int leaf_vtx = edge.second;
                        pending_vertices.push(leaf_vtx);
                        labels[leaf_vtx] = cur_label;
                    }
                    visited[cur_vtx] = true;
                }
            }
            cur_label++;
        }
    }

    // cout << "sir" << endl;
    // cout << cur_label << endl;
    // handle disconnected components
    for (int a=0; a<cur_label; a++) {
        double min_ab_dist = INF;
        int min_ai = -1;
        int min_bi = -1;
        for (int b=a+1; b<cur_label; b++) {
            for (int ai=0; ai<N; ai++) {
                if (labels[ai] == a) {
                    for (int bi=0; bi<N; bi++) {
                        if (labels[bi] == b) {
                            if (sq_eucl_dist[ai][bi] < min_ab_dist) {
                                min_ab_dist = sq_eucl_dist[ai][bi];
                                min_ai = ai;
                                min_bi = bi;
                            }
                        }
                    }
                }
            }
        }
        graph[min_ai].emplace_back( make_pair( sqrt(min_ab_dist), min_bi) );
        graph[min_bi].emplace_back( make_pair( sqrt(min_ab_dist), min_ai) );
    }

    // cout << "sir" << endl;
    // ===============================================
    // Step 4: dijkstra and eccentricity & centricity
    // ===============================================
    // change 
    int cur_node = 0;
    vector<bool> voi(N, false);
    for ( int i=0; i<N; i++ ) {
        if ( i == node_inds[cur_node] ) {
            voi[i] = true;
            cur_node++;
        }
    }

    vector<vector<double>> temp_dist_mat; // size should be nr_vert_nodes * nr_vert_points
    unordered_map<int, int> real_row;
    for ( int i=0; i<N; i++ ) {
        if ( voi[i] == false ) continue;

        real_row[i] = temp_dist_mat.size();
        vector<double> dijk_info = heap_shortest_path(i, graph);
        
        double alpha=0.4,  beta=0.2, gamma=0.4;
        double eccentricity = *std::max_element(dijk_info.begin(), dijk_info.end());
        double centricity = std::accumulate(dijk_info.begin(), dijk_info.end(), 0.0) / dijk_info.size();
        
        for(size_t j=0; j<dijk_info.size(); j++) 
           dijk_info[j]= alpha*dijk_info[j] + beta*eccentricity + gamma*centricity;

        temp_dist_mat.push_back(dijk_info);
    }

    // ========================================
    // Step 5: *Return
    // ========================================
    // return points_geo_dist_mat;
    for (int i=0; i<N; i++) 
        for (int j=0; j<M; j++) 
            dist_mat[i*M + j] = temp_dist_mat[real_row[i]][j] * radius;
}