#include <vector>
#include <cstring>
#include <map>
#include <utility>
#include <cmath>
#include <queue>
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <cmath>
#include <time.h>

using namespace std;


double voxel_info(double* points, int N, 
                  double* max_coord, double* min_coord,
                  double voxel_size, int n_voxels);

vector<double> heap_shortest_path(int src, vector<vector<pair<double,int>>> graph);

void voxel_geo_distance(double* points, int N,
                        int* node_inds, int M, 
                        double* dist_mat, 
                        double size_of_voxel, int n_voxels);




