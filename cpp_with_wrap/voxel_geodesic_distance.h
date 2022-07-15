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


pair<vector<double>, vector<double>> bbox(double* points, int N);

vector<double> heap_shortest_path(int src, vector<vector<pair<double,int>>> graph);

void voxel_geo_distance(double* points, double* nodes, int N, int M, double voxel_size, double* dist_mat);




