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
#include <stack>

using namespace std;

vector<double> heap_shortest_path(int src, vector<vector<pair<double,int>>> graph);
void pure_geod(double* points, int N, int* node_inds, int M, double* dist_mat, float radius);
