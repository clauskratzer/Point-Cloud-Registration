#include <vector>
#include <cstring>
#include <map>
#include <utility>
#include <cmath>
#include <queue>
#include <iostream>
#include "cloud.h"
using namespace std;


pair<PointXYZ, PointXYZ> bbox(vector<PointXYZ> point_cloud); 

vector<float> heap_shortest_path(int src, vector<vector<pair<float,int>>> graph);

vector<vector<float>> voxel_geo_distance(vector<PointXYZ> points,
                                         vector<PointXYZ> nodes,
                                         float  voxel_size);




