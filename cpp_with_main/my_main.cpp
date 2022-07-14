#include <iostream>
#include "cloud.h"
#include <random>
#include "voxel_geodesic_distance.h"


/*
test case of a c
*/
vector<PointXYZ> generate_point_cloud_C() {
    float xyz_arr[20][3] = {{0,0,0},{1,0,0},{2,0,0},{3,0,0},
                            {3,1,0},{3,2,0},{3,3,0},{0,1,0},
                            {0,2,0},{0,3,0},{0,0,1},{1,0,1},
                            {2,0,1},{3,0,1},{3,1,1},{3,2,1},
                            {3,3,1},{0,1,1},{0,2,1},{0,3,1}};
    vector<PointXYZ> test_cloud;
    for (int i=0; i<20; i++) {
        test_cloud.push_back(PointXYZ(xyz_arr[i][0],
                                      xyz_arr[i][1],
                                      xyz_arr[i][2]));
    }                 

    return test_cloud;
}


vector<PointXYZ> sample_nodes(vector<PointXYZ> points_pool, double rate) {
    // shuffle a index list
    vector<int> idx(points_pool.size());
    iota(begin(idx), end(idx), 0); //0 is the starting number

    random_device rd;
    mt19937 g(rd());
    shuffle(idx.begin(), idx.end(), g);

    // sample
    int sample_size = int(rate*idx.size());
    vector<PointXYZ> sample;
    for (int i=0; i<sample_size; i++) sample.push_back(points_pool[idx[i]]);
    return sample;
}


int main() {
    vector<PointXYZ> points = generate_point_cloud_C();
    vector<PointXYZ> nodes = sample_nodes(points, double(0.4));

    int N = points.size();
    int M = nodes.size();

    float voxel_size = 1.0;

    auto dist_mat = voxel_geo_distance(points, nodes, voxel_size);

    for (int i=0; i<dist_mat.size(); i++)
        for (int j=0; j<dist_mat.size(); j++)
            cout << points[i] << " is " << dist_mat[i][j] << " to " << points[j] << endl;

    return 0;
}