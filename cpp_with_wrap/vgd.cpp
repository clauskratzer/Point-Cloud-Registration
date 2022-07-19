// <%
// cfg['sources'] = ['voxel_geodesic_distance.cpp']
// cfg['compiler_args'] = ['-std=c++11', '-stdlib=libc++', '-mmacosx-version-min=10.7']
// cfg['include_dirs'] = ['/Users/hoijanlai/.pyenv/versions/pylab/lib/python3.7/site-packages/pybind11/include']
// setup_pybind11(cfg)
// %>
#include "voxel_geodesic_distance.h"
#include<pybind11/pybind11.h>
#include<pybind11/numpy.h>
#define NULL_SIZE 0.0
#define NULL_N -1

namespace py = pybind11;
using namespace pybind11::literals;


py::array_t<double> _geo_dist(py::array_t<double> py_point_cloud, py::array_t<int> py_node_inds, double voxel_size, int n_voxels) {
    // buffers
    auto buf_points = py_point_cloud.request();
    int n = buf_points.shape[0];

    auto buf_node_inds = py_node_inds.request();
    int m = buf_node_inds.shape[0];

    // pointers 
    double *ptr_points  = static_cast<double *>(buf_points.ptr);
    int *ptr_node_inds = static_cast<int *>(buf_node_inds.ptr);

    // results 
    py::array_t<double> result = py::array_t<double>(n*m);
    auto buf_result = result.request();
    double *dist_mat = static_cast<double *>(buf_result.ptr);

    // modify result buffer pointer 
    voxel_geo_distance(ptr_points, n, ptr_node_inds, m, dist_mat, voxel_size, n_voxels);

    result.resize({n, m});

    return result;

}


py::array_t<double> np_voxel_size_geo_dist(py::array_t<double> py_point_cloud, py::array_t<int> py_node_inds, double voxel_size) {
    return _geo_dist(py_point_cloud, py_node_inds, voxel_size, NULL_N);
}


py::array_t<double> np_num_voxel_geo_dist(py::array_t<double> py_point_cloud, py::array_t<int> py_node_inds, int n_voxels) {
    return _geo_dist(py_point_cloud, py_node_inds, NULL_SIZE, n_voxels);
}


PYBIND11_MODULE(vgd, m) {
    m.def("np_voxel_size_geo_dist", &np_voxel_size_geo_dist, "Calculate geodesic distances with voxel_size");
    m.def("np_num_voxel_geo_dist", &np_num_voxel_geo_dist, "Calculate geodesic distances with number of voxels");
}
