#include "pure_geod.h"
#include<pybind11/pybind11.h>
#include<pybind11/numpy.h>

namespace py = pybind11;
using namespace pybind11::literals;


py::array_t<double> np_graph_geo_dist(py::array_t<double> py_point_cloud, py::array_t<int> py_node_inds, float radius) {
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
    pure_geod(ptr_points, n, ptr_node_inds, m, dist_mat, radius);

    result.resize({n, m});

    return result;
}



PYBIND11_MODULE(ggd, m) {
    m.def("np_graph_geo_dist", &np_graph_geo_dist, "Calculate geodesic distances with graph");
}