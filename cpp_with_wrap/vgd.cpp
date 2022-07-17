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


py::array_t<double> _geo_dist(py::array_t<double> py_point_cloud, double voxel_size, int n_voxels) {
    auto in_buf = py_point_cloud.request();
    int n = in_buf.shape[0];
    // int d = in_buf.shape[1];

    double *in_ptr  = static_cast<double *>(in_buf.ptr);
    
    double* dist_mat = new double[n*n];

    voxel_geo_distance(in_ptr, in_ptr, n, n, dist_mat, voxel_size, n_voxels);

    auto result = py::array(
        py::buffer_info(
            dist_mat,                              /* Pointer to data (nullptr -> ask NumPy to allocate!) */
            sizeof(double),                        /* Size of one item */
            py::format_descriptor<double>::value,  /* Buffer format */
            in_buf.ndim,                           /* How many dimensions? */
            { n, n },                              /* Number of elements for each dimension */
            { sizeof(double), sizeof(double) }     /* Strides for each dimension */
        )
    );
    return result;

}


py::array_t<double> np_voxel_size_geo_dist(py::array_t<double> py_point_cloud, double voxel_size) {
    return _geo_dist(py_point_cloud, voxel_size, NULL_N);
}


py::array_t<double> np_num_voxel_geo_dist(py::array_t<double> py_point_cloud, int n_voxels) {
    return _geo_dist(py_point_cloud, NULL_SIZE, n_voxels);
}


PYBIND11_MODULE(vgd, m) {
    m.def("np_voxel_size_geo_dist", &np_voxel_size_geo_dist, "Calculate geodesic distances with voxel_size");
    m.def("np_num_voxel_geo_dist", &np_num_voxel_geo_dist, "Calculate geodesic distances with number of voxels");
}
