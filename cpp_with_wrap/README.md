### Usage
1. install pybind11 by
```
$ pip install pybind11
```
2. build 
```sh
$ python setup.py build_ext -i
```
3. run dummy test on letter C
```sh
python dummy_test.py
```
4. how to use in a script
```python
from wrap import np_voxel_geo_distance
dist_mat = np_voxel_geo_distance(points, voxel_size) # currently pairwise
```


### Reuse the wrapper for diffusion distance
* check out `voxel_geodesic_distance.cpp`
* the definition of voxel geo distance calculation is like:
    ```cpp
    void voxel_geo_distance(double* points, double* nodes, int N, int M, double voxel_size, double* dist_mat)
    ```
* so diffusion distance should probably looks like:
    ```cpp
    void diffusion_distance(double* points, double* nodes, int N, int M, double* dist_mat)
    ```
    * `points` and `nodes` are set of points as pointers (so we don't need to copy from numpy), and `N` and `M` are their sizes.
    * `dist_mat` is currently expecting size of `N*N` 


### TODO
more tests ... 
