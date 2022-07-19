### Usage
1. install pybind11 by
    ```bash
    $ pip install pybind11
    ```
2. build 
    ```bash
    $ python setup.py build_ext -i
    ```
3. run test on 4d point cloud
    ```bash
    $ python test_with_steve.py
    ```
    _note: you will need to input some numbers__
    ```bash
    Percentage of the Point Cloud to calculate: 0.25 # 3/4 of the whole point cloud will be discarded (by taking a patch, not sampling)
    Voxel Size: 0.03
    Sample rate for KDE: 0.1 # kde plot 
    ```
    Then you will get some visualization (KDE will need some time to plot) 
    ![Figure](./Figure.png)
4. how to use in a python script
    ```python
    from vgd import np_num_voxel_geo_dist
    from vgd import np_voxel_size_geo_dist

    point_cloud = ... # the whole point cloud
    node_inds = ... # a subset of point cloud indices 

    dist_mat = np_num_voxel_geo_dist(point_cloud, node_inds, 80) # number of voxels for the longest edge of the bounding box 
    dist_mat = np_voxel_size_geo_dist(point_cloud, node_inds, 0.03) # voxel_size of 0.03
    ```

### Insights
* take a look at [`voxel_geodesic_distance.cpp`](./voxel_geodesic_distance.cpp)
* `points` is a set of points as pointers (so we don't need to copy from numpy), and `N` is its size
* `node_inds` is a subset of indices from `range(N)`, with size `M`
* `dist_mat` is a pointer of `double`, expecting size of `N, M` 

### !!!
* Poor choice of `voxel_size` (too small) will render a graph that is "too" disconnected.
    * Can perturb the point cloud so there're more bridging points to connect separate components (Check [test_with_steve.py](./test_with_steve.py))

### TODO
- [x] Deal with nodes not points
#### Bug Fixed
eccentricity and centricity needs to calculate max, so it is affected by the INF distances. Now they are set to be 0 if is bigger than the longest posible distance between two points in a cube (3 times number of voxels)
```cpp
...
for (size_t i=0; i<dijk_info.size(); i++) 
    if (dijk_info[i] >= 600) // assuming at most 200^3 voxels, and no crazy spiral surface
        dijk_info[i] = 0.0;
...
```


