import numpy as np
# import cppimport
# vgd = cppimport.imp('wrap')
from wrap import np_voxel_geo_distance

test_pc = np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.],[3.,0.,0.],[3.,1.,0.],[3.,2.,0.],[3.,3.,0.],[0.,1.,0.],[0.,2.,0.],[0.,3.,0.],[0.,0.,1.],[1.,0.,1.],[2.,0.,1.],[3.,0.,1.],[3.,1.,1.],[3.,2.,1.],[3.,3.,1.],[0.,1.,1.],[0.,2.,1.],[0.,3.,1.]])

test = np_voxel_geo_distance(test_pc, 1.0)
print(type(test))
print(test.shape)
print(test)
