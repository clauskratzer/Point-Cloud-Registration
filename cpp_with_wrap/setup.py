import os, sys
from distutils.core import setup, Extension
from pybind11 import get_include

cpp_args = ['-std=c++11', '-stdlib=libc++']

ext_modules = [
    Extension(
    'wrap',
        ['voxel_geodesic_distance.cpp', 'wrap.cpp'],
        include_dirs=[get_include()],
    language='c++',
    extra_compile_args = cpp_args,
    ),
]

setup(
    name='wrap',
    version='0.0.1',
    description='voxels based geodesic distance calculation',
    ext_modules=ext_modules,
)