from distutils.core import setup, Extension
from pybind11 import get_include

cpp_args = ['-std=c++11', '-stdlib=libc++']

ext_modules = [
    Extension(
    'ggd',
        ['pure_geod.cpp', 'ggd.cpp'],
        include_dirs=[get_include()],
    language='c++',
    extra_compile_args = cpp_args,
    ),
]

setup(
    name='ggd',
    version='0.0.1',
    description='graph based geodesic distance calculation',
    ext_modules=ext_modules,
)