g++ -std=c++11 -c voxel_geodesic_distance.cpp
g++ -std=c++11 -c my_main.cpp
g++ voxel_geodesic_distance.o my_main.o -o main
