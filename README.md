# SLAM_Voronoi

Take occupancy grid maps created with SLAM algorithms and process them to compute the Voronoi graph in the free space.

After that you can use different planning algorithms to compute the sortest path between 2 points. Currently A* and ACO (Ant Colony Optimization) algorithms are implemented for the path planning.

### How to build

```bash
mkdir build && cd build
cmake ..
make
```

### Notes
You can find some maps inside the data directory. The parameters can be adjusted manually with the trackbars to adapt them to each map.

After building the project, the binaries are put in the bin directory.
