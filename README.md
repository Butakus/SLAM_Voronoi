# SLAM_Voronoi

Take occupancy grid maps created with SLAM algorithms and process them to compute the Voronoi graph in the free space.

Path planning algorithms can be used later on that graph to compute the shortest path between 2 points.

### How to install

```bash
mkdir build && cd build
cmake ..
make
```

### Notes
You can find some maps inside the data directory, but the parameters are currently adjusted to woek with map_0.png, and it shuold not work with other images.
