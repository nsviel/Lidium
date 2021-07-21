# Lidium
**Open source 3D point cloud processing.**

LiDAR (Light Detection and Ranging) sensors give huge amount of data, which generally need a variety of post-processing actions. These data are 3D point cloud containing the spatial coordinates and additionnal information like color or intensity of a numerous set of measures. Generally, the main use of these point clouds is the construction of a numerical 3D model of a real scene. The registration of point clouds took from different points of view, for example, is a painful task to achieve and requires usually some specific steps : cloud sampling and filtering, normal computation, just to name a few. But point clouds could also be particularly usefull with the support of the color or intensity information for object detection, material and structure recognition. A radiometric correction of the intensity is necessary to take advantage of this by-product information. Thus, we propose in this software an open source solution for visualization, easy manipulation and processing of point clouds, and some advanced algorithms for 1) radiometric correcttion of the the intensity information, 2) pairwise registration of multiple point cloud using different strategies.

## file format

Import: .PTS, .PTX, .PLY, .OBJ
Export: .PTS
 
## Libraries :
All of the code is written in C++, for performance purpose. The incorporated external libraries come from the open source world:
- OpenGL (https://www.opengl.org): Data visualization
- ImGui (https://github.com/ocornut/imgui): Graphical user interface
- PCL (https://pointclouds.org/): a set of processing algorithms
- Eigen (https://eigen.tuxfamily.org/): Some linear algebra
- FLANN (https://github.com/flann-lib/flann) & Nanoflann (https://github.com/jlblancoc/nanoflann) : to improve the nearest neighbor search speed
- Gnuplot (http://www.gnuplot.info/): Graphical data visualization
- OptimLib (https://www.kthohr.com/optimlib.html): Numerical optimization methods
- normals_Hough (https://github.com/aboulch/normals_Hough): Method for normal estimation
- OpenMP (https://www.openmp.org/): Multithreading
- FontAwesome (https://fontawesome.com/): Icons


## Installation : 

Simply run the script file 
```
cd /Lidium
./install.sh
```

OR build it manually

Install dependencies: 
```
sudo apt install build-essential freeglut3-dev libglfw3-dev libglew-dev libeigen3-dev libflann-dev libboost-all-dev libpng-dev libglm-dev libvtk7-dev gnuplot cmake
```
Install PCL:
```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build && cmake .. && make -j4 && make install 
```
Compile the executable:
```
 mkdir build && cd build && cmake .. && make -j4 && ./executable
```

