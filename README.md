# LidarSoft
**Open source 3D OpenGL based point cloud vizualizer and processing.**

## file format

Read .PTS, .PTX, .PLY, .OBJ.

Export in .PTS.
 
## External libraries :
- Eigen : some maths
- Gnuplot : data visualization
- ImGui : GUI
- FLANN : neirest neighbor search
- PCL : point cloud processing
- OpenMP : threading
- Icons : FontAwesome

## Installation : 

Simply run the script file 
```
cd /Dunesand
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

