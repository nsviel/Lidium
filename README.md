# Lidium
**Open source 3D point cloud processing.**

## file format

Import: .PTS, .PTX, .PLY, .OBJ
Export: .PTS
 
## External libraries :
- Eigen: Some maths
- Gnuplot: Data visualization
- ImGui: Graphical user interface
- FLANN: Nearest neighbor search
- PCL: point cloud processing
- OpenMP: threading
- FontAwesome: Icons

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

