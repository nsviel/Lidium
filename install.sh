#-------------
#INSTALLATION
#-------------

#!/bin/sh
echo Installation...

#Install tools
sudo apt install build-essential git cmake htop

#Install dependancies
sudo apt install libglfw3-dev libglew-dev libeigen3-dev libflann-dev libboost-all-dev libglm-dev gnuplot libvtk7-dev libarmadillo-dev libqhull-dev

#Install pcl library
#----freezing could occurs due to RAM failure -> make SWAP partition----#
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build && cmake .. && make -j4 && sudo make install 

#End
echo Installation finished

