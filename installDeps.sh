#!/bin/bash
mkdir /tmp/eaglevision
cd /tmp/eaglevision
wget -O wpiutil.zip http://first.wpi.edu/FRC/roborio/maven/release/edu/wpi/first/wpilib/wpiutil/1.0.2/wpiutil-1.0.2-desktop.zip
wget -O cscore.zip http://first.wpi.edu/FRC/roborio/maven/release/edu/wpi/cscore/cpp/cscore/1.0.2/cscore-1.0.2-linux.zip
wget -O ntcore.zip http://first.wpi.edu/FRC/roborio/maven/release/edu/wpi/first/wpilib/networktables/cpp/NetworkTables/3.1.7/NetworkTables-3.1.7-desktop.zip

unzip wpiutil.zip
unzip cscore.zip
unzip ntcore.zip

sudo cp -r /tmp/eaglevision/include/* /usr/local/include/
sudo cp -r /tmp/eaglevision/Linux/amd64/* /usr/local/lib
sudo ldconfig

wget -O opencv3.2.0.zip https://github.com/opencv/opencv/archive/3.2.0.zip
unzip opencv3.2.0.zip

cd /tmp/eaglevision/opencv-3.2.0/
cmake -D WITH_TBB=OFF -D WITH_OPENMP=OFF -D WITH_IPP=OFF -D ENABLE_PROFILING=ON -D CMAKE_BUILD_TYPE=Debug -D BUILD_EXAMPLES=OFF -D WITH_NVCUVID=OFF -D WITH_CUDA=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D WITH_CSTRIPES=OFF -D WITH_OPENCL=OFF CMAKE_INSTALL_PREFIX=/usr/local/ .
make -j8
sudo make install

echo "Dependency installation complete!" 
