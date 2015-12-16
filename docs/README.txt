AV-NUI
------

Dependencies:
-------------
- Boost 1.55:
sudo apt-get install libboost1.55-all-dev

- Eigen 3:	
sudo apt-get install libeigen3-dev

Set environment variable:
export EIGEN3_INCLUDE_PATH=/usr/include/eigen3/

- Google Glog:
sudo apt-get install libgoogle-glog-dev

Set environment variable:
export GLOG_ROOT=/usr/

- Google Protobuf:
sudo apt-get install libprotobuf-dev
sudo apt-get install protobuf-compiler 

- HDF 5:
sudo apt-get install libhdf5-dev 

- OpenCV:
sudo apt-get install libopencv-dev

- OpenGL:
sudo apt-get install mesa-common-dev
sudo apt-get install freeglut3-dev

- QT 5: 
URL: https://qt-project.org/search/tag/online~installer

Set environment variable:
export Qt5Widgets_DIR=[PATH_TO_QT]/Qt/5.2.1/gcc_64/

- Leap SDK (2.0.4) 
URL: https://developer.leapmotion.com/downloads/skeletal-beta?platform=linux&version=2.0.4.17546


Set environment variable:
LeapSDK_DIR=PATH_TO_LEAP_SDK

Caffe
-----

sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libboost-all-dev libhdf5-serial-dev

CMake command:
--------------
cmake -G "GENERATER" -DCMAKE_BUILD_TYPE=Debug -DWITH_EYE-TRACKING=ON -DWITH_GESTURE_RECOGNITION=ON PATH_TO_AV_NUI

cmake -G "KDevelop3 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DWITH_EYE-TRACKING=ON -DWITH_GESTURE_RECOGNITION=ON -DQt5Widgets_DIR=~/Qt/5.2.1/gcc_64/lib/cmake/Qt5Widgets -DGLOG_ROOT=/usr/ -DEIGEN3_INCLUDE_PATH=/usr/include/eigen3/ -DLeapSDK_DIR=/home/m_weber/develop/av-nui/3rdparty/x64/LeapSDK-2.1.0/ ~/develop/av-nui

cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DWITH_EYE-TRACKING=ON -DWITH_GESTURE_RECOGNITION=ON -DQt5Widgets_DIR=~/Qt/5.3/gcc_64/lib/cmake/Qt5Widgets -DGLOG_ROOT=/usr/ -DEIGEN3_INCLUDE_PATH=/usr/include/eigen3/ -DLeapSDK_DIR=/home/m_weber/develop/av-nui/3rdparty/x64/LeapSDK-2.2.5/ -DPMDSDK_DIR=/home/m_weber/develop/av-nui/3rdparty/x64/PMDSDK ~/develop/av-nui


Supported device:
-----------------
  - Leap Motion
  - Webcam
