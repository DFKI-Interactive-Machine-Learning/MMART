DESCRIPTION
===========
MMART is a command line tool to record raw data from several sensors that are connected to a computer. Prior to usage MMART has to be configured via an XML-file. To start a recording simply run `recorder.exe`, e.g. by using a command line shell of your choice. For setting up the system you can either use a pre-built package (TODO) or build it yourself from source.

Sensors
-------

* Intel RealSense
* Thalmic Labs Myo
* SoftKinetic DepthSense

...

Configuration
-------------
Several options can be set via the XML configuration file:

* Devices that shall be used
* Streams of that devices to record
* Target path

SETUP PRE-BUILT PACKAGE
=======================

* Install Visual C++ Redistributable Packages für Visual Studio 2013
* Install RealSense SDK Runtime and F200 DCM (v1.4)
* PCL 1.7.2 from http://unanancyowen.com/?p=1255&lang=en
* HDF5-1.8.**15**
* Install Myo Connect


BUILDING IN LINUX
=================

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
URL: http://download.qt.io/official_releases/online_installers/qt-unified-linux-x64-online.run

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

BUILDING IN WINDOWS
===================
Tools
-----

* cmake
* Visual Studio **2013** Community

Dependencies
------------
Always choose **x64** version of a dependency!

* HDF5-1.8.**15**
* PCL 1.7.2 from http://unanancyowen.com/?p=1255&lang=en
* SoftKinetic SDK 1.4.5 from web page (registration required)

...

Environment Variables
---------------------

*could be automated by script*


Targets
-------


Building
--------

References
----------

Please cite the following [paper](https://dl.acm.org/citation.cfm?id=2971459) [(BibTeX)](https://dl.acm.org/downformats.cfm?id=2971459&parent_id=2968219&expformat=bibtex) if you use MMART:

```
@inproceedings{Barz:2016:MMA:2968219.2971459,
 author = {Barz, Michael and Moniri, Mohammad Mehdi and Weber, Markus and Sonntag, Daniel},
 title = {Multimodal Multisensor Activity Annotation Tool},
 booktitle = {Proceedings of the 2016 ACM International Joint Conference on Pervasive and Ubiquitous Computing: Adjunct},
 series = {UbiComp '16},
 year = {2016},
 isbn = {978-1-4503-4462-3},
 location = {Heidelberg, Germany},
 pages = {17--20},
 numpages = {4},
 url = {http://doi.acm.org/10.1145/2968219.2971459},
 doi = {10.1145/2968219.2971459},
 acmid = {2971459},
 publisher = {ACM},
 address = {New York, NY, USA},
 keywords = {data annotation, data capture, multimodal, multisensor},
} 

```
