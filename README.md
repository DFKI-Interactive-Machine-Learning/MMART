DESCRIPTION
===========
`nui` is a command line tool to record raw data from several sensors that are connected to a computer. Prior to usage `nui` has to be configured via an XML-file. To start a recording simply run `recorder.exe`, e.g. by using a command line tool of your choice. For setting up the system you can either use a pre-built package (TODO) or build it yourself from source.

Sensors
-------

* Intel RealSense
* Thalmic Labs Myo
* SoftKinetic DepthSense

...

Configuration
-------------
The following options can be set via the XML configuration file:

...

SETUP PRE-BUILT PACKAGE
=======================

* Install Visual C++ Redistributable Packages für Visual Studio 2013
* Install RealSense SDK Runtime and F200 DCM (v1.4)
* PCL 1.7.2 from http://unanancyowen.com/?p=1255&lang=en


BUILDING IN LINUX
=================


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