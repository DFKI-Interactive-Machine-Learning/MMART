# This file is part of the AV-NUI project.
# Copyright (C) 2012-2015 DFKI GmbH. All rights reserved.
#
# Disclaimer:
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND
# CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.

###############################################################################
###                           AV_NUI BUILD SCRIPT                           ###
###############################################################################
cmake_minimum_required (VERSION 3.1.0)
#===================================================================================
# Project Settings
#===================================================================================
PROJECT(NUI)
SET(PROJECT_LONGNAME "NUI - Framework for Naturual User Interaction Devices")
SET(PROJECT_VERSION  "1.0")
SET(VERSION_MAJOR    1)
SET(VERSION_MINOR    0)
#===================================================================================
IF( NOT DEFINED CMAKE_USER_MAKE_RULES_OVERRIDE )
	SET( CMAKE_USER_MAKE_RULES_OVERRIDE "${CMAKE_SOURCE_DIR}/NUIDefaultFlags.cmake")
ENDIF( NOT DEFINED CMAKE_USER_MAKE_RULES_OVERRIDE )

#===================================================================================
# AV-NUI optionals:
#===================================================================================
#  --------------------------------- TECHNOLOGY ------------------------------------
OPTION( WITH_GESTURE_RECOGNITION "Compile with gesture recognition support"     OFF)
#  ---------------------------------- DEVICES --------------------------------------
OPTION( WITH_DEVICE_PMD_NANO     "Compile with PMD Nano support"                ON)
OPTION( WITH_DEVICE_INTEL_REALSENSE   "Compile with Intel Realsense support"    ON)
OPTION( WITH_DEVICE_DEPTHSENSE   "Compile with DepthSense support"              ON)
OPTION( WITH_DEVICE_LEAP_MOTION  "Compile with Leap Motion support"             ON)
OPTION( WITH_DEVICE_MYO          "Compile with MYO support"                     OFF)
OPTION( BUILD_SHARED_LIBS        "Build Shared Libs"	                        ON)
#===================================================================================
# Project wide cmake configuration:
#===================================================================================
SET( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/tools/cmake )
SET( NUI_ROOT_PATH ${PROJECT_SOURCE_DIR} )
#===================================================================================
# External Libs
#===================================================================================
IF( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
	# Set variables extracted from environment
	SET(EIGEN3_INCLUDE_DIR         $ENV{EIGEN3_INCLUDE_PATH})
	SET(PMDSDK_PATH                $ENV{PMDSDK_PATH})
	SET(LEAPSDK_PATH               $ENV{LEAPSDK_PATH})
	SET(MYO_PATH                   $ENV{MYO_PATH})
	SET(GRT_DIR                    $ENV{GRT_DIR})
	SET(GRPC_SRC_ROOT_FOLDER       $ENV{GRPC_DIR})
	SET(Qt5Widgets_DIR             $ENV{Qt5Widgets_DIR})
	SET(VTK_DIR                    $ENV{VTK_DIR})
	SET(DEPTHSENSE_DIR             $ENV{DEPTHSENSE_SDK})
	#   Take from documentation http://www.cmake.org/Wiki/BuildingWinDLL
	include (GenerateExportHeader)
ENDIF( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
#----------------------------------------------------------------------------
# Take care of Eigen 
#----------------------------------------------------------------------------
IF( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
	#Disable alignment for Eigen under 32 Bit
	IF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
		#add_definitions( -DEIGEN_DONT_ALIGN )
	ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
	#undef min and max
	add_definitions( -DNOMINMAX )
ENDIF( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
# OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
# Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
# LEAP MOTION
IF(WITH_DEVICE_LEAP_MOTION)
	FIND_PACKAGE(LeapSDK REQUIRED)
	ADD_DEFINITIONS( -DWITH_DEVICE_LEAP_MOTION=1)
ENDIF(WITH_DEVICE_LEAP_MOTION)
# MYO
IF(WITH_DEVICE_MYO)
	FIND_PACKAGE(Myo REQUIRED)
	ADD_DEFINITIONS( -DWITH_DEVICE_MYO=1)
ENDIF(WITH_DEVICE_MYO)
# PMD Nano
IF(WITH_DEVICE_PMD_NANO) 
	FIND_PACKAGE(PMDNano REQUIRED)
	ADD_DEFINITIONS( -DWITH_DEVICE_PMD_NANO=1)
ENDIF(WITH_DEVICE_PMD_NANO)
# Intel Realsense
IF(WITH_DEVICE_INTEL_REALSENSE) 
	FIND_PACKAGE(RSSDK REQUIRED)
	ADD_DEFINITIONS( -DWITH_DEVICE_INTEL_REALSENSE=1)
ENDIF(WITH_DEVICE_INTEL_REALSENSE)
# DepthSense
IF(WITH_DEVICE_DEPTHSENSE) 
	FIND_PACKAGE(DepthSenseSDK REQUIRED)
	ADD_DEFINITIONS( -DWITH_DEVICE_DEPTHSENSE=1)
ENDIF(WITH_DEVICE_DEPTHSENSE)

# HDF 5
FIND_PACKAGE(HDF5 COMPONENTS CXX REQUIRED)
# Google RPC framework
FIND_PACKAGE(GRPC REQUIRED)
# OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# PCL
FIND_PACKAGE(PCL 1.7.2 REQUIRED COMPONENTS common io visualization filters segmentation recognition)
#----------------------------------------------------------------------------
# Take care of BOOST configuration
#----------------------------------------------------------------------------
ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB=1) #tells boost to avoid auto-link
SET(Boost_ADDITIONAL_VERSIONS 1.54.0 1.55.0)
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_USE_MULTITHREAD ON)
SET(BOOST_MIN_VERSION "1.54.0")
ADD_DEFINITIONS(
  ${PCL_DEFINITIONS}
)
# BOOST
FIND_PACKAGE(Boost ${BOOST_MIN_VERSION} COMPONENTS date_time filesystem log program_options system thread REQUIRED)
# GRT extensions
IF( WITH_GESTURE_RECOGNITION )
	if(UNIX) 
		#Add C++11 and multithreading support
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")
	endif()
	FIND_PACKAGE(GRT REQUIRED)
	ADD_DEFINITIONS( -DWITH_GESTURE_RECOGNITION=1)
ELSEIF(WITH_GESTURE_RECOGNITION)
	ADD_DEFINITIONS( -DWITH_GESTURE_RECOGNITION=0)
ENDIF(WITH_GESTURE_RECOGNITION)
MESSAGE( "\n===================== AV-NUI ======================")
MESSAGE( "   WITH_DEVICE_PMD_NANO        : " ${WITH_DEVICE_PMD_NANO} )
MESSAGE( "   WITH_DEVICE_LEAP_MOTION     : " ${WITH_DEVICE_LEAP_MOTION} )
MESSAGE( "   WITH_DEVICE_MYO             : " ${WITH_DEVICE_MYO} )
MESSAGE( "   WITH_DEVICE_INTEL_REALSENSE : " ${WITH_DEVICE_INTEL_REALSENSE})
MESSAGE( "   WITH_DEVICE_DEPTHSENSE      : " ${WITH_DEVICE_DEPTHSENSE})
MESSAGE( "   WITH_GESTURE_RECOGNITION    : " ${WITH_GESTURE_RECOGNITION} )
MESSAGE( "   BUILD_SHARED_LIBS           : " ${BUILD_SHARED_LIBS} )
MESSAGE( "=====================================================")
MESSAGE( "   Protobuf generated path : ${ProtobufIncludePath}")
MESSAGE( "=====================================================")
# Information about includes and libraries
MESSAGE("\nProject Includes: ")
MESSAGE( "=====================================================")
MESSAGE("include         : ${PROJECT_SOURCE_DIR}/include") 
MESSAGE("OpenCV          : ${OpenCV_INCLUDE_DIRS}")  
MESSAGE("BOOST           : ${Boost_INCLUDE_DIRS}") 
MESSAGE("EIGEN 3         : ${EIGEN3_INCLUDE_DIR}") 
IF( WITH_DEVICE_LEAP_MOTION )
	INCLUDE_DIRECTORIES("${LEAP_SDK_INCLUDE_DIRS}")
	MESSAGE("Leap Motion     : ${LEAP_SDK_INCLUDE_DIRS}") 
ENDIF( WITH_DEVICE_LEAP_MOTION )
IF( WITH_DEVICE_MYO )
	MESSAGE("Myo             : ${MYO_SDK_INCLUDE_DIRS} ") 
ENDIF( WITH_DEVICE_MYO )
MESSAGE("hdf5            : ${HDF5_INCLUDE_DIRS}") 
MESSAGE("protobuf        : ${PROTOBUF_INCLUDE_DIRS}") 
IF( WITH_DEVICE_PMD_NANO )
	MESSAGE("pmdnano         : ${PMD_NANO_INCLUDE_DIRS}") 
	INCLUDE_DIRECTORIES("${PMD_NANO_INCLUDE_DIRS}")
ENDIF( WITH_DEVICE_PMD_NANO )
IF( WITH_DEVICE_INTEL_REALSENSE )
	MESSAGE("Intel RealSense : ${RSSDK_INCLUDE_DIRS}") 
	INCLUDE_DIRECTORIES("${RSSDK_INCLUDE_DIRS}")
ENDIF( WITH_DEVICE_INTEL_REALSENSE )
IF( WITH_DEVICE_DEPTHSENSE )
	MESSAGE("DepthSense      : ${DepthSenseSDK_INCLUDE_DIRS}") 
	INCLUDE_DIRECTORIES("${DepthSenseSDK_INCLUDE_DIRS}")
ENDIF( WITH_DEVICE_DEPTHSENSE )
IF( WITH_GESTURE_RECOGNITION )
	INCLUDE_DIRECTORIES("${GRT_INCLUDE_DIRS}")
	MESSAGE("GRT             : ${GRT_INCLUDE_DIRS}") 
ENDIF( WITH_GESTURE_RECOGNITION )
MESSAGE("PCL             : ${PCL_INCLUDE_DIRS}")
MESSAGE("Generated       : ${CMAKE_BINARY_DIR}/src")
MESSAGE("=====================================================")
MESSAGE("Libraries: ")
MESSAGE("=====================================================")
MESSAGE("OpenCV          : ${OpenCV_LIBS} ")  
MESSAGE("OpenGL          : ${OPENGL_LIBRARIES}")
MESSAGE("BOOST           : ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY}") 
IF( WITH_DEVICE_LEAP_MOTION )
	MESSAGE("Leap Motion     : ${LEAP_SDK_LIBRARIES} ") 
ENDIF( WITH_DEVICE_LEAP_MOTION )
IF( WITH_DEVICE_MYO )
	MESSAGE("Myo             : ${MYO_SDK_LIBRARIES} ") 
ENDIF( WITH_DEVICE_MYO )
IF(WITH_DEVICE_INTEL_REALSENSE) 
	MESSAGE("RealSense       : ${RSSDK_LIBRARY} ") 
ENDIF(WITH_DEVICE_INTEL_REALSENSE)
IF(WITH_DEVICE_DEPTHSENSE) 
	MESSAGE("DepthSense      : ${DepthSenseSDK_LIBRARY} ") 
ENDIF(WITH_DEVICE_DEPTHSENSE)
MESSAGE("hdf5            : ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES}") 
MESSAGE("protobuf        : ${PROTOBUF_LIBRARIES}") 
IF( WITH_DEVICE_PMD_NANO )
	MESSAGE("pmdnano         : ${PMD_NANO_LIBRARIES}") 
ENDIF( WITH_DEVICE_PMD_NANO )
IF( WITH_DEVICE_INTEL_REALSENSE )
	MESSAGE("Intel RealSense : ${RSSDK_LIBRARIES}") 
ENDIF( WITH_DEVICE_INTEL_REALSENSE )
IF( WITH_GESTURE_RECOGNITION )
	MESSAGE("GRT             : ${GRT_INCLUDE_LIBRARIES}") 
ENDIF( WITH_GESTURE_RECOGNITION )
MESSAGE("PCL             : ${PCL_LIBRARIES}")
MESSAGE( "=====================================================\n")

INCLUDE_DIRECTORIES(${ProtobufIncludePath})
ADD_SUBDIRECTORY(src)
