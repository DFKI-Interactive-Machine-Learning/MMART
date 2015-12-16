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

# Set path for binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
# Instruct CMake to run moc automatically when needed.
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
# Compiler Runtime DLLs
IF(MSVC)
  # Visual Studio
  set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP true)
  include(InstallRequiredSystemLibraries)
  install(FILES ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS} DESTINATION .)
ENDIF(MSVC)

include(${VTK_USE_FILE})
# Header with pre-compiled information
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/include/defines.h.cmake
               ${PROJECT_SOURCE_DIR}/include/defines.h)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/src "${CMAKE_BINARY_DIR}/src"
                    "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/ui" 
					"${OpenCV_INCLUDE_DIRS}" 
					"${Boost_INCLUDE_DIRS}" "${EIGEN3_INCLUDE_DIR}"  
					"${HDF5_INCLUDE_DIRS}"
					"${GRPC_INCLUDE_DIRS}"
					"${PROTOBUF_INCLUDE_DIRS}" 
					"${PCL_INCLUDE_DIRS}")
								  					
# =========================== Common definitions =================================
SET(COMMON_HEADERS ${PROJECT_SOURCE_DIR}/include/core/common.hpp
				   ${PROJECT_SOURCE_DIR}/include/core/errorcodes.hpp
				   ${PROJECT_SOURCE_DIR}/include/core/NUIDLLexport.hpp
				   ${PROJECT_SOURCE_DIR}/include/core/TripleBuffer.hxx
				   ${PROJECT_SOURCE_DIR}/include/eventmanager/Events.hpp
				   ${PROJECT_SOURCE_DIR}/include/eventmanager/IProcessingModule.hpp
				   ${PROJECT_SOURCE_DIR}/include/eventmanager/Modality.hpp
				   ${PROJECT_SOURCE_DIR}/include/inttypes.h
				   ${PROJECT_SOURCE_DIR}/include/eventmanager/SimpleEventListener.hpp)
SET(COMMON_SOURCES eventmanager/Events.cpp
				   eventmanager/IProcessingModule.cpp
				   eventmanager/Modality.cpp
				   eventmanager/SimpleEventListener.cpp)
ADD_LIBRARY(common ${LIB_TYPE} ${COMMON_SOURCES} ${COMMON_HEADERS})
TARGET_LINK_LIBRARIES(common interfaces ${Boost_LIBRARIES})
#TARGET_LINK_LIBRARIES(common interfaces ${PROTOBUF_LIBRARIES} ${Boost_LOG_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_LOG_LIBRARY} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES})
# Tidy up
UNSET(COMMON_HEADERS)
UNSET(COMMON_SOURCES)
# =========================== Config Library ==================================================
SET(CONFIG_SOURCES 	config/NUIconfig.cpp)
SET(CONFIG_HEADERS  ${PROJECT_SOURCE_DIR}/include/config/NUIconfig.hpp)
ADD_LIBRARY(config ${LIB_TYPE} ${CONFIG_SOURCES} ${CONFIG_HEADERS})
TARGET_LINK_LIBRARIES(config ${Boost_LIBRARIES})
# Tidy up
UNSET(CONFIG_SOURCES)
UNSET(CONFIG_HEADERS)
# =========================== GESTURE Recognition Library =================================
IF(WITH_GESTURE_RECOGNITION)
	INCLUDE_DIRECTORIES("${GRT_INCLUDE_DIRS}" 
						"${PROJECT_SOURCE_DIR}/gesture-recognition/include")
	MESSAGE("\nGesture Recognition includes: ")
	MESSAGE( "=====================================================")
	MESSAGE(" include     : ${PROJECT_SOURCE_DIR}/gesture-recognition/include") 
	MESSAGE(" GRT         : ${GRT_INCLUDE_DIRS}")  
	MESSAGE( "=====================================================\n")
	SET(GESTURERECOGNITION_HEADERS ${PROJECT_SOURCE_DIR}/gesture-recognition/include/FeatureSelectionFilter.hpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/include/LSTM.hpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/include/NormalizationFilter.hpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/include/HandGestureRecognition.hpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/include/GestureRecognition.hpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/include/ModelClassifier.hpp)
	SET(GESTURERECOGNITION_SOURCES ${PROJECT_SOURCE_DIR}/gesture-recognition/src/FeatureSelectionFilter.cpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/src/NormalizationFilter.cpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/src/LSTM.cpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/src/HandGestureRecognition.cpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/src/GestureRecognition.cpp
								   ${PROJECT_SOURCE_DIR}/gesture-recognition/src/ModelClassifier.cpp)
	ADD_LIBRARY(gesture_recognition ${LIB_TYPE} ${GESTURERECOGNITION_SOURCES} ${GESTURERECOGNITION_HEADERS})
	TARGET_LINK_LIBRARIES(gesture_recognition config common ${Boost_LIBRARIES}  ${GRT_LIBRARIES}  ${OpenCV_LIBS} ${PCL_LIBRARIES})
	UNSET(GESTURERECOGNITION_SOURCES)
	UNSET(GESTURERECOGNITION_HEADERS)
ENDIF(WITH_GESTURE_RECOGNITION)
# =========================== Eventmanager Library ================================================
SET(EVENTMANAGER_SOURCES controller/Controller.cpp
			 controller/TCPChannel.cpp
			 eventmanager/NUIEventManager.cpp
			 eventmanager/DataDumper.cpp
			 serialize/GPufferSerializer.cpp
			 serialize/HDF5Serializer.cpp
			 serialize/PointCloudSerializer.cpp
			 serialize/OpenCVSerializer.cpp)
			 
SET(EVENTMANAGER_HEADERS ${PROJECT_SOURCE_DIR}/include/controller/ControlChannel.hpp
                         ${PROJECT_SOURCE_DIR}/include/controller/Controller.hpp
						 ${PROJECT_SOURCE_DIR}/include/controller/TCPChannel.hpp
                         ${PROJECT_SOURCE_DIR}/include/eventmanager/NUIEventManager.hpp
						 ${PROJECT_SOURCE_DIR}/include/eventmanager/DataDumper.hpp
						 ${PROJECT_SOURCE_DIR}/include/serialize/GPufferSerializer.hpp
						 ${PROJECT_SOURCE_DIR}/include/serialize/HDF5Serializer.hpp
						 ${PROJECT_SOURCE_DIR}/include/serialize/OpenCVSerializer.hpp
						 ${PROJECT_SOURCE_DIR}/include/serialize/PointCloudSerializer.hpp)

ADD_LIBRARY(eventmanager ${LIB_TYPE} ${EVENTMANAGER_SOURCES} ${EVENTMANAGER_HEADERS})
TARGET_LINK_LIBRARIES(eventmanager config  cam interfaces common 
					${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES})
					
IF( WITH_GESTURE_RECOGNITION )
	TARGET_LINK_LIBRARIES(eventmanager gesture_recognition ${GRT_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
ENDIF( WITH_GESTURE_RECOGNITION )
					
IF( WITH_DEVICE_LEAP_MOTION )
	TARGET_LINK_LIBRARIES(eventmanager leapmotion ${LEAP_SDK_LIBRARIES})
ENDIF( WITH_DEVICE_LEAP_MOTION )

IF( WITH_DEVICE_MYO )
	TARGET_LINK_LIBRARIES(eventmanager myo ${MYO_SDK_LIBRARIES})
ENDIF( WITH_DEVICE_MYO )

IF( WITH_DEVICE_PMD_NANO )
	TARGET_LINK_LIBRARIES(eventmanager pmd ${PMD_NANO_LIBRARIES})
ENDIF( WITH_DEVICE_PMD_NANO )

IF( WITH_DEVICE_INTEL_REALSENSE )
	TARGET_LINK_LIBRARIES(eventmanager intelrealsense ${RSSDK_LIBRARIES})
ENDIF( WITH_DEVICE_INTEL_REALSENSE )

IF( WITH_DEVICE_DEPTHSENSE )
	TARGET_LINK_LIBRARIES(eventmanager depthsensecamera ${DepthSenseSDK_LIBRARY})
ENDIF( WITH_DEVICE_DEPTHSENSE )

# Tidy up
UNSET(EVENTMANAGER_SOURCES)
UNSET(EVENTMANAGER_HEADERS)
# =========================== Interfaces ===========================================================
PROTOBUF_GENERATE_PROTO_CPP(PROTO_SOURCES PROTO_HEADERS ${PROJECT_SOURCE_DIR}/interfaces/interface.proto)
PROTOBUF_GENERATE_GRPC_CPP(SERVICE_SOURCES SERVICE_HEADERS ${PROJECT_SOURCE_DIR}/interfaces/interface.proto)
SET(ProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
    CACHE INTERNAL "Path to generated protobuf files.")
SET(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
SET(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
SET(PROTO_SOURCES ${PROTO_SOURCES} PARENT_SCOPE)
SET(PROTO_HEADERS ${PROTO_HEADERS} PARENT_SCOPE)
SET_SOURCE_FILES_PROPERTIES(${PROTO_SRC} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)
MESSAGE("GRPC Services")
MESSAGE("=====================================================")
MESSAGE(" Binaries:")
MESSAGE("=====================================================")
MESSAGE(" Protoc : ${PROTOBUF_PROTOC_EXECUTABLE}")
MESSAGE(" Plugin : ${CPP_PLUGIN_PROTOC_EXECUTABLE}")
MESSAGE("=====================================================")
MESSAGE(" Generated sources:")
MESSAGE("=====================================================")
MESSAGE("  -> Interface file     : ${PROJECT_SOURCE_DIR}/interfaces/interface.proto")
MESSAGE("  -> Protocol headers   : ${PROTO_HEADERS}")
MESSAGE("  -> Protocol sources   : ${PROTO_SOURCES}")
MESSAGE("  -> GRPC headers       : ${SERVICE_HEADERS}")
MESSAGE("  -> GRPC sources       : ${SERVICE_SOURCES}")
MESSAGE("=====================================================\n")
ADD_LIBRARY(interfaces STATIC ${PROTO_SOURCES} ${PROTO_HEADERS})
TARGET_LINK_LIBRARIES(interfaces ${PROTOBUF_LIBRARIES})
ADD_LIBRARY(service ${LIB_TYPE} ${SERVICE_SOURCES} ${SERVICE_HEADERS})
TARGET_LINK_LIBRARIES(service interfaces ${PROTOBUF_LIBRARIES} ${GRPC_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(interfaces PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
# Tidy up
UNSET(PROTO_SOURCES)
UNSET(PROTO_HEADERS)
# =========================== LEAP Motion Wrapper Library =========================================
IF(WITH_DEVICE_LEAP_MOTION)
	INCLUDE_DIRECTORIES("${LEAP_SDK_INCLUDE_DIRS}")
	SET(LEAPMOTION_SOURCES devices/LEAPMotionWrapper.cpp)	
	SET(LEAPMOTION_HEADERS ${PROJECT_SOURCE_DIR}/include/devices/LEAPMotionWrapper.hpp)	 
	ADD_LIBRARY(leapmotion ${LIB_TYPE} ${LEAPMOTION_SOURCES} ${LEAPMOTION_HEADERS})
	TARGET_LINK_LIBRARIES(leapmotion config common ${LEAP_SDK_LIBRARIES} ${Boost_LIBRARIES})
	# Tidy up
	UNSET(LEAPMOTION_SOURCES)
	UNSET(LEAPMOTION_HEADERS)
ENDIF(WITH_DEVICE_LEAP_MOTION)
# =========================== Myo Wrapper Library ==================================================
IF(WITH_DEVICE_MYO)
	INCLUDE_DIRECTORIES("${MYO_SDK_INCLUDE_DIRS}")
	SET(MYO_SOURCES devices/MyoWrapper.cpp)	
	SET(MYO_HEADERS ${PROJECT_SOURCE_DIR}/include/devices/MyoWrapper.hpp)	 
	ADD_LIBRARY(myo ${LIB_TYPE} ${MYO_SOURCES} ${MYO_HEADERS})
	TARGET_LINK_LIBRARIES(myo config common ${MYO_SDK_LIBRARIES} ${Boost_LIBRARIES})
	# Tidy up
	UNSET(MYO_SOURCES)
	UNSET(MYO_HEADERS)
ENDIF(WITH_DEVICE_MYO)
# =========================== WebCam Wrapper Library ===============================================
SET(CAM_SOURCES devices/Webcam.cpp)
SET(CAM_HEADERS ${PROJECT_SOURCE_DIR}/include/devices/Webcam.hpp)	 
ADD_LIBRARY(cam ${LIB_TYPE} ${CAM_SOURCES} ${CAM_HEADERS})
TARGET_LINK_LIBRARIES(cam config common ${OpenCV_LIBS})
# Tidy up
UNSET(CAM_SOURCES)
UNSET(CAM_HEADERS)
# =========================== PMDNano Wrapper Library ===============================================
IF( WITH_DEVICE_PMD_NANO )
	SET(PMD_SOURCES devices/PMDNano.cpp)
	SET(PMD_HEADERS ${PROJECT_SOURCE_DIR}/include/devices/PMDNano.hpp)	 
	ADD_LIBRARY(pmd ${LIB_TYPE} ${PMD_SOURCES} ${PMD_HEADERS})
	TARGET_LINK_LIBRARIES(pmd config common ${PMD_NANO_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
	# Tidy up
	UNSET(PMD_SOURCES)
	UNSET(PMD_HEADERS)
ENDIF( WITH_DEVICE_PMD_NANO )
# =========================== Intel Realsense Wrapper Library =======================================
IF( WITH_DEVICE_INTEL_REALSENSE )
	SET(INTEL_REALSENSE_SOURCES devices/IntelRealSenseCamera.cpp)
	SET(INTEL_REALSENSE_HEADERS ${PROJECT_SOURCE_DIR}/include/devices/IntelRealSenseCamera.hpp)	 
	ADD_LIBRARY(intelrealsense ${LIB_TYPE} ${INTEL_REALSENSE_SOURCES} ${INTEL_REALSENSE_HEADERS})
	TARGET_LINK_LIBRARIES(intelrealsense config common ${RSSDK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
	# Tidy up
	UNSET(INTEL_REALSENSE_SOURCES)
	UNSET(INTEL_REALSENSE_HEADERS)
ENDIF( WITH_DEVICE_INTEL_REALSENSE )
# =========================== DepthSense Wrapper Library =======================================
IF( WITH_DEVICE_DEPTHSENSE )
	SET(DEPTHSENSE_SOURCES devices/DepthSenseCamera.cpp)
	SET(DEPTHSENSE_HEADERS ${PROJECT_SOURCE_DIR}/include/devices/DepthSenseCamera.hpp)	 
	ADD_LIBRARY(depthsensecamera ${LIB_TYPE} ${DEPTHSENSE_SOURCES} ${DEPTHSENSE_HEADERS})
	TARGET_LINK_LIBRARIES(depthsensecamera config common ${DepthSenseSDK_LIBRARY} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
	# Tidy up
	UNSET(DEPTHSENSE_SOURCES)
	UNSET(DEPTHSENSE_SOURCES)
ENDIF( WITH_DEVICE_DEPTHSENSE )

# ---------------------------------------------------------------------------
ADD_EXECUTABLE(recorder app/GestureRecoder_NUI.cpp)
add_definitions(${PCL_DEFINITIONS})
TARGET_LINK_LIBRARIES(recorder config common eventmanager leapmotion interfaces ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES}  ${PCL_LIBRARIES})
IF( WITH_DEVICE_DEPTHSENSE )
	TARGET_LINK_LIBRARIES(recorder depthsensecamera ${DepthSenseSDK_LIBRARY})
ENDIF( WITH_DEVICE_DEPTHSENSE )

IF( WITH_DEVICE_LEAP_MOTION )
	TARGET_LINK_LIBRARIES(recorder leapmotion ${LEAP_SDK_LIBRARIES})
ENDIF( WITH_DEVICE_LEAP_MOTION )

IF( WITH_DEVICE_MYO )
	TARGET_LINK_LIBRARIES(recorder myo ${MYO_SDK_LIBRARIES})
ENDIF( WITH_DEVICE_MYO )

IF( WITH_DEVICE_PMD_NANO )
	TARGET_LINK_LIBRARIES(recorder pmd ${PMD_NANO_LIBRARIES})
ENDIF( WITH_DEVICE_PMD_NANO )

IF( WITH_DEVICE_INTEL_REALSENSE )
	TARGET_LINK_LIBRARIES(recorder intelrealsense ${RSSDK_LIBRARIES})
ENDIF( WITH_DEVICE_INTEL_REALSENSE )
#----------------------------------------------------------------------------
# General CPack setup:
#----------------------------------------------------------------------------
set( CPACK_PACKAGE_NAME "NUI")
set( CPACK_PACKAGE_VENDOR "http://www.dfki.de/")
set( CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/docs/README.txt" )
set( CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/docs/Copyright.txt" )
set( CPACK_PACKAGE_CONTACT "http://www.dfki.de/" )
set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "Framework for Natural User Interaction.")
set( CPACK_PACKAGE_VERSION "0.1")
set( CPACK_PACKAGE_VERSION_MAJOR "0")
set( CPACK_PACKAGE_VERSION_MINOR "1")
set( CPACK_PACKAGE_VERSION_PATCH "0")
set( CPACK_PACKAGE_INSTALL_DIRECTORY "NUI")
set( CPACK_PACKAGE_FILE_NAME "NUI" )
#----------------------------------------------------------------------------
# Linux install
#----------------------------------------------------------------------------
if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
	set(SPECIFIC_SYSTEM_VERSION_NAME "${CMAKE_SYSTEM_NAME}")
	if(EXISTS "/etc/issue")
		set(LINUX_NAME "")
		file(READ "/etc/issue" LINUX_ISSUE)

		# Ubuntu
		#if( LINUX_ISSUE MATCHES "Ubuntu" OR LINUX_ISSUE MATCHES "Debian" )

			#message( "    DEBIAN --------------------------------")

			set( CMAKE_INSTALL_PREFIX "/usr" )
			set(CPACK_GENERATOR DEB)
			install( TARGETS car_nui RUNTIME DESTINATION share/avnui/bin COMPONENT Runtime )

			set( LIBS config eventmanager leapmotion interfaces ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES})

			string(REGEX MATCH "buntu ([0-9]+\\.[0-9]+)" UBUNTU "${LINUX_ISSUE}")
			set(LINUX_NAME "Ubuntu_${CMAKE_MATCH_1}")
		
			install( FILES ${CPACK_PACKAGE_DESCRIPTION_FILE} DESTINATION share/avnui/doc )
			install( FILES ${CPACK_RESOURCE_FILE_LICENSE} DESTINATION share/avnui/doc )

			set(CPACK_DEBIAN_PACKAGE_NAME "av-nui")
			set(CPACK_DEBIAN_PACKAGE_MAINTAINER "AV-NUI Team <markus.weber@dfki.de>")
			set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
 			set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
			set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/resources/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/resources/debian/prerm;")
		
		#endif( LINUX_ISSUE MATCHES "Ubuntu" OR LINUX_ISSUE MATCHES "Debian" )

	endif(EXISTS "/etc/issue")      

    # Configure desktop entry and install it
    set(DESKTOP_ENTRY_IN
        "${PROJECT_SOURCE_DIR}/resources/av-nui.desktop.in"
        )
    set(DESKTOP_ENTRY_OUT
        "${PROJECT_BINARY_DIR}/resources/av-nui.desktop"
        )

    install(FILES "${DESKTOP_ENTRY_OUT}"
        DESTINATION share/applications
        )

endif( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
# include cpack. This must be done last!
include( CPack )
