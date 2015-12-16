###############################################################################
# Find SoftKinectic DepthSense SDK
#
#     find_package(DepthSenseSDK)
#
# Variables defined by this module:
#
#  DepthSenseSDK_FOUND                 True if DepthSense SDK was found
#  DepthSenseSDK_INCLUDE_DIRS          The location(s) of DepthSense SDK headers
#  DepthSenseSDK_LIBRARIES             Libraries needed to use DepthSense SDK

find_path(DepthSenseSDK_DIR include/DepthSenseAPI.h
          PATHS "$ENV{DepthSenseSDK_DIR}"
		        "$ENV{PROGRAMFILES}/SoftKinetic/DepthSenseSDK"
				"$ENV{PROGRAMW6432}/SoftKinetic/DepthSenseSDK"
				"C:/Program Files (x86)/SoftKinetic/DepthSenseSDK"
				"C:/Program Files/SoftKinetic/DepthSenseSDK"
          DOC "DepthSense SDK directory")
		  
if(DepthSenseSDK_DIR)

  # Include directories
  set(DepthSenseSDK_INCLUDE_DIRS ${DepthSenseSDK_DIR}/include)
  mark_as_advanced(DepthSenseSDK_INCLUDE_DIRS)
  
  # Libraries
  set(DepthSenseSDK_RELEASE_NAME DepthSense.lib)
  find_library(DepthSenseSDK_LIBRARY
               NAMES ${DepthSenseSDK_RELEASE_NAME}
               PATHS "${DepthSenseSDK_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES  ${DepthSenseSDK_WINDOWS_CORRECT_PATH})
  find_library(DepthSenseSDK_LIBRARY_DEBUG
               NAMES ${DepthSenseSDK_DEBUG_NAME} ${DepthSenseSDK_RELEASE_NAME}
               PATHS "${DepthSenseSDK_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES  ${DepthSenseSDK_WINDOWS_CORRECT_PATH})
  if(NOT DepthSenseSDK_LIBRARY_DEBUG)
    set(DepthSenseSDK_LIBRARY_DEBUG ${DepthSenseSDK_LIBRARY})
  endif()
  set(DepthSenseSDK_LIBRARIES optimized ${DepthSenseSDK_LIBRARY})
  mark_as_advanced(DepthSenseSDK_LIBRARY)	
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DepthSenseSDK
  FOUND_VAR DepthSenseSDK_FOUND
  REQUIRED_VARS DepthSenseSDK_LIBRARIES DepthSenseSDK_INCLUDE_DIRS
  VERSION_VAR DepthSenseSDK_VERSION
)

