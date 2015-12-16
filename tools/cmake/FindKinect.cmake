#############################################################################
# - Find Kinect
#
# This module defines
#  - KINECT_FOUND
#  - KINECT_INCLUDE_PATH
#  - KINECT_LIBRARIES
#
# This modules supposes that the user might have provided (and looks at those places)
#  - KINECT_PATH
#  - KINECT_INCLUDE_PATH
#  - KINECT_LIB_PATH
#
#############################################################################

IF( KINECT_INCLUDE_PATH AND KINECT_LIBRARIES)
  SET( KINECT_FIND_QUIETLY TRUE ) # Already in cache, be silent.
ENDIF( KINECT_INCLUDE_PATH AND KINECT_LIBRARIES)

# Check of the ExternalToolsPath env var in windows:
set( _EXTERNAL_TOOLS_PATH $ENV{ExternalToolsPath} )
if( NOT EXTERNAL_TOOLS_PATH )
	if( _EXTERNAL_TOOLS_PATH )
		set( EXTERNAL_TOOLS_PATH $ENV{ExternalToolsPath} )
	endif( _EXTERNAL_TOOLS_PATH )
endif( NOT EXTERNAL_TOOLS_PATH )

#Here set the standard arguments in case of EXTERNAL_TOOLS_PATH
if(EXTERNAL_TOOLS_PATH)
	IF(NOT KINECT_PATH)
		SET(KINECT_PATH "${EXTERNAL_TOOLS_PATH}/Kinect")
	ENDIF(NOT KINECT_PATH)
endif(EXTERNAL_TOOLS_PATH)

#----------------------------------------------------------------------------
# Construct potential include and lib paths
#----------------------------------------------------------------------------
#TODO: find standard installation paths

SET(KINECT_POTENTIAL_INCPATH "${KINECT_INCLUDE_PATH}")
SET(KINECT_POTENTIAL_LIBPATH "${KINECT_LIB_PATH}")

SET(KINECT_POTENTIAL_INCPATH ${KINECT_POTENTIAL_INCPATH}
	#windows:
	${KINECT_PATH}/include
	"C:/Program Files (x86)/Code Laboratories/CL NUI Platform/SDK/Include"
)

SET(KINECT_POTENTIAL_LIBPATH "${KINECT_POTENTIAL_LIBPATH}"
	#windows:
	${KINECT_PATH}/lib
	"C:/Program Files (x86)/Code Laboratories/CL NUI Platform/SDK/Lib/"
	"C:/Program Files/Code Laboratories/CL NUI Platform/SDK/Lib/"
	"C:/Programme/Code Laboratories/CL NUI Platform/SDK/Lib/"
	"C:/Programme (x86)/Code Laboratories/CL NUI Platform/SDK/Lib/"
)


#----------------------------------------------------------------------------
# Set KINECT_INCLUDE_PATH. (The path to the include folder)
#----------------------------------------------------------------------------
FIND_PATH(KINECT_INCLUDE_PATH NAMES CLNUIDevice.h PATHS ${KINECT_POTENTIAL_INCPATH})

#message( "\n KINECT_INCLUDE_PATH: " )
#message( ${KINECT_INCLUDE_PATH} )

#----------------------------------------------------------------------------
# Set KINECT_LIBRARIES.
#----------------------------------------------------------------------------
#TODO: find standard library names

FIND_LIBRARY( KINECT_LIBRARIES NAMES CLNUIDevice PATHS ${KINECT_POTENTIAL_LIBPATH})

IF(KINECT_LIBRARIES)
    GET_FILENAME_COMPONENT(KINECT_LIB_PATH ${KINECT_LIBRARIES} PATH)
ENDIF(KINECT_LIBRARIES)

#message( "\n KINECT_LIBRARIES:" )
#message( ${KINECT_LIBRARIES} )

#----------------------------------------------------------------------------
# Set KINECT_FOUND.
#----------------------------------------------------------------------------
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Kinect DEFAULT_MSG KINECT_LIBRARIES KINECT_INCLUDE_PATH)

MARK_AS_ADVANCED(KINECT_LIBRARIES KINECT_INCLUDE_PATH KINECT_FOUND)
