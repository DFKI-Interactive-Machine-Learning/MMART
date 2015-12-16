#############################################################################
# - Find Leap SDK 
#
# This module defines
#  - LEAP_SDK_FOUND
#  - LEAP_SDK_INCLUDE_DIRS
#  - LEAP_SDK_LIBRARIES
#
# This modules supposes that the user might have provided (and looks at those places)
#  - LeapSDK_DIR 
#
#############################################################################

IF( LEAP_SDK_INCLUDE_DIRS AND LEAP_SDK_LIBRARIES)
  SET( LEAP_SDK_FIND_QUIETLY TRUE ) # Already in cache, be silent.
ENDIF( LEAP_SDK_INCLUDE_DIRS AND LEAP_SDK_LIBRARIES)

#----------------------------------------------------------------------------
# Construct potential include and lib paths
#----------------------------------------------------------------------------
set( LEAP_SDK_INSTALL_PATH "${LEAPSDK_PATH}" )

set( LEAP_SDK_POTENTIAL_INCPATH 
	# from user defined LeapSDK_DIR
	${LEAP_SDK_INSTALL_PATH}/include

	#Mac os X
	#linux:
)
unset( LEAP_WINDOWS_CORRECT_PATH )
if( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
	if( CMAKE_SIZEOF_VOID_P EQUAL 4 )	# Windows 32
		set( LEAP_WINDOWS_CORRECT_PATH x86 )
    endif( CMAKE_SIZEOF_VOID_P EQUAL 4 )
	if( CMAKE_SIZEOF_VOID_P EQUAL 8 )	# Windows 64
		set( LEAP_WINDOWS_CORRECT_PATH x64 )
    endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
endif( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )

set( LEAP_SDK_POTENTIAL_LIBPATH

	# external tools:
	${EXTERNAL_TOOLS_PATH}/LeapSDK/lib/libc++
	${EXTERNAL_TOOLS_PATH}/LeapSDK/${LEAP_WINDOWS_CORRECT_PATH}
	${EXTERNAL_TOOLS_PATH}/LeapSDK/lib

	# potentialy under application 3rdparty
	3rdparty/LeapSDK/lib/libc++
	3rdparty/LeapSDK/lib/${LEAP_WINDOWS_CORRECT_PATH}
	3rdparty/LeapSDK/lib
	../3rdparty/LeapSDK/lib/libc++
	../3rdparty/LeapSDK/lib

	# from user defined LeapSDK_DIR
	${LEAP_SDK_INSTALL_PATH}/lib/libc++
	${LEAP_SDK_INSTALL_PATH}/lib/${LEAP_WINDOWS_CORRECT_PATH}
	${LEAP_SDK_INSTALL_PATH}/lib

	#Mac OS X
	#linux:
)

#----------------------------------------------------------------------------
# Set LEAP_SDK_INCLUDE_DIRS. (The path to the include folder)
#----------------------------------------------------------------------------
find_path( LEAP_SDK_INCLUDE_DIRS NAMES Leap.h PATHS ${LEAP_SDK_POTENTIAL_INCPATH} )

#message( "\n LEAP_SDK_INCLUDE_DIRS: " )
#message( ${LEAP_SDK_INCLUDE_DIRS} )

#----------------------------------------------------------------------------
# Set LEAP_SDK_LIBRARIES.
#----------------------------------------------------------------------------
find_library( LEAP_SDK_LIBRARY NAMES Leap PATHS ${LEAP_SDK_POTENTIAL_LIBPATH} )

set( LEAP_SDK_LIBRARIES ${LEAP_SDK_LIBRARY} )

#----------------------------------------------------------------------------
# Set LEAP_SDK_FOUND.
#----------------------------------------------------------------------------
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LeapSDK DEFAULT_MSG LEAP_SDK_LIBRARIES LEAP_SDK_INCLUDE_DIRS)

MARK_AS_ADVANCED( LEAP_SDK_LIBRARIES LEAP_SDK_INCLUDE_DIRS LEAP_SDK_FOUND )
