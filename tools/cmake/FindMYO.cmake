#############################################################################
# - Find Myo SDK 
#
# This module defines
#  - MYO_SDK_FOUND
#  - MYO_SDK_INCLUDE_DIRS
#  - MYO_SDK_LIBRARIES
#
# This modules supposes that the user might have provided (and looks at those places)
#  - MyoSDK_DIR 
#
#############################################################################

IF( MYO_SDK_INCLUDE_DIRS AND MYO_SDK_LIBRARIES)
  SET( MYO_SDK_FIND_QUIETLY TRUE ) # Already in cache, be silent.
ENDIF( MYO_SDK_INCLUDE_DIRS AND MYO_SDK_LIBRARIES)

#----------------------------------------------------------------------------
# Construct potential include and lib paths
#----------------------------------------------------------------------------
SET( MYO_SDK_INSTALL_PATH "${MYO_PATH}" )
SET( MYO_SDK_POTENTIAL_INCPATH 
	# from user defined MyoSDK_DIR
	${MYO_SDK_INSTALL_PATH}/include

	#Mac os X
	#linux:
)
UNSET( MYO_WINDOWS_CORRECT_PATH )
IF( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )
	IF( CMAKE_SIZEOF_VOID_P EQUAL 4 )	# Windows 32
		SET( MYO_WINDOWS_CORRECT_PATH x86 )
    ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
	IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )	# Windows 64
		SET( MYO_WINDOWS_CORRECT_PATH x64 )
    ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
ENDIF( ${CMAKE_SYSTEM_NAME} MATCHES "Windows" )

SET( MYO_SDK_POTENTIAL_LIBPATH

	# external tools:
	${EXTERNAL_TOOLS_PATH}/MyoSDK/lib/libc++
	${EXTERNAL_TOOLS_PATH}/MyoSDK/${MYO_WINDOWS_CORRECT_PATH}
	${EXTERNAL_TOOLS_PATH}/MyoSDK/lib

	# potentialy under application 3rdparty
	3rdparty/MyoSDK/lib/libc++
	3rdparty/MyoSDK/lib/${MYO_WINDOWS_CORRECT_PATH}
	3rdparty/MyoSDK/lib
	../3rdparty/MyoSDK/lib/libc++
	../3rdparty/MyoSDK/lib

	# from user defined MyoSDK_DIR
	${MYO_SDK_INSTALL_PATH}/lib/libc++
	${MYO_SDK_INSTALL_PATH}/lib/${MYO_WINDOWS_CORRECT_PATH}
	${MYO_SDK_INSTALL_PATH}/lib

	#Mac OS X
	#linux:
)

#----------------------------------------------------------------------------
# Set MYO_SDK_INCLUDE_DIRS. (The path to the include folder)
#----------------------------------------------------------------------------
FIND_PATH( MYO_SDK_INCLUDE_DIRS NAMES myo/myo.hpp PATHS ${MYO_SDK_POTENTIAL_INCPATH} )

#message( "\n MYO_SDK_INCLUDE_DIRS: " )
#message( ${MYO_SDK_INCLUDE_DIRS} )

#----------------------------------------------------------------------------
# Set MYO_SDK_LIBRARIES.
#----------------------------------------------------------------------------
FIND_LIBRARY( MYO_SDK_LIBRARY NAMES myo64 PATHS ${MYO_SDK_POTENTIAL_LIBPATH} )

SET( MYO_SDK_LIBRARIES ${MYO_SDK_LIBRARY} )

#----------------------------------------------------------------------------
# Set MYO_SDK_FOUND.
#----------------------------------------------------------------------------
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MyoSDK DEFAULT_MSG MYO_SDK_LIBRARIES MYO_SDK_INCLUDE_DIRS)

MARK_AS_ADVANCED( MYO_SDK_LIBRARIES MYO_SDK_INCLUDE_DIRS MYO_SDK_FOUND )
