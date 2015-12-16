#############################################################################
# - Find PMD Nano
# 
# This module defines
#  - PMD_NANO_FOUND
#  - PMD_NANO_INCLUDE_DIRS
#  - PMD_NANO_LIBRARIES
#
# This modules supposes that the user might have provided (and looks at those places)
#  - PMDSDK_DIR 
#
#############################################################################

IF( PMD_NANO_INCLUDE_DIRS AND PMD_NANO_LIBRARIES)
  SET( PMD_NANO_FIND_QUIETLY TRUE ) # Already in cache, be silent.
ENDIF( PMD_NANO_INCLUDE_DIRS AND PMD_NANO_LIBRARIES)
#----------------------------------------------------------------------------
# Construct potential include and lib paths
#----------------------------------------------------------------------------
set( PMD_NANO_INSTALL_PATH "${PMDSDK_PATH}" )
set( PMD_NANO_POTENTIAL_INCPATH 
	# from user defined PMDSDK_DIR
	${PMD_NANO_INSTALL_PATH}/include
	#Mac os X
	#linux:
)
set( PMD_NANO_POTENTIAL_LIBPATH
	# from user defined PMDSDK_DIR
	${PMD_NANO_INSTALL_PATH}/bin
	#Mac OS X
	#linux:
	${PMD_NANO_INSTALL_PATH}/lib
)
#----------------------------------------------------------------------------
# Set PMD_NANO_INCLUDE_DIRS. (The path to the include folder)
#----------------------------------------------------------------------------
find_path( PMD_NANO_INCLUDE_DIRS NAMES pmdsdk2.h PATHS ${PMD_NANO_POTENTIAL_INCPATH} )
#----------------------------------------------------------------------------
# Set PMD_NANO_LIBRARIES.
#----------------------------------------------------------------------------
find_library( PMD_NANO_LIBRARY NAMES pmdaccess2 PATHS ${PMD_NANO_POTENTIAL_LIBPATH} )
set( PMD_NANO_LIBRARIES ${PMD_NANO_LIBRARY} )
#----------------------------------------------------------------------------
# Set PMD_NANO_FOUND.
#----------------------------------------------------------------------------
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PMDSDK DEFAULT_MSG PMD_NANO_LIBRARIES PMD_NANO_INCLUDE_DIRS)
MARK_AS_ADVANCED( PMD_NANO_LIBRARIES PMD_NANO_INCLUDE_DIRS PMD_NANO_FOUND )
