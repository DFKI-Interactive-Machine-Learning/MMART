#############################################################################
# - Find PMD Nano
# 
# This module defines
#  - GRT_FOUND
#  - GRT_INCLUDE_DIRS
#  - GRT_LIBRARIES
#
# This modules supposes that the user might have provided (and looks at those places)
#  - GRT_DIR 
#
#############################################################################

# Internal function: search for normal library as well as a debug one
#    if the debug one is specified also include debug/optimized keywords
#    in *_LIBRARIES variable
function(_find_libraries name filename)
   find_library(${name}_LIBRARY
       NAMES ${filename}
       PATHS ${GRT_POTENTIAL_LIBPATH}/Release)
   mark_as_advanced(${name}_LIBRARY)
   find_library(${name}_LIBRARY_DEBUG
       NAMES ${filename}
       PATHS ${GRT_POTENTIAL_LIBPATH}/Debug)
   mark_as_advanced(${name}_LIBRARY_DEBUG)
   if(NOT ${name}_LIBRARY_DEBUG)
      # There is no debug library
      set(${name}_LIBRARY_DEBUG ${${name}_LIBRARY} PARENT_SCOPE)
      set(${name}_LIBRARIES     ${${name}_LIBRARY} PARENT_SCOPE)
   else()
      # There IS a debug library
      set(${name}_LIBRARIES
          optimized ${${name}_LIBRARY}
          debug     ${${name}_LIBRARY_DEBUG}
          PARENT_SCOPE
      )
   endif()
endfunction()

IF( GRT_INCLUDE_DIRS AND GRT_LIBRARIES)
  SET( GRT_FIND_QUIETLY TRUE ) # Already in cache, be silent.
ENDIF( GRT_INCLUDE_DIRS AND GRT_LIBRARIES)
#----------------------------------------------------------------------------
# Construct potential include and lib paths
#----------------------------------------------------------------------------
SET( GRT_INSTALL_PATH "${GRT_DIR}" )
SET( GRT_POTENTIAL_INCPATH 
	# from user defined GRT_DIR
	${GRT_INSTALL_PATH}/include
	#Mac os X
	#linux:
)

SET( GRT_POTENTIAL_LIBPATH
	# from user defined GRT_DIR
	${GRT_INSTALL_PATH}/bin
	#Mac OS X
	#linux:
)
#----------------------------------------------------------------------------
# Set GRT_INCLUDE_DIRS. (The path to the include folder)
#----------------------------------------------------------------------------
FIND_PATH( GRT_INCLUDE_DIRS NAMES GRT/GRT.h PATHS ${GRT_POTENTIAL_INCPATH} )
#----------------------------------------------------------------------------
# Set GRT_LIBRARIES.
#----------------------------------------------------------------------------
_find_libraries(GRT grt)
#----------------------------------------------------------------------------
# Set GRT_FOUND.
#----------------------------------------------------------------------------
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GRT DEFAULT_MSG GRT_LIBRARIES GRT_INCLUDE_DIRS)

MARK_AS_ADVANCED( GRT_LIBRARIES GRT_INCLUDE_DIRS GRT_FOUND )
