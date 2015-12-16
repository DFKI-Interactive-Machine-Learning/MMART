#.rst:
# FindGRPC
# ------------
#
# Locate and configure the Google Protocol Buffers library and GRPC framework.
#
function(PROTOBUF_GENERATE_PROTO_CPP SRCS HDRS)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_GRPC_CPP() called without any proto files")
    return()
  endif()

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH) # This variable is common for all types of output.
    # Create an include path for each file specified
    foreach(FIL ${ARGN})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  else()
    set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  set(${SRCS})
  set(${HDRS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    list(APPEND ${SRCS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
    list(APPEND ${HDRS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")
	add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
             "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out  ${CMAKE_CURRENT_BINARY_DIR} ${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Running C++ protocol buffer compiler on ${FIL}"
      VERBATIM )
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

function(PROTOBUF_GENERATE_GRPC_CPP SRCS HDRS)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_GRPC_CPP() called without any proto files")
    return()
  endif()

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH) # This variable is common for all types of output.
    # Create an include path for each file specified
    foreach(FIL ${ARGN})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  else()
    set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  set(${SRCS})
  set(${HDRS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    list(APPEND ${SRCS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.cc")
    list(APPEND ${HDRS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.h")
    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.cc"
             "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
           --plugin=protoc-gen-grpc=${CPP_PLUGIN_PROTOC_EXECUTABLE}
           ${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Running gRPC C++ protocol buffer compiler on ${FIL}"
      VERBATIM)
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

# Internal function: search for normal library as well as a debug one
#    if the debug one is specified also include debug/optimized keywords
#    in *_LIBRARIES variable
function(_grpc_find_libraries name filename)
   find_library(${name}_LIBRARY
       NAMES ${filename}
       PATHS ${GRPC_SRC_ROOT_FOLDER}/lib/Release)
   mark_as_advanced(${name}_LIBRARY)

   find_library(${name}_LIBRARY_DEBUG
       NAMES ${filename}
       PATHS ${GRPC_SRC_ROOT_FOLDER}/lib/Debug)
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
# Internal function: find threads library
function(_protobuf_find_threads)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads)
    if(Threads_FOUND)
        list(APPEND PROTOBUF_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
        set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBRARIES}" PARENT_SCOPE)
    endif()
endfunction()

#
# Main.
#

# By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
  set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
endif()


# Google's provided vcproj files generate libraries with a "lib"
# prefix on Windows
if(MSVC)
    set(PROTOBUF_ORIG_FIND_LIBRARY_PREFIXES "${CMAKE_FIND_LIBRARY_PREFIXES}")
    set(CMAKE_FIND_LIBRARY_PREFIXES "lib" "")
	find_path(GRPC_SRC_ROOT_FOLDER gRPC.podspec) 
endif()

# The Protobuf library
_grpc_find_libraries(PROTOBUF protobuf)
#DOC "The Google Protocol Buffers RELEASE Library"

_grpc_find_libraries(PROTOBUF_LITE protobuf-lite)

# The Protobuf Protoc Library
_grpc_find_libraries(PROTOBUF_PROTOC protoc)

# The GRPC Library
_grpc_find_libraries(GRPC grpc)

# Restore original find library prefixes
if(MSVC)
    set(CMAKE_FIND_LIBRARY_PREFIXES "${PROTOBUF_ORIG_FIND_LIBRARY_PREFIXES}")
endif()

if(UNIX)
    _protobuf_find_threads()
endif()


# Find the include directory
find_path(GRPC_INCLUDE_DIR
    grpc/grpc.h
    PATHS ${GRPC_SRC_ROOT_FOLDER}/include
)
mark_as_advanced(GRPC_INCLUDE_DIR)

# Find the include directory
find_path(PROTOBUF_INCLUDE_DIR
    google/protobuf/service.h
    PATHS ${GRPC_SRC_ROOT_FOLDER}/include
)
mark_as_advanced(PROTOBUF_INCLUDE_DIR)

# Find the protoc Executable
find_program(PROTOBUF_PROTOC_EXECUTABLE
    NAMES protoc
    DOC "The Google Protocol Buffers Compiler"
    PATHS
    ${GRPC_SRC_ROOT_FOLDER}/protoc
)
mark_as_advanced(PROTOBUF_PROTOC_EXECUTABLE)

find_program(CPP_PLUGIN_PROTOC_EXECUTABLE
    NAMES grpc_cpp_plugin
    DOC "The Google Protocol CPP Plugin Compiler"
    PATHS
    ${GRPC_SRC_ROOT_FOLDER}/protoc/plugins
)
mark_as_advanced(CPP_PLUGIN_PROTOC_EXECUTABLE)

IF (PROTOBUF_LIBRARY AND PROTOBUF_INCLUDE_DIR AND PROTOBUF_PROTOC_EXECUTABLE)
  # in cache already
  SET(PROTOBUF_FOUND TRUE)
ENDIF (PROTOBUF_LIBRARY AND PROTOBUF_INCLUDE_DIR AND PROTOBUF_PROTOC_EXECUTABLE)

IF (GRPC_LIBRARY AND GRPC_INCLUDE_DIR)
  # in cache already
  SET(GRPC_FOUND TRUE)
ENDIF (GRPC_LIBRARY AND GRPC_INCLUDE_DIR)

if(GRPC_FOUND)
    set(GRPC_INCLUDE_DIRS ${GRPC_INCLUDE_DIR})
endif()

if(PROTOBUF_FOUND)
    set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIR})
endif()




