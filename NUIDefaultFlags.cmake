## NUIDefaultFlags.cmake - Initializes the standard flags for the compilers
##     created by CMake with custom values.
## Note: The values appended here to the flags should not depend on a
##     configuration option, as the values in this file are only written when
##     the cache variables are not set.
## Authors: Jean-Marc Hengen, jhenriques

#Append some switches when building with Visual Studio
IF(DEFINED MSVC)
    # Enable multiple processor build if at least Visual Studio 2008 is used
    IF(NOT MSVC_VERSION LESS 1500)
        SET(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} /MP")
        SET(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} /MP")
    ENDIF(NOT MSVC_VERSION LESS 1500)
    #SET(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} /wd4355 /wd4251 /wd4275 /wd4996")
    #SET(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} /wd4355 /wd4251 /wd4275 /wd4996")

	#ADD_DEFINITIONS(/W3 /wd4996)
ENDIF(DEFINED MSVC)

# Set default NUI C flags for gcc
IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS_DEBUG_INIT "-O0 -ggdb3")
    SET(CMAKE_C_FLAGS_RELEASE_INIT "-O2 -DNDEBUG")
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O1 -ggdb3")

    IF(NOT NoWall)
        SET(CMAKE_C_FLAGS_DEBUG_INIT "${CMAKE_C_FLAGS_DEBUG_INIT} -Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas -Wno-missing-field-initializers")
        SET(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "${CMAKE_C_FLAGS_RELWITHDEBINFO_INIT} -Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas -Wno-missing-field-initializers")
    ENDIF(NOT NoWall)
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# Set default NUI C++ flags for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG_INIT "-O0 -ggdb3")
    SET(CMAKE_CXX_FLAGS_RELEASE_INIT "-O2 -DNDEBUG")
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O1 -ggdb3")

    IF(NOT NoWall)
        SET(CMAKE_CXX_FLAGS_DEBUG_INIT "${CMAKE_CXX_FLAGS_DEBUG_INIT} -Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas -Wno-missing-field-initializers")
        SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "${CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT} -Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas -Wno-missing-field-initializers")
    ENDIF(NOT NoWall)
ENDIF(CMAKE_COMPILER_IS_GNUCXX)
