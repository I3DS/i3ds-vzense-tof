cmake_minimum_required(VERSION 3.0)

# This file is adapted from i3ds-basler

################################################################################
# Set some project wide defintions
################################################################################

project (i3ds-vzense-tof)

set (PROJECT_AUTHOR "SINTEF Digital")
set (PROJECT_URL "https://corob-x.eu/")
set (PROJECT_LICENSE "MPLv2")
set (PROJECT_VERSION "1.0")

enable_testing ()

# Builds for embedded systems are not tested
option(BUILD_EMBEDDED "Build for embedded systems" OFF)
option(BUILD_DEBUG "Enable debug-build with lots and lots of warnings" OFF)
option(I3DS_FRAMEWORK_DIR "Custom directory for i3ds (uses \"external\" ASN.1 messages)")

if (BUILD_EMBEDDED)
  MESSAGE(WARNING "Building Vzense for embedded systems is untested")
  if(NOT DEFINED CMAKE_FIND_ROOT_PATH)
    MESSAGE(FATAL_ERROR "CMAKE_FIND_ROOT_PATH path must be set when compiling for embedded")
  endif(NOT DEFINED CMAKE_FIND_ROOT_PATH)

  if (DEFINED BUILD_SYSROOT)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${BUILD_SYSROOT}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${BUILD_SYSROOT}")
  else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
  endif ()
endif (BUILD_EMBEDDED)

# Find i3ds dependencies
find_library(I3DS_MAIN_LIB i3ds HINTS ${I3DS_FRAMEWORK_DIR} PATH_SUFFIXES build/libi3ds)
find_library(I3DS_CLIENT_LIB i3ds_clients HINTS ${I3DS_FRAMEWORK_DIR} PATH_SUFFIXES build/clients)
find_library(I3DS_ASN1_LIB i3ds_asn1 HINTS ${I3DS_FRAMEWORK_DIR} PATH_SUFFIXES external/lib)
find_path(I3DS_COMMUNICATION_HPP_DIR i3ds/communication.hpp HINTS ${I3DS_FRAMEWORK_DIR} PATH_SUFFIXES libi3ds/include)
find_path(I3DS_SENSOR_CLIENT_HPP_DIR i3ds/sensor_client.hpp HINTS ${I3DS_FRAMEWORK_DIR} PATH_SUFFIXES clients/include)
find_path(I3DS_ASN1_HPP_DIR i3ds_asn1/Common.hpp HINTS ${I3DS_FRAMEWORK_DIR} PATH_SUFFIXES external/include)

set(I3DS_LIBS ${I3DS_MAIN_LIB} ${I3DS_CLIENT_LIB} ${I3DS_ASN1_LIB})
set(I3DS_INCLUDE ${I3DS_COMMUNICATION_HPP_DIR} ${I3DS_ASN1_HPP_DIR} ${I3DS_SENSOR_CLIENT_HPP_DIR})

if(NOT(I3DS_MAIN_LIB AND I3DS_CLIENT_LIB AND I3DS_ASN1_LIB AND I3DS_COMMUNICATION_HPP_DIR AND I3DS_SENSOR_CLIENT_HPP_DIR AND I3DS_ASN1_HPP_DIR))
  MESSAGE(FATAL_ERROR "Could not find some of the required i3ds libraries: ${I3DS_LIBS} ${I3DS_INCLUDE}")
endif()


################################################################################
# Check that the C++ compiler supports C++11 or C++0X, else issue warning.
################################################################################

include (CheckCXXCompilerFlag REQUIRED)

function(set_cxx_flag FLAG)
  check_cxx_compiler_flag (${FLAG} CXX_SUPPORT_FLAG)
  if (${CXX_SUPPORT_FLAG})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}" PARENT_SCOPE)
  else ()
    message(FATAL_ERROR "Failed setting ${FLAG}, please update CMakeLists")
  endif ()
endfunction(set_cxx_flag)

if (NOT CMAKE_CXX_FLAGS)

  set (DEBUG $ENV{DEBUG})

  if (${DEBUG})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
  else ()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  endif ()

  check_cxx_compiler_flag ("-std=c++11" CXX_SUPPORT_STD_11)

  if (${CXX_SUPPORT_STD_11})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  else ()
    message (WARNING "${CMAKE_CXX_COMPILER} does not support -std=c++11 flag")
  endif ()

  check_cxx_compiler_flag ("-Wall" CXX_SUPPORT_WALL)

  if (${CXX_SUPPORT_WALL})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  endif ()


  check_cxx_compiler_flag ("-fpic" CXX_SUPPORT_FPIC_1)
  check_cxx_compiler_flag ("-fPIC" CXX_SUPPORT_FPIC_2)

  if (${CXX_SUPPORT_FPIC_1})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
  elseif (${CXX_SUPPORT_FPIC_2})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  else ()
    message (WARNING "${CMAKE_CXX_COMPILER} does not support -fpic or -fPIC flags")
  endif ()

endif (NOT CMAKE_CXX_FLAGS)

if (${BUILD_DEBUG})
  set_cxx_flag("-ggdb3")
  set_cxx_flag("-Wall")
  set_cxx_flag("-pedantic")
  set_cxx_flag("-Wextra")
  set_cxx_flag("-Werror")
  set_cxx_flag("-Wfatal-errors")
  set_cxx_flag("-fsanitize=address")
endif ()


################################################################################
# Add subdirectories with sources
################################################################################

add_subdirectory (src)
