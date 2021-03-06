#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(naxes_motion_control)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
#find_package(OROCOS-RTT QUIET)
#if (NOT OROCOS-RTT_FOUND)
#  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
#endif()
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    rtt_ros
        )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories( include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(${USE_OROCOS_INCLUDE_DIRS})
# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libcartesian_motion_control-types-<target>.so
# and installs in the directory lib/orocos/@target@/cartesian_motion_control/types/
#
# The header will go in include/orocos/cartesian_motion_control/types/cartesian_motion_control/cartesian_motion_control-types.hpp
# So you can #include <cartesian_motion_control/cartesian_motion_control-types.hpp>
#
#orocos_typegen_headers(include/cartesian_motion_control/cartesian_motion_control-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(
    INCLUDE_DIRS include
      DEPENDS rtt_ros
      CATKIN_DEPENDS tf_conversions )
