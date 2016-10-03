

#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#


find_package(IsaeComponent REQUIRED)
include(${ISAE_COMPONENTS_HELPER}/isae.cmake)
#include(../CMakeHelpers/isae.cmake)

find_package(PkgConfig)
pkg_check_modules(PC_MAVLINK QUIET mavlink)

isae_orocos_comp(LDC TYPE_DEPENDS Datalogger common sensor command
 DEPENDS
INTERNAL_LIB_DEPENDS component_tools
EXTRA_INCLUDE_DIRS ${PC_MAVLINK_INCLUDE_DIRS}/v1.0/pixhawk)
