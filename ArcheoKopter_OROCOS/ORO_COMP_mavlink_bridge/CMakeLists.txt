cmake_minimum_required(VERSION 2.6.3)

include(../CMakeHelpers/isae.cmake)

find_package(PkgConfig)
pkg_check_modules(PC_MAVLINK QUIET mavlink)

isae_orocos_comp(mavlink_bridge TYPE_DEPENDS Datalogger FullGPS common sensor command
								INTERNAL_LIB_DEPENDS component_tools 
								EXTRA_INCLUDE_DIRS ${PC_MAVLINK_INCLUDE_DIRS}/v1.0/pixhawk)
