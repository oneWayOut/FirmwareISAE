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

isae_orocos_comp(LaserJNPK TYPE_DEPENDS Datalogger DEPENDS
						   INTERNAL_LIB_DEPENDS component_tools)

