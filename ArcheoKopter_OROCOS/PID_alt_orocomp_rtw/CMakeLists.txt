# OROCOS component generated from Simulink Model
# Date  : 26-Aug-2016
# Model : PID_alt

cmake_minimum_required(VERSION 2.6.3)

find_package(IsaeComponent REQUIRED)

include(${ISAE_COMPONENTS_HELPER}/isae.cmake)

isae_orocos_comp(PID_alt TYPE_DEPENDS common command sensor BlockUDP DEPENDS)
