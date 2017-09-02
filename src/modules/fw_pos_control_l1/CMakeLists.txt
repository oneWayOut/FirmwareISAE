
px4_add_module(
	MODULE modules__fw_pos_control_l1
	MAIN fw_pos_control_l1
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		FixedwingPositionControl.cpp
		Landingslope.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
