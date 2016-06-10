px4_add_module(
	MODULE examples__visionair_control
	MAIN ex_visionair_control
	STACK_MAIN 1200
	STACK_MAX 1300
	SRCS
		main.c
		params.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
