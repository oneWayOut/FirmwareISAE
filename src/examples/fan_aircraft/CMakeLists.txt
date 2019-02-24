

px4_add_module(
	MODULE examples__fan_aircraft
	MAIN fan_aircraft
	STACK_MAIN 2000
	SRCS
		fan_aircraft.cpp
	DEPENDS
	)
