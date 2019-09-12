#### 注意接线不要错

## TODO:
	1. navigator 中加航点判断特殊处理;
	2. L1 accept radius
	3. check rate of vehicle_global_position(125Hz),  this matters the drop accuracy!!

## NOTE:
	1. 串口文件设备，read函数为阻塞调用

	2. upload arduino program:  unplug all connections!!!

	3. make file proto is at boards/....

	4. gdb 调试，[按官网说法](https://dev.px4.io/master/en/debug/simulation_debugging.html):
~~~
	 export PX4_NO_OPTIMIZATION='^modules__navigator;^modules__fw_att_control$'
~~~
		跳过特定模块优化，断点调试仍不一致。

		后来在CMakeLists.txt中加"COMPILE_FLAGS -O0"才解决问题。　cflag在/cmake/px4_add_common_flags.cmake中设置，
	问题：　1.默认优化值是多少???? 2.为何再次调试时需要关地面站，否则程序死，与地面站udp通信有关??
	3. 在devguide上提交issue，
	４．可否使用qt, nemiver调试?
	5. 调试程序中设置值，attcontrol模块没收到的疑问!!!


	Build Type: MinSizeRel　？？？

	优化级别与CMakeLists.txt中的CMAKE_BUILD_TYPE变量有关，cmake中RelWithDebInfo will add compiler flags for generating debug information (the -g flag for GCC / clang), and will result in debuggable, yet much larger binaries.　MinSizeRel will add compiler flags for generating more compact binaries (the -Os flag for GCC / clang), possibly on the expense of program speed.






