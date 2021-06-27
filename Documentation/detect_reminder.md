1. input: deltaX, deltaY, dis;

2. mode: auto, stab, follow target;

3. message protocal, 消息精度，字节数，起始字节，终止字节；



搜索关键词 uav vision guide  formation
A guidance law for UAV autonomous aerial refueling based on the iterative computation method

https://www.sciencedirect.com/science/article/pii/S1000936114001113

2. vtol simulation, 有那些任务，启动执行顺序，参数文件；


3. 先做手动模式；

4. debug, 修改cmake, vscode?  gdb?

5. 最新版与flightgear交连  (子模块中有flightgear)



make px4_sitl gazebo_plane_gdb


make px4_sitl gazebo_tiltrotor

HEADLESS=1 make px4_sitl gazebo_plane  #Gazebo can be run in a headless mode in which the Gazebo UI is not launched. This starts up more quickly and uses less system resources




"/media/cai/work/Coding/Firmware/build/px4_sitl_default/bin/px4" "/media/cai/work/Coding/Firmware"/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t "/media/cai/work/Coding/Firmware"/test_data


"/media/cai/work/Coding/build-Firmware-Desktop-Default/bin/px4" "/media/cai/work/Coding/Firmware"/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t "/media/cai/work/Coding/Firmware"/test_data



"/media/cai/work/Coding/Firmware-build/bin/px4" "/media/cai/work/Coding/Firmware"/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t "/media/cai/work/Coding/Firmware"/test_data