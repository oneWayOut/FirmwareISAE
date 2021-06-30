1. input: deltaX, deltaY, dis;

2. mode: auto, stab, follow target;

3. message protocal, 消息精度，字节数，起始字节，终止字节；


make px4_fmu-v5_fixedwing   #pixhack v5;

make px4_sitl gazebo_plane_gdb


make px4_sitl gazebo_tiltrotor



## TODO 
 * mavlink test via tele2 with or without flow control: CTS, RTS;
 * mavlink msg format??? pymavlink?  https://mavlink.io/en/mavgen_python/  or https://github.com/mavlink/MAVSDK-Python
 * hardware connection?
 * 不用mavlink，直接使用串口? 需要测试!!!!  使用caiDrop分支上的代码，可以按V5配置编译
 * python or C language
 * use mavlink to print error msg;


https://docs.px4.io/master/en/peripherals/companion_computer_peripherals.html

## hardware setup
https://docs.px4.io/master/en/companion_computer/pixhawk_companion.html#hardware-setup

## software setup
https://docs.px4.io/master/en/companion_computer/pixhawk_companion.html#companion-computer-setup



搜索关键词 uav vision guide  formation
A guidance law for UAV autonomous aerial refueling based on the iterative computation method

https://www.sciencedirect.com/science/article/pii/S1000936114001113

2. vtol simulation, 有那些任务，启动执行顺序，参数文件；


3. 先做手动模式；

4. debug, 修改cmake, vscode?  gdb?

5. 最新版与flightgear交连  (子模块中有flightgear)





HEADLESS=1 make px4_sitl gazebo_plane  #Gazebo can be run in a headless mode in which the Gazebo UI is not launched. This starts up more quickly and uses less system resources



