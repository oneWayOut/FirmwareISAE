# funUAV

## TODO
 * buy wire.
 * Control Law
 * 锁尾控制
 * rcS 文件修改
 * remove or add modules in *nuttx_px4fmu-v2_default.cmake* 
   * add control law module
   * check magnetometer
   * disable distance_sensor
   * disable mc, fw, vtol;


   output channels :  4 servos, 1 rotor for tail lock: yaw control, 1 rotor for thrust.




 original diff_pres.error_count is useless


## reminder 

 * mixer 6001_hexa_x;

 * update rate of ORB_ID(vehicle_attitude) is about 250Hz, which is tested at FixedwingAttitudeControl.cpp.

 * usefull build commands:

 ~~~
make posix_sitl_default jmavsim
make posix_sitl_default gazebo
make px4fmu-v4_default
make px4fmu-v2_default
 ~~~

 * screen :

[Connec via FTDI 3.3V Cable to serial 4/5 in pixhawk1](https://dev.px4.io/en/debug/system_console.html)
use ls /dev/tty* and watch what changes when unplugging / replugging the USB device). Common names are /dev/ttyUSB0 and /dev/ttyACM0 for Linux and /dev/tty.usbserial-ABCBD for Mac OS.

 ~~~
screen /dev/ttyUSB0 57600 8N1
 ~~~

