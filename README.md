# funUAV

## TODO


 * hrt_elapsed_time; 参数的添加　0.5~1s;
  

 * remove or add modules in *nuttx_px4fmu-v2_default.cmake* 
   * add control law module
   * check magnetometer
   * disable distance_sensor
   * disable mc, fw, vtol;

 * change frequency of adc66v higher in sensors.cpp???? 

   uorb top  //TODO check delete useless topic publish; or change pub frequency

 * check sign of mix throttle in aileron




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



TEST  CBRK_USB_CHK for usb link arm




commander:

state_machine_helper.cpp : L993, press armed switch;

no print msg after press armed switch, but pwm output changed.

after give arm command to board:
commander.cpp L805  arm/disarm component command