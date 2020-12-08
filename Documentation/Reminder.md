## Reminder

### Mode 
 手动控位: Manual Mode
 手动控姿: Rattitude mode

### Build

~~~
make px4_fmu-v5_fixedwing upload
make px4_sitl gazebo_plane 
~~~

机架配置： 2100 使用AETRFG.main.mix

参数中禁用解锁对Airspeed Sensor的检查。禁用解锁对usb连接的检查。

## TODO

1. 增加用于手动控位的参数
2. change controller





2020.5.10
地面站查看servo有输出，但接舵机没反应。通过万用表测试，输出口电压为0，但RC in接收机处有电
2020.5.11
参数禁用电源检测解锁： CBRK_SUPPLY_CHK;  使用USB给飞控供电，使用UBEC给输出口供电，解锁后，舵机按预期动作。

2020.5.13
手动控位: Manual Mode
手动控姿: Rattitude mode