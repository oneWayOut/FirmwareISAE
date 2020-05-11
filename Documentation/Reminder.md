make px4_sitl gazebo_plane 

机架配置： 2100 使用AETRFG.main.mix

参数中禁用解锁对Airspeed Sensor的检查。禁用解锁对usb连接的检查。

## TODO

1. change mixer;  8 channels
2. change controller
3. add info to orb msg

make px4_fmu-v5_fixedwing upload


2020.5.10
地面站查看servo有输出，但接舵机没反应。通过万用表测试，输出口电压为0，但RC in接收机处有电
2020.5.11
参数禁用电源检测解锁： CBRK_SUPPLY_CHK;  使用USB给飞控供电，使用UBEC给输出口供电，解锁后，舵机按预期动作。