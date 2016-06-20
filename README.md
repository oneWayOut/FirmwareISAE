## DCAS Pixhawk Project ##

This repository holds the VisionAir flight control source code based on [PX4 flight stack](http://px4.io). 
Developed by DCAS of [ISAE Supaero](http://isae.fr/)

### Build guide

Install the needed toolchain as [here](http://dev.px4.io/starting-installing-linux.html) first. 
 
Then clone the code and compile it as below:

git clone https://openforge.isae.fr/git/visionair 

cd visionair 

git checkout -b caidev origin/caidev

git submodule update --init --recursive 

make px4fmu-v2_default


### Users ###

Use [QGround Control V2.9.7b Release](https://github.com/mavlink/qgroundcontrol/releases/tag/v2.9.7b) please.

### Developers ###

  * Keep updated with the latest px4 project(TODO).
  * [Developer Forum / Mailing list](http://groups.google.com/group/px4users)
  * [Guide for Contributions](https://github.com/PX4/Firmware/blob/master/CONTRIBUTING.md)
  * [Developer guide](http://dev.px4.io)



