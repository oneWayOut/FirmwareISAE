2019.11.5尝试调试，代码，ide调试不行，gdb调试也不行；


## TODO:
	1. navigator 中加航点判断特殊处理;
	2. L1 accept radius
	3. check rate of vehicle_global_position(125Hz),  this matters the drop accuracy!!
	4. 外置磁罗盘，校准, 方向必须一致 CUAV,
	5. 工控机上电后自动进入系统，不要用户名，密码，通过自启动脚本启动python程序;
	6. debug ide qt, 简化，使用一个目录??? 不用费时编译两次?
	7. 待确认截取图片尺寸是否正确cv2.imshow("2", imgray)

## NOTE:
	1. 串口文件设备，read函数为阻塞调用

	2. upload arduino program:  unplug all connections!!!

	3. make file proto is at boards/....

	4.　第一圈三个靶标仅将第一个作为航点加在航线中，作为开始识别的标识，过了该点即发送停止识别指令；第二圈三个均加在航线中；目前代码在接近DROP_TGTIDX_R2时，投放; 发布的position_setpoint_trip中目标点cur的经纬度是DROP_TGTIDX_R2+1点的经纬度

	5. IDE debug; cd ../Firmware-build
cmake ../Firmware -G "CodeBlocks - Unix Makefiles"



### 日志分析
过回放确定日志具体为哪一个，然后通过analysis.py分析对应的文件。

/media/cai/work/Coding/pyFlightAnalysis/src$ python3 analysis.py


	cheese 摄像头

### 仿真时设置飞机位置
	export PX4_HOME_LAT=34.6633145
	export PX4_HOME_LON=109.2372228

	export PX4_HOME_LAT=34.6753188
	export PX4_HOME_LON=109.38369484999998
	export PX4_HOME_ALT=364

### V1.9.2中消息的发布有时直接用如ORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub的对象，调用update函数，更加简洁；

### rangefinder数据发布后，ekf2模块获取，应该是融合至vehicle_local_position_s消息中后发布;





## LOG:

### 2019.11.2
　　　　自动起飞，航线飞行，投放均成功；
	
	发现问题或待办:
	1. 第二圈自动飞行时未按照航路飞，爬很高，怀疑是通过地面站上传航线出问题；rtk占用一定数传带宽？？
	2. 罗盘校准显示inconsitent;
	3. 准备降落时，航线偏离跑道，怀疑是航点定义偏差，有没有可能是磁罗盘原因？
	　　　下次试飞时，找跑道中两个点的位置连线；

	



### 2019.10.26
	10.22飞行，无法切mission; 接调试线后可切，自动起飞无响应；

	原因: navigator_main.cpp中px4_poll函数使用不当；导致导航任务一直死等。
	自动起飞无响应, 应设置RWTO等参数，令其为跑道起飞。
	使能超声波参数MBXX;  

	必须有takeoff点才能自动起飞；

### 2019.10.12
 FW_LND_USETER　Use terrain estimate (ground altitude from GPS) during landing; 应该设为false

### 2019.10.7
 1. ERROR: 无法切至mission模式;
    遥控器切换至mission模式是打印Critical: REJECT AUTO MISSION；　应该是Commander.cpp L2822行处打印，从地面站切mission应该不会打印该消息; 
    可在State_machine_helper.cpp中的main_state_transition()函数中加入调试代码看看,位置无效导致的?
    利用好QGC的　Mavlink Console

    
 2. ERROR: 锁定后舵机进入极位；
　3. ERROR: 插入电源线后, 飞控没上电;
 4. WARNING: 高度保持得不好；
 5. NOTE: 虽然校准后有两个磁罗盘，但通过改参数(CAL_MAG_PRIME)使用内置磁罗盘，并禁用(CAL_MAG0_EN)外置磁罗盘

### 2019.9.28
396809   mag0
396825   mag1

上网查外置磁罗盘，C-RTK 9p　可能没有包含磁罗盘，

待测试，链接调试器，下次试飞或换一个gps试试
自主起降是否需要超声波??




