# 2019.10.7
 1. ERROR: 无法切至mission模式;
    遥控器切换至mission模式是打印Critical: REJECT AUTO MISSION；　应该是Commander.cpp L2822行处打印，从地面站切mission应该不会打印该消息; 
    可在State_machine_helper.cpp中的main_state_transition()函数中加入调试代码看看,位置无效导致的?
    利用好QGC的　Mavlink Console
 2. ERROR: 锁定后舵机进入极位；
　3. ERROR: 插入电源线后, 飞控没上电;
 4. WARNING: 高度保持得不好；
 5. NOTE: 虽然校准后有两个磁罗盘，但通过改参数(CAL_MAG_PRIME)使用内置磁罗盘，并禁用(CAL_MAG0_EN)外置磁罗盘

# 2019.9.28
396809   mag0
396825   mag1

上网查外置磁罗盘，C-RTK 9p　可能没有包含磁罗盘，

待测试，链接调试器，下次试飞或换一个gps试试
自主起降是否需要超声波??


