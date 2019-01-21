/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

	 
#ifndef SDK_MIDDLEWARE_H_
#define SDK_MIDDLEWARE_H_

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <sys/statfs.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <float.h>
#include <stdint.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/task_stack_info.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>
#include <navigator/navigation.h>
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <systemlib/printload.h>
#include <systemlib/mavlink_log.h>
#include <version/version.h>


struct st_attitude_s
{
	float roll;/*rad*/
	float pitch;/*rad*/
	float yaw;/*rad*/
	float rollspeed;/*rad/s*/
	float pitchspeed;/*rad/s*/
	float yawspeed;/*rad/s*/
};

struct st_sensors_s
{
	float gyro_rad_s[3];/*rad/s*/
	float accelerometer_m_s2[3];/*m/s^2*/
	float magnetometer_ga[3];/*Gauss*/
	float baro_alt_meter;/*meter*/
	float baro_temp_celcius;/*celcius*/
	float baro_pressure_pa;/*pa*/
};

struct st_sl_status_s
{
	bool arm_state;/*应用层自己的锁，只能由命令加解锁，1为已解锁*/
	bool rc_signal_lost;/*遥控器信号是否丢失,1为丢失*/
	bool data_link_lost;/*地面站信号是否丢失，1为丢失*/
	float voltage_filtered_v;/*电池电压，单位是V*/
	bool ekf_is_rotary_wing;/*这决定了EKF的模式*/
};


struct st_controls_s {
	float control[14];/*14 channels of pwm output,-1\0\1 represent the mininum\neutral\maximum pulse width*/
};




extern int sl_app_main(int argc, char *argv[]);
extern int user1_1_main(int argc, char *argv[]);
extern int fun_user1_2(void);
extern int fun_user2_1(int argc, char *argv[]);
extern int fun_user2_2(void);

void api_set_ekf_rotary_wing(bool is_rotary_wing);


/******************
*	
*	函数名:api_send_controls_data
*	作用:输出pwm控制量
*	参数:14 channels of pwm output,-1\0\1 represent the mininum\neutral\maximum pulse width
*	注意；如果连续500ms未调用api_send_controls_data函数，pwm输出脉宽变为0.
*
*******************/
void api_send_controls_data(struct st_controls_s controls_value);

/*获取飞控数据api函数
*
*	参数:查看sdk.h或者对应的msg文件。
*	返回值:表示数据是否再上一次函数调用之后发生更新，要注意多任务对这个函数调用导致返回值总是为false的问题。
*
**********************/
extern bool api_get_sl_status(struct st_sl_status_s *pSlStatus);
extern bool api_get_rc_data(struct rc_channels_s *pRcData);/*成员含义查看rc_channels.msg*/
extern bool api_get_global_pos_data(struct vehicle_global_position_s *pGlobalPos);/*成员含义查看vehicle_global_position.msg*/
extern bool api_get_airspeed(struct airspeed_s *pAirSpd);/*成员含义查看airspeed.msg*/
extern bool api_get_gps_data(struct vehicle_gps_position_s *pGps);/*成员含义查看vehicle_gps_position.msg*/
extern bool api_get_sensors_data(struct st_sensors_s *pSens);/*数据更新频率200hz*/
extern bool api_get_att_data(struct st_attitude_s *pAtt);/*数据更新频率200hz*/
extern bool api_param_update(void);/*返回true表示参数有更新*/
extern bool api_get_new_airline(struct mission_s *pMission,bool *updated);/*updated成员返回航线是否被更新，返回true表明航线被更新且成功读取，struct mission_s成员定义查看mission.msg*/
extern bool api_get_way_point(int32_t index,struct mission_item_s *pMission_item);/*返回true表明成功保存第index个航点到pMission_item地址中*/
extern bool api_get_home_position(struct home_position_s *pHomePosition);/*获取home点，gps正常才有home点，返回值表明home点是否更新*/
extern void app_print_callback(void);/*在控制台敲命令"sdk status"会调用这个*/

extern sem_t _sdk_sem_newData;/*姿态数据有更新，这个信号量就释放,频率200HZ*/




#endif
