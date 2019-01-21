/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

/**
 * @file sdk.c
 *
 * Simple sdk middleware for flight data. 
 *
 * @author chenjian

 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#include <ctype.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <string.h>

#include <uORB/uORB.h>
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
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/home_position.h>

#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <systemlib/printload.h>
#include <systemlib/mavlink_log.h>
#include <version/version.h>

#include "sdk.h"


static bool main_thread_should_exit = false;		/**< Deamon exit flag */
static bool sdk_thread_running = false;			/**< Deamon status flag */

static perf_counter_t        _perf_att_copys;
static perf_counter_t        _perf_sens_copys;
static perf_counter_t        _perf_glo_pos_copys;
static perf_counter_t        _perf_gps_pos_copys;
static perf_counter_t        _perf_rc_copys;
static perf_counter_t        _perf_control_outputs;
static perf_counter_t        _perf_api_outputs;

struct sdk_orb_bufs_s
{
	struct vehicle_command_s buf_cmd;
	struct vehicle_status_s buf_status;
	struct sensor_combined_s buf_sensor;
	struct vehicle_attitude_s buf_att;
	struct vehicle_global_position_s buf_global_pos;
	struct vehicle_gps_position_s buf_gps_pos;
	struct rc_channels_s buf_rc;
	struct airspeed_s buf_airspeed; 
	struct battery_status_s buf_battery;
	struct telemetry_status_s buf_telemetry;
	struct system_power_s buf_system_power;
	struct actuator_controls_s buf_actuator_controls_0;
	struct actuator_controls_s buf_actuator_controls_1;
	struct vehicle_magnetometer_s buf_vehicle_magnetometer;
	struct vehicle_air_data_s buf_vehicle_air;
	struct vehicle_land_detected_s buf_land_detected;
	struct parameter_update_s buf_parameter_update;
	struct mission_s buf_mission;
	struct home_position_s buf_home_position;
	bool cmd_updated;
	bool status_updated;
	bool sensor_updated;
	bool att_updated;
	bool global_pos_updated;
	bool gps_pos_updated;
	bool rc_updated;
	bool airspeed_updated; 
	bool battery_updated;
	bool telemetry_updated;
	bool system_power_updated;
	bool vehicle_magnetometer_updated;
	bool vehicle_air_updated;
	bool parameter_updated;
	bool mission_updated;
	bool home_position_updated;
};

enum SDK_ORBSUB {
	SDK_SUB_CMD				= 0,
	SDK_SUB_STATUS			= 1,
	SDK_SUB_SENSOR			= 2,
	SDK_SUB_ATT				= 3,
	SDK_SUB_GLOBAL_POS		= 4,
	SDK_SUB_GPS_POS			= 5,
	SDK_SUB_RC				= 6,
	SDK_SUB_AIRSPEED		= 7,
	SDK_SUB_BATTERY			= 8,
	SDK_SUB_TELEMETRY		= 9,
	SDK_SUB_SYSTEM_POWER	= 10,
	SDK_SUB_MASTER_MAG		= 11,
	SDK_SUB_VEHICLE_BARO	= 12,
	SDK_SUB_PARAM_UPDATE	= 13,
	SDK_SUB_MISSION			= 14,
	SDK_SUB_HOME_POSITION	= 15,
	SDK_SUB_MAX
};


__EXPORT bool _sdk_smartLong_armed = false;
__EXPORT bool _sdk_ekf_is_rotary_wing = true;



static struct sdk_orb_bufs_s _sdk_orb_bufs;
static sem_t _sdk_sems_array[SDK_SUB_MAX];
static sem_t _sdk_output_sem;
sem_t _sdk_sem_newData;
static orb_advert_t _sdk_actuators_0 = NULL;
static orb_advert_t _sdk_actuators_1 = NULL;
static orb_advert_t _sdk_landDetectedPub = NULL;


int sdk_output_thread_main(int argc, char *argv[]);
static void sdk_status(void);
int sdk_input_thread_main(int argc, char *argv[]);/* Mainloop of sdk deamon.*/
__EXPORT int sdk_main(int argc, char *argv[]);




#if 0
struct orbs_buf_s{
	unsigned orbs_num;
	int **pSubs;
	sem_t **pSems;
	bool *pUpdated;
	void **data;
};
bool orb_buf_init(struct orb_buf_s *bufs,unsigned bufs_num)
{
	
}
bool recv_all_orb_to_buf(int *subs,unsigned subs_num,struct orb_buf_s *bufs)
{
	bool updated = false;
	for(unsigned i=0;i<subs_num;i++)
	{
		orb_check(subs[i].sub, &updated);
		if (updated)
		{
			sem_wait(&bufs[i].sem);
			orb_copy(bufs[i].meta,subs[i].sub,bufs.data);
			bufs[i].updated = true;
			sem_post(&bufs[i].sem);
		}	
	}
	return true;
}
#endif

int sdk_output_thread_main(int argc, char *argv[])/*Priority must be set to Max*/
{
	while (!main_thread_should_exit)
	{
		sem_wait(&_sdk_output_sem);
		if (_sdk_actuators_0 != NULL) {
			orb_publish(ORB_ID(actuator_controls_0), _sdk_actuators_0, &_sdk_orb_bufs.buf_actuator_controls_0);

		} else {
			_sdk_actuators_0 = orb_advertise(ORB_ID(actuator_controls_0), &_sdk_orb_bufs.buf_actuator_controls_0);
		}

		if (_sdk_actuators_1 != NULL) {
			orb_publish(ORB_ID(actuator_controls_1), _sdk_actuators_1, &_sdk_orb_bufs.buf_actuator_controls_1);

		} else {
			_sdk_actuators_1 = orb_advertise(ORB_ID(actuator_controls_1), &_sdk_orb_bufs.buf_actuator_controls_1);
		}
		perf_count(_perf_control_outputs);
	}
	return 0;
}

void api_set_ekf_rotary_wing(bool is_rotary_wing)
{
	_sdk_ekf_is_rotary_wing = is_rotary_wing;
}

void api_send_controls_data(struct st_controls_s controls_value)
{
	if(_sdk_output_sem.semcount < 0){
		/*This task does not require a mutex lock due to the setting of priority and scheduling mode*/
		_sdk_orb_bufs.buf_actuator_controls_1.timestamp = _sdk_orb_bufs.buf_actuator_controls_0.timestamp = hrt_absolute_time();		
		memcpy(&_sdk_orb_bufs.buf_actuator_controls_0.control[0], &controls_value.control[0], sizeof(float) * 8);
		memcpy(&_sdk_orb_bufs.buf_actuator_controls_1.control[0], &controls_value.control[8], sizeof(float) * 6);
		perf_count(_perf_api_outputs);
		sem_post(&_sdk_output_sem);
	}
	else{
		/*sdk_output_thread has the max priority,so semcont is always < 0*/
		printf("api_send_controls_data: _sdk_output_sem.semcount =%d\n",_sdk_output_sem.semcount);
		//sleep(1);
		//ASSERT(0);
	}
}


bool api_get_sl_status(struct st_sl_status_s *pSlStatus)
{
	bool ret = false;
	sem_wait(&_sdk_sems_array[SDK_SUB_STATUS]);
	pSlStatus->data_link_lost = _sdk_orb_bufs.buf_status.data_link_lost;
	pSlStatus->rc_signal_lost = _sdk_orb_bufs.buf_status.rc_signal_lost;
	ret = (ret || _sdk_orb_bufs.status_updated);
	 _sdk_orb_bufs.status_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_STATUS]);
	
	if(pSlStatus->arm_state != _sdk_smartLong_armed){
		pSlStatus->arm_state = _sdk_smartLong_armed;
		ret = true;
	}
	pSlStatus->ekf_is_rotary_wing = _sdk_ekf_is_rotary_wing;//Internal value,don't need to change return value.

	sem_wait(&_sdk_sems_array[SDK_SUB_BATTERY]);
	pSlStatus->voltage_filtered_v = _sdk_orb_bufs.buf_battery.voltage_filtered_v; 
	ret = (ret || _sdk_orb_bufs.battery_updated);
	_sdk_orb_bufs.battery_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_BATTERY]);

	return ret;
}


bool api_get_rc_data(struct rc_channels_s *pRcData)
{
	bool ret;

	sem_wait(&_sdk_sems_array[SDK_SUB_RC]);
	memcpy(pRcData, &_sdk_orb_bufs.buf_rc, sizeof(_sdk_orb_bufs.buf_rc));
	ret = _sdk_orb_bufs.rc_updated;
	_sdk_orb_bufs.rc_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_RC]);
	
	return ret;
}



bool api_get_global_pos_data(struct vehicle_global_position_s *pGlobalPos)
{
	bool ret;

	sem_wait(&_sdk_sems_array[SDK_SUB_GLOBAL_POS]);
	memcpy(pGlobalPos, &_sdk_orb_bufs.buf_global_pos, sizeof(_sdk_orb_bufs.buf_global_pos));
	ret = _sdk_orb_bufs.global_pos_updated;
	_sdk_orb_bufs.global_pos_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_GLOBAL_POS]);
	
	return ret;
}



bool api_get_airspeed(struct airspeed_s *pAirSpd)
{
	bool ret;

	sem_wait(&_sdk_sems_array[SDK_SUB_AIRSPEED]);
	memcpy(pAirSpd, &_sdk_orb_bufs.buf_airspeed, sizeof(_sdk_orb_bufs.airspeed_updated));
	ret = _sdk_orb_bufs.airspeed_updated;
	_sdk_orb_bufs.airspeed_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_AIRSPEED]);
	
	return ret;
}


bool api_get_gps_data(struct vehicle_gps_position_s *pGps)
{
	bool ret;

	sem_wait(&_sdk_sems_array[SDK_SUB_GPS_POS]);
	memcpy(pGps, &_sdk_orb_bufs.buf_gps_pos, sizeof(_sdk_orb_bufs.buf_gps_pos));
	ret = _sdk_orb_bufs.gps_pos_updated;
	_sdk_orb_bufs.gps_pos_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_GPS_POS]);
	
	return ret;
}


bool api_get_sensors_data(struct st_sensors_s *pSens)
{
	bool ret;
	
	sem_wait(&_sdk_sems_array[SDK_SUB_SENSOR]);
	for(unsigned i=0;i<3;i++){
		pSens->accelerometer_m_s2[i] = _sdk_orb_bufs.buf_sensor.accelerometer_m_s2[i];
		pSens->gyro_rad_s[i] = _sdk_orb_bufs.buf_sensor.gyro_rad[i];		
	}
	ret = _sdk_orb_bufs.sensor_updated;
	_sdk_orb_bufs.sensor_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_SENSOR]);

	sem_wait(&_sdk_sems_array[SDK_SUB_MASTER_MAG]);
	pSens->magnetometer_ga[0] = _sdk_orb_bufs.buf_vehicle_magnetometer.magnetometer_ga[0];
	pSens->magnetometer_ga[1] = _sdk_orb_bufs.buf_vehicle_magnetometer.magnetometer_ga[1];
	pSens->magnetometer_ga[2] = _sdk_orb_bufs.buf_vehicle_magnetometer.magnetometer_ga[2];
	_sdk_orb_bufs.vehicle_magnetometer_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_MASTER_MAG]);

	sem_wait(&_sdk_sems_array[SDK_SUB_VEHICLE_BARO]);
	pSens->baro_alt_meter = _sdk_orb_bufs.buf_vehicle_air.baro_alt_meter;
	pSens->baro_pressure_pa = _sdk_orb_bufs.buf_vehicle_air.baro_pressure_pa;
	pSens->baro_temp_celcius = _sdk_orb_bufs.buf_vehicle_air.baro_temp_celcius;
	_sdk_orb_bufs.vehicle_air_updated = false;
	sem_post(&_sdk_sems_array[SDK_SUB_VEHICLE_BARO]);	

	return ret;
}



bool api_get_att_data(struct st_attitude_s *pAtt)
{
	bool ret;
	sem_wait(&_sdk_sems_array[SDK_SUB_ATT]);
	
	float q0 = _sdk_orb_bufs.buf_att.q[0];
	float q1 = _sdk_orb_bufs.buf_att.q[1];
	float q2 = _sdk_orb_bufs.buf_att.q[2];
	float q3 = _sdk_orb_bufs.buf_att.q[3];
	pAtt->roll =  atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
	pAtt->pitch = asinf(2*(q0*q2 - q3*q1));
	pAtt->yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));;
	pAtt->rollspeed = _sdk_orb_bufs.buf_att.rollspeed;
	pAtt->pitchspeed = _sdk_orb_bufs.buf_att.pitchspeed;
	pAtt->yawspeed =  _sdk_orb_bufs.buf_att.yawspeed;
	ret = _sdk_orb_bufs.att_updated;
	
	_sdk_orb_bufs.att_updated = false;
  	sem_post(&_sdk_sems_array[SDK_SUB_ATT]);
	return ret;
}

bool api_param_update(void)
{
	bool ret;
	sem_wait(&_sdk_sems_array[SDK_SUB_PARAM_UPDATE]);
	ret = _sdk_orb_bufs.parameter_updated;
	_sdk_orb_bufs.parameter_updated = false;
  	sem_post(&_sdk_sems_array[SDK_SUB_PARAM_UPDATE]);
	return ret;
}

bool api_get_new_airline(struct mission_s *pMission,bool *updated)
{
	sem_wait(&_sdk_sems_array[SDK_SUB_MISSION]);
	//memcpy(pMission, &_sdk_orb_bufs.buf_mission, sizeof(_sdk_orb_bufs.buf_mission));
	*updated = _sdk_orb_bufs.mission_updated;
	_sdk_orb_bufs.mission_updated = false;
  	sem_post(&_sdk_sems_array[SDK_SUB_MISSION]);

	if(true == *updated){
		/* lock MISSION_STATE item */
		int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);
		if (dm_lock_ret != 0) {
			PX4_ERR("DM_KEY_MISSION_STATE lock failed");
		}
		int dm_read_size = dm_read(DM_KEY_MISSION_STATE, 0, pMission, sizeof(struct mission_s));
		/* unlock MISSION_STATE item */
		if (dm_lock_ret == 0) {
			dm_unlock(DM_KEY_MISSION_STATE);
		}
		
		if(dm_read_size == sizeof(struct mission_s)){
			if ((pMission->dataman_id != DM_KEY_WAYPOINTS_OFFBOARD_0) && (pMission->dataman_id != DM_KEY_WAYPOINTS_OFFBOARD_1)){
				pMission->timestamp = hrt_absolute_time();
				pMission->dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
				dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, pMission, sizeof(struct mission_s));
				orb_advert_t mission_pub = orb_advertise(ORB_ID(mission), pMission);
				orb_unadvertise(mission_pub);				
				printf("sdk.c: dataman_id is not offboard !!!\n");
			}
			memcpy(&_sdk_orb_bufs.buf_mission, pMission, sizeof(struct mission_s));
			return true;
		}
	}
	
	return false;
}

bool api_get_way_point(int32_t index,struct mission_item_s *pMission_item)
{
	ssize_t retlen = 0;
	bool ret = false;

	sem_wait(&_sdk_sems_array[SDK_SUB_MISSION]);
	if(index >= _sdk_orb_bufs.buf_mission.count){
		ret = -1;
	}
	else{
		retlen = dm_read(_sdk_orb_bufs.buf_mission.dataman_id,index,pMission_item,sizeof(struct mission_item_s));
		if(retlen == sizeof(struct mission_item_s)){
			ret = true;
		}
		else{
			ret = false;
		}
	}
	sem_post(&_sdk_sems_array[SDK_SUB_MISSION]);
	return ret;
}

bool api_get_home_position(struct home_position_s *pHomePosition)
{
	bool ret;
	sem_wait(&_sdk_sems_array[SDK_SUB_HOME_POSITION]);
	memcpy(pHomePosition,&_sdk_orb_bufs.buf_home_position,sizeof(struct home_position_s));
	ret = _sdk_orb_bufs.home_position_updated;
	_sdk_orb_bufs.home_position_updated = false;
  	sem_post(&_sdk_sems_array[SDK_SUB_HOME_POSITION]);
	return ret;
}

static void sdk_status()
{
	perf_print_counter(_perf_att_copys);
	perf_print_counter(_perf_sens_copys);
	perf_print_counter(_perf_glo_pos_copys);
	perf_print_counter(_perf_gps_pos_copys);
	perf_print_counter(_perf_rc_copys);
	perf_print_counter(_perf_control_outputs);
	perf_print_counter(_perf_api_outputs);
	printf("lock state:%s\t",_sdk_smartLong_armed ? "armed" :"disarm");
	printf("ekf wing type:%s\n",_sdk_ekf_is_rotary_wing ? "rotary_wing" :"fix_wing");
}


static void
sdk_usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
	PX4_WARN("usage: sdk {start|stop|status}\n");
}



int sdk_input_thread_main(int argc, char *argv[])
{
	int subs[SDK_SUB_MAX];
	memset(subs, -1, sizeof(subs));

	subs[SDK_SUB_AIRSPEED] = orb_subscribe(ORB_ID(airspeed));
	subs[SDK_SUB_ATT] = orb_subscribe(ORB_ID(vehicle_attitude));
	subs[SDK_SUB_STATUS] = orb_subscribe(ORB_ID(battery_status));
	subs[SDK_SUB_CMD] = orb_subscribe(ORB_ID(vehicle_command));
	subs[SDK_SUB_GLOBAL_POS] = orb_subscribe(ORB_ID(vehicle_global_position));
	subs[SDK_SUB_GPS_POS] = orb_subscribe(ORB_ID(vehicle_gps_position));
	subs[SDK_SUB_RC] = orb_subscribe(ORB_ID(rc_channels));
	subs[SDK_SUB_SENSOR] = orb_subscribe(ORB_ID(sensor_combined));
	subs[SDK_SUB_STATUS] = orb_subscribe(ORB_ID(vehicle_status));
	//subs[SDK_SUB_SYSTEM_POWER] = orb_subscribe(ORB_ID(system_power));
	subs[SDK_SUB_TELEMETRY] = orb_subscribe(ORB_ID(telemetry_status));
	subs[SDK_SUB_MASTER_MAG] = orb_subscribe(ORB_ID(vehicle_magnetometer));
	subs[SDK_SUB_VEHICLE_BARO] = orb_subscribe(ORB_ID(vehicle_air_data));
	subs[SDK_SUB_PARAM_UPDATE]= orb_subscribe(ORB_ID(parameter_update));
	subs[SDK_SUB_MISSION] = orb_subscribe(ORB_ID(mission));
	subs[SDK_SUB_HOME_POSITION] = orb_subscribe(ORB_ID(home_position));

	orb_set_interval(subs[SDK_SUB_SENSOR], 5);//decrease to 200hz
	

	memset(&_sdk_orb_bufs, 0, sizeof(_sdk_orb_bufs));
	_sdk_orb_bufs.parameter_updated = true;/*刚开始启动默认参数已更新，以便应用层读取*/
	_sdk_orb_bufs.mission_updated = true;/*方便应用层在刚启动的时候更新航点*/

	_perf_att_copys = perf_alloc(PC_COUNT, "att_copys");
	_perf_sens_copys = perf_alloc(PC_COUNT, "sens_copys");
	_perf_glo_pos_copys = perf_alloc(PC_COUNT, "glo_pos_copys");
	_perf_gps_pos_copys = perf_alloc(PC_COUNT, "gps_pos_copys");
	_perf_rc_copys = perf_alloc(PC_COUNT, "rc_copys");
	_perf_control_outputs = perf_alloc(PC_COUNT, "control_outputs");
	_perf_api_outputs = perf_alloc(PC_COUNT, "perf_api_outputs");


	for(unsigned i=0;i<SDK_SUB_MAX;i++)
	{
		sem_init(&_sdk_sems_array[i], 0, 1);/*互斥型*/
	}
	sem_init(&_sdk_output_sem,0,0);/*触发型*/
	sem_init(&_sdk_sem_newData,0,0);
	
	
	/* wakeup source */
	px4_pollfd_struct_t poll_fds[1];
	poll_fds[0].fd = subs[SDK_SUB_SENSOR];/*sensors orb更稳定，更可靠，更适合做时钟*/
	poll_fds[0].events = POLLIN;

	sdk_thread_running = true;
	while (!main_thread_should_exit)
	{
		/* wait for up to 20ms for data*/
		int pret = px4_poll(&poll_fds[0], 1, 20);

		/* if pret == 0 it timed out - periodic check for should_exit(), etc. */
		if (pret <=0) {
			
			PX4_WARN("sdk task can't get sensors data\n");
			usleep(1000000);
			continue;
		}

		bool updated = false;
		orb_check(subs[SDK_SUB_AIRSPEED], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_AIRSPEED]);
			orb_copy(ORB_ID(airspeed), subs[SDK_SUB_AIRSPEED], &_sdk_orb_bufs.buf_airspeed);
			_sdk_orb_bufs.airspeed_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_AIRSPEED]);
		}

		updated = false;
		orb_check(subs[SDK_SUB_ATT], &updated);/*注:刚上电有6秒时间ekf没有启动导致att收不到数据*/
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_ATT]);
			orb_copy(ORB_ID(vehicle_attitude), subs[SDK_SUB_ATT], &_sdk_orb_bufs.buf_att);
			_sdk_orb_bufs.att_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_ATT]);

			perf_count(_perf_att_copys);
		}

		
		updated = false;
		orb_check(subs[SDK_SUB_BATTERY], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_BATTERY]);
			orb_copy(ORB_ID(battery_status), subs[SDK_SUB_BATTERY], &_sdk_orb_bufs.buf_battery);
			_sdk_orb_bufs.battery_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_BATTERY]);
		}

		
		updated = false;
		orb_check(subs[SDK_SUB_CMD], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_CMD]);
			orb_copy(ORB_ID(vehicle_command), subs[SDK_SUB_CMD], &_sdk_orb_bufs.buf_cmd);
			_sdk_orb_bufs.cmd_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_CMD]);
		}

		updated = false;
		orb_check(subs[SDK_SUB_GLOBAL_POS], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_GLOBAL_POS]);
			orb_copy(ORB_ID(vehicle_global_position), subs[SDK_SUB_GLOBAL_POS], &_sdk_orb_bufs.buf_global_pos);
			_sdk_orb_bufs.global_pos_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_GLOBAL_POS]);

			perf_count(_perf_glo_pos_copys);
		}

		updated = false;
		orb_check(subs[SDK_SUB_GPS_POS], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_GPS_POS]);
			orb_copy(ORB_ID(vehicle_gps_position), subs[SDK_SUB_GPS_POS], &_sdk_orb_bufs.buf_gps_pos);
			_sdk_orb_bufs.gps_pos_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_GPS_POS]);

			perf_count(_perf_gps_pos_copys);
		}

		updated = false;
		orb_check(subs[SDK_SUB_RC], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_RC]);
			orb_copy(ORB_ID(rc_channels), subs[SDK_SUB_RC], &_sdk_orb_bufs.buf_rc);
			_sdk_orb_bufs.rc_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_RC]);

			perf_count(_perf_rc_copys);
		}

		updated = false;
		orb_check(subs[SDK_SUB_SENSOR], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_SENSOR]);
			orb_copy(ORB_ID(sensor_combined), subs[SDK_SUB_SENSOR], &_sdk_orb_bufs.buf_sensor);
			_sdk_orb_bufs.sensor_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_SENSOR]);

			perf_count(_perf_sens_copys);
		}

		updated = false;
		orb_check(subs[SDK_SUB_STATUS], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_STATUS]);
			orb_copy(ORB_ID(vehicle_status), subs[SDK_SUB_STATUS], &_sdk_orb_bufs.buf_status);
			_sdk_orb_bufs.status_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_STATUS]);
		}

		/*updated = false;
		orb_check(subs[SDK_SUB_SYSTEM_POWER], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_SYSTEM_POWER]);
			orb_copy(ORB_ID(system_power), subs[SDK_SUB_SYSTEM_POWER], &_sdk_orb_bufs.buf_system_power);
			_sdk_orb_bufs.system_power_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_SYSTEM_POWER]);
		}*/

		updated = false;
		orb_check(subs[SDK_SUB_TELEMETRY], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_TELEMETRY]);
			orb_copy(ORB_ID(telemetry_status), subs[SDK_SUB_TELEMETRY], &_sdk_orb_bufs.buf_telemetry);
			_sdk_orb_bufs.telemetry_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_TELEMETRY]);
		}

		updated = false;
		orb_check(subs[SDK_SUB_MASTER_MAG], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_MASTER_MAG]);
			orb_copy(ORB_ID(vehicle_magnetometer), subs[SDK_SUB_MASTER_MAG], &_sdk_orb_bufs.buf_vehicle_magnetometer);
			_sdk_orb_bufs.vehicle_magnetometer_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_MASTER_MAG]);
		}

		updated = false;
		orb_check(subs[SDK_SUB_VEHICLE_BARO], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_VEHICLE_BARO]);
			orb_copy(ORB_ID(vehicle_air_data), subs[SDK_SUB_VEHICLE_BARO], &_sdk_orb_bufs.buf_vehicle_air);
			_sdk_orb_bufs.vehicle_air_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_VEHICLE_BARO]);
		}		

		if(_sdk_sem_newData.semcount <= 0){
			sem_post(&_sdk_sem_newData);
		}

		/*对实时性要求不高的放在下面*/
		if(hrt_elapsed_time(&_sdk_orb_bufs.buf_land_detected.timestamp) >= 1000000){
			_sdk_orb_bufs.buf_land_detected.timestamp = hrt_absolute_time();
			_sdk_orb_bufs.buf_land_detected.landed = !_sdk_smartLong_armed;
			_sdk_orb_bufs.buf_land_detected.freefall = false;
			_sdk_orb_bufs.buf_land_detected.ground_contact = false;
			_sdk_orb_bufs.buf_land_detected.maybe_landed = false;
			if (_sdk_landDetectedPub != NULL) {
				orb_publish(ORB_ID(vehicle_land_detected), _sdk_landDetectedPub, &_sdk_orb_bufs.buf_land_detected);
			} else {
				_sdk_landDetectedPub = orb_advertise(ORB_ID(vehicle_land_detected), &_sdk_orb_bufs.buf_land_detected);
			}
		}
		

		updated = false;
		orb_check(subs[SDK_SUB_PARAM_UPDATE], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_PARAM_UPDATE]);
			orb_copy(ORB_ID(parameter_update), subs[SDK_SUB_PARAM_UPDATE], &_sdk_orb_bufs.buf_parameter_update);
			_sdk_orb_bufs.parameter_updated = true;
			sem_post(&_sdk_sems_array[SDK_SUB_PARAM_UPDATE]);
		}

		
		updated = false;
		orb_check(subs[SDK_SUB_MISSION], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_MISSION]);
			orb_copy(ORB_ID(mission), subs[SDK_SUB_MISSION], &_sdk_orb_bufs.buf_mission);
			_sdk_orb_bufs.mission_updated = true;
			printf("sdk.c:i recived new mission\n");
			sem_post(&_sdk_sems_array[SDK_SUB_MISSION]);
		}

		updated = false;
		orb_check(subs[SDK_SUB_HOME_POSITION], &updated);
		if (updated)
		{
			sem_wait(&_sdk_sems_array[SDK_SUB_HOME_POSITION]);
			orb_copy(ORB_ID(home_position), subs[SDK_SUB_HOME_POSITION], &_sdk_orb_bufs.buf_home_position);
			_sdk_orb_bufs.home_position_updated = true;
			printf("sdk.c:i recived new home position\n");
			sem_post(&_sdk_sems_array[SDK_SUB_HOME_POSITION]);
		}
	}

	for(unsigned i=0;i<SDK_SUB_MAX;i++)
	{
		orb_unsubscribe(subs[i]);
		sem_destroy(&_sdk_sems_array[i]);
	}
	sem_destroy(&_sdk_output_sem);
	sem_destroy(&_sdk_sem_newData);
	perf_free(_perf_att_copys);
	perf_free(_perf_sens_copys);
	perf_free(_perf_glo_pos_copys);
	perf_free(_perf_gps_pos_copys);
	perf_free(_perf_rc_copys);
	perf_free(_perf_control_outputs);
	perf_free(_perf_api_outputs);

	sdk_thread_running = false;
	return 0;
}



int sdk_main(int argc, char *argv[])
{
	if (argc < 2) {
		sdk_usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (sdk_thread_running) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}


		int task_priority = SCHED_PRIORITY_DEFAULT + (SCHED_PRIORITY_MAX - SCHED_PRIORITY_DEFAULT) / 2;

		main_thread_should_exit = false;
		px4_task_spawn_cmd("sdk_input_thread",
						 SCHED_PRIORITY_MAX - 10,
						 task_priority,
						 3400,
						 sdk_input_thread_main,
						 (char * const *)argv);

		/* wait for the task to launch */
		unsigned const max_wait_us = 1000000;
		unsigned const max_wait_steps = 2000;

		unsigned i;
		for (i = 0; i < max_wait_steps; i++) {
			usleep(max_wait_us / max_wait_steps);
			if (sdk_thread_running) {
				break;
			}
		}

		if(i < max_wait_steps){
			px4_task_spawn_cmd("sdk_output_thread",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX,/*Never change the priority*/
						 1200,
						 sdk_output_thread_main,
						 (char * const *)argv);

			px4_task_spawn_cmd("sl_app_main",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 3400,
						 sl_app_main,
						 (char * const *)argv);
		}

		return !(i < max_wait_steps);
	}

	/*if (!strcmp(argv[1], "stop")) {
		if (!sdk_thread_running) {
			PX4_WARN("not started");
		}

		main_thread_should_exit = true;
		return 0;
	}*/

	if (!sdk_thread_running) {
		PX4_WARN("not started\n");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		sdk_status();
		app_print_callback();
		return 0;
	}


	if (!strcmp(argv[1], "arm")) {
		if(true == _sdk_smartLong_armed){
			PX4_WARN("sdk: adk already armed\n");
			return 1;
		}
		
		if(_sdk_orb_bufs.buf_status.arming_state == VEHICLE_STATUS_ARMING_STATE_ARMED){
			_sdk_smartLong_armed = true;
		}else{
			char output_str[30];
			switch(_sdk_orb_bufs.buf_status.arming_state){
				case VEHICLE_STATUS_ARMING_STATE_INIT:
					sprintf(output_str,"init");
					break;
				case VEHICLE_STATUS_ARMING_STATE_STANDBY:
					sprintf(output_str,"standby");
					break;
				case VEHICLE_STATUS_ARMING_STATE_STANDBY_ERROR:
					sprintf(output_str,"standby error");
					break;
				case VEHICLE_STATUS_ARMING_STATE_REBOOT:
					sprintf(output_str,"reboot");
					break;
				case VEHICLE_STATUS_ARMING_STATE_IN_AIR_RESTORE:
					sprintf(output_str,"air restore");
					break;
				default:
					sprintf(output_str,"unknow");
					break;
			}
			PX4_WARN("sdk: pix is in %s state,don't allow arm",output_str);
			return 1;
		}
		return 0;
	}

	if (!strcmp(argv[1], "disarm")) {
		if(false == _sdk_smartLong_armed){
			PX4_WARN("sdk: adk not arm\n");
			return 1;
		}
		_sdk_smartLong_armed = false;

		/* send this to itself */
		param_t sys_id_param = param_find("MAV_SYS_ID");
		param_t comp_id_param = param_find("MAV_COMP_ID");
		int32_t sys_id;
		int32_t comp_id;
		if (param_get(sys_id_param, &sys_id)) {
			errx(1, "PRM SYSID");
		}
		if (param_get(comp_id_param, &comp_id)) {
			errx(1, "PRM CMPID");
		}
		struct vehicle_command_s cmd;
		memset(&cmd, 0, sizeof(cmd));
		cmd.timestamp = hrt_absolute_time(),
		cmd.param5 = 0.0f;
		cmd.param6 = 0.0f;
		/* request disarm */
		cmd.param1 = 0.0f;
		cmd.param2 = 0.0f;
		cmd.param3 = 0.0f;
		cmd.param4 = 0.0f;
		cmd.param7 = 0.0f;
		cmd.command = VEHICLE_COMMAND_VEHICLE_CMD_COMPONENT_ARM_DISARM;
		cmd.target_system = (uint8_t)sys_id;
		cmd.target_component = (uint8_t)comp_id;
		cmd.source_system = (uint8_t)sys_id +1;
		cmd.source_component = (uint8_t)comp_id +1;
		/* ask to confirm command */
		cmd.confirmation = 1;
		/* send command once */
		orb_advert_t cmd_adv = orb_advertise(ORB_ID(vehicle_command), &cmd);
		orb_unadvertise(cmd_adv);
		return 0;
	}
	
	sdk_usage("unrecognized command");
	return 1;
}
