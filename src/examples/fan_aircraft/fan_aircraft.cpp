
/**
 * @file fan_aircraft.c
 * fan craft control
 *
 * @author cai  cia120@163.com
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <matrix/matrix/helper_functions.hpp>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>


#include <parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>  //TODO delete
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/adc_66v_raw.h>

//TODO input :
#include <uORB/topics/manual_control_setpoint.h>  //remote controller
#include <uORB/topics/vehicle_control_mode.h>   //armed status
//other topics????
//can fill in  vehicle_rates_setpoint message with our sepecific stuff.
// OUTPUT:
#include <uORB/topics/actuator_controls.h>  //	_actuators_id = ORB_ID(actuator_controls_0);

//


#include "params.h"

using matrix::wrap_pi;


//param handle   refer to examples/fixedwing_control:
static struct fan_params        params_st;
static struct fan_param_handles p_h;

/* subscribe to sensor_combined topic */
static int sensor_sub;
static int adc_sub;
static int v_att_sub;
static int param_sub;

static int v_ctrl_mode_sub;	/**< vehicle control mode subscription */
static int m_ctrl_sp_sub;


static struct sensor_combined_s sensor_comb;   //TODO: delete
static struct adc_66v_raw_s  adc66v = {0};
static struct vehicle_attitude_s		v_att = {0};		/**< vehicle attitude */


static struct manual_control_setpoint_s m_ctrl_sp = {0};	/**< manual control setpoint */
static struct vehicle_control_mode_s    v_ctrl_mode = {0};	/**< vehicle control mode */
static struct actuator_controls_s       actuators0 = {0};		/**< actuator controls. NOTE: change to local?*/



static void parameters_init(void)
{
	p_h.cai_test_h   =  param_find("CAI_TESTPARAM");
	p_h.adc360_val_h =  param_find("CAI_ADC360_VAL");
}

static void parameters_update(void)
{
	param_get(p_h.cai_test_h,   &(params_st.cai_test));
	param_get(p_h.adc360_val_h, &(params_st.adc360_val));
}


static void poll_topic_msgs(void)
{

	bool _updated;


	orb_check(adc_sub, &_updated);
	if (_updated)
	{
		orb_copy(ORB_ID(adc_66v_raw), adc_sub, &adc66v);
		//TODO filter the value to degree
	}


	orb_check(sensor_sub, &_updated);
	if (_updated)
	{
		orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor_comb);
	}


	/* params update*/
	orb_check(param_sub, &_updated);
	if (_updated)
	{
		struct parameter_update_s params_update;
		orb_copy(ORB_ID(parameter_update), param_sub, &params_update);

		/* if a param update occured, re-read our parameters */
		//parameter test
		parameters_update();

		PX4_INFO("caiParam:\t%8.4f\n", (double)params_st.cai_test);
	}



	orb_check(v_ctrl_mode_sub, &_updated);
	if (_updated)
	{
		 orb_copy(ORB_ID(vehicle_control_mode), v_ctrl_mode_sub, &v_ctrl_mode);
	}

	orb_check(m_ctrl_sp_sub, &_updated);
	if (_updated)
	{
		 orb_copy(ORB_ID(manual_control_setpoint), m_ctrl_sp_sub, &m_ctrl_sp);
	}
}


extern "C" __EXPORT  int fan_aircraft_main(int argc, char *argv[]);

int fan_aircraft_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	hrt_abstime last_run = hrt_absolute_time();

	parameters_init();
	parameters_update();


	sensor_sub      = orb_subscribe(ORB_ID(sensor_combined));
	adc_sub         = orb_subscribe(ORB_ID(adc_66v_raw));
	v_att_sub       = orb_subscribe(ORB_ID(vehicle_attitude));
	param_sub       = orb_subscribe(ORB_ID(parameter_update));

	v_ctrl_mode_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	m_ctrl_sp_sub   = orb_subscribe(ORB_ID(vehicle_control_mode));


	orb_advert_t actuator0_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators0);


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = v_att_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};


	while (1) {
		/* wait for sensor update of 1 file descriptor for 80 ms */
		int poll_ret = px4_poll(fds, 1, 80);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");
			continue;
		} 

		if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			PX4_ERR("ERROR return value from poll(): %d", poll_ret);

			/* sleep a bit before next try */
			usleep(100000);
			continue;

		}


		/* only run controller if attitude changed */
		if (fds[0].revents & POLLIN) {

			//get dt, for integration
			const hrt_abstime now = hrt_absolute_time();

			//printf("mc: dt = %d\n", now-last_run);

			float dt = (now - last_run) / 1e6f;
			last_run = now;


			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;
			} else if (dt > 0.02f) {
				dt = 0.02f;
			}


			/* obtained data for the first file descriptor */
			orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);


			//poll other topic messages to our concern
			poll_topic_msgs();







			//TODO control law
			//rotation matrix:
			matrix::Dcmf _R = matrix::Quatf(v_att.q);
			matrix::Eulerf euler_angles(_R);
			//float _roll  = euler_angles.phi();
			//float _pitch = euler_angles.theta();
			float _yaw   = euler_angles.psi();

			//
			//

			float _yaw_rate_max = math::radians(100);  //in mc_att module is 200 deg/s;
			float yaw_target = wrap_pi(_yaw + _yaw_rate_max * m_ctrl_sp.r * dt);

			PX4_INFO("yaw_target: %8.4f\n", (double)yaw_target);
			


			//OUTPUT to actuators:
			orb_publish(ORB_ID(actuator_controls_0), actuator0_pub, &actuators0);





			//print example
			/*
			PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				 (double)sensor_comb.accelerometer_m_s2[0],
				 (double)sensor_comb.accelerometer_m_s2[1],
				 (double)sensor_comb.accelerometer_m_s2[2]);*/

		}

	}

	PX4_INFO("exiting");

	return 0;
}
