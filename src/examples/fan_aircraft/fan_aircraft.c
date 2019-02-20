
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

#include <parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/adc_66v_raw.h>




#include "params.h"




//param handle   refer to examples/fixedwing_control:
static param_t temp_p;  

/* subscribe to sensor_combined topic */
static int sensor_sub;
static int adc_sub;
static int v_att_sub;
static int param_sub;


static struct sensor_combined_s sensor_comb;
static struct adc_66v_raw_s  adc66v = {0};
static struct vehicle_attitude_s		v_att = {0};		/**< vehicle attitude */



static void parameters_init(void)
{
	/* PID parameters */
	temp_p 	=	param_find("CAI_TESTPARAM");
}

static void parameters_update(float * pVal)
{
	param_get(temp_p, pVal);

}


static void poll_topic_msgs(void)
{

	bool _updated;


	orb_check(adc_sub, &_updated);
	if (_updated)
	{
		orb_copy(ORB_ID(adc_66v_raw), adc_sub, &adc66v);
		//other things to do
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
		float temp = 0;
		parameters_update(&temp);

		PX4_INFO("caiParam:\t%8.4f\n", (double)temp);
	}

	

	

}


__EXPORT int fan_aircraft_main(int argc, char *argv[]);

int fan_aircraft_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	parameters_init();
	float temp = 0;
	parameters_update(&temp);


	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	adc_sub  = orb_subscribe(ORB_ID(adc_66v_raw));
	v_att_sub     = orb_subscribe(ORB_ID(vehicle_attitude));
	param_sub = orb_subscribe(ORB_ID(parameter_update));


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
			/* obtained data for the first file descriptor */
			orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);


			poll_topic_msgs();


			//TODO delete later
			PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				 (double)sensor_comb.accelerometer_m_s2[0],
				 (double)sensor_comb.accelerometer_m_s2[1],
				 (double)sensor_comb.accelerometer_m_s2[2]);


			
		}

	}

	PX4_INFO("exiting");

	return 0;
}
