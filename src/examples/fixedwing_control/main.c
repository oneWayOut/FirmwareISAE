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

/**
 * @file main.c
 *
 * Example implementation of a fixed wing attitude controller. This file is a complete
 * fixed wing controller for manual attitude control or auto waypoint control.
 * There is no need to touch any other system components to extend / modify the
 * complete control architecture.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* process-specific header files */
#include "params.h"

/* Prototypes */


__EXPORT int ex_fixedwing_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);


/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;

/*cai test mixer */
static float    control_value = 0.0f;
static unsigned control_channel  = 0;



/* Main Thread */
int fixedwing_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("[example fixedwing control] started");

	/* initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &p);



	/*
	 * Declare and safely initialize all structs to zero.
	 *
	 * These structs contain the system state and things
	 * like attitude, position, the current waypoint, etc.
	 */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct position_setpoint_s global_sp;
	memset(&global_sp, 0, sizeof(global_sp));

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));


	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	/*
	 * Advertise that this controller will publish actuator
	 * control values and the rate setpoint
	 */
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int global_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Setup of loop */

	struct pollfd fds[2] = {{ .fd = param_sub, .events = POLLIN },
		{ .fd = att_sub, .events = POLLIN }
	};

	while (!thread_should_exit) {

		/*
		 * Wait for a sensor or param update, check for exit condition every 500 ms.
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new attitude measurement or
		 * a param update is published. So no latency in contrast to the polling
		 * design pattern (do not confuse the poll() system call with polling).
		 *
		 * This design pattern makes the controller also agnostic of the attitude
		 * update speed - it runs as fast as the attitude updates with minimal latency.
		 */
		int ret = poll(fds, 2, 500);

		if (ret < 0) {
			/*
			 * Poll error, this will not really happen in practice,
			 * but its good design practice to make output an error message.
			 */
			warnx("poll error");

		} else if (ret == 0) {
			/* no return value = nothing changed for 500 ms, ignore */
		} else {

			/* only update parameters if they changed */
			if (fds[0].revents & POLLIN) {
				/* read from param to clear updated flag (uORB API requirement) */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);

				/* if a param update occured, re-read our parameters */
				parameters_update(&ph, &p);
			}

			/* only run controller if attitude changed */
			if (fds[1].revents & POLLIN) {


				/* Check if there is a new position measurement or position setpoint */
				bool pos_updated;
				orb_check(global_pos_sub, &pos_updated);
				bool global_sp_updated;
				orb_check(global_sp_sub, &global_sp_updated);
				bool manual_sp_updated;
				orb_check(manual_sp_sub, &manual_sp_updated);

				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

				if (global_sp_updated) {
					struct position_setpoint_triplet_s triplet;
					orb_copy(ORB_ID(position_setpoint_triplet), global_sp_sub, &triplet);
					memcpy(&global_sp, &triplet.current, sizeof(global_sp));
				}

				if (manual_sp_updated)
					/* get the RC (or otherwise user based) input */
				{
					orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
				}

				/* check if the throttle was ever more than 50% - go later only to failsafe if yes */
				if (isfinite(manual_sp.z) &&
				    (manual_sp.z >= 0.6f) &&
				    (manual_sp.z <= 1.0f)) {
				}

				/* get the system status and the flight mode we're in */
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

				/* publish rates */
				orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

				if (control_channel>=1 && control_channel<=4)
				{
					for (int i = 0; i < 4; ++i)
					{
						if(i+1 == control_channel)
							actuators.control[i] = (float)control_value;
					}
				}
				else
				{
					for (int i = 0; i < 4; ++i)
					{
						actuators.control[i] = 0.0f;
					}
				}					

				/* sanity check and publish actuator outputs */
				if (isfinite(actuators.control[0]) &&
				    isfinite(actuators.control[1]) &&
				    isfinite(actuators.control[2]) &&
				    isfinite(actuators.control[3])) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

					if (verbose) {
						warnx("published");
					}
				}
			}
		}
	}

	printf("[ex_fixedwing_control] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	fflush(stdout);

	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: ex_fixedwing_control {start|stop|status}\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int ex_fixedwing_control_main(int argc, char *argv[])
{
	char *ep;
	int ch;

	while ((ch = getopt(argc - 1, &argv[1], "c:v:n:")) != EOF)
	{
		switch(ch)
		{
		case 'c':
			control_channel = strtoul(optarg, &ep, 0);
			if(control_channel>4||control_channel==0)
			{
				printf("set channel error\n");
				control_channel = 0;
			}
			else
				printf("set channel = %d\n", control_channel); //cai may change type.
			break;
		case 'v':
			control_value = strtod(optarg, &ep);  // cai TODO: change function
			//printf("optarg = %s\n", optarg);
			if (control_value>=-1.0f && control_value<=1.0f)
			{
				printf("set value =  %d\n", (int)(control_value*100.0f));   //cai may scale to int value;
			}
			else
			{
				printf("set value error\n");
				control_value = 0.0;
			}
			break;
		case 'n':       //for negative value input, because 'strtod' recognizes negative value as zero.
			control_value = -strtod(optarg, &ep);  // cai TODO: change function
			//printf("optarg = %s\n", optarg);
			if (control_value>=-1.0f && control_value<=1.0f)
			{
				printf("set value =  %d\n", (int)(control_value*100.0f));   //cai may scale to int value;
			}
			else
			{
				printf("set value error\n");
				control_value = 0.0;
			}
			break;
		default:
			break;
		}
	}

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ex_fixedwing_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("ex_fixedwing_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 fixedwing_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tex_fixedwing_control is running\n");

		} else {
			printf("\tex_fixedwing_control not started\n");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "set")) 
	{

		printf("\tset control input\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



