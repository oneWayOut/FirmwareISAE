
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
#include <uORB/topics/vehicle_local_position.h>  //changed by cai
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/control_state.h>   //added by cai
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/*cai for pwm control*/
#include <sys/ioctl.h>
#include "drivers/drv_pwm_output.h"

/* process-specific header files */
#include "params.h"


#define PI 3.1415926536f

/* Prototypes */

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
__EXPORT int ex_visionair_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int visionair_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);




void pwm_outtest(const struct manual_control_setpoint_s *manual_sp, 
	     const struct vehicle_local_position_s *plocal_pos, const struct vehicle_attitude_s * att);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;

/* cai pwm control */
static int pwm_fd = 0;



static float SEUIL_HAUT_GAZ_MODE2 = 0.65f;   //TODO: change the variables' name
static float SEUIL_BAS_GAZ_MODE2  = 0.35f;
static float GAZ_BASCUL = 0.0f;

static float cmd_1_maxK[4] = {1.0f, 1.45f, 2.0f, 0.25f}; //max_phic, max_tetc, max_psipc, max_vzc





/**
 * pwm output test based on the estimator data
 * @param
 */
void pwm_outtest(const struct manual_control_setpoint_s *manual_sp, 
	     const struct vehicle_local_position_s *plocal_pos, const struct vehicle_attitude_s * att)
{
	static float Hpc = 0.0f;   //Altitude  ???

	static float cmd_1[4] = {0.0f};  //roll, pitch, yaw, throttle :  PHIC, TETC, PSIC, GAZC    
	static float cmd_2[4] = {};      //com_phi, com_tet, com_psi


	/***************    calculate: PHIC, TETC, PSIC, GAZC       ********************/

	cmd_1[0] =  manual_sp->y * cmd_1_maxK[0];
	cmd_1[1] =  PI/2 + manual_sp->x * cmd_1_maxK[1];
	cmd_1[2] += (manual_sp->r * cmd_1_maxK[2])*dT;   //cai TODO: dT unknown?????
	//TODO:  mod(cmd_1)  ??????
	//
	
	if (manual_sp->z > SEUIL_HAUT_GAZ_MODE2)
	{
		Hpc += (manual_sp->z - SEUIL_HAUT_GAZ_MODE2) / (1-SEUIL_HAUT_GAZ_MODE2) * cmd_1_maxK[3] * dT
	}
	else if (manual_sp->z < SEUIL_HAUT_GAZ_MODE2)
	{
		if (cmd_1[3] > 0.1)
		{
			Hpc -= (SEUIL_BAS_GAZ_MODE2 - manual_sp->z)/(SEUIL_BAS_GAZ_MODE2 - 0) * cmd_1_maxK[3] * dT;
		}
	}
	else   //????
		;


	//??????
	TiGaz += KiHp * (Hpc - Hp) * Dt;

	//TODO: vz_acc is vertical speed, nz0 is vertical acc;  both in earth frame;
	cmd_1[3] = GAZ_BASCUL+ khp * (Hpc - Hp) + TiGaz + kvz * vz_acc + knz0 * nz0 ; 


	/***************    calculate: com_phi, com_tet, com_psi      ********************/
	// use euler angles, anguler rates, 





}


/* Main Thread */
int visionair_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("[example visionair control] started");

	/* initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &p);


	/*
	 * PX4 uses a publish/subscribe design pattern to enable
	 * multi-threaded communication.
	 *
	 * The most elegant aspect of this is that controllers and
	 * other processes can either 'react' to new data, or run
	 * at their own pace.
	 *
	 * PX4 developer guide:
	 * https://pixhawk.ethz.ch/px4/dev/shared_object_communication
	 *
	 * Wikipedia description:
	 * http://en.wikipedia.org/wiki/Publish–subscribe_pattern
	 *
	 */




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

	//added by cai
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));

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

	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	/* Setup of loop */

	struct pollfd fds[3] = {{ .fd = param_sub, .events = POLLIN },
		{ .fd = att_sub, .events = POLLIN },
		{ .fd = local_pos_sub, .events = POLLIN }
	};

	/* open for ioctl only */
	pwm_fd = open(PWM_OUTPUT0_DEVICE_PATH, 0);

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
		int ret = poll(fds, 3, 500);

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

			if (fds[2].revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

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
                    //cai test for read pwm input
                    if (verbose) {
                        warnx("manual input x=%f, y=%f, z=%f, r=%f;\n", (double)manual_sp.x, (double)manual_sp.y, (double)manual_sp.z, (double)manual_sp.r);
                    }
				}

				/* check if the throttle was ever more than 50% - go later only to failsafe if yes */
				if (isfinite(manual_sp.z) &&
				    (manual_sp.z >= 0.6f) &&
				    (manual_sp.z <= 1.0f)) {
				}

				/* get the system status and the flight mode we're in */
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);


                if(pos_updated)
                {
                    orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
                }


                /* publish actuator controls */
                actuators.control[0] = manual_sp.y;
                actuators.control[1] = manual_sp.x;
                actuators.control[2] = manual_sp.r;
                actuators.control[3] = manual_sp.z;
                actuators.timestamp = hrt_absolute_time();
                actuators.timestamp_sample = actuators.timestamp;




				/* publish rates */
                orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

                //warnx("set acuators0 = %f; 1 = %f; 2 = %f; 3 = %f\n", (double)actuators.control[0], (double)actuators.control[1],(double)actuators.control[2],(double)actuators.control[3]);

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

	printf("[ex_visionair_control] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	fflush(stdout);

	close(pwm_fd);

	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: ex_visionair_control {start|stop|status}\n\n");
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
int ex_visionair_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ex_visionair_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("ex_visionair_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 visionair_control_thread_main,
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
			printf("\tex_visionair_control is running\n");

		} else {
			printf("\tex_visionair_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



