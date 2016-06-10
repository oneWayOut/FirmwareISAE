
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

/**
 * Control roll and pitch angle.
 *
 * This very simple roll and pitch controller takes the current roll angle
 * of the system and compares it to a reference. Pitch is controlled to zero and yaw remains
 * uncontrolled (tutorial code, not intended for flight).
 *
 * @param att_sp The current attitude setpoint - the values the system would like to reach.
 * @param att The current attitude. The controller should make the attitude match the setpoint
 * @param rates_sp The angular rate setpoint. This is the output of the controller.
 */
void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators);

/**
 * Control heading.
 *
 * This very simple heading to roll angle controller outputs the desired roll angle based on
 * the current position of the system, the desired position (the setpoint) and the current
 * heading.
 *
 * @param pos The current position of the system
 * @param sp The current position setpoint
 * @param att The current attitude
 * @param att_sp The attitude setpoint. This is the output of the controller
 */
void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp);


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

void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators)
{

	/*
	 * The PX4 architecture provides a mixer outside of the controller.
	 * The mixer is fed with a default vector of actuator controls, representing
	 * moments applied to the vehicle frame. This vector
	 * is structured as:
	 *
	 * Control Group 0 (attitude):
	 *
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 *    4  -  flaps  (-1..+1)
	 *    ...
	 *
	 * Control Group 1 (payloads / special):
	 *
	 *    ...
	 */


	/*
	 * Calculate roll error and apply P gain
	 */
	float roll_err = att->roll - att_sp->roll_body;
	actuators->control[0] = roll_err * p.roll_p;

	/*
	 * Calculate pitch error and apply P gain
	 */
	float pitch_err = att->pitch - att_sp->pitch_body;
	actuators->control[1] = pitch_err * p.pitch_p;
}

void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp)
{

	/*
	 * Calculate heading error of current position to desired position
	 */

	float bearing = get_bearing_to_next_waypoint(pos->lat, pos->lon, sp->lat, sp->lon);

	/* calculate heading error */
	float yaw_err = att->yaw - bearing;
	/* apply control gain */
	att_sp->roll_body = yaw_err * p.hdng_p;

	/* limit output, this commonly is a tuning parameter, too */
	if (att_sp->roll_body < -0.6f) {
		att_sp->roll_body = -0.6f;

	} else if (att_sp->roll_body > 0.6f) {
		att_sp->roll_body = 0.6f;
	}
}

/**
 * pwm output test based on the estimator data
 * @param
 */
void pwm_outtest(const struct manual_control_setpoint_s *manual_sp, 
	     const struct vehicle_local_position_s *plocal_pos, const struct vehicle_attitude_s * att)
{
	int ret = 0;
	unsigned pwm_value[6] = {0};

	float v_z = 0.0;    // m/s
    float roll = 0.0;   // degree

#if 0
    /*use the throttle to test motor*/
    pwm_value[0] = (unsigned)((manual_sp->z+1.0f)*1000.0f);
    pwm_value[1] = pwm_value[0];

    /*use only the pitch input to test servo*/
    pwm_value[2] = (unsigned)((manual_sp->x+3.0f)*500.0f);
    pwm_value[3] = pwm_value[2];
    pwm_value[4] = pwm_value[2];
    pwm_value[5] = pwm_value[2];
#endif

    roll  = att->roll*180.0f/3.1415926f;

    if (roll >= 20.0f)
        roll = 20.0f;
    else if(roll <= -20.0f)
        roll  = -20.0f;
    else
        { ;}

    v_z = plocal_pos->vz;
    if(v_z >= 1.0f)
        v_z = 1.0f;
    else if(v_z <= -1.0f)
        v_z = -1.0f;
    else
        { ;}

    pwm_value[0] = (unsigned)((roll+60.0f)*25.0f);
    pwm_value[1] = pwm_value[0];

    pwm_value[2] = (unsigned)((v_z+3.0f)*500.0f);
    pwm_value[3] = pwm_value[2];
    pwm_value[4] = pwm_value[2];
    pwm_value[5] = pwm_value[2];



    for(unsigned i = 0; i<6; i++)
    {
        ret = ioctl(pwm_fd, PWM_SERVO_SET(i), pwm_value[i]);
        if (ret != OK) {
            err(1, "PWM_SERVO_SET(%d)", i);
        }
    }
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

               // warnx("att.roll = %f, attsp.rollbody = %f, p.roll_p=%f\n", (double)(att.roll*1000.0f), (double)att_sp.roll_body, (double)p.roll_p);

               // control_heading(&global_pos, &global_sp, &att, &att_sp);
               // control_attitude(&att_sp, &att, &rates_sp, &actuators);
               // pwm_outtest(&manual_sp, &local_pos, &att);







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



