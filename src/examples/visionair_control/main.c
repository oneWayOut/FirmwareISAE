
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
#include <uORB/topics/vehicle_global_position.h>  //???
#include <uORB/topics/position_setpoint_triplet.h>  //??
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


/* process-specific header files */
#include "params.h"


#define PI 3.1415926536f   //TODO, use the original macro.

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




void visionair_controller(const struct manual_control_setpoint_s *pManual_sp, 
	const struct vehicle_local_position_s *pLocal_pos, 
	const struct control_state_s * pCtrl_state, 
	struct actuator_controls_s *pActuators);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;



//Altitude, when switch from mannual mode to altitude mode, we should remember the altitude
static float Hpc = 0.0f;   



/**
 * Limit angle to -PI~PI
 * @param  angle input angle(rad)
 * @return       out put angle(rad)
 */
static float modAngle(float angle);
static float modAngle(float angle)
{
	int i;

	i = (int)((angle+PI)*0.5f/PI);

	return angle - PI * (float)(i*2);
}

/**
 * [q_2_euler quaternion to euler from Quaternion.hpp]
 * @param data  [quaternion]
 * @param euler [description]
 */
static void q_2_euler(const float data[], float euler[]);
static void q_2_euler(const float data[], float euler[])
{
	euler[0] = atan2f(2.0f * (data[0] * data[1] + data[2] * data[3]), 1.0f - 2.0f * (data[1] * data[1] + data[2] * data[2]));
	euler[1] = asinf(2.0f * (data[0] * data[2] - data[3] * data[1]));
	euler[2] = atan2f(2.0f * (data[0] * data[3] + data[1] * data[2]), 1.0f - 2.0f * (data[2] * data[2] + data[3] * data[3]));
}


void visionair_controller(const struct manual_control_setpoint_s *pManual_sp, 
	const struct vehicle_local_position_s *pLocal_pos, 
	const struct control_state_s * pCtrl_state, 
	struct actuator_controls_s *pActuators)
{

	static float SEUIL_HAUT_GAZ_MODE2 = 0.65f;   //TODO: change the variables' name
	static float SEUIL_BAS_GAZ_MODE2  = 0.35f;
	static float GAZ_BASCUL = 0.0f;

	// commands from remote controller, manual set point;
	float OrdreRoll     = pManual_sp->y;
	float OrdrePitch    = pManual_sp->x;
	float OrdreYaw      = pManual_sp->r;
	float OrdreThrottle = pManual_sp->z;


	//roll, pitch, yaw, (angle commands in earth frame) throttle
	float PHIC = 0.0f;
	float TETC = 0.0f;
	static float PSIC = 0.0f;
	static float GAZC = 0.0f;

	// coefficient to compute PHIC, TETC, PSIC, GAZC   
	float max_phic = 1.0f;
	float max_tetc = 1.57f;
	float max_psipc = 2.0f;
	float max_vzc = 0.25f;

	float khp = 0.035f;
	float kvz = -0.1;

	static float TiGaz = 0.0f; // integrator

	

	float Hp = -pLocal_pos->z;
	float vz_acc = -pLocal_pos->vz; //TODO, check the direction




	float KiHp = 0.0088f;


	// get dT
	static uint64_t last_run = 0;
	float dT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	/* guard against too small (< 2ms) and too large (> 20ms) dt's */
	//cai TODO: maybe we need to check; as poll timeout in mainloop is 100ms
	if (dT < 0.002f) {
		dT = 0.002f;
	} else if (dT > 0.02f) {
		dT = 0.02f;
	}


	/***************    calculate: PHIC, TETC, PSIC, GAZC       ********************/

	PHIC =  OrdreRoll * max_phic;         //-1~1
	TETC =  PI/2 + OrdrePitch * max_tetc;  //
	PSIC += (OrdreYaw * max_psipc)*dT;
	PSIC = modAngle(PSIC);  

	// printf("roll = %d, pitch = %d, yaw = %d\t", (int)(pManual_sp->y*1000.0f),(int)(pManual_sp->x*1000.0f),(int)(pManual_sp->r*1000.0f));
	// printf("PHIC = %d\n", (int)(cmd_1[0]*180.0f*1000.0f/PI));
	//TODO: if we want to see these variables in GCS through mavlink, put 
	//them in vehicle_rates_setpoint or similar msgs that are not used.


	switch(pManual_sp->mode_switch)
	{
	//TODO: have to check the grammer of below, maybe only supported by CPP file.
	//case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
	case 3:
		Hpc = Hp;
		TiGaz = 0.0f;
		GAZC           = OrdreThrottle ;
		GAZ_BASCUL     = OrdreThrottle;
		break;
	//case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
	case 2:
		if (OrdreThrottle > SEUIL_HAUT_GAZ_MODE2)
		{
			Hpc += (OrdreThrottle - SEUIL_HAUT_GAZ_MODE2) / (1-SEUIL_HAUT_GAZ_MODE2) * max_vzc * dT;
		}
		else if (OrdreThrottle < SEUIL_BAS_GAZ_MODE2)
		{
			if (GAZC > 0.1f)
			{
				Hpc -= (SEUIL_BAS_GAZ_MODE2 - OrdreThrottle)/(SEUIL_BAS_GAZ_MODE2 - 0) * max_vzc * dT;
			}
		}
		else
		{
			;
		}

		TiGaz += KiHp * (Hpc - Hp) * dT;

		GAZC = GAZ_BASCUL+ khp * (Hpc - Hp) + TiGaz + kvz * vz_acc; 

		break;
	default :
		break;
	}



	/***************    calculate: com_phi, com_tet, com_psi      ********************/
	// use euler angles, anguler rates, 
	
 	//roll pitch yaw commands in UAV frame
	float com_phi, com_tet, com_psi;

	float euler_angles[3];

	q_2_euler(pCtrl_state->q, euler_angles);

	//euler angles
	float PHI = euler_angles[0];
	float TET = euler_angles[1];
	float PSI = euler_angles[2];

	//and angular rates
	float P = pCtrl_state->roll_rate;
	float Q = pCtrl_state->pitch_rate;
	float R = pCtrl_state->yaw_rate;

	float phip,  psip;

	float sintet = sinf(TET);	
	float costet = cosf(TET) ;	
	//float sinphi = sinf(PHI) ;		
	float cosphi = cosf(PHI) ;

	//proportional coefficients
	float kphi = 0.8f;
	float ktet = 0.8f;
	float kpsi = 1.0f;

	//derivative coefficients
	float kphip = -0.12f;
	float kq    = -0.12f;	
	float kpsip = -0.2f;


	float dpsi;

	static int sature = 0;
	// Pitch angle
	com_tet = ktet * (TETC - TET) + kq * Q ;

	// Roll angle
	phip = P*costet + R*sintet ;						//roll derivative
	com_phi = kphi * (PHIC - PHI) + kphip * phip ;

	// Yaw 
	psip = (-P*sintet + R*costet)/cosphi ;				//yaw derivative
	dpsi = modAngle(PSIC - PSI) ;

	if ( (sature == 1) && (dpsi<0) )
		dpsi += 2*PI ;
	if ( (sature == -1) && (dpsi>0) )
		dpsi -= 2*PI ;
	sature = 0 ;
	if (dpsi > PI/2)		// Ecrêtage de l'écart à 90°
	{
		dpsi = PI/2 ;
		sature = 1 ;
	}
	if (dpsi < - PI/2)
	{
		dpsi = -PI/2 ;
		sature=-1 ;
	}	
	
	com_psi = kpsi * dpsi + kpsip * psip ;

	//COM_ROULIS, COM_TANGAGE, COM_LACET, COM_GAZ, 	
	pActuators->control[0] = com_phi * costet - com_psi * sintet ;	// roll
	pActuators->control[1] = com_tet ;
	pActuators->control[2] = com_phi * sintet + com_psi * costet ;

	//TODO: please be aware that the code below is anti roll disturbation,
	//and we need to change the mixer also.
	pActuators->control[3] = GAZC + 0.02f*P;
	pActuators->control[4] = GAZC - 0.02f*P;

	pActuators->timestamp = hrt_absolute_time();
	pActuators->timestamp_sample = pActuators->timestamp;
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
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	//cai TODO: the four msgs above may not be useful anymore





	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));

	//added by cai
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct control_state_s ctrl_state;
	memset(&ctrl_state, 0, sizeof(ctrl_state));

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
	//int att_sub        = orb_subscribe(ORB_ID(vehicle_attitude));
	int manual_sp_sub  = orb_subscribe(ORB_ID(manual_control_setpoint));
	//int vstatus_sub    = orb_subscribe(ORB_ID(vehicle_status));
	int param_sub      = orb_subscribe(ORB_ID(parameter_update));
	int local_pos_sub  = orb_subscribe(ORB_ID(vehicle_local_position));
	int ctrl_state_sub = orb_subscribe(ORB_ID(control_state));

	/* Setup of loop */

	struct pollfd fds[2] = {{ .fd = param_sub, .events = POLLIN },
		                 { .fd = ctrl_state_sub, .events = POLLIN }
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
		int ret = poll(fds, 2, 100);

		if (ret < 0) {
			/*
			 * Poll error, this will not really happen in practice,
			 * but its good design practice to make output an error message.
			 */
			warnx("poll error");
			usleep(100000);
			continue;
		}
		if (ret == 0) {
			/* no return value = nothing changed for 500 ms, ignore */
			warnx("poll timeout visionair");
			continue;
		}



		/* only update parameters if they changed */
		//cai TODO: maybe we need to put some PID parameters here
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag (uORB API requirement) */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), param_sub, &update);

			/* if a param update occured, re-read our parameters */
			parameters_update(&ph, &p);
		}


		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {

			/* copy control state topic */
			orb_copy(ORB_ID(control_state), ctrl_state_sub, &ctrl_state);


			/* check for updates in other topics */
			bool updated = false;
			
			orb_check(local_pos_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
			}

			updated = false;
			orb_check(manual_sp_sub, &updated);
			if (updated)  /* get the RC (or otherwise user based) input */				
			{
				orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
                //cai test for read pwm input
                if (verbose) {
                    warnx("manual input x=%f, y=%f, z=%f, r=%f;\n", (double)manual_sp.x, (double)manual_sp.y, (double)manual_sp.z, (double)manual_sp.r);
                }
			}

			//cai: currently we don't need the two msgs below:
			/* get the system status and the flight mode we're in */
			//orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);
			/* get a local copy of attitude */
			//orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			//


			/* check if the throttle was ever more than 50% - go later only to failsafe if yes */
			if (isfinite(manual_sp.z) &&
			    (manual_sp.z >= 0.6f) &&
			    (manual_sp.z <= 1.0f)) {
			}

			//run the controller!!
			visionair_controller(&manual_sp, &local_pos, &ctrl_state, &actuators);



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

	printf("[ex_visionair_control] exiting, stopping all motors.\n");
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



