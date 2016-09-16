  
/**
 * @file main.c
 *
 * the control law of this file is immigrated from the original control law code of
 * VisionAir developed by DCAS of ISAE. Note that the hovering mode is the neutral position, 
 * that means in this code the body frame axises is a bit different.
 * @author CAI Dongcai	<dacid120@gmail.com>
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
__EXPORT int visionair_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int visionair_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);



/**
 * VisionAir controller, 
 * @param pManual_sp  [description]
 * @param pLocal_pos  [description]
 * @param pCtrl_state [pro]
 * @param pAtt        [description]
 * @param pActuators  [description]
 */
static void visionair_controller(const struct manual_control_setpoint_s *pManual_sp, 
	const struct vehicle_local_position_s *pLocal_pos,
	const struct control_state_s * pCtrl_state,
	const struct vehicle_attitude_s *pAtt,
	struct actuator_controls_s *pActuators);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params va_p;        //VisionAir parameters
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
#if 0  /*this is created by CAI, not work properly for some value. e.g. -1.1*M_PI_F  */
	int i;

	i = (int)((angle+M_PI_F)*0.5f/M_PI_F);

	return angle - M_PI_F * (float)(i*2);
#endif

//use the original mod function
	if (angle > M_PI_F)
		angle += - 2 * M_PI_F ;
	if (angle < - M_PI_F)
		angle += 2 * M_PI_F ;

	return angle ;
}





static void visionair_controller(const struct manual_control_setpoint_s *pManual_sp, 
	const struct vehicle_local_position_s *pLocal_pos,
	const struct control_state_s * pCtrl_state,
	const struct vehicle_attitude_s *pAtt,
	struct actuator_controls_s *pActuators)
{

	static float SEUIL_HAUT_GAZ_MODE2 = 0.65f;   //TODO: change the variables' name
	static float SEUIL_BAS_GAZ_MODE2  = 0.35f;
	static float GAZ_BASCUL = 0.0f;


	//scale the RC inputs (roll, pitch, yaw) to angles(rad)
	static float scaled_rc_att[3] = {0.0f};

	//throttle command
	static float GAZC = 0.0f;


	//height integrator
	static float TiGaz = 0.0f;

	float Hp     = -pLocal_pos->z;
	float vz_acc = -pLocal_pos->vz;


	// get dT
	static uint64_t last_run = 0;
	float dT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	/* guard against too small (< 2ms) and too large (> 20ms) dt's */
	if (dT < 0.002f) {
		dT = 0.002f;
	} else if (dT > 0.02f) {
		dT = 0.02f;
	}


	/***************    calculate: scaled_rc_att,  GAZC       ********************/
	// scale rc attitude control from input  commands to angles(rad)
	scaled_rc_att[0] =  pManual_sp->y * va_p.rc_cmd_k[0];         //-1~1
	scaled_rc_att[1] =  pManual_sp->x * va_p.rc_cmd_k[1];  //change from 0~170 to -90~80 (degree)
	scaled_rc_att[2] += (pManual_sp->r * va_p.rc_cmd_k[2])*dT;
	scaled_rc_att[2] = modAngle(scaled_rc_att[2]);

	//TODO: if we want to see these variables in GCS through mavlink, put 
	//them in vehicle_rates_setpoint or similar msgs that are not used.

	switch(pManual_sp->mode_switch)
	{
	//the statement below need C++ support
	//case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
	case 3:   //direct control of throttle
		Hpc = Hp;
		TiGaz = 0.0f;
		GAZC           = pManual_sp->z;  // pManual_sp->z is RC throttle input. 0~1
		GAZ_BASCUL     = pManual_sp->z;
		break;
	//case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
	case 2:  //throttle is used to control vz speed
		if (pManual_sp->z > SEUIL_HAUT_GAZ_MODE2)
		{
			Hpc += (pManual_sp->z - SEUIL_HAUT_GAZ_MODE2) / (1-SEUIL_HAUT_GAZ_MODE2) * va_p.rc_cmd_k[3] * dT;
		}
		else if (pManual_sp->z < SEUIL_BAS_GAZ_MODE2)
		{
			if (GAZC > 0.1f)
			{
				Hpc -= (SEUIL_BAS_GAZ_MODE2 - pManual_sp->z)/(SEUIL_BAS_GAZ_MODE2 - 0) * va_p.rc_cmd_k[3] * dT;
			}
		}
		else
		{
			;
		}

		TiGaz += va_p.h_PID[1] * (Hpc - Hp) * dT;

		GAZC = GAZ_BASCUL+ va_p.h_PID[0] * (Hpc - Hp) + TiGaz + va_p.h_PID[2] * vz_acc; 

		break;
	default : //just protection in case we switch to a third postion and don't have any throttle command
		Hpc = Hp;
		TiGaz = 0.0f;
		GAZC           = pManual_sp->z ;
		GAZ_BASCUL     = pManual_sp->z;
		break;
	}



	/***************    calculate: com_att      ********************/
	// input: euler angles, anguler rates.  Using PD control

 	//roll pitch yaw commands (in earth frame or from pilot's view?), there is mixing in roll and yaw depeding on pitch angle
	float com_att[3] = {0.0f};


	//printf("r,p,y _Q = %.4f, %.4f, %.4f\n", (double)pAtt->roll, (double)pAtt->pitch, (double)pAtt->yaw);


	float sin_pitch = sinf(pAtt->pitch);
	float cos_pitch = cosf(pAtt->pitch);

	// Roll command
	float roll_rate_tmp = pCtrl_state->roll_rate * cos_pitch + pCtrl_state->yaw_rate * sin_pitch ;  // roll derivative, note changes of the rotation axis, and which is the positive rotation side
	com_att[0] = va_p.att_P[0] * (scaled_rc_att[0] - pAtt->roll) + va_p.att_D[0] * roll_rate_tmp ;

	// Pitch command
	com_att[1] = va_p.att_P[1] * (scaled_rc_att[1] - pAtt->pitch) + va_p.att_D[1] * pCtrl_state->pitch_rate ;

	// Yaw command
	float yaw_error = modAngle(scaled_rc_att[2] - pAtt->yaw) ;

	static int sature = 0;
	if ( (sature == 1) && (yaw_error<0) )
		yaw_error += 2*M_PI_F ;
	if ( (sature == -1) && (yaw_error>0) )
		yaw_error -= 2*M_PI_F ;

	if (yaw_error > M_PI_F/2)		// Ecrêtage de l'écart à 90°
	{
		yaw_error = M_PI_F/2 ;
		sature = 1 ;
	}
	else if (yaw_error < - M_PI_F/2)
	{
		yaw_error = -M_PI_F/2 ;
		sature=-1 ;
	}
	else
	{
		sature = 0;
	}

	//yaw_rate_tmp = (-pCtrl_state->roll_rate*sin_pitch + pCtrl_state->yaw_rate*cos_pitch)/cosphi ;	
	//yaw derivative  TODO, why devide by cosphi????? 
	float yaw_rate_tmp = (-pCtrl_state->roll_rate * sin_pitch + pCtrl_state->yaw_rate * cos_pitch);

	com_att[2] = va_p.att_P[2] * yaw_error + va_p.att_D[2] * yaw_rate_tmp ;



	/***************    calculate actuator_controls_s, which will be used in mixer ********************/
	//COM_ROULIS, COM_TANGAGE, COM_LACET, COM_GAZ in the original visionair code
	//(the commands below are in body frame ?)  note the different coordinate frames(pilot's commands to control)
	pActuators->control[0] = com_att[0] * cos_pitch - com_att[2] * sin_pitch ;	// roll
	pActuators->control[1] = com_att[1] ;                                       // pitch
	pActuators->control[2] = com_att[0] * sin_pitch + com_att[2] * cos_pitch ;  // yaw

	//please be aware that the code below is anti yaw (roll) disturbation,
	//2016.9.16 change pCtrl_state->roll_rate to pCtrl_state->yaw_rate as the body coordinates change because of using Pixhawk
	pActuators->control[3] = GAZC + va_p.kgp * pCtrl_state->yaw_rate;   //0.02 is kgp in original VisionAir sourcecode.
	pActuators->control[4] = GAZC - va_p.kgp * pCtrl_state->yaw_rate;

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
	warnx("[visionair control] started");

	/* initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &va_p);



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
	//cai TODO: the 3 msgs above may not be useful anymore



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
	int att_sub        = orb_subscribe(ORB_ID(vehicle_attitude));
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
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag (uORB API requirement) */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), param_sub, &update);

			/* if a param update occured, re-read our parameters */
			parameters_update(&ph, &va_p);			
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
			updated = false;
			orb_check(att_sub, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			}


			/* check if the throttle was ever more than 50% - go later only to failsafe if yes */
			if (isfinite(manual_sp.z) &&
			    (manual_sp.z >= 0.6f) &&
			    (manual_sp.z <= 1.0f)) {
			}

			//run the controller!!
			visionair_controller(&manual_sp, &local_pos, &ctrl_state, &att, &actuators);



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

	printf("[visionair_control] exiting, stopping all motors.\n");
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

	fprintf(stderr, "usage: visionair_control {start|stop|status}\n\n");
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
int visionair_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("visionair_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("visionair_control",
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
			printf("\tvisionair_control is running\n");

		} else {
			printf("\tvisionair_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

