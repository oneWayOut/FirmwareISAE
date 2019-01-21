#ifndef WIN32

#include "../sdk.h"
#include "smoothModeValues.h"

#define  _ENABLE_PRINT_DEBUG 0



//lonPitch
PARAM_DEFINE_FLOAT(K_LONPITCH, 0.15f);
PARAM_DEFINE_FLOAT(K_LONQ, 0.01f);
PARAM_DEFINE_FLOAT(K_LONVX, 1.5f);
PARAM_DEFINE_FLOAT(K_ILONVX, 0.3f);

//latRoll
PARAM_DEFINE_FLOAT(K_LATBANK, 0.15f);
PARAM_DEFINE_FLOAT(K_LATP, 0.01f);
PARAM_DEFINE_FLOAT(K_LATVY, 1.6f);
PARAM_DEFINE_FLOAT(K_ILATVY, 0.3f);

//tail
PARAM_DEFINE_FLOAT(K_TR, 0.25f);
PARAM_DEFINE_FLOAT(K_ITR, 0.3f);


PARAM_DEFINE_FLOAT(K_CAZ, 0.4f);
PARAM_DEFINE_FLOAT(K_ICAZ, 1.0f);
PARAM_DEFINE_FLOAT(K_CVZ, 2.0f);


//PARAM_DEFINE_FLOAT(K_LONPITCHQ, 0.5f);
//PARAM_DEFINE_FLOAT(K_ILONQ, 1.0f);
//PARAM_DEFINE_FLOAT(K_LATBANKP, 0.5f);
//PARAM_DEFINE_FLOAT(K_ILATP, 1.0f);

PARAM_DEFINE_FLOAT(K_LONPITCH_HR, 0.05f);
PARAM_DEFINE_FLOAT(K_LATBANK_HR, 0.02f);
PARAM_DEFINE_FLOAT(K_LAND, 1.0f);

//const  param
PARAM_DEFINE_FLOAT(C_DELTAT0, -4.7621f);

PARAM_DEFINE_FLOAT(C_DELTA_LON0, 0.0142f);
PARAM_DEFINE_FLOAT(C_PITCHLON0, -2.0f);

PARAM_DEFINE_FLOAT(C_DELTALAT0, 0.0192f);
PARAM_DEFINE_FLOAT(C_BANKLAT0, 2.2771f);


PARAM_DEFINE_FLOAT(C_DELTAC0, 8.0f);


extern sem_t _user_sem_100Hz;

#else

#include "simAdapter.h"
#include "stdio.h"
#include "math.h"
#endif

static struct st_attitude_s         attData;
static struct rc_channels_s         rc_ctrl;
static struct vehicle_global_position_s  glbPos;

static struct st_controls_s         controls_value;
static struct st_sl_status_s slStatus;


//deltaC
static float thrustCurve[3][2] = {{0, 0}, {0.5, 7}, {1, 14}};

static float pitchCurve[3][2] = {{-1, -10}, {0, 0}, {1, 10}};

static float rollCurve[3][2] = {{-1,-10}, {0, 0,}, {1, 10}};

static float yawCurve[3][2] = {{-1, -60}, {0, 0}, {1, 60}};

//static float deltaT0Curve[3][2] = {{0, 1}, {7, 5}, {14, 9}};

static float deltaC = 0;  //collective pitch
static float cBelow2T = 0; //time of deltaC below 2;


//to reset integration or not;
static bool armResetInt = false;     //reset integration as armed switch
static bool modeResetInt = false;    //reset integration as mode switch




//parameters::
//lonPitch
static float k_lonpitch = 0.1;
static float k_lonq = 0.01;
static float k_lonvx = 1.5;
static float k_ilonvx = 0.3;

//latRoll
static float k_latbank = 0.1;
static float k_latp = 0.01;
static float k_latvy = 1.6;
static float k_ilatvy = 0.3;

//tail
static float k_tr = 0.25;
static float k_itr = 0.3;


static float k_caz = 0.4;
static float k_icaz = 0.2;
static float k_cvz = 2.0;

//new params
//static float k_lonpitchq = 0.5;
//static float k_ilonq = 1;
//static float k_latbankp = 0.5;
//static float k_ilatp = 1;

static float k_lonpitch_hr = 0.2f;
static float k_latbank_hr = 0.1f;

static float k_land = 1.0;



//constant paramenters::   add as param
static float c_deltaT0   = -4.7621f;  //give a const first;

static float c_delta_lon0 = 0.0142;
static float c_pitchlon0 = -0.0178;

static float c_deltaLat0 = 0.0192;
static float c_bankLat0 = 2.2771;
static float c_deltaC0  = 8.0;


static float d_T  = 0.01;  //d_T for each loop period (currently is 0.01 second)


static struct st_attitude_s attData_Deg;   //attitude data in degree (not radians)

static float h_r = 0;   //height above ground


static SMOOTH_MODE_VALUES smoothDeltaC;   //1=assist , 2 = manual;
static SMOOTH_MODE_VALUES smoothDeltaLon; //1= assist & h_r<5; 2 = assit & h_r>5; 3=manual
static SMOOTH_MODE_VALUES smoothDeltaLat; //1= assist & h_r<5; 2 = assit & h_r>5; 3=manual




/*handle for GCS communication*/
static orb_advert_t u1_1_mavlink_log_pub = NULL;
struct st_sensors_s sensor_data;


static float interpolation(float val, float xy[][2], int size);
static float constrain(float val, float min, float max);
static float controlLonPitch(float RcPitch, float vx, float dt, int mode);
static float controlLat(float RcRoll, float vy, float dt, int mode);
static float controlTailClctPth(float RcYaw, float dt);
static float constrainRate(float lastVal, float thisVal, float rate, float dt);
static void initParams(void);
static void getParamsValue(void);
static float controlDeltaC(float RcThrotle, float dt);


param_t _param_K_LONPITCH;
param_t _param_K_LONQ;
param_t _param_K_LONVX;
param_t _param_K_ILONVX;
param_t _param_K_LATBANK;
param_t _param_K_LATP;
param_t _param_K_LATVY;
param_t _param_K_ILATVY;
param_t _param_K_TR;
param_t _param_K_ITR;
param_t _param_K_CAZ;
param_t _param_K_ICAZ;
param_t _param_K_CVZ;
param_t _param_C_DELTAT0;
param_t _param_C_DELTA_LON0;
param_t _param_C_PITCHLON0;
param_t _param_C_DELTALAT0;
param_t _param_C_BANKLAT0;
param_t _param_C_DELTAC0;
param_t _param_K_LONPITCH_HR;
param_t _param_K_LATBANK_HR;
param_t _param_K_LAND;






static void initParams(void)
{

	//lonPitch
	// param_set(param_find("K_LONPITCH"), &k_lonpitch);
#if 0
	(void)param_find("K_LONPITCH");
	(void)param_find("K_LONQ");
	(void)param_find("K_LONVX");
	(void)param_find("K_ILONVX");
	(void)param_find("K_LATBANK");
	(void)param_find("K_LATP");
	(void)param_find("K_LATVY");
	(void)param_find("K_ILATVY");
	(void)param_find("K_TR");
	(void)param_find("K_ITR");

	(void)param_find("K_CAZ");
	(void)param_find("K_ICAZ");
	(void)param_find("K_CVZ");

	(void)param_find("C_DELTAT0");
	(void)param_find("C_DELTA_LON0");
	(void)param_find("C_PITCHLON0");
	(void)param_find("C_DELTALAT0");
	(void)param_find("C_BANKLAT0");
	(void)param_find("C_DELTAC0");

	//(void)param_find("K_LONPITCHQ");
	//(void)param_find("K_ILONQ");
	//(void)param_find("K_LATBANKP");
	//(void)param_find("K_ILATP");
	(void)param_find("K_LONPITCH_HR");
	(void)param_find("K_LATBANK_HR");
	(void)param_find("K_LAND");
#endif








	_param_K_LONPITCH = param_find("K_LONPITCH");
	_param_K_LONQ = param_find("K_LONQ");
	_param_K_LONVX = param_find("K_LONVX");
	_param_K_ILONVX = param_find("K_ILONVX");
	_param_K_LATBANK = param_find("K_LATBANK");
	_param_K_LATP = param_find("K_LATP");
	_param_K_LATVY = param_find("K_LATVY");
	_param_K_ILATVY = param_find("K_ILATVY");
	_param_K_TR = param_find("K_TR");
	_param_K_ITR = param_find("K_ITR");

	_param_K_CAZ = param_find("K_CAZ");
	_param_K_ICAZ = param_find("K_ICAZ");
	_param_K_CVZ = param_find("K_CVZ");


	_param_C_DELTAT0 = param_find("C_DELTAT0");

	_param_C_DELTA_LON0 = param_find("C_DELTA_LON0");
	_param_C_PITCHLON0 = param_find("C_PITCHLON0");

	_param_C_DELTALAT0 = param_find("C_DELTALAT0");
	_param_C_BANKLAT0 = param_find("C_BANKLAT0");

	_param_C_DELTAC0 = param_find("C_DELTAC0");

	_param_K_LONPITCH_HR = param_find("K_LONPITCH_HR");
	_param_K_LATBANK_HR = param_find("K_LATBANK_HR");
	_param_K_LAND = param_find("K_LAND");



	getParamsValue();
}


static void getParamsValue(void)
{
	//lonPitch
	param_get(_param_K_LONPITCH, &k_lonpitch);
	param_get(_param_K_LONQ, &k_lonq);
	param_get(_param_K_LONVX, &k_lonvx);
	param_get(_param_K_ILONVX, &k_ilonvx);

	//latRoll
	param_get(_param_K_LATBANK, &k_latbank);
	param_get(_param_K_LATP, &k_latp);
	param_get(_param_K_LATVY, &k_latvy);
	param_get(_param_K_ILATVY, &k_ilatvy);

	//tail
	param_get(_param_K_TR, &k_tr);
	param_get(_param_K_ITR, &k_itr);


	param_get(_param_K_CAZ, &k_caz);
	param_get(_param_K_ICAZ, &k_icaz);
	param_get(_param_K_CVZ, &k_cvz);


	//param_get("K_LONPITCHQ", &k_lonpitchq);
	//param_get("K_ILONQ", &k_ilonq);
	//param_get("K_LATBANKP", &k_latbankp);
	//param_get("K_ILATP", &k_ilatp);

	param_get(_param_K_LONPITCH_HR, &k_lonpitch_hr);
	param_get(_param_K_LATBANK_HR, &k_latbank_hr);
	param_get(_param_K_LAND, &k_land);


	param_get(_param_C_DELTAT0, &c_deltaT0);

	param_get(_param_C_DELTA_LON0, &c_delta_lon0);
	param_get(_param_C_PITCHLON0, &c_pitchlon0);

	param_get(_param_C_DELTALAT0, &c_deltaLat0);
	param_get(_param_C_BANKLAT0, &c_bankLat0);


	param_get(_param_C_DELTAC0, &c_deltaC0);
}



//interpolation  chazhi
static float interpolation(float val, float xy[][2], int size)
{
	int ix = 0;

	if (val <= xy[0][0])
	{
		return xy[0][1];
	}
	if (val >= xy[size-1][0])
	{
		return xy[size-1][1];
	}


	for (ix = 0; ix < size; ++ix)
	{
		if (val >= xy[ix][0])
		{
			return xy[ix][1] + (val-xy[ix][0])* (xy[ix+1][1] - xy[ix][1])/(xy[ix+1][0] - xy[ix][0]);
		}
	}
	
	printf("error in interpolation\n");
	return 0.0;
}


//constrain function
static float constrain(float val, float min, float max)
{
	if (val>=max)
		return max;

	if (val<=min)
		return min;

	return val;
}


/**
 * [constrainRate constrain the rate of a command.]
 * @param  lastVal [description]
 * @param  thisVal [description]
 * @param  rate    [IMPMORT NOTE: rate should be a postive val]
 * @param  dt      [description]
 * @return         [description]
 */
static float constrainRate(float lastVal, float thisVal, float rate, float dt)
{
	if (thisVal > lastVal+rate*dt)
	{
		return lastVal+rate*dt;
	}
	if (thisVal < lastVal-rate*dt)
	{
		return lastVal-rate*dt;
	}

	return thisVal;
}



static float controlDeltaC(float RcThrotle, float dt)
{
	float az;
	float azcmd;
	float retVal;
	static float integration = 0;
	static float lastDeltaC0 = 0;

	static float lastDeltaC = 0;

	struct st_sensors_s _sens_data;


	if (!(h_r<5 && rc_ctrl.channels[5]>0))
	{
		//deltaC0
		retVal = constrainRate(lastDeltaC0, c_deltaC0, 2.0, dt);
		lastDeltaC0 = retVal;

		//proportion

		api_get_sensors_data(&_sens_data);

		//convert local to NED coordinate
		az = -(float)(sin(attData.pitch))*_sens_data.accelerometer_m_s2[0] + 
		     (float)(sin(attData.roll)*cos(attData.pitch))*_sens_data.accelerometer_m_s2[1] +
		     (float)(cos(attData.roll)*cos(attData.pitch))*_sens_data.accelerometer_m_s2[2] +9.81f;



		RcThrotle = constrain(RcThrotle, 0, 1);

		azcmd = -k_cvz*(RcThrotle*4.0f - 2.0f + glbPos.vel_d);  //todo check sign of glbPos.vel_d

		azcmd = constrain(azcmd, -5.0, 5.0);

		retVal += constrain(k_caz*az, -8.0f, 8.0f);


		//integration
		if (armResetInt)
		{
			integration = 0;
		}
		else
		{
			integration += constrain(az-azcmd, -5.0, 5.0)*dt*k_icaz;

			integration = constrain(integration, -8.0, 8.0);
		}

		retVal += integration;
	}
	else //(h_r<5 && rc_ctrl.channels[5]>0)   land detect switch
	{
		retVal = constrainRate(lastDeltaC, 0, k_land, dt);
	}

	lastDeltaC = retVal;

	return retVal;
}


/**
 * [controlLonPitch description]
 * @param  RcPitch [description]
 * @param  vx      [description]
 * @param  dt      [description]
 * @param  mode    [0/1 = manual/assit]
 * @return         [description]
 */
static float controlLonPitch(float RcPitch, float vx, float dt, int mode)
{
	static float integration = 0;
	static float last_Delta_lon0 = 0;

	static float lastVxloncmd = 0;

	uint8_t smoothMode = 0;

	float temp = 0;

	float retVal = 0;

	float vxloncmd = interpolation(RcPitch, pitchCurve, 3);

	vxloncmd = constrainRate(lastVxloncmd, vxloncmd, 5, dt);
	lastVxloncmd = vxloncmd;


	//constrain rate of delta_lon0
	retVal          = constrainRate(last_Delta_lon0, c_delta_lon0, 1, dt);
	last_Delta_lon0 = retVal;

	retVal += constrain(k_lonpitch*(attData_Deg.pitch - c_pitchlon0), -3, 3);

	retVal += constrain(k_lonq*attData_Deg.pitchspeed, -3.0, 3.0);

	if (mode == 1) //assit mode
	{
		if (h_r<5)
		{
			smoothMode = 1;

			//pitchhr
			if (fabs(attData_Deg.pitch)<=5)
				temp = 0;
			else if (attData_Deg.pitch<-5)
			{
				temp = attData_Deg.pitch +5;
				retVal -= constrain(k_lonpitch_hr * temp * temp, -15, 15);
			}
			else
			{
				temp = attData_Deg.pitch - 5;
				retVal += constrain(k_lonpitch_hr * temp * temp, -15, 15);
			}
		}
		else
		{
			smoothMode = 2;
		}

		//pitchcmd
		retVal += k_lonvx * (vxloncmd-vx);

		//calculate integration
		if (armResetInt || modeResetInt || (cBelow2T>5 && h_r<5))
		{
			integration = 0;
		}
		else
		{
			integration += constrain(vxloncmd-vx, -4.0, 4.0)*dt*k_ilonvx;
			integration = constrain(integration, -10.0, 10.0);
		
			retVal += integration;
		}
	}
	else //manual mode
	{
		smoothMode = 3;
		retVal += RcPitch*18.0f;
	}

	return smoothUpdate(&smoothDeltaLon, smoothMode, retVal);
}

static float controlLat(float RcRoll, float vy, float dt, int mode)
{
	static float integration = 0;
	static float lastDeltaLat0 = 0;
	static float lastVylatcmd = 0;

	float retVal = 0;
	float temp   = 0;
	uint8_t smoothMode = 0;

	float vylatcmd = interpolation(RcRoll, rollCurve, 3);

	vylatcmd = constrainRate(lastVylatcmd, vylatcmd, 3, dt);
	lastVylatcmd = vylatcmd;

	//constrain rate of deltaLat0    for power on 
	retVal = constrainRate(lastDeltaLat0, -c_deltaLat0, 1, dt);
	lastDeltaLat0 = retVal;

	retVal += constrain(k_latbank* (attData_Deg.roll - c_bankLat0), -3.0, 3.0);

	retVal += constrain(k_latp * attData_Deg.rollspeed, -2.0, 2.0);

	if (mode==1) //assist mode
	{
		if (h_r<5)
		{
			smoothMode = 1;

			//bank_hr
			if (fabs(attData_Deg.roll)<=5)
				temp = 0;
			else if(attData_Deg.roll<-5)
			{
				temp = attData_Deg.roll+5.0f;
				retVal -= constrain(k_latbank_hr * temp * temp, -10, 10);
			}
			else
			{
				temp = attData_Deg.roll - 5.0f;
				retVal += constrain(k_latbank_hr * temp * temp, -10, 10);
			}
		}
		else
		{
			smoothMode = 2;
		}

		//bankcmd
		retVal += k_latvy * (vy-vylatcmd);

		//calculate integration
		if ((cBelow2T>5 && h_r<5) || armResetInt || modeResetInt)
		{
			integration = 0;
		}
		else
		{
			integration -= constrain(vylatcmd-vy, -3.0, 3.0)*dt*k_ilatvy;
			integration = constrain(integration, -10.0, 10.0);	

			retVal += integration;
		}
	}
	else
	{
		smoothMode = 3;
		retVal -= RcRoll*12.0f;
	}

	return smoothUpdate(&smoothDeltaLat, smoothMode, retVal);
}


/**
 * [control tail collective pitch]
 * @param  RcYaw  [description]
 * @param  dt     [description]
 * @return        [description]
 */
static float controlTailClctPth(float RcYaw, float dt)
{
	// float k_tr = 0.1;
	// float k_itr = 0.5;

	float rtCmd;
	float retVal;

	static float integration = 0;


	//calculate deltaT0;
	//retVal = interpolation(deltaC, deltaT0Curve, 3);
	retVal   = c_deltaT0;  //give a const first; caitodo

	retVal += k_tr * attData_Deg.yawspeed;


	if ((cBelow2T > 5 && h_r<5) || armResetInt)  // below 2, reset integration
	{
		integration = 0;
	}
	else
	{
		//calculate integration
		rtCmd = interpolation(RcYaw, yawCurve, 3);

		integration += k_itr * (rtCmd - attData_Deg.yawspeed) * dt;

		integration = constrain(integration, -20, 20);


		retVal -= integration;
		retVal -= RcYaw*10.0f;
	}

	return retVal;
}




#ifndef WIN32
int user1_1_main(int argc, char *argv[])
{
#endif
	float vx =0, vy= 0;

	float cosYaw, sinYaw;


	float deltaLon = 0.0;
	float deltaLat = 0.0;
	float deltaT;


	//float rcRollTemp = 0;


	//command in assit mode
	// static float deltaLonAssit = 0;
	// static float deltaLatAssit = 0;
	// static float deltaCAssit = 0;

	// //command in manual mode
	// static float deltaLonManual = 0;
	// static float deltaLatManual = 0;
	// static float deltaCManual   = 0;


	static int mode = -1;
	static unsigned int counter = 0;

	static bool lastArmed = false;

	static float homeAlt = 0;


#ifndef WIN32

	initParams();


	while(1)
	{
		sem_wait(&_user_sem_100Hz);  //10ms
#else
void user1_1_main(void)
	{
#endif

		counter++;

		api_get_sl_status(&slStatus);
		if (!slStatus.arm_state)  // not armed 
		{
			lastArmed    = false;
	

			if (counter%125 == 0)
				printf("cai not Armed\n");
#ifndef WIN32
			continue;
#endif
		}

		api_get_global_pos_data(&glbPos);
		api_get_sensors_data(&sensor_data);

		//armed event,  get home altitude.
		if (lastArmed == false)
		{
			lastArmed       = true;
			homeAlt         = sensor_data.baro_alt_meter;

			armResetInt    = true;

			smoothInit(&smoothDeltaC,  2);
			smoothInit(&smoothDeltaLon, 3);
			smoothInit(&smoothDeltaLat, 3);
		}

		h_r  = sensor_data.baro_alt_meter - homeAlt;


		//updateParams();
		if (api_param_update())
		{
			getParamsValue();
		}

		//get the control input from RC, NOTE: maybe we need filter later;
		//caitodo considering RC signal lost
		if(api_get_rc_data(&rc_ctrl))
		{
			//rcRollTemp = -rc_ctrl.channels[0];
			;
		}

		if (counter%125 == 0)
		{
#if _ENABLE_PRINT_DEBUG
			printf("RCin 1~4 = %5.3f, %5.3f, %5.3f, %5.3f;\n",(double)rc_ctrl.channels[0],         (double)rc_ctrl.channels[1],(double)rc_ctrl.channels[2],(double)rc_ctrl.channels[3]);
			printf("RCin 5~8 = %5.3f, %5.3f, %5.3f, %5.3f;\n",(double)rc_ctrl.channels[4],(double)rc_ctrl.channels[5],(double)rc_ctrl.channels[6],(double)rc_ctrl.channels[7]);
#endif
			counter = 0;
		}


		//caitodo check return value;
		api_get_att_data(&attData);

		//convert radians to degree
		attData_Deg.roll       = attData.roll * 57.29578f;
		attData_Deg.pitch      = attData.pitch * 57.29578f;
		attData_Deg.yaw        = attData.yaw * 57.29578f;
		attData_Deg.rollspeed  = attData.rollspeed * 57.29578f;
		attData_Deg.pitchspeed = attData.pitchspeed * 57.29578f;
		attData_Deg.yawspeed   = attData.yawspeed * 57.29578f;
		/*attData_Deg.rollacc    = attData.rollacc * 57.29578f;
		attData_Deg.pitchacc   = attData.pitchacc * 57.29578f;
		attData_Deg.yawacc     = attData.yawacc * 57.29578f;*/



		deltaT   = controlTailClctPth(rc_ctrl.channels[3],  d_T);

		
		//assist mode
		//caitodo  change to channel 6 later.
		if (rc_ctrl.channels[7] <= 0.0f)
		{
			if (mode!=1) //mode switch event
			{
				modeResetInt = true;

				mode = 1;
				printf("Entering Assist Mode\n");
				#ifndef WIN32
				mavlink_and_console_log_info(&u1_1_mavlink_log_pub, "Entering Assist Mode\n");
				#endif
			}


			deltaC = controlDeltaC(rc_ctrl.channels[2], d_T);
			deltaC = smoothUpdate(&smoothDeltaC, 1, deltaC);


			//calculate vx and vy
			cosYaw = cos(glbPos.yaw);
			sinYaw = sin(glbPos.yaw);
			vx =  glbPos.vel_n * cosYaw + glbPos.vel_e * sinYaw;
			vy = -glbPos.vel_n * sinYaw + glbPos.vel_e * cosYaw;


			// 0=roll, 1=pitch, 2=thrust, 3=yaw;
			deltaLat = controlLat(rc_ctrl.channels[0], vy, d_T, mode);
			deltaLon = controlLonPitch(rc_ctrl.channels[1], vx, d_T, mode);
		}
		//manual mode
		else
		{
			if (mode!=0)
			{
				mode = 0;
				printf("Entering Manual Mode\n");
				#ifndef WIN32
				mavlink_and_console_log_info(&u1_1_mavlink_log_pub, "Entering Manual Mode\n");
				#endif
			}
			deltaC   = interpolation(rc_ctrl.channels[2], thrustCurve, 3);
			deltaC   = smoothUpdate(&smoothDeltaC, 2, deltaC);

			deltaLat = controlLat(rc_ctrl.channels[0], vy, d_T, mode);
			deltaLon = controlLonPitch(rc_ctrl.channels[1], vx, d_T, mode);
		}

#if _ENABLE_PRINT_DEBUG

		if (counter%200 ==0)
		{
			printf("dLat,dLon,dC,dT = %5.3f; %5.3f; %5.3f; %5.3f\n", (double)deltaLat, (double)deltaLon, (double)deltaC, (double)deltaT);
		}
#endif


		if (counter%2 == 0)
		{
			//control output distribution
			controls_value.control[0] = 2.0f*(0.0218f*deltaLon+ 0.0237f*deltaC+1.6f) - 3.0f;
			controls_value.control[1] = 2.0f*(-0.0109f*deltaLon -0.0364f*deltaLat + 0.0237f*deltaC +1.5f) - 3.0f;
			controls_value.control[2] = 2.0f* ( 0.0109f*deltaLon -0.0364f*deltaLat - 0.0237f*deltaC + 1.5f) - 3.0f;

			controls_value.control[3] = 2.0f*(0.0129f*deltaT+1.5f) - 3.0f;

			//rotor and tailrotor
			controls_value.control[4] =  (rc_ctrl.channels[4]<=0.0f)? -1:1;
			controls_value.control[5] =  (rc_ctrl.channels[5]<=0.0f)? -1:1;


			api_send_controls_data(controls_value);
		}


		// if(true == api_get_att_data(&attData))
		// {
		// 	printf("user1_1_main:roll=%6.4f pitch=%6.4f\n",(double)attData.roll,(double)attData.pitch);
		// }
		
		if (deltaC <2)
			cBelow2T += d_T;
		else
			cBelow2T = 0;

		armResetInt = false;
		modeResetInt = false;
 	}

#ifndef WIN32
	return 0;
}

void app_print_callback(void){
	printf("  app:\n");
	printf("attData:roll=%.3f pitch=%.3f yaw=%.3f yawspd=%.3f\n",(double)attData.roll,(double)attData.pitch,\
		(double)attData.yaw,(double)attData.yawspeed);	
	printf("deltaC = %.3f\n",(double)deltaC);
}
                                                  
#endif
