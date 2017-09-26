
#include "FixedwingPositionControl.hpp"

#define G_CONST 9.81f



static unsigned int g_counter =0;

//caitodo add battery_status for thrust compensate

extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

FixedwingPositionControl *l1_control::g_control;
static int _control_task = -1;			///< task handle for sensor task */

FixedwingPositionControl::FixedwingPositionControl() :
	_actuators_0_pub(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control"))
{
	_parameter_handles._k_QE  = 	param_find("FW0_PR_P");
	_parameter_handles._k_i_QE  = 	param_find("FW0_PR_I");
	_parameter_handles._k_pitch_E  = 	param_find("FW0_P_P");
	_parameter_handles._k_HE  = 	param_find("FW0_H_P");
	_parameter_handles._k_HdotE  = 	param_find("FW0_HDOT_P");
	_parameter_handles._k_i_HdotE  = 	param_find("FW0_HDOT_I");
	_parameter_handles._k_Vmax  = 	param_find("FW0_AIRSPD_MAX");
	_parameter_handles._k_Vmin  = 	param_find("FW0_AIRSPD_MIN");
	_parameter_handles._k_Vcru  = 	param_find("FW0_AIRSPD_CRU");
	_parameter_handles._k_AxP  = 	param_find("FW0_AX_P");
	_parameter_handles._k_i_AxP  = 	param_find("FW0_AX_I");
	_parameter_handles._k_Vas  = 	param_find("FW0_AIRSPD_P");
	_parameter_handles._k_yaw_R  = 	param_find("FW0_Y_P");
	_parameter_handles._k_YR  = 	param_find("FW0_TRACK_P");
	_parameter_handles._k_RR  = 	param_find("FW0_YR_P");
	_parameter_handles._k_YA  = 	param_find("FW0_TRACK2R_P");
	_parameter_handles._k_i_YA  = 	param_find("FW0_TRACK2R_I");
	_parameter_handles._k_yaw_A  = 	param_find("FW0_Y2R_P");
	_parameter_handles._k_roll_A  = 	param_find("FW0_R_P");
	_parameter_handles._k_PA  = 	param_find("FW0_RR_P");
	_parameter_handles._k_i_PA  = 	param_find("FW0_RR_I");



	_parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
	_parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
	_parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
	_parameter_handles.land_flare_pitch_min_deg = param_find("FW_LND_FL_PMIN");
	_parameter_handles.land_flare_pitch_max_deg = param_find("FW_LND_FL_PMAX");
	_parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
	_parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
	_parameter_handles.land_use_terrain_estimate = param_find("FW_LND_USETER");
	_parameter_handles.land_airspeed_scale = param_find("FW_LND_AIRSPD_SC");

	/* fetch initial parameter values */
	parameters_update();

	_takeoff_state = -1;
	_land_state = -1;

	_actuators = {};



	//cai publish ORB_ID(fw_pos_ctrl_status) for navigator mission checker
	param_get(_parameter_handles.land_slope_angle, &(land_slope_angle));
	param_get(_parameter_handles.land_H1_virt, &(land_H1_virt));
	param_get(_parameter_handles.land_flare_alt_relative, &(land_flare_alt_relative));
	param_get(_parameter_handles.land_thrust_lim_alt_relative, &(land_thrust_lim_alt_relative));
	// param_get(_parameter_handles.land_heading_hold_horizontal_distance,
	// 	  &(land_heading_hold_horizontal_distance));
	// param_get(_parameter_handles.land_flare_pitch_min_deg, &(land_flare_pitch_min_deg));
	// param_get(_parameter_handles.land_flare_pitch_max_deg, &(land_flare_pitch_max_deg));
	// param_get(_parameter_handles.land_use_terrain_estimate, &(land_use_terrain_estimate));
	// param_get(_parameter_handles.land_airspeed_scale, &(land_airspeed_scale));
	// param_get(_parameter_handles.vtol_type, &(vtol_type));

	_landingslope.update(radians(land_slope_angle), land_flare_alt_relative,
			     land_thrust_lim_alt_relative, land_H1_virt);
	
	/* Update and publish the navigation capabilities */
	_fw_pos_ctrl_status.landing_slope_angle_rad = radians(5.0f);
	_fw_pos_ctrl_status.landing_horizontal_slope_displacement = _landingslope.horizontal_slope_displacement();
	_fw_pos_ctrl_status.landing_flare_length = _landingslope.flare_length();
	fw_pos_ctrl_status_publish();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	l1_control::g_control = nullptr;
}

int
FixedwingPositionControl::parameters_update()
{

	param_get(_parameter_handles._k_QE, &_k_QE);
	param_get(_parameter_handles._k_i_QE, &_k_i_QE);
	param_get(_parameter_handles._k_pitch_E, &_k_pitch_E);
	param_get(_parameter_handles._k_HE, &_k_HE);
	param_get(_parameter_handles._k_HdotE, &_k_HdotE);
	param_get(_parameter_handles._k_i_HdotE, &_k_i_HdotE);
	param_get(_parameter_handles._k_Vmax, &_k_Vmax);
	param_get(_parameter_handles._k_Vmin, &_k_Vmin);
	param_get(_parameter_handles._k_Vcru, &_k_Vcru);
	param_get(_parameter_handles._k_AxP, &_k_AxP);
	param_get(_parameter_handles._k_i_AxP, &_k_i_AxP);
	param_get(_parameter_handles._k_Vas, &_k_Vas);
	param_get(_parameter_handles._k_yaw_R, &_k_yaw_R);
	param_get(_parameter_handles._k_YR, &_k_YR);
	param_get(_parameter_handles._k_RR, &_k_RR);
	param_get(_parameter_handles._k_YA, &_k_YA);
	param_get(_parameter_handles._k_i_YA, &_k_i_YA);
	param_get(_parameter_handles._k_yaw_A, &_k_yaw_A);
	param_get(_parameter_handles._k_roll_A, &_k_roll_A);
	param_get(_parameter_handles._k_PA, &_k_PA);
	param_get(_parameter_handles._k_i_PA, &_k_i_PA);


	return PX4_OK;
}

/**
 * [FixedwingPositionControl::vehicle_control_mode_poll description]
 */
void FixedwingPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		_last_armed = _control_mode.flag_armed;
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}



//reset integral here

void
FixedwingPositionControl::vehicle_status_poll()
{
	static uint8_t lastMode = 0;
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		// if (_attitude_setpoint_id == nullptr) {
		// 	_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
		// }
		// 
		if (lastMode==vehicle_status_s::NAVIGATION_STATE_MANUAL && _vehicle_status.nav_state != lastMode)
		{
			reset_integral();
		}

		if (lastMode==vehicle_status_s::NAVIGATION_STATE_STAB && _vehicle_status.nav_state != lastMode)
		{
			reset_att_integral();
		}

		lastMode = _vehicle_status.nav_state;
	}
}

void
FixedwingPositionControl::vehicle_land_detected_poll()
{
	bool updated;

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

void
FixedwingPositionControl::manual_control_setpoint_poll()
{
	bool manual_updated;

	/* Check if manual setpoint has changed */
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}
}



void FixedwingPositionControl::global_pos_poll()
{
	static float lastHeight = 0.0f;
	static float lastHeightDot = 0.0f;
	bool updated;
	/* Check if manual setpoint has changed */
	orb_check(_global_pos_sub, &updated);

	if (updated) {
		/* load local copies */
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

		//cai get height and heightDot, todo, maybe we need filter later.....
		_height = lastHeight*0.95f + _global_pos.alt*0.05f;
		_heightDot = lastHeightDot*0.95f-_global_pos.vel_d*0.05f;
		lastHeight = _height;
		lastHeightDot = _heightDot;
	}
}

			




void
FixedwingPositionControl::control_state_update()
{
	static float lastAirSpd  = 0.0f;
	static float last_x_acc  = 0.0f;


	orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

	if (_ctrl_state.airspeed_valid)
	{
		_airspeed_last_received = hrt_absolute_time();
		_airspeed_valid = true;
	} else {
		/* no valid airspeed for one second */
		if ((hrt_absolute_time() - _airspeed_last_received) > 1e6) {
			_airspeed_valid = false;
			printf("airspeed invalid\n");
			mavlink_log_info(&_mavlink_log_pub, "#airpeed invalid!");
		}
	}


	if (_airspeed_valid)
	{
		_airspd_constrain = constrain(_ctrl_state.airspeed, _k_Vmin, _k_Vmax);
		_scaler = _k_Vcru/_airspd_constrain;
	} else {
		_scaler = 1.0f;
		_airspd_constrain = _k_Vcru;
	}


//filter x_acc and airspeed
	_airspd_filtered = 0.95f*lastAirSpd + 0.05f*_ctrl_state.airspeed;
	_x_acc           = 0.95f*last_x_acc + 0.05f*_ctrl_state.x_acc;

	lastAirSpd  = _airspd_filtered;
	last_x_acc  = _x_acc;



	/* set rotation matrix and euler angles */
	math::Quaternion q_att(_ctrl_state.q);
	math::Vector<3> euler_angles =  q_att.to_euler();
	_roll    = euler_angles(0);
	_pitch   = euler_angles(1);
	_yaw     = euler_angles(2);
}

void
FixedwingPositionControl::position_setpoint_triplet_poll()
{
	/* check if there is a new setpoint */
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)
		{
			if (_land_state == 0)
			{
				//note: only do this when climbing down, not flare.
				_dH = _pos_sp_triplet.previous.alt - _pos_sp_triplet.current.alt;
				_dL = get_distance_to_next_waypoint(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon, _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);
			}
		}
	}
}

void
FixedwingPositionControl::task_main_trampoline(int argc, char *argv[])
{
	l1_control::g_control = new FixedwingPositionControl();

	if (l1_control::g_control == nullptr) {
		PX4_WARN("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	l1_control::g_control->task_main();
	delete l1_control::g_control;
	l1_control::g_control = nullptr;
}

void
FixedwingPositionControl::fw_pos_ctrl_status_publish()
{
	_fw_pos_ctrl_status.timestamp = hrt_absolute_time();

	if (_fw_pos_ctrl_status_pub != nullptr) {
		orb_publish(ORB_ID(fw_pos_ctrl_status), _fw_pos_ctrl_status_pub, &_fw_pos_ctrl_status);

	} else {
		_fw_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_fw_pos_ctrl_status);
	}
}

//caitodo checkthis
float
FixedwingPositionControl::get_terrain_altitude_takeoff(float takeoff_alt,
		const vehicle_global_position_s &global_pos)
{
	if (PX4_ISFINITE(global_pos.terrain_alt) && global_pos.terrain_alt_valid) {
		return global_pos.terrain_alt;
	}

	return takeoff_alt;
}


void FixedwingPositionControl::reset_integral(void)
{
	_roll_integ = 0.0f;
	_pitch_integ = 0.0f;
	_thrust_integ = 0.5f;  // initial value is 0.5

	_height_integ = 0.0f;  //for pitch control
	_track_integ = 0.0f;  //for roll control
}

void FixedwingPositionControl::reset_att_integral(void)
{
	_height_integ = 0.0f;  //for pitch control
	_track_integ = 0.0f;  //for roll control
}




//this function is from ecl_l1_pos_controller.cpp
math::Vector<2> FixedwingPositionControl::get_local_planar_vector(const math::Vector<2> &origin, const math::Vector<2> &target) const
{
	/* this is an approximation for small angles, proposed by [2] */

	math::Vector<2> out(math::radians((target(0) - origin(0))), math::radians((target(1) - origin(1))*cosf(math::radians(origin(0)))));

	return out * static_cast<float>(CONSTANTS_RADIUS_OF_EARTH);
}

/**
 * calculate:
 *  _trackError
 *    _trackAngle; */
void FixedwingPositionControl::calcTrackInfo()
{
	math::Vector<2> vector_A;
	math::Vector<2> vector_B{(float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon};

	if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF)	{
		vector_A = _tkoff_wp;
	} else if (!_pos_sp_triplet.previous.valid) {
		vector_A = vector_B;
	} else {
		vector_A = {(float)_pos_sp_triplet.previous.lat, (float)_pos_sp_triplet.previous.lon};
	}


	math::Vector<2> vector_cur{(float)_global_pos.lat, (float)_global_pos.lon};

	math::Vector<2> vector_AB = get_local_planar_vector(vector_A, vector_B);
	/*
	 * check if waypoints are on top of each other. If yes,
	 * skip A and directly continue to B
	 */
	if (vector_AB.length() < 1.0e-6f) {
		vector_AB = get_local_planar_vector(vector_cur, vector_B);
	}

	vector_AB.normalize();

	math::Vector<2> vector_A_to_airplane = get_local_planar_vector(vector_A, vector_cur);

	//caitodo check the sign of the value
	_trackError =  -vector_A_to_airplane % vector_AB;


	if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF)	{
		_trackAngle = _tkoff_yaw;
	} else {

		//cdctodo test 
		_trackAngle = get_bearing_to_next_waypoint(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon, _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);
	}
	
	// _trackErrorVel = ground_speed_vector % vector_AB;
}


float FixedwingPositionControl::softenCmd(float lastCmd, float thisCmd, float dt, float rate)
{
	return constrain(thisCmd, lastCmd-rate*dt, lastCmd+rate*dt);
}


void FixedwingPositionControl::control_thrust(float v_dmd, float dt)
{
	//demanded acceleration X;
	float _accX_dmd   =  _k_Vas*(v_dmd - _airspd_filtered);
	_accX_dmd = constrain(_accX_dmd, -2.0f, 2.0f);

	float temp = _accX_dmd - _x_acc + G_CONST*sinf(_pitch);



	_thrust_integ += dt * _k_i_AxP * constrain(temp, -2.0f, 2.0f);
	_thrust_integ = constrain(_thrust_integ, -0.3f, 1.0f);

	float p_term = _k_AxP * temp;
	p_term       = constrain(p_term, -0.3f, 0.3f);

	_thrust_cmd = p_term + _thrust_integ;
}





/**
 * @param
 * @param
 * caitod  add mode param to this function
 */
void FixedwingPositionControl::control_pitch(int method, float height_dot_dmd, float dt)
{
	static float last_pitch_dmd = 0.0f;

	float pitch_dmd = 0.0f;

	//calculate  pitch_rate_dmd in diffrent modes
	if (method == 1) {  //take off
		pitch_dmd = radians(15.0f);
		last_pitch_dmd = pitch_dmd;
	} else 	{  // mission

		_height_integ += constrain(height_dot_dmd - _heightDot, -0.5f, 0.5f) * dt * _k_i_HdotE;
		_height_integ = constrain(_height_integ, radians(-15.0f), radians(15.0f));

		pitch_dmd = _height_integ + _k_HdotE * (height_dot_dmd - _heightDot);
		//pitch_dmd = softenCmd(last_pitch_dmd, pitch_dmd, dt, radians(2.0f));
		//caitodo change cmd constrain rate
		pitch_dmd = softenCmd(last_pitch_dmd, pitch_dmd, dt, radians(20.0f));
		pitch_dmd = constrain(pitch_dmd, radians(-15.0f), radians(15.0f));
		last_pitch_dmd = pitch_dmd;
	}


	if (_takeoff_state>=1 && g_counter%20 == 0)
	{
		printf("hdotdmd= %.3f; pitch dmd = %.3f;  ", double(height_dot_dmd), double(pitch_dmd));
		//printf("pRateDmd = %.3f; ", (double)(_k_pitch_E * _wrap_pi(pitch_dmd - _pitch)));
	}


	control_pitch_rate(pitch_dmd, dt);
}

void FixedwingPositionControl::control_pitch_rate(float pitch_dmd, float dt)
{

	float pitch_rate_dmd = 0.0f;


	pitch_rate_dmd = constrain(_k_pitch_E * (pitch_dmd - _pitch), -radians(3.0f), radians(3.0f));
	pitch_rate_dmd += (G_CONST/_airspd_constrain)*cosf(_pitch)*tanf(_roll)*sinf(_roll);

	float pitch_rate_error = _ctrl_state.pitch_rate - pitch_rate_dmd;

	// if (_takeoff_state>=1 && g_counter%20 == 0)
	// {
	// 	printf("p Rate E = %.3f\n", (double)pitch_rate_error);
	// }

	//_k_i_QE = 3.0f;  //caitodo delete
	_pitch_integ += _k_i_QE * dt* constrain(pitch_rate_error, -radians(5.0f), radians(5.0f));
	_pitch_integ = constrain(_pitch_integ, -1.0f, 1.0f);


	float p_term = _k_QE * pitch_rate_error;
	p_term = constrain(p_term, -0.5f, 0.5f);

	_pitch_cmd = _scaler * (p_term + _pitch_integ);
}



/**
 * [FixedwingPositionControl::control_roll description]
 * @param method [1=level plane; 2=normal control roll]
 * @param dt     [description]
 */
void FixedwingPositionControl::control_roll(int method, float dt)
{
	static float last_roll_dmd = 0.0f;

	
	if (method == 1)
	{
		_roll_cmd = _k_PA * _ctrl_state.roll_rate + _k_roll_A * _roll;
	}else{
		//_trackError = 0.0f;
		_track_integ += _k_i_YA * dt * constrain(_trackError, -5.0f, 5.0f);
		_track_integ = constrain(_track_integ, -radians(2.0f), radians(2.0f));


		//_trackAngle = 1.666f;
		float roll_dmd = _k_yaw_A * _wrap_pi(_trackAngle -_yaw) - _k_YA * _trackError - _track_integ;
		//roll_dmd = 0.0f;

		roll_dmd = softenCmd(last_roll_dmd, roll_dmd, dt, radians(10.0f));
		roll_dmd = constrain(roll_dmd, -radians(30.0f), radians(30.0f));
		last_roll_dmd = roll_dmd;

		control_roll_rate(roll_dmd, dt);


		// if (g_counter%20 == 0)
		// {
		// 	printf("dY = %.3f, rolldmd = %.3f;\n", double(_trackError), double(roll_dmd));
		// 	printf("trackError = %.3f\n", (double)(_wrap_pi(_trackAngle -_yaw)));
		// }
	}
	g_counter++;
}


void FixedwingPositionControl::control_roll_rate(float roll_dmd, float dt)
{
	float roll_rate_dmd = _k_roll_A * (roll_dmd - _roll) - (G_CONST/_airspd_constrain)*tanf(_roll)*sinf(_pitch);
	roll_rate_dmd  = constrain(roll_rate_dmd, -radians(10.0f), radians(10.0f));

	_roll_integ += _k_i_PA*dt * constrain(_ctrl_state.roll_rate - roll_rate_dmd, -radians(10.0f), radians(10.0f));
	
	_roll_integ = constrain(_roll_integ, -1.0f, 1.0f);

	_roll_cmd = _scaler *( _k_PA *(_ctrl_state.roll_rate - roll_rate_dmd) + _roll_integ);
}

/**
 * [FixedwingPositionControl::control_yaw description]
 * @param method  [1=  below 2 meters, ]
 * @param yaw_dmd [description]
 * @param dt      [description]
 */
void FixedwingPositionControl::control_yaw(int method, float yaw_dmd, float dt)
{
	if (method==1) {
		float yaw_term = constrain(_k_yaw_R * _wrap_pi(_yaw - yaw_dmd), radians(-5.0f), radians(5.0f));
		float track_term = constrain(_k_YR * _trackError, radians(-3.0f), radians(3.0f));

		_yaw_cmd     = (_k_Vmin/(_airspd_filtered+_k_Vmin)) * (yaw_term+track_term + _k_RR * _ctrl_state.yaw_rate);
		_wheel_cmd   = _yaw_cmd;
	} else {
		float yaw_rate_dmd = G_CONST/_airspd_constrain * tanf(_roll)*cosf(_roll)*cosf(_pitch);

		_yaw_cmd   = _scaler * _k_RR*(_ctrl_state.yaw_rate - yaw_rate_dmd);
		_wheel_cmd = 0;
	}
}



bool
FixedwingPositionControl::control_position(const math::Vector<2> &curr_pos, const math::Vector<2> &ground_speed,
		const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	
	if (!_control_mode.flag_armed)
	{
		reset_integral();
		return false;
	}

	static hrt_abstime last_called = 0;

	float dt = 0.01f;
	if (last_called > 0) {
		dt = hrt_elapsed_time(&last_called) * 1e-6f;
	}
	last_called = hrt_absolute_time();


	bool setpoint = true;

	_att_sp.fw_control_yaw = false;		// by default we don't want yaw to be contoller directly with rudder
	_att_sp.apply_flaps = false;		// by default we don't use flaps


	// l1 navigation logic breaks down when wind speed exceeds max airspeed
	// compute 2D groundspeed from airspeed-heading projection
	math::Vector<2> air_speed_2d{_airspd_filtered * cosf(_yaw), _airspd_filtered * sinf(_yaw)};
	math::Vector<2> nav_speed_2d{0.0f, 0.0f};

	// angle between air_speed_2d and ground_speed
	float air_gnd_angle = acosf((air_speed_2d * ground_speed) / (air_speed_2d.length() * ground_speed.length()));

	// if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
	if ((fabsf(air_gnd_angle) > M_PI_F) || (ground_speed.length() < 3.0f)) {
		nav_speed_2d = air_speed_2d;

	} else {
		nav_speed_2d = ground_speed;
	}


	/* save time when airplane is in air */
	if (!_was_in_air && !_vehicle_land_detected.landed) {
		_was_in_air = true;
		_time_went_in_air = hrt_absolute_time();
		_takeoff_ground_alt = _global_pos.alt;
	}

	/* reset flag when airplane landed */
	if (_vehicle_land_detected.landed && _was_in_air) {
		_was_in_air = false;

		_takeoff_state = -1;
	}



	if (_control_mode.flag_control_auto_enabled && pos_sp_curr.valid) {
		/* AUTONOMOUS FLIGHT */
		//cainote: including LOITER, RCRECOVER.....

		_control_mode_current = FW_POSCTRL_MODE_AUTO;

		/* reset hold altitude */
		_hold_alt = _global_pos.alt;

		/* reset hold yaw */
		_hdg_hold_yaw = _yaw;


		/* current waypoint (the one currently heading for) */
		math::Vector<2> curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);

		/* Initialize attitude controller integrator reset flags to 0 */
		_att_sp.roll_reset_integral = false;
		_att_sp.pitch_reset_integral = false;
		_att_sp.yaw_reset_integral = false;

		//calculate tranck Error speed, and track angle
		calcTrackInfo();


		static int statusCount = -1;
	//	static bool lastStatus = true;
		static hrt_abstime land_low_alt_T{0};
		float height_dot_dmd = 0.0f;
		switch(pos_sp_curr.type)
		{
		case position_setpoint_s::SETPOINT_TYPE_IDLE:
			_att_sp.thrust = 0.0f;
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;
			break;
		case position_setpoint_s::SETPOINT_TYPE_TAKEOFF:
			_land_state = 0;
			switch(_takeoff_state)
			{
			case -1:   //takeoff uninitialized
				if (_control_mode.flag_armed)
				{
					reset_integral();

					_takeoff_state = 0;

					_tkoff_alt   = _global_pos.alt;
					_tkoff_wp(0) = (float)_global_pos.lat;
					_tkoff_wp(1) = (float)_global_pos.lon;
					_tkoff_yaw = get_bearing_to_next_waypoint(_global_pos.lat, _global_pos.lon, _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);
					mavlink_log_info(&_mavlink_log_pub, "#cdcTakeoff started");

					printf("tk alt = %.3f, yaw = %.3f;\n", (double)_tkoff_alt,  (double)_tkoff_yaw);
				}
		
				_pitch_cmd = 0.0f;
				_roll_cmd  = 0.0f;
				_yaw_cmd   = 0.0f;
				_thrust_cmd = 0.0f;
				break;
			case 0:  //on runway and armed
				_pitch_cmd = 0.0f;

				//calculate thrust cmd
				control_thrust(_k_Vcru, dt);
				control_roll(1, dt);
				control_yaw(1, _tkoff_yaw, dt);

				if (_airspd_filtered > _k_Vmin) {
					statusCount++;
					if (statusCount>=6){
						_takeoff_state = 1;
						mavlink_log_info(&_mavlink_log_pub, "#cdcTakeoff airspeed reached");
						printf("airspeed reached");

						statusCount = 0;
					}
				} else {
					statusCount = 0;
				}
				break;
			case 1:  //reached airspeed,  front wheel liftoff
				control_thrust(_k_Vcru, dt);

				//calculate demanded pitch;
				control_pitch(1, 0.0f, dt);
				control_roll(1, dt);
				control_yaw(1, _tkoff_yaw, dt);

				if (_global_pos.alt > _tkoff_alt+15.0f) {
					statusCount++;
					if (statusCount>=6) {
						_takeoff_state = 2;
						mavlink_log_info(&_mavlink_log_pub, "#cdcNavigating to waypoint");
						printf("#cdcNavigating to waypoint");
						statusCount = 0;
					}
				} else {
					statusCount = 0;
				}
				break;
			case 2:  //consider takeoff waypoint as a common navigating waypoint!!!
				height_dot_dmd = _k_HE * (pos_sp_curr.alt - _height);
				height_dot_dmd = constrain(height_dot_dmd, -3.0f, 3.0f);
				control_pitch(2, height_dot_dmd,  dt);

				control_roll(2, dt);
				control_thrust(_k_Vcru, dt);
				control_yaw(2, _tkoff_yaw, dt);
				break;
			}

			break;
		case position_setpoint_s::SETPOINT_TYPE_POSITION:
			if (_land_state == -1)
			{
				mavlink_log_info(&_mavlink_log_pub, "#switch to waypoint");
			}
			_land_state = 0;
			// reset statusCount when waypoint transition?????
			statusCount = 0;
			

			//calculate demanded pitch rate
			height_dot_dmd = _k_HE * (pos_sp_curr.alt - _height);
			height_dot_dmd = constrain(height_dot_dmd, -3.0f, 3.0f);

			control_pitch(2, height_dot_dmd,  dt);
			control_roll(2, dt);
			control_thrust(_k_Vcru, dt);
			control_yaw(2, _trackAngle, dt);
			break;
		case position_setpoint_s::SETPOINT_TYPE_LAND:
			if (g_counter%20==0)
			{
				printf("state, dh, dL = %d, %.3f, %.3f\n",_land_state, double(_dH), double(_dL));
			}
			switch(_land_state)
			{
			case 0:
				{
				float height_dmd = 	get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);
				height_dmd *= _dH/_dL; //relative height


				height_dot_dmd = _k_HE *(height_dmd+ _tkoff_alt - _global_pos.alt) + ground_speed.length() *tanf(_dH/_dL);
				height_dot_dmd = constrain(height_dot_dmd, -3.0f, 3.0f);
				control_pitch(2, height_dot_dmd, dt);

				control_roll(2, dt);
				control_thrust(15.0f, dt);
				control_yaw(2, _trackAngle, dt);

				if (_global_pos.alt < _tkoff_alt +15.0f){
					statusCount++;
					if (statusCount>=6)	{
						_land_state = 1;

						mavlink_log_info(&_mavlink_log_pub, "#land flare!!");
						statusCount = 0;
					}
				} else{
					statusCount = 0;
				}


				}
				break;
			case 1:  //flare
				height_dot_dmd = ground_speed.length() *tanf(_dH/_dL) * expf(0.1f*(_global_pos.alt- _tkoff_alt-15.0f));
				if (height_dot_dmd > -0.4f)
				{
					height_dot_dmd = -0.4f;
				}
				height_dot_dmd = constrain(height_dot_dmd, -3.0f, 3.0f);
				control_pitch(2, height_dot_dmd, dt);
				control_roll(2, dt);
				control_thrust(_k_Vmin, dt);
				control_yaw(2, _trackAngle, dt);

				if (_global_pos.alt < _tkoff_alt +2.0f) {
					statusCount++;
					if (statusCount>=6) {
						_land_state = 2;
						mavlink_log_info(&_mavlink_log_pub, "#landing to ground!!");
						statusCount = 0;
					}
				} else {
					statusCount = 0;
				}
				break;
			case 2:  // landing to ground!!
				if (_global_pos.alt < _tkoff_alt +2.0f) {
					//stop motors if height below 2m for 5 seconds!!
					if (hrt_absolute_time() - land_low_alt_T > 5000000)
					{
						if (_land_state != -1)
						{
							mavlink_log_info(&_mavlink_log_pub, "#landed, stop motors!!");
						}
						_roll_cmd = 0.0f;
						_pitch_cmd = 0.0f;
						_yaw_cmd = 0.0f;
						_thrust_cmd = 0.0f;
						_wheel_cmd = 0.0f;
						_land_state = -1;

						//reset all integrators!!
						reset_integral();
						break;  // break the control calculation in advance!!!!
					}
				} else {
					land_low_alt_T = hrt_absolute_time();
				}


				height_dot_dmd = ground_speed.length() *tanf(_dH/_dL) * expf(0.1f*(_global_pos.alt- _tkoff_alt-15.0f));
				if (height_dot_dmd > -0.4f)
				{
					height_dot_dmd = -0.4f;
				}
				height_dot_dmd = constrain(height_dot_dmd, -3.0f, 3.0f);
				control_pitch(2, height_dot_dmd, dt);

				control_roll(1, dt);
				control_thrust(8.0f, dt);
				control_yaw(1, _trackAngle, dt);
				break;
			default :
				break;
			}
			break;
		default:
			break;
		}
	} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB) {
		/* in stablize  mode */
		manual_control_setpoint_poll();

		//caitodo check sign
		control_roll_rate(_manual.y*radians(20.0f), dt);

		control_pitch_rate(-_manual.x*radians(15.0f), dt);
		_yaw_cmd   = _scaler * _k_RR* _ctrl_state.yaw_rate + _manual.r;

		_thrust_cmd = _manual.z;

		//reset part of the integral
	} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL) {
		/* manual/direct control */
		manual_control_setpoint_poll();

		_roll_cmd = _manual.y ;
		_pitch_cmd = -_manual.x;  //caitodo check sign here
		_yaw_cmd = _manual.r;
		_thrust_cmd = _manual.z;
		//caitodo check wheel control channel
		//
		//todo reset integer
	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;

		/* do not publish the setpoint */
		setpoint = false;

		// reset hold altitude
		_hold_alt = _global_pos.alt;

	}


	return setpoint;
}


void
FixedwingPositionControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vehicle_status_sub, 200);
	/* rate limit vehicle land detected updates to 5Hz */
	orb_set_interval(_vehicle_land_detected_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update() != PX4_OK) {
		/* parameter setup went wrong, abort */
		PX4_WARN("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;

	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		vehicle_control_mode_poll();

		vehicle_land_detected_poll();
		vehicle_status_poll();

		/* only update parameters if they changed */
		if ((fds[0].revents & POLLIN) != 0) {
			/* read from param to clear updated flag */
			parameter_update_s update {};
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if position changed */
		if ((fds[1].revents & POLLIN) != 0) {
			perf_begin(_loop_perf);

			control_state_update();

			global_pos_poll();
			position_setpoint_triplet_poll();

			math::Vector<2> curr_pos((float)_global_pos.lat, (float)_global_pos.lon);
			math::Vector<2> ground_speed(_global_pos.vel_n, _global_pos.vel_e);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current)) {
				//_att_sp.timestamp = hrt_absolute_time();


				_actuators.control[actuator_controls_s::INDEX_ROLL]  = constrain(-_roll_cmd, -1.0f, 1.0f);
				_actuators.control[actuator_controls_s::INDEX_PITCH]  = constrain(-_pitch_cmd, -1.0f, 1.0f);
				_actuators.control[actuator_controls_s::INDEX_YAW]  = constrain(_yaw_cmd, -1.0f, 1.0f);
				_actuators.control[actuator_controls_s::INDEX_THROTTLE]  = constrain(_thrust_cmd, 0.0f, 1.0f);
				_actuators.control[actuator_controls_s::INDEX_FLAPS]  = constrain(_wheel_cmd, -1.0f, 1.0f);

				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;



				_actuators.control[actuator_controls_s::INDEX_FLAPS] = 0.0f;

				static int mycounter = 0;
				if (_takeoff_state>=1)
				{

					if (mycounter%20 == 0)
						printf("roll = %.3f; pitch = %.3f, yaw= %.3f;\n",  (double)_roll_cmd, (double)_pitch_cmd, (double)_yaw_cmd);

					
				}






				mycounter++;

				
		
				if (_actuators_0_pub != nullptr) {
					orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
				} else {
					_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
				}

				//caitodo check if we need to publish ORB_ID(fw_pos_ctrl_status)
				
				// /* XXX check if radius makes sense here */
				// float turn_distance = _l1_control.switch_distance(100.0f);

				// /* lazily publish navigation capabilities */
				// if ((hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) > 1000000)
				//     || (fabsf(turn_distance - _fw_pos_ctrl_status.turn_distance) > FLT_EPSILON
				// 	&& turn_distance > 0)) {

				// 	/* set new turn distance */
				// 	_fw_pos_ctrl_status.turn_distance = turn_distance;

				// 	_fw_pos_ctrl_status.nav_roll = _l1_control.nav_roll();
				// 	_fw_pos_ctrl_status.nav_pitch = get_tecs_pitch();
				// 	_fw_pos_ctrl_status.nav_bearing = _l1_control.nav_bearing();

				// 	_fw_pos_ctrl_status.target_bearing = _l1_control.target_bearing();
				// 	_fw_pos_ctrl_status.xtrack_error = _l1_control.crosstrack_error();

				// 	math::Vector<2> curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);

				// 	_fw_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(curr_pos(0), curr_pos(1), curr_wp(0), curr_wp(1));

				// 	fw_pos_ctrl_status_publish();
				// }
			}

			perf_end(_loop_perf);
		}
	}

	_task_running = false;

	PX4_WARN("exiting.\n");

	_control_task = -1;
}




int
FixedwingPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("fw_pos_ctrl_l1",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&FixedwingPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int fw_pos_control_l1_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: fw_pos_control_l1 {start|stop|status}");
		return 1;
	}

	if (strcmp(argv[1], "start") == 0) {

		if (l1_control::g_control != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		if (OK != FixedwingPositionControl::start()) {
			warn("start failed");
			return 1;
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (l1_control::g_control == nullptr || !l1_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}

		printf("\n");

		return 0;
	}

	if (strcmp(argv[1], "stop") == 0) {
		if (l1_control::g_control == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		delete l1_control::g_control;
		l1_control::g_control = nullptr;
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		if (l1_control::g_control != nullptr) {
			PX4_INFO("running");
			return 0;
		}

		PX4_WARN("not running");
		return 1;
	}

	PX4_WARN("unrecognized command");
	return 1;
}
