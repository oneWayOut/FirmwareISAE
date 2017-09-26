

#ifndef FIXEDWINGPOSITIONCONTROL_HPP_
#define FIXEDWINGPOSITIONCONTROL_HPP_

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <cfloat>

#include "Landingslope.hpp"

#include <drivers/drv_hrt.h>
//#include <ecl/l1/ecl_l1_pos_controller.h>
//#include <external_lgpl/tecs/tecs.h>
#include <geo/geo.h>
//#include <launchdetection/LaunchDetector.h>
#include <mathlib/mathlib.h>
//#include <runway_takeoff/RunwayTakeoff.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
//#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

//cai new added
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

#define HDG_HOLD_DIST_NEXT 		3000.0f 	// initial distance of waypoint in front of plane in heading hold mode
#define HDG_HOLD_REACHED_DIST 		1000.0f 	// distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
#define HDG_HOLD_SET_BACK_DIST 		100.0f 		// distance by which previous waypoint is set behind the plane
#define HDG_HOLD_YAWRATE_THRESH 	0.15f 		// max yawrate at which plane locks yaw for heading hold mode
#define HDG_HOLD_MAN_INPUT_THRESH 	0.01f 		// max manual roll/yaw input from user which does not change the locked heading
#define T_ALT_TIMEOUT 			1		// time after which we abort landing if terrain estimate is not valid
#define THROTTLE_THRESH 0.05f				///< max throttle from user which will not lead to motors spinning up in altitude controlled modes
#define MANUAL_THROTTLE_CLIMBOUT_THRESH 0.85f		///< a throttle / pitch input above this value leads to the system switching to climbout mode
#define ALTHOLD_EPV_RESET_THRESH 5.0f

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

//using namespace launchdetection;
//using namespace runwaytakeoff;

class FixedwingPositionControl
{
public:
	FixedwingPositionControl();
	~FixedwingPositionControl();
	FixedwingPositionControl(const FixedwingPositionControl &) = delete;
	FixedwingPositionControl operator=(const FixedwingPositionControl &other) = delete;

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	static int	start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	math::Matrix<3, 3> _R_nb;				///< current attitude
	float _roll{0.0f};
	float _pitch{0.0f};
	float _yaw{0.0f};

	//cai add:
	float _roll_cmd{0.0f};
	float _pitch_cmd{0.0f};
	float _yaw_cmd{0.0f};
	float _thrust_cmd{0.0f};
	float _wheel_cmd{0.0f};

	float _roll_integ{0.0f};
	float _pitch_integ{0.0f};
	float _thrust_integ{0.5f};  // initial value is 0.5

	float _height_integ{0.0f};  //for pitch control
	float _track_integ{0.0f};  //for roll control

	float _scaler{0.0f};   //   Vcru/V
	float _airspd_constrain{0.0f};
	float _x_acc{0.0f};             //filtered x_acc
	float _airspd_filtered{0.0f};   //filtered airspeed


	float _height{0.0f};
	float _heightDot{0.0f};

	float _trackError{0.0f};
	float _trackErrorVel{0.0f};
	float _trackAngle{0.0f};

	float _tkoff_yaw{0.0f};
	float _tkoff_alt{0.0f};
	math::Vector<2> _tkoff_wp;
	int _takeoff_state{-1};  //-1=uninitialized takeoff waypoint; 
	                          //0=slide on runway; 1=reached airspeed; 2=fly;

	int _land_state{-1};  //-1 = at land; 0=fly; 1=flare; 2=low than 2 meters;  


	float _dH{0.0f};
	float _dL{0.0f};

	bool _last_armed{false};


	orb_advert_t	_actuators_0_pub;
	struct actuator_controls_s	_actuators;		/**< actuator control inputs */

	Landingslope _landingslope;   //caitodo delete later



	control_state_s			_ctrl_state {};			///< control state */
	manual_control_setpoint_s	_manual {};			///< r/c channel data */
	position_setpoint_triplet_s	_pos_sp_triplet {};		///< triplet of mission items */
	vehicle_global_position_s	_global_pos {};			///< global vehicle position */
	vehicle_attitude_setpoint_s	_att_sp {};			///< vehicle attitude setpoint */

	vehicle_command_s		_vehicle_command {};		///< vehicle commands */
	vehicle_control_mode_s		_control_mode {};		///< control mode */
	vehicle_status_s		_vehicle_status {};		///< vehicle status */

	fw_pos_ctrl_status_s		_fw_pos_ctrl_status {};		///< navigation capabilities */
	vehicle_land_detected_s		_vehicle_land_detected {};	///< vehicle land detected */
	



	orb_advert_t	_mavlink_log_pub{nullptr};

	bool		_task_should_exit{false};		///< if true, sensor task should exit */
	bool		_task_running{false};			///< if true, task is running in its mainloop */

	int		_global_pos_sub{-1};
	int		_pos_sp_triplet_sub{-1};
	int		_ctrl_state_sub{-1};			///< control state subscription */
	int		_control_mode_sub{-1};			///< control mode subscription */
	int		_vehicle_command_sub{-1};		///< vehicle command subscription */
	int		_vehicle_status_sub{-1};		///< vehicle status subscription */
	int		_vehicle_land_detected_sub{-1};		///< vehicle land detected subscription */
	int		_params_sub{-1};			///< notification of parameter updates */
	int		_manual_control_sub{-1};		///< notification of manual control updates */

	orb_advert_t	_attitude_sp_pub{nullptr};		///< attitude setpoint */
	orb_advert_t	_fw_pos_ctrl_status_pub{nullptr};	///< navigation capabilities publication */





	perf_counter_t	_loop_perf;				///< loop performance counter */

	float	_hold_alt{0.0f};				///< hold altitude for altitude mode */
	float	_takeoff_ground_alt{0.0f};			///< ground altitude at which plane was launched */
	float	_hdg_hold_yaw{0.0f};				///< hold heading for velocity mode */
	bool	_hdg_hold_enabled{false};			///< heading hold enabled */
	bool	_yaw_lock_engaged{false};			///< yaw is locked for heading hold */
	float	_althold_epv{0.0f};				///< the position estimate accuracy when engaging alt hold */
	bool	_was_in_deadband{false};			///< wether the last stick input was in althold deadband */

	position_setpoint_s _hdg_hold_prev_wp {};		///< position where heading hold started */
	position_setpoint_s _hdg_hold_curr_wp {};		///< position to which heading hold flies */





	hrt_abstime _time_started_landing{0};			///< time at which landing started */

	float _t_alt_prev_valid{0};				///< last terrain estimate which was valid */
	hrt_abstime _time_last_t_alt{0};			///< time at which we had last valid terrain alt */

	float _flare_height{0.0f};				///< estimated height to ground at which flare started */
	float _flare_curve_alt_rel_last{0.0f};
	float _target_bearing{0.0f};				///< estimated height to ground at which flare started */

	bool _was_in_air{false};				///< indicated wether the plane was in the air in the previous interation*/
	hrt_abstime _time_went_in_air{0};			///< time at which the plane went in the air */

	/* throttle and airspeed states */
	bool _airspeed_valid{false};				///< flag if a valid airspeed estimate exists
	hrt_abstime _airspeed_last_received{0};			///< last time airspeed was received. Used to detect timeouts.

	float _groundspeed_undershoot{0.0f};			///< ground speed error to min. speed in m/s





	// estimator reset counters
	uint8_t _pos_reset_counter{0};				///< captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter{0};				///< captures the number of times the estimator has reset the altitude state


	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_POSITION,
		FW_POSCTRL_MODE_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER};		///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.


/******* parameters  ***************/
	float _k_QE;
	float _k_i_QE;
	float _k_pitch_E;
	float _k_HE;
	float _k_HdotE;
	float _k_i_HdotE;
	float _k_Vmax;
	float _k_Vmin;
	float _k_Vcru;
	float _k_AxP;
	float _k_i_AxP;
	float _k_Vas;
	float _k_yaw_R;
	float _k_YR;
	float _k_RR;
	float _k_YA;
	float _k_i_YA;
	float _k_yaw_A;
	float _k_roll_A;
	float _k_PA;
	float _k_i_PA;
	
	//caitodo delete later
	float land_slope_angle;
	float land_H1_virt;
	float land_flare_alt_relative;
	float land_flare_pitch_min_deg;
	float land_flare_pitch_max_deg;
	float land_thrust_lim_alt_relative;
	float land_heading_hold_horizontal_distance;
	float land_use_terrain_estimate;
	float land_airspeed_scale;
/******* parameters  ***************/

	struct {
		param_t _k_QE;
		param_t _k_i_QE;
		param_t _k_pitch_E;
		param_t _k_HE;
		param_t _k_HdotE;
		param_t _k_i_HdotE;
		param_t _k_Vmax;
		param_t _k_Vmin;
		param_t _k_Vcru;
		param_t _k_AxP;
		param_t _k_i_AxP;
		param_t _k_Vas;
		param_t _k_yaw_R;
		param_t _k_YR;
		param_t _k_RR;
		param_t _k_YA;
		param_t _k_i_YA;
		param_t _k_yaw_A;
		param_t _k_roll_A;
		param_t _k_PA;
		param_t _k_i_PA;

		/*caitodo to delete later, land*/
		param_t land_slope_angle;
		param_t land_H1_virt;
		param_t land_flare_alt_relative;
		param_t land_flare_pitch_min_deg;
		param_t land_flare_pitch_max_deg;
		param_t land_thrust_lim_alt_relative;
		param_t land_heading_hold_horizontal_distance;
		param_t land_use_terrain_estimate;
		param_t land_airspeed_scale;
	} _parameter_handles {};				///< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void    reset_integral(void);
	void    reset_att_integral(void);
	void    calcTrackInfo();
	float   softenCmd(float lastCmd, float thisCmd, float dt, float rate);
	void    control_thrust(float v_dmd, float dt);
	void    control_pitch(int method, float height_dot_dmd, float dt);
	void    control_roll(int method, float dt);
	void    control_yaw(int method, float yaw_dmd, float dt);
	void    control_pitch_rate(float pitch_dmd, float dt);
	void    control_roll_rate(float roll_dmd, float dt);

	math::Vector<2> get_local_planar_vector(const math::Vector<2> &origin, const math::Vector<2> &target) const;

	// Update subscriptions
	void		control_state_update();
	void        global_pos_poll();
	void		manual_control_setpoint_poll();
	void		position_setpoint_triplet_poll();
	void		vehicle_command_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_land_detected_poll();
	void		vehicle_status_poll();

	// publish navigation capabilities
	void		fw_pos_ctrl_status_publish();

	
	/**
	 * Return the terrain estimate during takeoff or takeoff_alt if terrain estimate is not available
	 */
	float		get_terrain_altitude_takeoff(float takeoff_alt, const vehicle_global_position_s &global_pos);



	bool		control_position(const math::Vector<2> &curr_pos, const math::Vector<2> &ground_speed,
					 const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	void		reset_landing_state();
};

namespace l1_control
{
extern FixedwingPositionControl *g_control;
} // namespace l1_control

#endif // FIXEDWINGPOSITIONCONTROL_HPP_
