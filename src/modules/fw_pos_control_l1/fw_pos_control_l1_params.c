
/**
 * @file fw_pos_control_l1_params.c
 *
 * Parameters defined by the L1 position control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 */




/**
 * Pitch rate proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_PR_P,	2.0f);

/**
 * Pitch rate integral gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_PR_I,	3.0f);

/**
 * Pitch proportional gain
 * 
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_P_P,	0.5f);

/**
 * Height proportional gain   
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_H_P,	0.1f);

/**
 * Vertical speed proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_HDOT_P,	0.03f);

/**
 * Vertical speed integral gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_HDOT_I,	0.02f);

/**
 * Max speed
 * 
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_AIRSPD_MAX,	25.0f);

/**
 * Min speed
 * 
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_AIRSPD_MIN,	15.0f);

/**
 * Cruise speed
 * 
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_AIRSPD_CRU,	20.0f);

/**
 * Forward acceleration proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_AX_P,	0.05f);

/**
 * Forward acceleration integral gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_AX_I,	0.2f);

/**
 * Airspeed proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_AIRSPD_P,	0.5f);

/**
 * Yaw proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_Y_P,	2.5f);

/**
 * Track error proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_TRACK_P,	0.01f);

/**
 * Yaw rate proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_YR_P,	1.00f);

/**
 * Track error to roll proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_TRACK2R_P,	0.005f);

/**
 * Track error to roll integral gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_TRACK2R_I,	0.002f);

/**
 * Yaw to roll proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_Y2R_P,	1.0f);

/**
 * Roll proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_R_P,	0.5f);

/**
 * Roll rate proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_RR_P,	1.0f);

/**
 * Roll rate integral gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW0_RR_I,	1.0f);








/**
 * Landing slope angle
 *
 * @unit deg
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 *
 *
 * @unit m
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HVIRT, 10.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * @unit m
 * @min 0.0
 * @max 25.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 8.0f);

/**
 * Landing throttle limit altitude (relative landing altitude)
 *
 * Default of -1.0 lets the system default to applying throttle
 * limiting at 2/3 of the flare altitude.
 *
 * @unit m
 * @min -1.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_TLALT, -1.0f);

/**
 * Landing heading hold horizontal distance
 *
 * @unit m
 * @min 0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HHDIST, 15.0f);

/**
 * Use terrain estimate during landing
 *
 * @boolean
 * @group FW L1 Control
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during flare, a positive sign means nose up
 * Applied once FW_LND_TLALT is reached
 *
 * @unit deg
 * @min 0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during flare, a positive sign means nose up
 * Applied once FW_LND_TLALT is reached
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMAX, 15.0f);

/**
 * Min. airspeed scaling factor for landing
 *
 * Multiplying this factor with the minimum airspeed of the plane
 * gives the target airspeed the landing approach.
 * FW_AIRSPD_MIN * FW_LND_AIRSPD_SC
 *
 * @unit norm
 * @min 1.0
 * @max 1.5
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_AIRSPD_SC, 1.3f);





/**
 * Climbout Altitude difference
 *
 * If the altitude error exceeds this parameter, the system will climb out
 * with maximum throttle and minimum airspeed until it is closer than this
 * distance to the desired altitude. Mostly used for takeoff waypoints / modes.
 * Set to 0 to disable climbout mode (not recommended).
 *
 * @unit m
 * @min 0.0
 * @max 150.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_CLMBOUT_DIFF, 10.0f);