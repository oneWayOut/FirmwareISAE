
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
PARAM_DEFINE_FLOAT(FW_PR_P,	2.0f);

/**
 * Pitch rate integral gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I,	3.0f);

/**
 * Pitch proportional gain
 * 
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_P_P,	0.5f);

/**
 * Height proportional gain   
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_H_P,	0.1f);

/**
 * Vertical speed proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_HDOT_P,	0.03f);

/**
 * Vertical speed integral gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_HDOT_I,	0.02f);

/**
 * Max speed
 * 
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX,	20f);

/**
 * Min speed
 * 
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN,	10f);

/**
 * Cruise speed
 * 
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_CRU,	15f);

/**
 * Forward acceleration proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_AX_P,	0.05f);

/**
 * Forward acceleration integral gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_AX_I,	0.2f);

/**
 * Airspeed proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_P,	0.5f);

/**
 * Yaw proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_Y_P,	2.5f);

/**
 * Track error proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_TRACK_P,	0.01f);

/**
 * Yaw rate proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P,	1.00f);

/**
 * Track error to roll proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_TRACK2R_P,	0.005f);

/**
 * Track error to roll integral gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_TRACK2R_I,	0.002f);

/**
 * Yaw to roll proportional gain
 * 
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_Y2R_P,	1.0f);

/**
 * Roll proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_R_P,	0.5f);

/**
 * Roll rate proportional gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P,	1.0f);

/**
 * Roll rate integral gain
 * 
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group FW PLANE Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I,	1.0f);
