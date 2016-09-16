#include "params.h"


/* controller parameters, use max. 15 characters for param name! */



/**
 * Roll command scale
 *
 * TODO some description
 *
 * @unit rad
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_RCROLL_K, 1.0f);

/**
 * Pitch command scale
 *
 * TODO some description
 *
 * @unit rad
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_RCPITCH_K, 1.57f);

/**
 * Yaw command scale
 *
 * TODO some description
 *
 * @unit rad
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_RCYAW_K, 2.0f);

/**
 * Throttle command scale
 *
 * TODO some description
 *
 * @unit rad
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_RCTHRTT_K, 0.25f);


/**
 * height P
 *
 * TODO some description
 *
 * @decimal 4
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_H_P, 0.035f);

/**
 * height I
 *
 * TODO some description
 *
 * @decimal 5
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_H_I, 0.0088f);

/**
 * height D
 *
 * TODO some description
 *
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_H_D, -0.1f);


/**
 * roll p
 *
 * TODO some description
 *
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_ROLL_P, 2.0f);

/**
 * pitch p
 *
 * TODO some description
 *
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_PITCH_P, 2.0f);

/**
 * yaw p
 *
 * TODO some description
 *
 * @decimal 3
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_YAW_P, 1.0f);


/**
 * roll d
 *
 * TODO some description
 *
 * @decimal 4
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_ROLL_D, -0.35f);

/**
 * pitch d
 *
 * TODO some description
 *
 * @decimal 4
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_PITCH_D, -0.35f);

/**
 * yaw d
 *
 * TODO some description
 *
 * @decimal 4
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_YAW_D, -0.2f);

/**
 * kgp
 *
 * anti yaw (originally roll) disturbation in motor channel
 *
 * @decimal 4
 * @group VisionAir Control
 */
PARAM_DEFINE_FLOAT(VA_KGP, 0.03f);


int parameters_init(struct param_handles *h)
{
	h->rc_roll_k     = param_find("VA_RCROLL_K");
	h->rc_pitch_k    = param_find("VA_RCPITCH_K");
	h->rc_yaw_k      = param_find("VA_RCYAW_K");	
	h->rc_throttle_k = param_find("VA_RCTHRTT_K");

	h->h_p = param_find("VA_H_P");
	h->h_i = param_find("VA_H_I");
	h->h_d = param_find("VA_H_D");

	h->roll_p  = param_find("VA_ROLL_P");
	h->pitch_p = param_find("VA_PITCH_P");
	h->yaw_p   = param_find("VA_YAW_P");

	h->roll_d  = param_find("VA_ROLL_D");
	h->pitch_d = param_find("VA_PITCH_D");
	h->yaw_d   = param_find("VA_YAW_D");

	h->kgp     = param_find("VA_KGP");

	return OK;
}


#define MY_DEBUG_PARAM 0
#if MY_DEBUG_PARAM
#include <stdio.h>
#endif

int parameters_update(const struct param_handles *h, struct params *p)
{
	param_get(h->rc_roll_k,     &(p->rc_cmd_k[0]));
	param_get(h->rc_pitch_k,    &(p->rc_cmd_k[1]));
	param_get(h->rc_yaw_k,      &(p->rc_cmd_k[2]));	
	param_get(h->rc_throttle_k, &(p->rc_cmd_k[3]));

	param_get(h->h_p, &(p->h_PID[0]));
	param_get(h->h_i, &(p->h_PID[1]));
	param_get(h->h_d, &(p->h_PID[2]));

	param_get(h->roll_p,  &(p->att_P[0]));
	param_get(h->pitch_p, &(p->att_P[1]));
	param_get(h->yaw_p,   &(p->att_P[2]));

	param_get(h->roll_d,  &(p->att_D[0]));
	param_get(h->pitch_d, &(p->att_D[1]));
	param_get(h->yaw_d,   &(p->att_D[2]));

	param_get(h->kgp,     &(p->kgp));

#if MY_DEBUG_PARAM
	printf("commands_k: %.3f, %.3f, %.3f, %.3f\n", (double)p->rc_cmd_k[0], (double)p->rc_cmd_k[1], (double)p->rc_cmd_k[2], (double)p->rc_cmd_k[3]);

	printf("h pid: %.4f, %.4f, %.4f\n", (double)p->h_PID[0], (double)p->h_PID[1], (double)p->h_PID[2]);

	printf("att p: %.3f, %.3f, %.3f\n", (double)p->att_P[0], (double)p->att_P[1], (double)p->att_P[2]);

	printf("att d: %.3f, %.3f, %.3f\n", (double)p->att_D[0], (double)p->att_D[1], (double)p->att_D[2]);
#endif
	return OK;
}