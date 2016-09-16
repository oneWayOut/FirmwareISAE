#include <systemlib/param/param.h>

struct params {
	//coeficients used to scale the rc input
	float rc_cmd_k[4];

	//height PID
	float h_PID[3];

	//attitude error propotional gain
	float att_P[3];

	//attitude derivative gain (to real attitude rates, not attitdue error)
	float att_D[3];

	//anti yaw (originally roll) disturbation in motor channel
	float kgp;
};

struct param_handles {
	param_t rc_roll_k;
	param_t rc_pitch_k;
	param_t rc_yaw_k;
	param_t rc_throttle_k;

	param_t h_p;
	param_t h_i;
	param_t h_d;

	param_t roll_p;
	param_t pitch_p;
	param_t yaw_p;

	param_t roll_d;
	param_t pitch_d;
	param_t yaw_d;

	param_t kgp;
};




/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct param_handles *h, struct params *p);
