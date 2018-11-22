
/*
 * @file params.h
 *
 * Definition of parameters for fan aircraft
 */

#include <parameters/param.h>

struct params {
	float hdng_p;
	float roll_p;
	float pitch_p;
};

struct param_handles {
	param_t hdng_p;
	param_t roll_p;
	param_t pitch_p;
};
