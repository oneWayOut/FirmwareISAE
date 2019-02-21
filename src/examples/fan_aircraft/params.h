
/*
 * @file params.h
 *
 * Definition of parameters for fan aircraft
 */

#include <parameters/param.h>

struct fan_params {
	float cai_test;
	int   adc360_val;
};

struct fan_param_handles {
	param_t cai_test_h;
	param_t adc360_val_h;
};
