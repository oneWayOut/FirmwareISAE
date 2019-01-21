/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_config.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <mixer/mixer.h>
#include <pwm_limit/pwm_limit.h>
#include <rc/sbus.h>

#include <uORB/topics/actuator_controls.h>

#include "mixer.h"

extern "C" {
	/* #define DEBUG */
#include "px4io.h"
}

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		500000

/* current servo arm/disarm state */
static volatile bool mixer_servos_armed = false;
static volatile bool should_arm = false;
static volatile bool should_arm_nothrottle = false;
static volatile bool should_always_enable_pwm = false;
static volatile bool in_mixer = false;
static volatile uint64_t temp_fmu_recv_time = 0; 
static volatile uint64_t temp_raw_recv_time = 0; 
static volatile uint64_t last_fmu_update = 0;


extern int _sbus_fd;

/* selected control values and count for mixing */
enum mixer_source {
	MIX_NONE,
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE,
	MIX_OVERRIDE_FMU_OK
};

static volatile mixer_source source;

static int mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &control);
static int mixer_mix_threadsafe(float *outputs, volatile uint16_t *limits);

static MixerGroup mixer_group(mixer_callback, 0);
static int float_controls_to_pwm_us(uint16_t max_array[],uint16_t min_array[],uint16_t mid_array[],uint16_t* output,float *input);

int mixer_mix_threadsafe(float *outputs, volatile uint16_t *limits)
{
	/* poor mans mutex */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) == 0) {
		return 0;
	}

	in_mixer = true;
	int mixcount = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT);
	*limits = mixer_group.get_saturation_status();
	in_mixer = false;

	return mixcount;
}


static int float_controls_to_pwm_us(uint16_t max_array[],uint16_t min_array[],uint16_t mid_array[],uint16_t* output,float *input)
{
	if((max_array == NULL)|(min_array==NULL)|(mid_array==NULL)|\
		(output==NULL)|(input==NULL)){
			return -1;
	}

	for(uint16_t i=0;i<PX4IO_SERVO_COUNT;i++){
		if(fabsf(input[i]) > 1.5f){
			output[i] = 0;
		}
		else{
			if(input[i] < 0.0f){
				output[i] = mid_array[i] + (((int)mid_array[i] - (int)min_array[i]) * input[i]);
			}
			else{
				output[i] = mid_array[i] + (((int)max_array[i] - (int)mid_array[i]) * input[i]);
			}
		}
	}
	return 0;
}


void
mixer_tick(void)
{
	static float float_controls[PX4IO_SERVO_COUNT];
	static uint16_t temp_servos[PX4IO_SERVO_COUNT];
	static volatile bool need_to_init_timer = true;

	if(need_to_init_timer == true){
		up_pwm_servo_arm(true);
		need_to_init_timer = false;
	}
	PX4_CRITICAL_SECTION(temp_fmu_recv_time = system_state.fmu_data_received_time);
	PX4_CRITICAL_SECTION(temp_raw_recv_time = system_state.raw_pwm_received_time);


	/* check that we are receiving fresh data from the FMU */
	if ((temp_fmu_recv_time == 0) ||
	    hrt_elapsed_time(&temp_fmu_recv_time) > FMU_INPUT_DROP_LIMIT_US) {

		/* too long without FMU input, time to go to failsafe */
		if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {
			isr_debug(1, "AP RX timeout");
		}

		PX4_ATOMIC_MODIFY_CLEAR(r_status_flags, (PX4IO_P_STATUS_FLAGS_FMU_OK));
		PX4_ATOMIC_MODIFY_OR(r_status_alarms, PX4IO_P_STATUS_ALARMS_FMU_LOST);

	} else {
		PX4_ATOMIC_MODIFY_OR(r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_OK);
		/* this flag is never cleared once OK */
		PX4_ATOMIC_MODIFY_OR(r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED);
	}

	if (temp_fmu_recv_time != last_fmu_update) {
		last_fmu_update = temp_fmu_recv_time;
		up_pwm_update();/*this sentence is useless,because all pwm channel is not in OneShot mode*/
	}

	if(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK){
		for(uint16_t index=0;index<PX4IO_SERVO_COUNT;index++){
			float_controls[index]= REG_TO_FLOAT(r_page_controls[index]);
			if(r_setup_pwm_reverse &  (((uint16_t)1) << index) ){
				float_controls[index] *= -1.0f;
			}
		}
		float_controls_to_pwm_us(&r_page_servo_control_max[0], &r_page_servo_control_min[0], \
							&r_page_servo_control_mid[0],&temp_servos[0], &float_controls[0]);
	}else{
		for(uint16_t index=0;index<PX4IO_SERVO_COUNT;index++){
			temp_servos[index] = 0;
		}
	}
	
	if(!(r_status_flags&PX4IO_P_STATUS_FLAGS_RAW_PWM)){
		memcpy(r_page_servos, temp_servos, sizeof(uint16_t)*PX4IO_SERVO_COUNT);
	}
	else{
		memcpy(r_page_servos, r_page_direct_pwm, sizeof(uint16_t)*PX4IO_SERVO_COUNT);
		if(hrt_elapsed_time(&temp_raw_recv_time) > 500000){
			r_status_flags &= ~PX4IO_P_STATUS_FLAGS_RAW_PWM;
		}
	}

	for (uint16_t i = 0; i < PX4IO_SERVO_COUNT; i++) {
		up_pwm_servo_set(i, r_page_servos[i]);
	}
}

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	control = 0.0f;

	if (control_group >= PX4IO_CONTROL_GROUPS) {
		return -1;
	}

	switch (source) {
	case MIX_FMU:
		if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}

		return -1;

	case MIX_OVERRIDE:
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;
		}

		return -1;

	case MIX_OVERRIDE_FMU_OK:

		/* FMU is ok but we are in override mode, use direct rc control for the available rc channels. The remaining channels are still controlled by the fmu */
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;

		} else if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}

		return -1;

	case MIX_FAILSAFE:
	case MIX_NONE:
		control = 0.0f;
		return -1;
	}

	/* apply trim offsets for override channels */
	if (source == MIX_OVERRIDE || source == MIX_OVERRIDE_FMU_OK) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_ROLL) {
			control *= REG_TO_FLOAT(r_setup_scale_roll);
			control += REG_TO_FLOAT(r_setup_trim_roll);

		} else if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			   control_index == actuator_controls_s::INDEX_PITCH) {
			control *= REG_TO_FLOAT(r_setup_scale_pitch);
			control += REG_TO_FLOAT(r_setup_trim_pitch);

		} else if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			   control_index == actuator_controls_s::INDEX_YAW) {
			control *= REG_TO_FLOAT(r_setup_scale_yaw);
			control += REG_TO_FLOAT(r_setup_trim_yaw);
		}
	}

	/* limit output */
	if (control > 1.0f) {
		control = 1.0f;

	} else if (control < -1.0f) {
		control = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if ((pwm_limit.state == PWM_LIMIT_STATE_RAMP) || (should_arm_nothrottle && !should_arm)) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			control = 0.0f;
		}
	}

	/* only safety off, but not armed - set throttle as invalid */
	if (should_arm_nothrottle && !should_arm) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* mark the throttle as invalid */
			control = NAN;
		}
	}

	return 0;
}

/*
 * XXX error handling here should be more aggressive; currently it is
 * possible to get STATUS_FLAGS_MIXER_OK set even though the mixer has
 * not loaded faithfully.
 */

static char mixer_text[PX4IO_MAX_MIXER_LENGTH];		/* large enough for one mixer */
static unsigned mixer_text_length = 0;
static bool mixer_update_pending = false;

int
mixer_handle_text_create_mixer()
{
	/* only run on update */
	if (!mixer_update_pending) {
		return 0;
	}

	/* do not allow a mixer change while safety off and FMU armed */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return 1;
	}

	/* abort if we're in the mixer - it will be tried again in the next iteration */
	if (in_mixer) {
		return 1;
	}

	/* process the text buffer, adding new mixers as their descriptions can be parsed */
	unsigned resid = mixer_text_length;
	mixer_group.load_from_buf(&mixer_text[0], resid);

	/* if anything was parsed */
	if (resid != mixer_text_length) {

		isr_debug(2, "used %u", mixer_text_length - resid);

		/* copy any leftover text to the base of the buffer for re-use */
		if (resid > 0) {
			memmove(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);
			/* enforce null termination */
			mixer_text[resid] = '\0';
		}

		mixer_text_length = resid;
	}

	mixer_update_pending = false;

	return 0;
}

int
mixer_handle_text(const void *buffer, size_t length)
{
	/* do not allow a mixer change while safety off and FMU armed */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return 1;
	}

	/* disable mixing, will be enabled once load is complete */
	PX4_ATOMIC_MODIFY_CLEAR(r_status_flags, PX4IO_P_STATUS_FLAGS_MIXER_OK);

	/* abort if we're in the mixer - the caller is expected to retry */
	if (in_mixer) {
		return 1;
	}

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;

	isr_debug(2, "mix txt %u", length);

	if (length < sizeof(px4io_mixdata)) {
		return 0;
	}

	unsigned text_length = length - sizeof(px4io_mixdata);

	switch (msg->action) {
	case F2I_MIXER_ACTION_RESET:
		isr_debug(2, "reset");

		/* THEN actually delete it */
		mixer_group.reset();
		mixer_text_length = 0;

	/* FALLTHROUGH */
	case F2I_MIXER_ACTION_APPEND:
		isr_debug(2, "append %d", length);

		/* check for overflow - this would be really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			PX4_ATOMIC_MODIFY_CLEAR(r_status_flags, PX4IO_P_STATUS_FLAGS_MIXER_OK);
			return 0;
		}

		/* check if the last item has been processed - bail out if not */
		if (mixer_update_pending) {
			return 1;
		}

		/* append mixer text and nul-terminate, guard against overflow */
		memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		isr_debug(2, "buflen %u", mixer_text_length);

		/* flag the buffer as ready */
		mixer_update_pending = true;

		break;
	}

	return 0;
}

void
mixer_set_failsafe()
{
	/*
	 * Check if a custom failsafe value has been written,
	 * or if the mixer is not ok and bail out.
	 */

	if ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) ||
	    !(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {
		return;
	}

	/* set failsafe defaults to the values for all inputs = 0 */
	float	outputs[PX4IO_SERVO_COUNT];
	unsigned mixed;

	if (REG_TO_FLOAT(r_setup_slew_max) > FLT_EPSILON) {
		/* maximum value the outputs of the multirotor mixer are allowed to change in this cycle
		 * factor 2 is needed because actuator outputs are in the range [-1,1]
		 */
		float delta_out_max = 2.0f * 1000.0f * dt / (r_page_servo_control_max[0] - r_page_servo_control_min[0]) / REG_TO_FLOAT(
					      r_setup_slew_max);
		mixer_group.set_max_delta_out_once(delta_out_max);
	}

	/* update parameter for mc thrust model if it updated */
	if (update_mc_thrust_param) {
		mixer_group.set_thrust_factor(REG_TO_FLOAT(r_setup_thr_fac));
		update_mc_thrust_param = false;
	}

	/* mix */
	mixed = mixer_mix_threadsafe(&outputs[0], &r_mixer_limits);

	/* scale to PWM and update the servo outputs as required */
	for (unsigned i = 0; i < mixed; i++) {

		/* scale to servo output */
		r_page_servo_failsafe[i] = (outputs[i] * 600.0f) + 1500;

	}

	/* disable the rest of the outputs */
	for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++) {
		r_page_servo_failsafe[i] = 0;
	}

}
