/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file pwm_params_main.c
 *
 * Parameters defined for PWM output.
 *
 */

/**
 * Invert direction of main output channel 1
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV1, 0);

/**
 * Invert direction of main output channel 2
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV2, 0);

/**
 * Invert direction of main output channel 3
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV3, 0);

/**
 * Invert direction of main output channel 4
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV4, 0);

/**
 * Invert direction of main output channel 5
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV5, 0);

/**
 * Invert direction of main output channel 6
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV6, 0);

/**
 * Invert direction of main output channel 7
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV7, 0);

/**
 * Invert direction of main output channel 8
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV8, 0);

/**
 * Invert direction of main output channel 9
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV9, 0);

/**
 * Invert direction of main output channel 10
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV10, 0);

/**
 * Invert direction of main output channel 11
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV11, 0);

/**
 * Invert direction of main output channel 12
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV12, 0);

/**
 * Invert direction of main output channel 13
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV13, 0);

/**
 * Invert direction of main output channel 14
 *
 * Enable to invert the channel.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_REV14, 0);

/**
 * Set the minimum PWM for main output channel 1
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN1, 1000);

/**
 * Set the minimum PWM for main output channel 2
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN2, 1000);

/**
 * Set the minimum PWM for main output channel 3
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN3, 1000);

/**
 * Set the minimum PWM for main output channel 4
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN4, 1000);

/**
 * Set the minimum PWM for main output channel 5
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN5, 1000);

/**
 * Set the minimum PWM for main output channel 6
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN6, 1000);

/**
 * Set the minimum PWM for main output channel 7
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN7, 1000);

/**
 * Set the minimum PWM for main output channel 8
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN8, 1000);

/**
 * Set the minimum PWM for main output channel 9
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN9, 1000);

/**
 * Set the minimum PWM for main output channel 10
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN10, 1000);

/**
 * Set the minimum PWM for main output channel 11
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN11, 1000);

/**
 * Set the minimum PWM for main output channel 12
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN12, 1000);

/**
 * Set the minimum PWM for main output channel 13
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN13, 1000);

/**
 * Set the minimum PWM for main output channel 14
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN14, 1000);

/**
 * Set the maximum PWM for main outputs channel 1
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX1, 2000);

/**
 * Set the maximum PWM for main outputs channel 2
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX2, 2000);

/**
 * Set the maximum PWM for main outputs channel 3
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX3, 2000);

/**
 * Set the maximum PWM for main outputs channel 4
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX4, 2000);

/**
 * Set the maximum PWM for main outputs channel 5
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX5, 2000);

/**
 * Set the maximum PWM for main outputs channel 6
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX6, 2000);

/**
 * Set the maximum PWM for main outputs channel 7
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX7, 2000);

/**
 * Set the maximum PWM for main outputs channel 8
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX8, 2000);

/**
 * Set the maximum PWM for main outputs channel 9
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX9, 2000);

/**
 * Set the maximum PWM for main outputs channel 10
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX10, 2000);

/**
 * Set the maximum PWM for main outputs channel 11
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX11, 2000);

/**
 * Set the maximum PWM for main outputs channel 12
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX12, 2000);

/**
 * Set the maximum PWM for main outputs channel 13
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX13, 2000);

/**
 * Set the maximum PWM for main outputs channel 14
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX14, 2000);

/**
 * Set the middle PWM for main outputs channel 1
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID1, 1500);

/**
 * Set the middle PWM for main outputs channel 2
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID2, 1500);

/**
 * Set the middle PWM for main outputs channel 3
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID3, 1500);

/**
 * Set the middle PWM for main outputs channel 4
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID4, 1500);

/**
 * Set the middle PWM for main outputs channel 5
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID5, 1500);

/**
 * Set the middle PWM for main outputs channel 6
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID6, 1500);

/**
 * Set the middle PWM for main outputs channel 7
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID7, 1500);

/**
 * Set the middle PWM for main outputs channel 8
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID8, 1500);

/**
 * Set the middle PWM for main outputs channel 9
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID9, 1500);

/**
 * Set the middle PWM for main outputs channel 10
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID10, 1500);

/**
 * Set the middle PWM for main outputs channel 11
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID11, 1500);

/**
 * Set the middle PWM for main outputs channel 12
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID12, 1500);

/**
 * Set the middle PWM for main outputs channel 13
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID13, 1500);

/**
 * Set the middle PWM for main outputs channel 14
 *
 * Set to 1500 for industry default.
 *
 * @reboot_required true
 *
 * @min 1250
 * @max 1750
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MID14, 1500);

