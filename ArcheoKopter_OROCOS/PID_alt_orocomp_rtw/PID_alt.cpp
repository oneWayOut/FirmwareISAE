//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID_alt.cpp
//
// Code generated for Simulink model 'PID_alt'.
//
// Model version                  : 1.3
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Fri Aug 26 15:21:38 2016
//
// Target selection: orocomp.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "PID_alt.h"
#include "PID_alt_private.h"

// Model step function
void PID_altModelClass::step()
{
  real32_T rtb_FilterCoefficient;
  real32_T Sum1;

  // Sum: '<S1>/Sum1' incorporates:
  //   Inport: '<Root>/_alititude_measurement_input'
  //   Inport: '<Root>/_alititude_reference_input'

  Sum1 = PID_alt_U._alititude_reference_input -
    PID_alt_U._alititude_measurement_input;

  // Gain: '<S2>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S2>/Filter'
  //   Gain: '<S2>/Derivative Gain'
  //   Sum: '<S2>/SumD'

  rtb_FilterCoefficient = (0.322809219F * Sum1 - PID_alt_DW.Filter_DSTATE) *
    122.874237F;

  // Outport: '<Root>/_direct_alti_control_output' incorporates:
  //   DiscreteFilter: '<S1>/Discrete Filter'
  //   Gain: '<S1>/Gain2'
  //   Gain: '<S2>/Proportional Gain'
  //   Sum: '<S1>/Sum3'
  //   Sum: '<S2>/Sum'

  PID_alt_Y._direct_alti_control_output = (Sum1 + rtb_FilterCoefficient) * 0.11F
    + 0.01F * PID_alt_DW.DiscreteFilter_states * 0.04F;

  // Update for DiscreteIntegrator: '<S2>/Filter'
  PID_alt_DW.Filter_DSTATE += 0.01F * rtb_FilterCoefficient;

  // Update for DiscreteFilter: '<S1>/Discrete Filter'
  PID_alt_DW.DiscreteFilter_states = Sum1 - (-PID_alt_DW.DiscreteFilter_states);
}

// Model initialize function
void PID_altModelClass::initialize()
{
  // Registration code

  // states (dwork)
  (void) memset((void *)&PID_alt_DW, 0,
                sizeof(DW_PID_alt_T));

  // external inputs
  (void) memset((void *)&PID_alt_U, 0,
                sizeof(ExtU_PID_alt_T));

  // external outputs
  PID_alt_Y._direct_alti_control_output = 0.0F;
}

// Model terminate function
void PID_altModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
PID_altModelClass::PID_altModelClass()
{
}

// Destructor
PID_altModelClass::~PID_altModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
