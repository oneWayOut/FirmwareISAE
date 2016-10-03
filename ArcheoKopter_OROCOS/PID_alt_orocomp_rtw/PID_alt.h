//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID_alt.h
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
#ifndef RTW_HEADER_PID_alt_h_
#define RTW_HEADER_PID_alt_h_
#include <string.h>
#ifndef PID_alt_COMMON_INCLUDES_
# define PID_alt_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // PID_alt_COMMON_INCLUDES_

#include "PID_alt_types.h"

using namespace ISAE;

// Macros for accessing real-time model data structure

// Block states (auto storage) for system '<Root>'
typedef struct {
  real32_T Filter_DSTATE;              // '<S2>/Filter'
  real32_T DiscreteFilter_states;      // '<S1>/Discrete Filter'
} DW_PID_alt_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real32_T _alititude_reference_input; // '<Root>/_alititude_reference_input'
  real32_T _alititude_measurement_input;// '<Root>/_alititude_measurement_input' 
} ExtU_PID_alt_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real32_T _direct_alti_control_output;// '<Root>/_direct_alti_control_output'
} ExtY_PID_alt_T;

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model PID_alt
class PID_altModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_PID_alt_T PID_alt_U;

  // External outputs
  ExtY_PID_alt_T PID_alt_Y;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  PID_altModelClass();

  // Destructor
  ~PID_altModelClass();

  // protected data and function members
 protected:
  // Block states
  DW_PID_alt_T PID_alt_DW;

  // private data and function members
 private:
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PID_alt'
//  '<S1>'   : 'PID_alt/PID_archeo'
//  '<S2>'   : 'PID_alt/PID_archeo/PID Controller'

#endif                                 // RTW_HEADER_PID_alt_h_

//
// File trailer for generated code.
//
// [EOF]
//
