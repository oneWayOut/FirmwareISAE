//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ORO_COMP_PID_alt.h
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
#ifndef RTW_HEADER_ORO_COMP_PID_alt_h_
#define RTW_HEADER_ORO_COMP_PID_alt_h_

// Model header file(s)
#include "PID_alt.h"

// Orocos headers
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Property.hpp>

namespace ISAE
{
  using namespace std;
  using namespace RTT;

  // =====================================================================
  // Orocos Component class defintion
  // =====================================================================
  class PID_alt : public TaskContext , PID_altModelClass
  {
   protected:
    InputPort< real32_T > _alititude_reference_input;
    InputPort< real32_T > _alititude_measurement_input;
    OutputPort< real32_T > _direct_alti_control_output;
    void ResetAndStop();
    void DisconnectOutput();
    string _principal_output_name = "";
   public:
    PID_alt(const string & name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
  };
}

#endif                                 // RTW_HEADER_ORO_COMP_PID_alt_h_

//
// File trailer for generated code.
//
// [EOF]
//
