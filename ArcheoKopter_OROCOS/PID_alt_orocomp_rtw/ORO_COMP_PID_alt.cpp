//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ORO_COMP_PID_alt.cpp
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
#include "ORO_COMP_PID_alt.h"

namespace ISAE
{
  // =====================================================================
  // Constructor
  // =====================================================================
  PID_alt::PID_alt(const string& name) : TaskContext(name, PreOperational)
  {
    // ==< Inputs >===============================
    this->ports()->addPort( "_alititude_reference_input",
      _alititude_reference_input ).doc(
      "_alititude_reference_input : InputPort< real32_T >" );
    this->ports()->addPort( "_alititude_measurement_input",
      _alititude_measurement_input ).doc(
      "_alititude_measurement_input : InputPort< real32_T >" );

    // ==< Outputs >==============================
    this->ports()->addPort( "_direct_alti_control_output",
      _direct_alti_control_output ).doc(
      "_direct_alti_control_output : OutputPort< real32_T >" );

    // ==< Properties >===========================
    this->addProperty("PrincipalOutput", _principal_output_name).doc(
      "Name of the principal output that will be disconnected with the DisconnectOutput() Operation.");

    // Operations
    this->addOperation( "ResetAndStop", &PID_alt::ResetAndStop, this, OwnThread)
      .doc("Reset and stop the component with a non-blocking call.");
    this->addOperation( "DisconnectOutput", &PID_alt::DisconnectOutput, this,
                       OwnThread)
      .doc("Disconnect the principal output (PrincipalOutput property) with a non-blocking call.");
  }

  // =====================================================================
  // Configuration code.
  // =====================================================================
  bool PID_alt::configureHook()
  {
    // Initialize the model with its default values
    // internal states and tunables parameters are set here
    this->initialize();

    // set to periodic execution mode.
    return this->setPeriod(0.01);
  }

  // =====================================================================
  // Start up code.
  // =====================================================================
  bool PID_alt::startHook()
  {
    // Return false to abort start up.
    return true;
  }

  // =====================================================================
  // This function is periodicaly called by the Execution Engine.
  // =====================================================================
  void PID_alt::updateHook()
  {
    // ==< Code for External Inputs >=============
    {
      // Read InputPort _alititude_reference_input
      real32_T value;
      if (_alititude_reference_input.read(value) == NewData) {
        PID_alt_U._alititude_reference_input = value;
      }
    }

    {
      // Read InputPort _alititude_measurement_input
      real32_T value;
      if (_alititude_measurement_input.read(value) == NewData) {
        PID_alt_U._alititude_measurement_input = value;
      }
    }

    // ===< Call Model baserate step >============
    this->step();

    // ==< Code for External Outputs >============
    {
      // Write OutputPort _direct_alti_control_output
      real32_T value = PID_alt_Y._direct_alti_control_output;
      _direct_alti_control_output.write(value);
    }
  }

  // =====================================================================
  // This function is called when the task is stopped.
  // =====================================================================
  void PID_alt::stopHook()
  {
    // Your stop code after last updateHook()
  }

  // =====================================================================
  // This function is called when the task is being deconfigured.
  // =====================================================================
  void PID_alt::cleanupHook()
  {
    // Cleanup model workspace if necessary
    this->terminate();
  }

  // =====================================================================
  // This operation re-initializes and stops the component.
  // =====================================================================
  void PID_alt::ResetAndStop()
  {
    // Re-initialize the model with its default values
    // internal states and tunables parameters, in case it is restarted
    this->initialize();

    // Stop the component
    this->stop();
  }

  // =====================================================================
  // This operation disconnects the principal output port.
  // =====================================================================
  void PID_alt::DisconnectOutput()
  {
    RTT::base::PortInterface* principal_output_port = this->getPort
      (_principal_output_name);
    if (principal_output_port) {
      principal_output_port->disconnect();
    } else {
      std::cerr << "PID_alt: " <<
        "The PrincipalOutput property must be defined to disconnect the principal output port."
        << std::endl;
    }
  }
}

// =====================================================================
// Orocos Component
// =====================================================================
ORO_CREATE_COMPONENT(ISAE::PID_alt);

//
// File trailer for generated code.
//
// [EOF]
//
