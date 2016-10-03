#ifndef OROCOS_ISAE_LDC_COMP_HPP
#define OROCOS_ISAE_LDC_COMP_HPP

#include <ComponentBase.hpp>

#include <mavlink.h>
//#include <ORO_COMP_mavlink_bridge/mavlink_bridge.hpp>
#include <actuator_control_target.h>
#include <attitude.h>
#include <attitude_target.h>
#include <imu.h>
#include <battery.h>
#include <px4_aux_data.h>


//#include <px4_mode.h>
//#include "home/dmia/workspaces/workspaceQtCreator/ORO_COMP_Mav2-buildLocal/ORO_TYPE_common/px4_mode.h"
#include "../ORO_TYPE_common/px4_mode.h"
#include "../ORO_TYPE_common/fault_detection.h"
#include <Full_GPS.h>

namespace ISAE{

class LDC : public ISAE::component_base
{

private:

    //reception variable ports
    float _laser_distance;
    mavlink_manual_control_t _manual_control;
    ISAE::attitude_euler _attitude;
    ISAE::fault_detection _fault_detection;
    mavlink_highres_imu_t _imu;

    ISAE::attitude_target _att_tgt_out;
    ISAE::actuator_control_target _act_tgt_out;
    bool _control_mode;



    ISAE::battery _bat;
    ISAE::PX4_aux_data _px4_aux_data;
    ISAE::PX4_mode_flags _px4_mode = 0;
    ISAE::attitude_target _att_tgt_feedback;
    STR_GPSFUSION  _GPI_tmp;
    mavlink_local_position_ned_t  _local_pos;

    // Display Boolean
    bool _b_imu_press;
    bool _b_elr_dspl;
    bool _b_laser_dspl;
    bool _b_manual_dspl;
    bool _b_fault_dspl;

    bool _b_upt_port;

    bool _b_drt_ctrl_dspl;
    bool _b_att_ctrl_dspl;
    bool _b_ctrl_dspl;
    bool _b_gpi_dspl;

    //variable loop
    float _pow =0;
    float _incre = 0.001;
    bool _b_simul_Outputs;

    //array containing the values of the left
    float outputsValue[8] = {0,0,0,0,0,0,0,0};
    /* Information we want to grab from pixhawk */

    //laser data transaction
    float AltLaser_Average;
    float Laser1;
    float Laser2;//
    float Laser3;
    float Laser4;
    float Laser5;
    float Laser6;
    float Laser7;
    float Laser8;
    float Laser9;
    float Laser10;
    int InitLaser = 1;

    //laser control law
    float laser_alti_control =0;
    float AltLaser_Reference =20;
    float trim_laser=0;
     //PID1
    float Filter_DSTATE_laser=0;
    float DiscreteFilter_states_laser=0;
    float kp_alt_laser=0.11;
    float kd_alt_laser=0.322809213106724;
    float kn_alt_laser=122.874234041716;
    float filter_gain_laser=0.04;

    //PressureAlt control law
    float pressure_alti_control =0;
    float PresLaser_Reference =62;
    float trim_pressure=0;
     //PID1
    float Filter_DSTATE_pressure=0;
    float DiscreteFilter_states_pressure=0;
    float kp_alt_pressure=0.11;
    float kd_alt_pressure=0.322809213106724;
    float kn_alt_pressure=122.874234041716;
    float filter_gain_pressure=0.04;


    //PID2
    /*float KpLaser = 0.07538; // Gain proportionnel PIDLaser
    float KdLaser = 0.02442312; // Gain derivatif PIDLaser
    float KiLaser = 0.04; // Gain integral PIDLaser
    float Nlaser = 25.147; // Coef filtre derivee PIDLaser
    float p1 = 0;
    float p2 = 0;
    float err1 = 0;
    float err2 = 0;*/


//    std::string logHook() const;
//    std::string logHeader() const;

    // creation des port d'entrée et de sorti
    RTT::OutputPort<ISAE::actuator_control_target> _direct_control_output;
    RTT::OutputPort<ISAE::attitude_target> _attitude_control_output;
    RTT::OutputPort<bool> _control_mode_output;

    RTT::InputPort<mavlink_manual_control_t> _manual_control_input;
    RTT::InputPort<ISAE::attitude_euler> _attitude_input;
    RTT::InputPort<float> _laser_distance_input;
    RTT::InputPort<mavlink_highres_imu_t> _imu_input;
    RTT::InputPort<ISAE::fault_detection> _fault_detection_input;

    RTT::InputPort<STR_GPSFUSION> _GPI_Control_input;
    RTT::InputPort<ISAE::attitude_target> _attitude_control_feedback_input;
    RTT::InputPort<ISAE::battery> _battery_input;
    RTT::InputPort<ISAE::PX4_mode_flags> _px4_mode_input;
    RTT::InputPort<ISAE::PX4_aux_data> _px4_aux_data_input;
    RTT::InputPort<mavlink_local_position_ned_t> _local_position_input;


    //altitude modification and control
    void CalLaserAltitudeAverage(float tet, float phi,float DLaser);
    void LaserAltitudePIDcontrol(float dt);
    void PressureAltitudePIDcontrol(float dt);
    void ControlModeSwitch();

  public:
    LDC(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void resetDisplay();
    //port
    void updateDataPort();


    //fonction display
    void toggleDisplay(char i);
    void displayAttitudeEuler();
    void displayAttitudeControlFeedBack();
    void displayImuPressure();
    void displayGpi();
    void displayLaserData();
    void displayManualControl();
    void displayFaultDetect();
    void displayDirectControl();\
    void displayAttitudeControl();
    void displayControlMode();
    void splitDisplay();

    // fonction send attitude target
    void sendAttitudeTarget(ISAE::attitude_target m_tmp );
    void sendAttitudeTarget(uint8_t type_mask, float roll, float pitch, float yaw, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust /*[-1,1]*/);
    void sendAttitudeTarget_attitude(float roll, float pitch, float yaw );
    void sendAttitudeTarget_roll_rate(  float body_roll_rate);
    void sendAttitudeTarget_pitch_rate(  float body_pitch_rate);
    void sendAttitudeTarget_yaw_rate(  float body_yaw_rate);
    void sendAttitudeTarget_thrust(  float thrust);

    //fonction de modification de valeur des sortie
    void setOutputs(float tab);
    void setOutput(int nbO, float val);
    void simulOutputs();
    void sendDirectControlMSG();
    void sendControlModeMSG();


};
}/*end namespace ISAE*/
#endif /*OROCOS_ISAE_LDC_COMP_HPP*/
