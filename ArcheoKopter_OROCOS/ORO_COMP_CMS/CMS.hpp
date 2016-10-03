#ifndef OROCOS_ISAE_CMS_COMP_HPP
#define OROCOS_ISAE_CMS_COMP_HPP

#include <ComponentBase.hpp>

#include <mavlink.h>
//#include <ORO_COMP_mavlink_bridge/mavlink_bridge.hpp>
#include <actuator_control_target.h>
#include <attitude.h>
#include <attitude_target.h>
#include <imu.h>
#include <battery.h>
#include <px4_aux_data.h>
#include <fstream>





//#include <px4_mode.h>
//#include "home/dmia/workspaces/workspaceQtCreator/ORO_COMP_Mav2-buildLocal/ORO_TYPE_common/px4_mode.h"
#include "../ORO_TYPE_common/px4_mode.h"
#include "../ORO_TYPE_common/fault_detection.h"
#include "../ORO_TYPE_common/fault_code.h"
#include <Full_GPS.h>

namespace ISAE{

class CMS : public ISAE::component_base
{

private:

    //reception variable ports
    float _laser_distance;
    ISAE::attitude_euler _attitude;
    mavlink_highres_imu_t _imu;
    ISAE::fault_detection _fault_detection;
    //fstream MFR;


    // Display Boolean
    bool _b_imu_press;
    bool _b_elr_dspl;
    bool _b_laser_dspl;
    bool _b_fault_dspl;

    bool _b_upt_port;

    //laser data transaction
    ISAE::fault_code _fault_code;

    int nErrorLaserN;
    int nErrorIMUN;
    int nErrorPressureN;

// indicate a good data or not
    bool Laser_status;
    bool IMU_status;
    bool Pressure_status;

    int InitLaser = 1;
    int InitIMU = 1;
    int InitPressure = 1;


    float oldLaser;
    float oldPitch;
    float oldRoll;
    float oldPressure;

// threshold to detect a the fault
    float Laser_error=0.1;
    float IMU_error=0.005;
    float Pressure_error=0.1;
//    std::string logHook() const;
//    std::string logHeader() const;

    // creation des port d'entrée et de sorti
    RTT::OutputPort<ISAE::attitude_euler> _attitude_output;
    RTT::OutputPort<mavlink_highres_imu_t> _imu_output;
    RTT::OutputPort<float> _laser_distance_output;
    RTT::OutputPort<ISAE::fault_detection> _fault_detection_output;

    RTT::InputPort<ISAE::attitude_euler> _attitude_input;
    RTT::InputPort<float> _laser_distance_input;
    RTT::InputPort<mavlink_highres_imu_t> _imu_input;

    //altitude modification and control
    void LaserFaultDetection();
    void IMUFaultDetection();
    void BaroFaultDetection();

  public:
    CMS(std::string const& name);
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
    void displayImuPressure();
    void displayLaserData();
    void displayFaultDetect();
    void splitDisplay();

    // fonction send Fault detection
    void sendFaultDetectionMSG();


};
}/*end namespace ISAE*/
#endif /*OROCOS_ISAE_CMS_COMP_HPP*/
