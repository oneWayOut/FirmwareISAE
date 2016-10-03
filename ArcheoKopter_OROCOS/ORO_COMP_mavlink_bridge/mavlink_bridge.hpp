#ifndef _MAVLINK_BRIDGE_COMPONENT_H_
#define _MAVLINK_BRIDGE_COMPONENT_H_


#include <ComponentBase.hpp>

#include <mavlink.h>


#include "mavlink_device.hpp"

#include <actuator_control_target.h>
#include <attitude.h>
#include <attitude_target.h>
#include <imu.h>
#include <battery.h>
#include <px4_aux_data.h>
#include <px4_mode.h>
#include <Full_GPS.h>
//#include "ORO_TYPE_GPI_gps/GPI_gps.h"
//#include <struct_donnee.h>

namespace ISAE {


class mavlink_bridge : public ISAE::component_base
{

private:


	bool _quit;
	std::string _in_interface, _out_interface;
    bool affiche;
	/* Information we want to grab from pixhawk */
	float _static_pressure;
	float _dynamic_pressure;
	ISAE::attitude_euler _attitude;
   //ISAE::imu _imu;
    mavlink_highres_imu_t _imu;
    mavlink_local_position_ned_t _local_pos;
    mavlink_manual_control_t _manual_control;

	ISAE::battery _bat;
    ISAE::PX4_aux_data _px4_aux_data;


	uint16_t _servo_raw[8];
	uint8_t  arm_flag;
	uint32_t custom_mode_flag;
	ISAE::attitude_target _att_tgt_in;
	ISAE::attitude_target _att_tgt_feedback;
    STR_GPSFUSION  _GPI_tmp;

	ISAE::PX4_mode_flags _px4_mode = 0;

	std::string logHook() const;
	std::string logHeader() const;

	mavlink_device_t device_in, device_out;

	void process_message(const mavlink_message_t& res);

	void process_direct_control();
	void process_attitude_control();

    RTT::InputPort<ISAE::actuator_control_target> _direct_control_input;
    RTT::InputPort<ISAE::attitude_target> _attitude_control_input;
    RTT::InputPort<bool> _control_mode_input;

    RTT::OutputPort<mavlink_manual_control_t> _manual_control_output;
    RTT::OutputPort<STR_GPSFUSION> _GPI_Control_output;
    RTT::OutputPort<ISAE::attitude_target> _attitude_control_feedback;
    RTT::OutputPort<ISAE::attitude_euler> _attitude_output;
    RTT::OutputPort<mavlink_highres_imu_t> _imu_output;
    RTT::OutputPort<ISAE::battery> _battery_output;
    RTT::OutputPort<ISAE::PX4_mode_flags> _px4_mode_output;
    RTT::OutputPort<ISAE::PX4_aux_data> _px4_aux_data_output;
    RTT::OutputPort<mavlink_local_position_ned_t> _local_position_output;
    // Services
    void armCmd();
    void disarmCmd();
    void offboardCmd();
    void manualCmd();
    void servoCmd(int voie, int pwm);

public:

	mavlink_bridge(const std::string& name);
	bool configureHook();
	void updateHook();
	void cleanupHook();
	bool breakUpdateHook();






    //------Structure de Donnée-------



};
}

#endif	/* _MAVLINK_BRIDGE_COMPONENT_H_ */
