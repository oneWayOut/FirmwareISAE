#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <rtt/Component.hpp>
#include "mavlink_bridge.hpp"
#include "type_gpi.h"

const std::string ISAE::component_base::name = "mavlink_bridge";
const std::string ISAE::component_base::version = "0.2";
const std::string ISAE::component_base::doc = "\
This component allows to make an Orocos bridge between PixHawk \
(typically read using a serial link) and a ground station (udp link). \
The configuration of {input, output} device accept the following entry: \n\
	- none => do nothing\n\
	- serial:/dev/xxx:baudrate\n\
	- udp:host:port:[local_port]\n\n\
Moreover, the bridge allows to export several informations to the Orocos world, \
and to receive command for the autopilot (in particular for the offboard mode).";

namespace ISAE {
mavlink_bridge::mavlink_bridge(const std::string& name) :
		ISAE::component_base(name),
        _in_interface("serial:/dev/ttyUSB0:921600"),
        _out_interface("udp:127.0.0.1:14550"),
		_static_pressure(0),
		_dynamic_pressure(0),
        _direct_control_input("direct_control_input"),
        _attitude_control_input("attitude_control_input"),
        _control_mode_input("control_mode_input"),
        _attitude_control_feedback("attitude_control_feedback"),
        _manual_control_output("manual_control_output"),
        _attitude_output("attitude_output"),
        //we give the name that we will use in the script.ops
        _imu_output("imu_output"),
        _battery_output("battery_output"),
        _px4_aux_data_output("px4_aux_data_output"),
        _px4_mode_output("px4_mode_output"),
        arm_flag(0), custom_mode_flag(0)

{
    addProperty("affiche", affiche).
            doc("boolean affiche");
	addProperty("input_interface", _in_interface).
		doc("input interface description");
	addProperty("output_interface", _out_interface).
		doc("output interface description");

    this->ports()->addPort("_direct_control_input",_direct_control_input).doc("reception of direct control");
    this->ports()->addPort("_attitude_control_input",_attitude_control_input).doc("reception attitue target");
    this->ports()->addPort("_control_mode_input",_control_mode_input).doc("reception of control mode");

    this->ports()->addPort("_attitude_output",_attitude_output).doc("send of attitude");
    this->ports()->addPort("_manual_control_output",_manual_control_output).doc("send of manual control target");
    this->ports()->addPort("_imu_output",_imu_output).doc("send of imu");

    this->ports()->addPort("_battery_output",_battery_output).doc("send batteery stat");
    this->ports()->addPort("_px4_mode_output",_px4_mode_output).doc("send px4 mod");
    this->ports()->addPort("_px4_aux_data_output",_px4_aux_data_output).doc("send px4 data");
    this->ports()->addPort("_GPI_Control_output",_GPI_Control_output).doc("send gps pos");
    this->ports()->addPort("_local_position_output",_local_position_output).doc("send local position (alt)");
    this->ports()->addPort("_attitude_control_feedback_output",_attitude_control_feedback).doc("send control feed back");

    // Services
    addOperation("armCmd", &mavlink_bridge::armCmd, this);
    addOperation("disarmCmd", &mavlink_bridge::disarmCmd, this);
    addOperation("offboardCmd", &mavlink_bridge::offboardCmd, this);
    addOperation("manualCmd", &mavlink_bridge::manualCmd, this);
    addOperation("servoCmd",&mavlink_bridge::servoCmd, this);



}


bool mavlink_bridge::configureHook()
{
	bool res = ISAE::component_base::configureHook();
	if (!res) return res;

	// will throw an exception if not ok
	device_in = make_mavlink_device(_in_interface);
	device_out = make_mavlink_device(_out_interface);
    affiche=false;
	return true;
}

void mavlink_bridge::process_message(const mavlink_message_t& m)
{
	bool process_something = true;

	switch (m.msgid) {
		case MAVLINK_MSG_ID_SYS_STATUS:
		{
#ifdef DEBUG
			std::cerr << "Process a status message" << std::endl;
#endif
			mavlink_sys_status_t status;
			mavlink_msg_sys_status_decode(&m, &status);

            //std::cout << "Present......"<< status.onboard_control_sensors_present << "Enable......"<< status.onboard_control_sensors_enabled << "Health......"<< status.onboard_control_sensors_health<< std::endl;

			_bat.sensor_timestamp = 0;
			_bat.system_timestamp = RTT::os::TimeService::Instance()->getNSecs();
			_bat.voltage = (float) status.voltage_battery / 1000.0;
			if (status.current_battery == -1) 
				_bat.current = -1.0;
			else
				_bat.current = (float)status.current_battery / 100.0;
			_bat.remaining = (float)status.battery_remaining / 100.0;

            // PX4 Motors data (mkblctrl on i2c)
            _px4_aux_data.motor_speed[0] = status.errors_count1;
            _px4_aux_data.motor_speed[1] = status.errors_count2;
            // PX4 flaps positions from adc 3.3v
            _px4_aux_data.left_flap_value = status.errors_count3;
            _px4_aux_data.right_flap_value = status.errors_count4;

			_battery_output.write(_bat);
            _px4_aux_data_output.write(_px4_aux_data);
		}
	
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
#ifdef DEBUG
            std::cerr << "Process an heartbeat message" << std::endl;
#endif
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&m, &heartbeat);
			if (heartbeat.base_mode != 0 )
				arm_flag = heartbeat.base_mode;
			if (heartbeat.custom_mode != 0 )
				custom_mode_flag = heartbeat.custom_mode;
			//printf("heartbeat mode: %x and custom: %x \n", arm_flag, custom_mode_flag);

			if ((custom_mode_flag & 0x60000) == 0x60000){
				_px4_mode |= PX4_mode::FLAG_OFFBOARD_MODE_ENABLED; //offboard mode enabled
			} else {
				_px4_mode &= ~PX4_mode::FLAG_OFFBOARD_MODE_ENABLED;
			}

			if ((arm_flag & MAV_MODE_FLAG_SAFETY_ARMED) == MAV_MODE_FLAG_SAFETY_ARMED){
				_px4_mode |= PX4_mode::FLAG_ARMED; //px4 armed
			} else {
				_px4_mode &= ~PX4_mode::FLAG_ARMED; //px4 disarmed
			}

			_px4_mode_output.write(_px4_mode);

            break;
        }

		case MAVLINK_MSG_ID_ATTITUDE:
		{
#ifdef DEBUG
			std::cerr << "Process an attitude message" << std::endl;
#endif
			mavlink_attitude_t attitude;
			mavlink_msg_attitude_decode(&m, &attitude);

			_attitude.timestamp = attitude.time_boot_ms; // XXX use system timestamp ?
			_attitude.frame = frame_t::BODY_NED;
			_attitude.roll = attitude.roll;
			_attitude.pitch = attitude.pitch;
			_attitude.yaw = attitude.yaw;
			_attitude.rollspeed = attitude.rollspeed;
			_attitude.pitchspeed = attitude.pitchspeed;
			_attitude.yawspeed = attitude.yawspeed;

			_attitude_output.write(_attitude);

			break;
		}

        case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
#ifdef DEBUG
			std::cerr << "Process an high_res message" << std::endl;
#endif
           // std::cout << "----------------------------------------"<< MAVLINK_MSG_ID_HIGHRES_IMU << std::endl;
			mavlink_highres_imu_t imu;
			mavlink_msg_highres_imu_decode(&m, &imu);

            _imu.time_usec = imu.time_usec * 1000.0;
            //_imu.system_timestamp = RTT::os::TimeService::Instance()->getNSecs();
            _imu.xacc= imu.xacc;
            _imu.yacc = imu.yacc;
            _imu.zacc = imu.zacc;
            _imu.xgyro = imu.xgyro;
            _imu.ygyro = imu.ygyro;
            _imu.zgyro = imu.zgyro;
            _imu.xmag = imu.xmag;
            _imu.ymag = imu.ymag;
            _imu.zmag = imu.zmag;
            _imu.abs_pressure = imu.abs_pressure;
            _imu.diff_pressure = imu.diff_pressure;
            _imu.pressure_alt = imu.pressure_alt;

			_imu_output.write(_imu);

			break;
		}

        case MAVLINK_MSG_ID_MANUAL_CONTROL:
		{
#ifdef DEBUG
			std::cerr << "Process an manual control message" << std::endl;
#endif

			mavlink_manual_control_t control;
			mavlink_msg_manual_control_decode(&m, &control);

            _manual_control.target=control.target;
            _manual_control.x = control.x;
            _manual_control.y = control.y;
            _manual_control.z = control.z;
            _manual_control.r = control.r;
            _manual_control.buttons = control.buttons;

            _manual_control_output.write(_manual_control);

			break;
		}

		case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		{
#ifdef DEBUG
			std::cerr << "Process an servo_output_raw message" << std::endl;
#endif
			mavlink_servo_output_raw_t raw_output;
			mavlink_msg_servo_output_raw_decode(&m, &raw_output);
			
			memcpy(_servo_raw, &raw_output, sizeof(_servo_raw));
			break;
	}

		case MAVLINK_MSG_ID_RC_CHANNELS:
		{
#ifdef DEBUG
		std::cerr << "Process an rc_channels message" << std::endl;
#endif
		mavlink_rc_channels_t rc_output;
		mavlink_msg_rc_channels_decode(&m, &rc_output);

        //std::cout << "rc_output.chan2_raw------" << rc_output.chan2_raw<< std::endl;

		// The RC channel 8 is the RC offboard switch defined in the PX4
		if (rc_output.chan8_raw > 1700){
			_px4_mode |= PX4_mode::FLAG_RC_OFFBOARD_MODE_SWITCH; //offboard switch on the RC
		} else {
			_px4_mode &= ~PX4_mode::FLAG_RC_OFFBOARD_MODE_SWITCH;
		}

		_px4_mode_output.write(_px4_mode);

		break;
	}

		case MAVLINK_MSG_ID_ATTITUDE_TARGET:
		{
#ifdef DEBUG
			std::cerr << "Process an attitude_target message" << std::endl;
#endif
			mavlink_attitude_target_t att_tgt_feedback_message;
			mavlink_msg_attitude_target_decode(&m, &att_tgt_feedback_message);
			mavlink_quaternion_to_euler(att_tgt_feedback_message.q, &_att_tgt_feedback.roll, &_att_tgt_feedback.pitch, &_att_tgt_feedback.yaw);
			_att_tgt_feedback.timestamp = att_tgt_feedback_message.time_boot_ms;
			_att_tgt_feedback.thrust = att_tgt_feedback_message.thrust;
			_att_tgt_feedback.body_roll_rate = att_tgt_feedback_message.body_roll_rate;
			_att_tgt_feedback.body_pitch_rate = att_tgt_feedback_message.body_pitch_rate;
			_att_tgt_feedback.body_yaw_rate = att_tgt_feedback_message.body_yaw_rate;
			_attitude_control_feedback.write(_att_tgt_feedback);
		break;
	}

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
    #ifdef DEBUG
            std::cerr << "Process an GSP_GLOBAL_POSITION MSG" << std::endl;
    #endif
            mavlink_global_position_int_t global_position_int;
            mavlink_msg_global_position_int_decode(&m, &global_position_int);

            _GPI_tmp.POSLLH_Data.timeStamp= RTT::os::TimeService::Instance()->getNSecs();
            _GPI_tmp.POSLLH_Data.latitude=((double)global_position_int.lat)/(double)(10^7);
            _GPI_tmp.POSLLH_Data.longitude=((double)global_position_int.lon)/(double)(10^7);
            _GPI_tmp.VELNED_Data.speedD=((double)global_position_int.vx)/(double)(100);
            _GPI_tmp.VELNED_Data.speedE=((double)global_position_int.vy)/(double)(100);
            _GPI_tmp.VELNED_Data.speedN=((double)global_position_int.vz)/(double)(100);
            _GPI_Control_output.write(_GPI_tmp);


        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
#ifdef DEBUG
        std::cerr << "process a adsb vehicule" << std::endl;
#endif

        mavlink_local_position_ned_t local_pos;
        mavlink_msg_local_position_ned_decode(&m, &local_pos);


        _local_pos.vz=local_pos.vz;
        _local_position_output.write(_local_pos);


        break;

    }



		default:
			process_something = false;
#ifdef DEBUG
			std::cerr << "Received an unprocessed message of kind " << (unsigned int)m.msgid << std::endl;
#endif
			break;
	};

	if (process_something)
		generate_isae_log();
}

std::string 
mavlink_bridge::logHeader() const
{
	std::ostringstream oss;
	oss << "# timestamp(ns)\tstatic_pressure(mb)\tdynamic_pressure(mb)";
	oss << "\ttimestampPX4\troll(rad)\tpitch(rad)\tyaw(rad)";
	oss << "\ttimestamp_tgt\troll_tgt_in(rad)\tpitch_tgt_in(rad)\tyaw_tgt_in(rad)\tthrust_tgt_in";
	oss << "\tarm_flag\tcustom_mode_flag)";
	oss << "\tcontrol_x\tcontrol_y\tcontrol_z\tcontrol_r\tcontrol_buttons";
	for (size_t i = 0; i < sizeof(_servo_raw) / sizeof(_servo_raw[0]); ++i)
		oss << "\tservo" << std::dec << i << "_raw(us)";
	oss << "\ttimestamp_tgt_feedback\troll_tgt_feedback(rad)\tpitch_tgt_feedback(rad)\tyaw_tgt_feedback(rad)\tthrust_tgt_feedback";
	oss << "\n";

	return oss.str();
}

std::string
mavlink_bridge::logHook() const
{
	std::ostringstream oss;
	oss << RTT::os::TimeService::Instance()->getNSecs() << "\t";
	oss << _static_pressure << "\t" << _dynamic_pressure << "\t";
	oss << _attitude.timestamp << "\t" << _attitude.roll << "\t" << _attitude.pitch << "\t" << _attitude.yaw;
	oss << "\t" << _att_tgt_in.timestamp << "\t" << _att_tgt_in.roll << "\t" << _att_tgt_in.pitch;
	oss << "\t" << _att_tgt_in.yaw << "\t" << _att_tgt_in.thrust;
	oss << "\t" << arm_flag << "\t" << custom_mode_flag;
    oss << "\t" << _manual_control.x << "\t" << _manual_control.y << "\t" << _manual_control.z;
    oss << "\t" << _manual_control.r << "\t" << _manual_control.buttons;
	for (size_t i = 0; i < sizeof(_servo_raw) / sizeof(_servo_raw[0]); ++i)
		oss << "\t" << _servo_raw[i];
	oss << "\t" << _att_tgt_feedback.timestamp << "\t" << _att_tgt_feedback.roll << "\t" << _att_tgt_feedback.pitch;
	oss << "\t" << _att_tgt_feedback.yaw << "\t" << _att_tgt_feedback.thrust;
	oss << "\n";

	return oss.str();
}

void mavlink_bridge::process_direct_control()
{
    ISAE::actuator_control_target c;

    if (_direct_control_input.read(c) == RTT::NewData) {
        mavlink_set_actuator_control_target_t mav_c;
        mav_c.time_usec = c.time_usec;
        mav_c.target_system = 1;
        mav_c.target_component = 0;
        for (size_t i = 0; i < 8; ++i)
            mav_c.controls[i] = c.controls[i];

//		if (display_debug) {
//			std::cerr << "Sent controls ";
//			for (size_t i = 0; i < 8; ++i)
//				std::cerr << mav_c.controls[i] << " ";
//			std::cerr << std::endl;
//		}
        std::cout << "-------------------------------------"<<  std::endl;
        std::cout << "Actuator control target"<<  std::endl;
        std::cout << "  Pitch   : "<< mav_c.controls[1] << std::endl;
        std::cout << "  Roll    : "<< mav_c.controls[0] << std::endl;
        std::cout << "  Yaw     : "<< mav_c.controls[2] << std::endl;
        std::cout << "  Thrust  : "<< mav_c.controls[3] << std::endl;

        mavlink_message_t m;
        mavlink_msg_set_actuator_control_target_encode(1, 0, &m, &mav_c);
        device_in->write_message(m);
    }

}

void mavlink_bridge::process_attitude_control()
{
    if (_attitude_control_input.read(_att_tgt_in) == RTT::NewData) {
        mavlink_set_attitude_target_t mav_c;
        mav_c.time_boot_ms = _att_tgt_in.timestamp;
        mav_c.target_system = 1;
        mav_c.target_component = 0;
        mav_c.type_mask = _att_tgt_in.type_mask;
        mavlink_euler_to_quaternion(_att_tgt_in.roll, _att_tgt_in.pitch, _att_tgt_in.yaw, mav_c.q);
        mav_c.body_roll_rate = _att_tgt_in.body_roll_rate;
        mav_c.body_pitch_rate = _att_tgt_in.body_pitch_rate;
        mav_c.body_yaw_rate = _att_tgt_in.body_yaw_rate;
        mav_c.thrust = _att_tgt_in.thrust;

      std::cout << "-------------------------------------"<<  std::endl;
        std::cout << "Attitude control target"<<  std::endl;
       // std::cout << "  type_mask  : "<<  mav_c.type_mask << std::endl;
        std::cout << "  Pitch  : "<<  _att_tgt_in.roll << std::endl;
        std::cout << "  Roll   : "<<  _att_tgt_in.pitch << std::endl;
        std::cout << "  Yaw    : "<<  _att_tgt_in.yaw << std::endl;
        std::cout << "  Thrust : "<<  mav_c.thrust << std::endl;


        mavlink_message_t m;
        mavlink_msg_set_attitude_target_encode(1, 0, &m, &mav_c);
        device_in->write_message(m);
    }
}


void mavlink_bridge::updateHook()

{

    mavlink_message_t in_m, out_m;

    _quit = false;

    while (!_quit) {
         new_period();

        int err = device_in->read_message(in_m);
		if (err == 1) {
			process_message(in_m);           
            device_out->write_message(in_m);
		}

		int err2 = device_out->read_message(out_m);
		if (err2 == 1) 
			device_in->write_message(out_m);

        bool control_mode;
        control_mode = false;

        if (_control_mode_input.read(control_mode) == RTT::NewData) {
            if(!control_mode)
                process_attitude_control();
            else
                process_direct_control();
        }

        if (err == 0 && err2 == 0)
            usleep(10000);

    end_period();
    }



}

void mavlink_bridge::cleanupHook()
{
	device_in.release();
	device_out.release();
}

bool 
mavlink_bridge::breakUpdateHook() 
{
	_quit = true;
	return true;
}

void mavlink_bridge::armCmd() {
    mavlink_command_long_t mav_c;

    if((arm_flag & 0x80) != 0x80) {

       std::cout << "armCmd!!"  <<  std::endl;
       mav_c.command = 400; // arm code
       mav_c.target_system = 1;
       mav_c.target_component = 0;
       mav_c.confirmation = 0;
       mav_c.param1 = 1;    // arming
       mav_c.param2 = 0;
       mav_c.param3 = 0;
       mav_c.param4 = 0;
       mav_c.param5 = 0;
       mav_c.param6 = 0;
       mav_c.param7 = 0;

       mavlink_message_t m;
       mavlink_msg_command_long_encode(1, 0, &m, &mav_c);
       device_in->write_message(m);
    }
}

void mavlink_bridge::disarmCmd() {
    mavlink_command_long_t mav_c;

    if((arm_flag | 0x7F) != 0x7F) {
       mav_c.command = 400; // arm code
       mav_c.target_system = 1;
       mav_c.target_component = 0;
       mav_c.confirmation = 0;
       mav_c.param1 = 0;    // disarming
       mav_c.param2 = 0;
       mav_c.param3 = 0;
       mav_c.param4 = 0;
       mav_c.param5 = 0;
       mav_c.param6 = 0;
       mav_c.param7 = 0;

       mavlink_message_t m;
       mavlink_msg_command_long_encode(1, 0, &m, &mav_c);
       device_in->write_message(m);
    }
}

void mavlink_bridge::offboardCmd() {
    mavlink_command_long_t long_command_msg;
    mavlink_set_mode_t mode;

    if( ((arm_flag & 0x80) == 0x80) && ((custom_mode_flag & 0x60000) != 0x60000) ) {
        std::cout << "offboardCmd"  <<  std::endl;
        long_command_msg.command = 92;
        long_command_msg.target_system = 1;
        long_command_msg.target_component = 0;
        long_command_msg.confirmation = 0;
        long_command_msg.param1 = 1;    // new mode = offboard
        long_command_msg.param2 = 0;
        long_command_msg.param3 = 0;
        long_command_msg.param4 = 0;
        long_command_msg.param5 = 0;
        long_command_msg.param6 = 0;
        long_command_msg.param7 = 0;

        mavlink_message_t long_msg;
        mavlink_msg_command_long_encode(1, 0, &long_msg, &long_command_msg);
        device_in->write_message(long_msg);

        mode.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode.custom_mode = 0x60000;  //PX4_CUSTOM_MAIN_MODE_OFFBOARD
        mode.target_system = 1;

        mavlink_message_t mode_msg;
        mavlink_msg_set_mode_encode(1, 0, &mode_msg, &mode);
        device_in->write_message(mode_msg);
    }
}

void mavlink_bridge::manualCmd() {
    mavlink_command_long_t long_command_msg;
    mavlink_set_mode_t mode;

//	if( ((arm_flag & 0x80) == 0x80) && ((custom_mode_flag & 0x10000) != 0x10000) ) {
    if( ((custom_mode_flag & 0x10000) != 0x10000) ) {
        std::cout << "manualCmd"  <<  std::endl;
		long_command_msg.command = 92;
        long_command_msg.target_system = 1;
        long_command_msg.target_component = 0;
        long_command_msg.confirmation = 0;
        long_command_msg.param1 = 0;    // OFFBOARD = False => new mode = manual
        long_command_msg.param2 = 0;
        long_command_msg.param3 = 0;
        long_command_msg.param4 = 0;
        long_command_msg.param5 = 0;
        long_command_msg.param6 = 0;
        long_command_msg.param7 = 0;

        mavlink_message_t long_msg;
        mavlink_msg_command_long_encode(1, 0, &long_msg, &long_command_msg);
        device_in->write_message(long_msg);

		mode.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode.custom_mode = 0x10000;  //PX4_CUSTOM_MAIN_MODE_MANUAL
        mode.target_system = 1;

        mavlink_message_t mode_msg;
        mavlink_msg_set_mode_encode(1, 0, &mode_msg, &mode);
        device_in->write_message(mode_msg);
    }
}

void mavlink_bridge::servoCmd(int voie, int pwm) {
    mavlink_command_long_t long_command_msg;

       long_command_msg.command = 183;          // MAV_CMD_DO_SET_SERVO code
       long_command_msg.target_system = 1;      // System which should execute the command
       long_command_msg.target_component = 0;   // Component which should execute the command, 0 for all components
       long_command_msg.confirmation = 0;       // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
       long_command_msg.param1 = voie;          // voie 0 = 1A, voie 1 = 2A, voie 2 = 3A, voie 3 = 4A et voie 4 = 5A

       if (pwm > 2000) long_command_msg.param2 = 2000;
       else if (pwm < 1000) long_command_msg.param2 = 1000;
       else long_command_msg.param2 = pwm;      //PWM value to output, in microseconds (typically 1000 to 2000).

       long_command_msg.param3 = 0;
       long_command_msg.param4 = 0;
       long_command_msg.param5 = 0;
       long_command_msg.param6 = 0;
       long_command_msg.param7 = 0;     

       mavlink_message_t long_msg;
       mavlink_msg_command_long_encode(1, 0, &long_msg, &long_command_msg);
       device_in->write_message(long_msg);

       std::cout << "PWM value: " << long_command_msg.param2 <<  " channel : " << long_command_msg.param1 << std::endl;
}

}
// Define component as deployable in the library
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(ISAE::mavlink_bridge)
