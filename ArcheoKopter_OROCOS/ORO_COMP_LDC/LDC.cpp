#include "LDC.hpp"
#include <rtt/Component.hpp>

#include <iostream>

const std::string ISAE::component_base::name = "AltitudeHoldControl";
const std::string ISAE::component_base::version = "0.1";
const std::string ISAE::component_base::doc = "XXX To be documented\n";

namespace ISAE {

LDC::LDC(std::string const& name) :
    ISAE::component_base(name)
{
    //set the name of the port in the deployer
    this->ports()->addPort("_direct_control_output",_direct_control_output).doc("send direct control motor");
    this->ports()->addPort("_attitude_control_output",_attitude_control_output).doc("send attitude target");
    this->ports()->addPort("_control_mode_output",_control_mode_output).doc("send control mode");


    this->ports()->addPort("_manual_control_input",_manual_control_input).doc("reception of manual control target");
    this->ports()->addPort("_attitude_input",_attitude_input).doc("reception attitude");
    this->ports()->addPort("_laser_distance_input",_laser_distance_input).doc("reception laser distance");
    this->ports()->addPort("_imu_input",_imu_input).doc("reception imut");
    this->ports()->addPort("_fault_detection_input",_fault_detection_input).doc("reception fault detection");

    this->ports()->addPort("_battery_input",_battery_input).doc("reception battery");
    this->ports()->addPort("_px4_mode_input",_px4_mode_input).doc("reception px4 mod");
    this->ports()->addPort("_GPI_Control_input",_GPI_Control_input).doc("reception gps pos");
    this->ports()->addPort("_px4_aux_data_input",_px4_aux_data_input).doc("reception px4 data");
    this->ports()->addPort("_local_position_input",_local_position_input).doc("reception local position (alt)");
    this->ports()->addPort("_attitude_control_feedback_input",_attitude_control_feedback_input).doc("reception of control feed back");
    // to access the variable on the deployer:
    addProperty("eulerDispl", _b_elr_dspl).
            doc("boolean display attitude euler");
    addProperty("pressDispl", _b_imu_press).
            doc("boolean imu pression display");
    addProperty("laser", _b_laser_dspl).
            doc("boolean laser data display");
    addProperty("manualDispl", _b_manual_dspl).
            doc("manual control display");
    addProperty("faultDispl", _b_fault_dspl).
            doc("fault detection display");

    addProperty("directCTRLDispl",_b_drt_ctrl_dspl).
            doc("boolean display direct control");
    addProperty("attitudeCTRLDispl",_b_att_ctrl_dspl).
            doc("boolean display attitude control");
    addProperty("CTRLmodeDispl",_b_ctrl_dspl).
            doc("boolean display control mode");
    addProperty("updatePort", _b_upt_port).
            doc("boolean update port");

    addProperty("gpiDispl", _b_gpi_dspl).
            doc("boolean gps display");
    addProperty("looper", _b_simul_Outputs).
            doc("boolean loop");
    addProperty("pow", _pow).
            doc("powermotor loop value");
    addProperty("incre", _incre).
            doc("increment loop value");


    // add function access to the deployer
    addOperation("tog", &LDC::toggleDisplay, this).
    doc("reverse the display chosen i:imu  e:euler  c: controle feed back  g: gpi  l:laser");
    addOperation("resDspl", &LDC::resetDisplay, this).
    doc("reset all display to none");

}

bool LDC::configureHook() {
  bool res = ISAE::component_base::configureHook();  
  _b_upt_port = true;

  _b_imu_press=false;
  _b_elr_dspl = false;
  _b_laser_dspl=false;
  _b_manual_dspl=false;
  _b_fault_dspl=false;

  _b_drt_ctrl_dspl=false;
  _b_att_ctrl_dspl=false;
  _b_ctrl_dspl=false;

  _b_gpi_dspl=false;
  _b_simul_Outputs=false;

  _fault_detection.laser=false;
  _fault_detection.imu=false;
  _fault_detection.barometer=false;
  _control_mode=false;

  if (!res) return res;

  std::cout << "LDC configured !" <<std::endl;
  return true;
}

bool LDC::startHook() {
  bool res = ISAE::component_base::startHook();
  //std::cout << this->setPeriod(2)<< std::endl;
  //std::cout << this->getPeriod()<< std::endl;

  if (!res) return res;

  std::cout << "LDC started !" <<std::endl;
  return true;
}



void LDC::updateHook() {
  new_period();



  //std::cout << "yoyo"<<std::endl;


  if (display_debug){
        std::cout << component_name() <<"-> updateHook()" << std::endl;
  }
    //std::cout << component_name() <<"-> updateHook()" << std::endl;
  if(_b_simul_Outputs){
    simulOutputs();
    //putVal(_power);
  }
  //data port getter
   updateDataPort();
   ControlModeSwitch();
   if (_control_mode)
       sendDirectControlMSG();
   else
       sendAttitudeTarget(_att_tgt_out);

   sendControlModeMSG();

    //ligne de separation
    splitDisplay();
   //display attitude euler
    displayAttitudeEuler();
    //display attitude control feedback
    displayAttitudeControlFeedBack();
    displayImuPressure();
    displayManualControl();
    displayGpi();
    displayLaserData();
    displayFaultDetect();
    displayDirectControl();\
    displayAttitudeControl();
    displayControlMode();

  end_period();
}

 void LDC::CalLaserAltitudeAverage(float tet, float phi,float DLaser){
       float DLaserCorr = 0;

       DLaserCorr = cos(tet)*cos(phi)*DLaser ;

       if(InitLaser){
           Laser10 = DLaserCorr;
           Laser9 = DLaserCorr;
           Laser8 = DLaserCorr;
           Laser7 = DLaserCorr;
           Laser6 = DLaserCorr;
           Laser5 = DLaserCorr;
           Laser4 = DLaserCorr;
           Laser3 = DLaserCorr;
           Laser2 = DLaserCorr;
           Laser1 = DLaserCorr;
           InitLaser = 0;
       } else{
           Laser10 = Laser9;
           Laser9 = Laser8;
           Laser8 = Laser7;
           Laser7 = Laser6;
           Laser6 = Laser5;
           Laser5 = Laser4;
           Laser4 = Laser3;
           Laser3 = Laser2;
           Laser2 = Laser1;
           Laser1 = DLaserCorr;
       }


     AltLaser_Average = 0.1 * Laser1 + 0.1 * Laser2 + 0.1 * Laser3 + 0.1 * Laser4 + 0.1 * Laser5 + 0.1 * Laser6 + 0.1 * Laser7  + 0.1 * Laser8  + 0.1 * Laser9  + 0.1 * Laser10;
     /*std::cout << "   pitch : "<< tet<< std::endl;
     std::cout << "   roll : "<< phi<< std::endl;
     std::cout << "   DLaser : "<< DLaser<< std::endl;
     std::cout << "   DLaserCorr : "<< DLaserCorr<< std::endl;
     std::cout << "   AltLaser_Average : "<< AltLaser_Average<< std::endl;*/
 }

 void LDC::LaserAltitudePIDcontrol(float dt){
     //pid 1
     float FilterCoefficient;
     float error;

     // error: '<S1>/error' incorporates:
     //   Inport: '<Root>/_alititude_measurement_input'
     //   Inport: '<Root>/_alititude_reference_input'

     error = AltLaser_Average - AltLaser_Reference;

     // Gain: '<S2>/Filter Coefficient' incorporates:
     //   DiscreteIntegrator: '<S2>/Filter'
     //   Gain: '<S2>/Derivative Gain'
     //   Sum: '<S2>/SumD'

     FilterCoefficient = (kd_alt_laser * error - Filter_DSTATE_laser) * kn_alt_laser;

     // Outport: '<Root>/_laser_alti_control_output' incorporates:
     //   DiscreteFilter: '<S1>/Discrete Filter'
     //   Gain: '<S1>/Gain2'
     //   Gain: '<S2>/Proportional Gain'
     //   Sum: '<S1>/Sum3'
     //   Sum: '<S2>/Sum'

     laser_alti_control = (error + FilterCoefficient) * kp_alt_laser
       + dt * DiscreteFilter_states_laser * filter_gain_laser;

     // Update for DiscreteIntegrator: '<S2>/Filter'
     Filter_DSTATE_laser += dt * FilterCoefficient;

     // Update for DiscreteFilter: '<S1>/Discrete Filter'
     DiscreteFilter_states_laser = error - (-DiscreteFilter_states_laser);

//pid2
    /* float err; //Erreur d'altitude
     float b0, b1, b2, a0, a1, a2; //Variables pour calcul de la loi PID
     float OrdrePasColLaserlin; //Ordre de pas de la loi PID
     err = AltLaser_Average - AltLaser_Reference;
     b0 = KpLaser*(1.0+Nlaser*dt) + KiLaser*dt*(1.0+Nlaser*dt) + KdLaser*Nlaser;
     b1 = KpLaser*(2.0+Nlaser*dt)+KiLaser*dt+2.0*KdLaser*Nlaser;
     b1 = -b1;
     b2 = KdLaser * Nlaser+KpLaser;
     a0 = 1.0 + Nlaser*dt;
     a1 = 2.0 + Nlaser*dt;
     a1 = -a1;
     a2 = 1.0f;
     OrdrePasColLaserlin = -a1/a0*p1-a2/a0*p2+b0/a0*err+b1/a0*err1+b2/a0*err2;
     p2 = p1;
     p1 = OrdrePasColLaserlin;
     err2 = err1;
     err1 = err;
     laser_alti_control=OrdrePasColLaserlin;
     return laser_alti_control;*/
 }

 void LDC::PressureAltitudePIDcontrol(float dt){
      //pid 1
      float FilterCoefficient;
      float error;

      // error: '<S1>/error' incorporates:
      //   Inport: '<Root>/_alititude_measurement_input'
      //   Inport: '<Root>/_alititude_reference_input'

      error = _imu.pressure_alt - PresLaser_Reference;

      // Gain: '<S2>/Filter Coefficient' incorporates:
      //   DiscreteIntegrator: '<S2>/Filter'
      //   Gain: '<S2>/Derivative Gain'
      //   Sum: '<S2>/SumD'

      FilterCoefficient = (kd_alt_pressure * error - Filter_DSTATE_pressure) * kn_alt_pressure;

      // Outport: '<Root>/_pressure_alti_control_output' incorporates:
      //   DiscreteFilter: '<S1>/Discrete Filter'
      //   Gain: '<S1>/Gain2'
      //   Gain: '<S2>/Proportional Gain'
      //   Sum: '<S1>/Sum3'
      //   Sum: '<S2>/Sum'

      pressure_alti_control = (error + FilterCoefficient) * kp_alt_pressure
        + dt * DiscreteFilter_states_pressure * filter_gain_pressure;

      // Update for DiscreteIntegrator: '<S2>/Filter'
      Filter_DSTATE_pressure += dt * FilterCoefficient;

      // Update for DiscreteFilter: '<S1>/Discrete Filter'
      DiscreteFilter_states_pressure = error - (-DiscreteFilter_states_pressure);

  }

 void LDC::ControlModeSwitch(){
     if(!(_fault_detection.laser) && !(_fault_detection.imu)){
         CalLaserAltitudeAverage(_attitude.pitch,_attitude.roll,_laser_distance);
         LaserAltitudePIDcontrol(0.01);//need comment

         _control_mode=false;
         _att_tgt_out.pitch=(float)_manual_control.x/1000*1.57; //maximum rotation rate 30 degree per second
         _att_tgt_out.roll=(float)_manual_control.r/1000*1.57;  //maximum rotation rate 30 degree per second
         _att_tgt_out.yaw=(float)_manual_control.y/1000*1.57;   //maximum rotation rate 30 degree per second
         _att_tgt_out.thrust= laser_alti_control+trim_laser;//need comment and set limitation
         _att_tgt_out.type_mask=0x07;
         _att_tgt_out.timestamp=RTT::os::TimeService::Instance()->getNSecs();

     }else if(_fault_detection.laser && !(_fault_detection.imu) && !(_fault_detection.barometer)){
         PressureAltitudePIDcontrol(0.01);//need comment and set pressure attitude reference

         _control_mode=false;
         _att_tgt_out.pitch=(float)_manual_control.x/1000*1.57; //maximum rotation rate 30 degree per second
         _att_tgt_out.roll=(float)_manual_control.r/1000*1.57;  //maximum rotation rate 30 degree per second
         _att_tgt_out.yaw=(float)_manual_control.y/1000*1.57;   //maximum rotation rate 30 degree per second
         _att_tgt_out.thrust= pressure_alti_control+trim_pressure;//need comment and set limitation
         _att_tgt_out.type_mask=0x07;
         _att_tgt_out.timestamp=RTT::os::TimeService::Instance()->getNSecs();

     }else if(_fault_detection.imu && !(_fault_detection.barometer)){
         PressureAltitudePIDcontrol(0.01);

         _control_mode=true;

         _act_tgt_out.controls[0]=(float)_manual_control.r/1000;//need comment and modification
         _act_tgt_out.controls[1]=(float)_manual_control.x/1000;
         _act_tgt_out.controls[2]=(float)_manual_control.y/1000;
         _act_tgt_out.controls[3]=(float)pressure_alti_control+trim_pressure;
         for (size_t i = 4; i < 8; ++i)
             _act_tgt_out.controls[i] = 0;
         _act_tgt_out.time_usec=RTT::os::TimeService::Instance()->getNSecs();

     }else {
         _control_mode=true;

         _act_tgt_out.controls[0]=(float)_manual_control.r/1000;
         _act_tgt_out.controls[1]=(float)_manual_control.x/1000;
         _act_tgt_out.controls[2]=(float)_manual_control.y/1000;
         _act_tgt_out.controls[3]=(float)_manual_control.z/1000;
         for (size_t i = 4; i < 8; ++i)
             _act_tgt_out.controls[i] = 0;
         _act_tgt_out.time_usec=RTT::os::TimeService::Instance()->getNSecs();
     }

 }

//var the value of the 8 output, to 0 to 1
void LDC::simulOutputs(){

            if(_pow>=0.99 ){

                _incre=-0.001;
              }
            if(_pow<=0.02){

                  _incre=0.001;
              }

              _pow=_pow+_incre;


              setOutputs(_pow);

}
//set one of the output between 0-7
void LDC::setOutput(int nO, float val){

    outputsValue[nO] = val;

}


// set the 8 output to a value
void LDC::setOutputs(float val){


        for(int i = 0 ; i <= 7 ; i++){

             outputsValue[i] = val;

        }


}

//function which update the reception variables of the input ports
void LDC::updateDataPort() {
    //std::cout << "update"<<  std::endl;
        if(_b_upt_port){
                if (_imu_input.read(_imu) == RTT::NewData) {

                        //std::cout << "IMU UPDATE"<<  std::endl;
                }
                if (_GPI_Control_input.read(_GPI_tmp)== RTT::NewData) {

                        //std::cout << "GPI UPDATE"<<  std::endl;
                }
                if (_attitude_control_feedback_input.read(_att_tgt_feedback)== RTT::NewData) {

                        //std::cout << "ATTITUDE CONTROL FEEDBACK UPDATE"<<  std::endl;
                }
                if (_attitude_input.read(_attitude)== RTT::NewData) {

                    //std::cout << "   roll : "<< ((_attitude.roll)*180)/3.14<< std::endl;

                        //std::cout << "ATTITUDE INPUT UPDATE"<<  std::endl;
                       //std::cout << "   pitch : "<< ((_attitude.pitch)*180)/3.14<< std::endl;
                }
                if (_battery_input.read(_bat)== RTT::NewData) {

                        //std::cout << "BATTERRY UPDATE"<<  std::endl;
                }
                if (_px4_mode_input.read(_px4_mode)== RTT::NewData) {

                        //std::cout << "PX4 MODE IN UPDATE"<<  std::endl;
                }
                if (_px4_aux_data_input.read(_px4_aux_data)== RTT::NewData) {

                        //std::cout << "PX4 AUX DATA IN UPDATE"<<  std::endl;
                       //std::cout << _px4_aux_data.nb_motors<<  std::endl;
                }
                if (_local_position_input.read(_local_pos)== RTT::NewData) {

                        //std::cout << "local position ned  UPDATE"<<  std::endl;

                }
                if (_laser_distance_input.read(_laser_distance)== RTT::NewData) {

                        //std::cout << "laser Data  UPDATE"<<  std::endl;
                        //std::cout << "D Laser: " << _alt_ground_laser<< std::endl;
                }
                if (_manual_control_input.read(_manual_control)== RTT::NewData) {

                        //std::cout << "laser Data  UPDATE"<<  std::endl;
                       //std::cout << "manual control y: " << _manual_control.y<< std::endl;
                }
                if (_fault_detection_input.read(_fault_detection)== RTT::NewData) {

                       // std::cout << "laser Data  UPDATE"<<  std::endl;
                        //std::cout << "D Laser: " << _alt_ground_laser<< std::endl;
                }
        }

}

// move off all display value
void LDC::resetDisplay(){

    _b_drt_ctrl_dspl=false;
    _b_elr_dspl=false;
    _b_gpi_dspl=false;
    _b_imu_press=false;
    _b_laser_dspl=false;
    _b_manual_dspl=false;
    _b_fault_dspl=false;
    _b_att_ctrl_dspl=false;
    _b_ctrl_dspl=false;

}

// active one display value with a specific char
void LDC::toggleDisplay(char i){

    switch (i) {
        case 'a':
        {
           _b_att_ctrl_dspl=!_b_att_ctrl_dspl;
        break;
        }
        case 'c':
        {
           _b_ctrl_dspl=!_b_ctrl_dspl;
        break;
        }
        case 'd':
        {
            _b_drt_ctrl_dspl=!_b_drt_ctrl_dspl;
        break;
         }
        case 'e':
        {
            _b_elr_dspl=!_b_elr_dspl;
        break;
         }
        case 'f':
        {
            _b_fault_dspl=!_b_fault_dspl;
        break;
        }
        case 'g':
        {
            _b_gpi_dspl=!_b_gpi_dspl;
        break;
         }
        case 'i':
        {
             _b_imu_press=!_b_imu_press;
        break;
         }
        case 'l':
        {
            _b_laser_dspl=!_b_laser_dspl;
        break;
         }
        case 'm':
        {
            _b_manual_dspl=!_b_manual_dspl;
        break;
        }

         default:

            break;
        };
    }





//just split the display when one of display value are activate
void LDC::splitDisplay(){
     if(_b_elr_dspl || _b_drt_ctrl_dspl || _b_gpi_dspl || _b_imu_press || _b_laser_dspl||_b_manual_dspl||_b_fault_dspl){
         std::cout << " "<<  std::endl;
         std::cout << "________________________________________________________ "<<  std::endl;
         std::cout << " "<<  std::endl;
     }
}

//function which display attitude euler send by mavlink in the port _attitude_input
void LDC::displayDirectControl(){
    if(_b_drt_ctrl_dspl){
        std::cout << "Actuator control target"<<  std::endl;
        std::cout << "  Pitch   : "<< _act_tgt_out.controls[1] << std::endl;
        std::cout << "  Roll    : "<< _act_tgt_out.controls[0] << std::endl;
        std::cout << "  Yaw     : "<< _act_tgt_out.controls[2] << std::endl;
        std::cout << "  Thrust  : "<< _act_tgt_out.controls[3] << std::endl;
    }
}

void LDC::displayAttitudeControl(){
    if(_b_att_ctrl_dspl){
        std::cout << "Attitude control target"<<  std::endl;
        std::cout << "  Pitch rate  : "<< _att_tgt_out.body_pitch_rate << std::endl;
        std::cout << "  Roll rate   : "<< _att_tgt_out.body_roll_rate << std::endl;
        std::cout << "  Yaw rate    : "<< _att_tgt_out.body_yaw_rate << std::endl;
        std::cout << "  Thrust      : "<< _att_tgt_out.thrust << std::endl;
    }
}

void LDC::displayControlMode(){
    if(_b_ctrl_dspl){
        std::cout << "Control Mode"<<  std::endl;
        std::cout << "  0-attitude control/1-actuator control  : "<< _control_mode << std::endl;
    }
}

void LDC::displayAttitudeEuler(){

     if(_b_elr_dspl){
                   //lign break
                   /*std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;*/
                   //lign break


                  /* std::cout << "Attitude"<<  std::endl;
                   std::cout << "  Pitch : "<< ((_attitude.pitch)*180)/3.14<< std::endl;
                   std::cout << "  Roll  : "<< ((_attitude.roll)*180)/3.14<< std::endl;
                   std::cout << "  Yaw   : "<< ((_attitude.yaw)*180)/3.14<< std::endl;


                   std::cout << "Attitude angle rate"<<  std::endl;
                   std::cout << "  Pitch_rate : "<< ((_attitude.pitchspeed)*180)/3.14<< std::endl;
                   std::cout << "  Roll_rate  : "<< ((_attitude.rollspeed)*180)/3.14<< std::endl;
                   std::cout << "  Yaw_rate   : "<< ((_attitude.yawspeed)*180)/3.14<< std::endl;*/
                   std::cout << "Attitude"<<  std::endl;
                   std::cout << "  Pitch : "<< _attitude.pitch<< std::endl;
                   std::cout << "  Roll  : "<< _attitude.roll<< std::endl;
                   /*std::cout << "  Yaw   : "<< _attitude.yaw<< std::endl;

                   *std::cout << "Attitude angle rate"<<  std::endl;
                   std::cout << "  Pitch_rate : "<< _attitude.pitchspeed<< std::endl;
                   std::cout << "  Roll_rate  : "<< _attitude.rollspeed<< std::endl;
                   std::cout << "  Yaw_rate   : "<< _attitude.yawspeed<< std::endl;*/
    }

}


//function which display attitude control feedback send by mavlink in the port _attitude_input
void LDC::displayAttitudeControlFeedBack(){

     if(_b_drt_ctrl_dspl){
                     //lign break
                    /* std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;*/
                     //lign break



                   std::cout << "pry_rate_radio co:"<<  std::endl;
                   std::cout << "   P_rate : "<< ((_att_tgt_feedback.body_pitch_rate)*180)/3.14<< std::endl;
                   std::cout << "   R_rate : "<< ((_att_tgt_feedback.body_roll_rate)*180)/3.14<< std::endl;
                   std::cout << "   Y_rate : "<< ((_att_tgt_feedback.body_yaw_rate)*180)/3.14<< std::endl;

    }

}

void LDC::displayImuPressure(){

     if(_b_imu_press){
                     //lign break
                     /*std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;*/
                     //lign break

          //         std::cout << "Static pressure: "<< _imu.abs_pressure<< std::endl;
                   std::cout << "Barometer"<<std::endl;
                   std::cout << "  Pressure altitude   : "<< _imu.pressure_alt<<  std::endl;

               //    std::cout << "Vertical speed: " << _local_pos.vz<< std::endl;



    }
}
void LDC::displayGpi(){

    if(_b_gpi_dspl){
                    //lign break
                    /*std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;*/
                    //lign break

                  std::cout << "Latitude: " << _GPI_tmp.POSLLH_Data.latitude<< std::endl;
                  std::cout << "Longitude: " << _GPI_tmp.POSLLH_Data.longitude<< std::endl;



   }

}

void LDC::displayLaserData(){

    if(_b_laser_dspl){
                    //lign break
                    /*std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;*/
                    //lign break
                  std::cout << "Laser"<<std::endl;
                  std::cout << "  Laser distance : " << _laser_distance<< std::endl;
               //   std::cout << "Laser_altitude_Average: " << AltLaser_Average<< std::endl;
                 // std::cout << "PID_Control_output: " << laser_alti_control<< std::endl;



   }

}

void LDC::displayManualControl(){

    if(_b_manual_dspl){
                    //lign break
                    /*std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;*/
                    //lign break
                  std::cout << "Manual control"<<std::endl;
                  std::cout << "  Pitch input  : " << _manual_control.x<< std::endl;
                  std::cout << "  Roll input   : " << _manual_control.r<< std::endl;
                  std::cout << "  Yawl input   : " << _manual_control.y<< std::endl;
                  std::cout << "  Thrust input : " << _manual_control.z<< std::endl;

   }

}

void LDC::displayFaultDetect(){

    if(_b_fault_dspl){
                    //lign break
                    /*std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;*/
                    //lign break

                  std::cout << "Fault detection"<<std::endl;
                  std::cout << "  Laser     : " << _fault_detection.laser<< std::endl;
                  std::cout << "  IMU       : " << _fault_detection.imu<< std::endl;
                  std::cout << "  Barometer : " << _fault_detection.barometer<< std::endl;

   }

}


// function which take a table of float( the actuators) and send this table to mavlink
void LDC::sendDirectControlMSG(){
  /*  ISAE::actuator_control_target m_tmp;

    for(int i = 0 ; i<8 ; i++){
        m_tmp.controls[i]=outputsValue[i];
        //std::cout << m_tmp.controls[i]<<  std::endl;
    }
     //std::cout << m_tmp.controls[0]<<   std::endl;
    _direct_control_output.write(m_tmp);*/

    _direct_control_output.write(_act_tgt_out);
}

void LDC::sendControlModeMSG(){
    _control_mode_output.write(_control_mode);
}

/* function which take a table of float( the actuators) and send this table to mavlink
void LDC::sendAltitudeControlMSG(){
    float m_altitude =0;
    m_altitude = altitudePIDcontrol(0.01)+trim_laser;
    _altitude_control_output.write(m_altitude);

}*/


/* function which take the parameters of a attitude Target variable and send then to mavlink

type_mask: mappings: Si un des bits est à 1, l'entrée corespondantes est ignoré
    -->
        bit 1 : body roll rate
        bit 2 : body pitch rate
        bit 3 : body yaw rate
        bit 4->6 : reserved
        bit 7 : thrust
        bit 8 : attitude
        donc pour transmettre roll, pitch, yaw le mask doit être à 7
*/

            //call with attitude_target variabl<< m_tmp.controls[i]<< e
            void LDC::sendAttitudeTarget(ISAE::attitude_target m_tmp ){

                _attitude_control_output.write(m_tmp);

            }

            //call with the variable of an attitude target
            void LDC::sendAttitudeTarget(uint8_t type_mask, float roll, float pitch, float yaw, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust /*[-1,1]*/){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=type_mask;
                        m_tmp.roll=roll;
                        m_tmp.pitch=pitch;
                        m_tmp.yaw=yaw;
                        m_tmp.body_pitch_rate=body_pitch_rate;
                        m_tmp.body_roll_rate=body_roll_rate;
                        m_tmp.body_yaw_rate=body_yaw_rate;
                        m_tmp.thrust=thrust;


                _attitude_control_output.write(m_tmp);

            }
            // modify just the attitude
            void LDC::sendAttitudeTarget_attitude(float roll, float pitch, float yaw ){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0x47;
                        m_tmp.roll=roll;
                        m_tmp.pitch=pitch;
                        m_tmp.yaw=yaw;


                _attitude_control_output.write(m_tmp);

            }
            //modify just the roll rate
            void LDC::sendAttitudeTarget_roll_rate(  float body_roll_rate){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0xc6;
                        m_tmp.body_roll_rate=body_roll_rate;


                _attitude_control_output.write(m_tmp);

            }
            //modify just the pitch rate
            void LDC::sendAttitudeTarget_pitch_rate(  float body_pitch_rate){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0xc5;
                        m_tmp.body_pitch_rate=body_pitch_rate;


                _attitude_control_output.write(m_tmp);

            }
            //modify just the yaw rate
            void LDC::sendAttitudeTarget_yaw_rate(  float body_yaw_rate){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0xc3;
                        m_tmp.body_pitch_rate=body_yaw_rate;

                _attitude_control_output.write(m_tmp);

            }
            //modify just the thrust
            void LDC::sendAttitudeTarget_thrust(  float thrust){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0x87;
                        m_tmp.thrust=thrust;

                _attitude_control_output.write(m_tmp);

            }


void LDC::stopHook() {
  ISAE::component_base::stopHook();
  std::cout << "LDC executes stopping !" << std::endl;
}

void LDC::cleanupHook() {
  ISAE::component_base::cleanupHook();
  std::cout << "LDC cleaning up !" << std::endl;
}

} /*end namespace ISAE*/

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(LDC)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ISAE::LDC)
