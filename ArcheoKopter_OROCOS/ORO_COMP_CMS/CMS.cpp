#include "CMS.hpp"
#include <rtt/Component.hpp>

#include <iostream>

const std::string ISAE::component_base::name = "CentralMaintenanceSystem";
const std::string ISAE::component_base::version = "0.1";
const std::string ISAE::component_base::doc = "XXX To be documented\n";

namespace ISAE {

CMS::CMS(std::string const& name) :
    ISAE::component_base(name)
{
    //set the name of the port in the deployer
    this->ports()->addPort("_attitude_output",_attitude_output).doc("send attitude");
    this->ports()->addPort("_imu_output",_imu_output).doc("send IMU");
    this->ports()->addPort("_laser_distance_output",_laser_distance_output).doc("send laser distance");
    this->ports()->addPort("_fault_detection_output",_fault_detection_output).doc("send fault detection");


    this->ports()->addPort("_attitude_input",_attitude_input).doc("reception attitude");
    this->ports()->addPort("_laser_distance_input",_laser_distance_input).doc("reception laser distance");
    this->ports()->addPort("_imu_input",_imu_input).doc("reception imut");

    // to access the variable on the deployer:
    addProperty("eulerDispl", _b_elr_dspl).
            doc("boolean display attitude euler");
    addProperty("pressDispl", _b_imu_press).
            doc("boolean imu pression display");
    addProperty("laser", _b_laser_dspl).
            doc("boolean laser data display");
    addProperty("faultDispl", _b_fault_dspl).
            doc("fault detection display");

    addProperty("updatePort", _b_upt_port).
            doc("boolean update port");



    // add function access to the deployer
    addOperation("tog", &CMS::toggleDisplay, this).
    doc("reverse the display chosen i:imu  e:euler  c: controle feed back  g: gpi  l:laser");
    addOperation("resDspl", &CMS::resetDisplay, this).
    doc("reset all display to none");

}

bool CMS::configureHook() {
  bool res = ISAE::component_base::configureHook();  
  _b_upt_port = true;
  _b_imu_press=false;
  _b_elr_dspl = false;
  _b_laser_dspl=false;
  _b_fault_dspl=false;

  _fault_detection.laser=false;
  _fault_detection.imu=false;
  _fault_detection.barometer=false;

    nErrorLaserN =0;
    nErrorIMUN=0;
    nErrorPressureN=0;

    Laser_status=true;
    IMU_status=true;
    Pressure_status=true;

  if (!res) return res;

  std::cout << "CMS configured !" <<std::endl;
  return true;
}

bool CMS::startHook() {
  bool res = ISAE::component_base::startHook();
  //std::cout << this->setPeriod(2)<< std::endl;
  //std::cout << this->getPeriod()<< std::endl;

  if (!res) return res;

  std::cout << "CMS started !" <<std::endl;
  return true;
}



void CMS::updateHook() {
  new_period();


  //std::cout << "yoyo"<<std::endl;


  if (display_debug){
        std::cout << component_name() <<"-> updateHook()" << std::endl;
  }
    //std::cout << component_name() <<"-> updateHook()" << std::endl;

  //data port getter
   updateDataPort();

 /*  _fault_detection.barometer=false;
   _fault_detection.imu=false;
   _fault_detection.laser=false;*/

   sendFaultDetectionMSG();
    //ligne de separation
    splitDisplay();
   //display attitude euler
    displayAttitudeEuler();
    //display attitude control feedback
    //display imu pressure
    displayImuPressure();

    displayLaserData();

  end_period();
}


//function which update the reception variables of the input ports
void CMS::updateDataPort() {
    //std::cout << "update"<<  std::endl;
        if(_b_upt_port){
                if (_imu_input.read(_imu) == RTT::NewData) {

                    _imu_output.write(_imu);
                    //fault detection mode
                    BaroFaultDetection();
                    if (Pressure_status){
                         _imu_output.write(_imu);
                    }else{
                        _fault_code.componentID=3;
                        _fault_code.timestamp=RTT::os::TimeService::Instance()->getNSecs();

                    }


                        //std::cout << "IMU UPDATE"<<  std::endl;
                }                
                if (_attitude_input.read(_attitude)== RTT::NewData) {

                    _attitude_output.write(_attitude);

                    IMUFaultDetection();
                    if (IMU_status){
                        _attitude_output.write(_attitude);
                    }else{
                        _fault_code.componentID=2;
                        _fault_code.timestamp=RTT::os::TimeService::Instance()->getNSecs();

                    }

                }
                if (_laser_distance_input.read(_laser_distance)== RTT::NewData) {

                    _laser_distance_output.write(_laser_distance);
                    LaserFaultDetection();
                    if (Laser_status)
                        _laser_distance_output.write(_laser_distance);
                    else{
                        _fault_code.componentID=1;
                        _fault_code.timestamp=RTT::os::TimeService::Instance()->getNSecs();


                    }

                }

        }
}

void CMS::LaserFaultDetection(){
    if(_fault_detection.laser==true){

        if(_laser_distance!=int(_laser_distance)){
            if(abs(_laser_distance-oldLaser)<Laser_error){
                oldLaser=_laser_distance;
                nErrorLaserN =nErrorLaserN-1;
                Laser_status=false;
            }else {
                nErrorLaserN =10;
                _fault_code.faultCode=1;
                Laser_status=false;
            }
        }else {
            nErrorLaserN =10;
            _fault_code.faultCode=2;
            Laser_status=false;
        }

        if(nErrorLaserN==0){
            _fault_detection.laser=false;
        }
        return;

    } else if((InitLaser==1)&&(_laser_distance!=int(_laser_distance))) {

        oldLaser=_laser_distance;
        nErrorLaserN =0;
        Laser_status=true;

        InitLaser++;

        return;

    }else if(InitLaser!=1) {

        if(_laser_distance!=int(_laser_distance)){
            if(abs(_laser_distance-oldLaser)<Laser_error){
                oldLaser=_laser_distance;
                nErrorLaserN =0;
                Laser_status=true;
            }else{
                nErrorLaserN++;
                _fault_code.faultCode=1;
                Laser_status=false;
            }
        }else{
            nErrorLaserN++;
            _fault_code.faultCode=2;
            Laser_status=false;
        }

        if(nErrorLaserN==10){
            _fault_detection.laser=true;
        }

       return;
      } else{
        nErrorLaserN++;
        _fault_code.faultCode=1;
        Laser_status=false;
        return;
    }
}

void CMS::IMUFaultDetection(){
    if(_fault_detection.imu==true){

         if((_attitude.pitch!=int(_attitude.pitch)) &&(_attitude.roll!=int(_attitude.roll))){
             if((abs(_attitude.pitch-oldPitch)<IMU_error) &&(abs(_attitude.roll-oldRoll)<IMU_error) ){
                 oldPitch=_attitude.pitch;
                 oldRoll=_attitude.roll;
                 nErrorIMUN =nErrorIMUN-1;
                 IMU_status=false;
             }else {
                 nErrorIMUN =10;
                 _fault_code.faultCode=1;
                 IMU_status=false;
             }
         }else {
             nErrorIMUN =10;
             _fault_code.faultCode=2;
             IMU_status=false;
         }

         if(nErrorIMUN==0){
             _fault_detection.imu=false;
         }
         return;

     }else if((InitIMU==1)&&(_attitude.pitch!=int(_attitude.pitch)) &&(_attitude.roll!=int(_attitude.roll))) {

         oldPitch=_attitude.pitch;
         oldRoll=_attitude.roll;
         nErrorIMUN =0;
         IMU_status=true;

         InitIMU++;

         return;

     }else if(InitIMU!=1) {

        if((_attitude.pitch!=int(_attitude.pitch)) &&(_attitude.roll!=int(_attitude.roll))){
            if((abs(_attitude.pitch-oldPitch)<IMU_error) &&(abs(_attitude.roll-oldRoll)<IMU_error) ){
                oldPitch=_attitude.pitch;
                oldRoll=_attitude.roll;
                 nErrorIMUN =0;
                 IMU_status=true;
             }else{
                 nErrorIMUN++;
                 _fault_code.faultCode=1;
                 IMU_status=false;
             }
         }else{
             nErrorIMUN++;
             _fault_code.faultCode=2;
             IMU_status=false;
         }

         if(nErrorIMUN==10){
             _fault_detection.imu=true;
         }
         return;

       } else{
         nErrorIMUN++;
         _fault_code.faultCode=1;
         IMU_status=false;
         return;
     }

}

void CMS::BaroFaultDetection(){
    if(_fault_detection.barometer==true){

         if(_imu.pressure_alt!=int(_imu.pressure_alt)){
             if(abs(_imu.pressure_alt-oldPressure)<Pressure_error){
                 oldPressure=_laser_distance;
                 nErrorPressureN =nErrorPressureN-1;
                 Pressure_status=false;
             }else {
                 nErrorPressureN =10;
                 _fault_code.faultCode=1;
                 Pressure_status=false;
             }
         }else {
             nErrorPressureN =10;
             _fault_code.faultCode=2;
             Pressure_status=false;
         }

         if(nErrorPressureN==0){
             _fault_detection.barometer=false;
         }
         return;

     }else if((InitPressure==1)&&(_imu.pressure_alt!=int(_imu.pressure_alt))) {

         oldPressure=_imu.pressure_alt;
         nErrorPressureN =0;
         Pressure_status=true;

         InitPressure++;

         return;

     }else if(InitPressure!=1) {

         if(_imu.pressure_alt!=int(_imu.pressure_alt)){
             if(abs(_imu.pressure_alt-oldPressure)<Pressure_error){
                 oldPressure=_imu.pressure_alt;
                 nErrorPressureN =0;
                 Pressure_status=true;
             }else{
                 nErrorPressureN++;
                 _fault_code.faultCode=1;
                 Pressure_status=false;
             }
         }else{
             nErrorPressureN++;
             _fault_code.faultCode=2;
             Pressure_status=false;
         }

         if(nErrorPressureN==10){
             _fault_detection.barometer=true;
         }

         return;
       } else{
         nErrorPressureN++;
         _fault_code.faultCode=1;
         Pressure_status=false;
         return;
     }
}

// move off all display value
void CMS::resetDisplay(){

    _b_elr_dspl=false;
    _b_imu_press=false;
    _b_laser_dspl=false;
    _b_fault_dspl=false;

}

// active one display value with a specific char
void CMS::toggleDisplay(char i){

    switch (i) {
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
         default:

            break;
        };
    }





//just split the display when one of display value are activate
void CMS::splitDisplay(){
     if(_b_elr_dspl || _b_imu_press || _b_laser_dspl||_b_fault_dspl){
         std::cout << " "<<  std::endl;
         std::cout << "________________________________________________________ "<<  std::endl;
         std::cout << " "<<  std::endl;
     }
}

//function which display attitude euler send by mavlink in the port _attitude_input
void CMS::displayAttitudeEuler(){

     if(_b_elr_dspl){

                   std::cout << "pry"<<  std::endl;
                   std::cout << "   pitch : "<< ((_attitude.pitch)*180)/3.14<< std::endl;
                   std::cout << "   roll : "<< ((_attitude.roll)*180)/3.14<< std::endl;
                   std::cout << "   yaw : "<< ((_attitude.yaw)*180)/3.14<< std::endl;


                   std::cout << "pry_rate:"<<  std::endl;
                   std::cout << "   P_rate : "<< ((_attitude.pitchspeed)*180)/3.14<< std::endl;
                   std::cout << "   R_rate : "<< ((_attitude.rollspeed)*180)/3.14<< std::endl;
                   std::cout << "   Y_rate : "<< ((_attitude.yawspeed)*180)/3.14<< std::endl;

    }

}


//function which display attitude control feedback send by mavlink in the port _attitude_input

void CMS::displayImuPressure(){

     if(_b_imu_press){


                   std::cout << "Altitude  pressure: "<< _imu.pressure_alt<<  std::endl;

    }
}

void CMS::displayLaserData(){

    if(_b_laser_dspl){

                  std::cout << "D Laser: " << _laser_distance<< std::endl;




   }

}


void CMS::displayFaultDetect(){

    if(_b_fault_dspl){

                  std::cout << "Laser" << _fault_detection.laser<< std::endl;
                  std::cout << "IMU" << _fault_detection.imu<< std::endl;
                  std::cout << "Barometer" << _fault_detection.barometer<< std::endl;

   }

}


// function which take a table of float( the actuators) and send this table to mavlink
void CMS::sendFaultDetectionMSG(){
    _fault_detection_output.write(_fault_detection);
}

void CMS::stopHook() {
  ISAE::component_base::stopHook();
  std::cout << "CMS executes stopping !" << std::endl;
}

void CMS::cleanupHook() {
  ISAE::component_base::cleanupHook();
  std::cout << "CMS cleaning up !" << std::endl;
}

} /*end namespace ISAE*/

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(CMS)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ISAE::CMS)
