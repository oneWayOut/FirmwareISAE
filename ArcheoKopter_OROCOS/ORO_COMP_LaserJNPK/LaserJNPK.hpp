#ifndef OROCOS_ISAE_LASERJNPK_COMP_HPP
#define OROCOS_ISAE_LASERJNPK_COMP_HPP

#include <ComponentBase.hpp>
#include "serie_laser.hpp"

namespace ISAE{

class LaserJNPK : public ISAE::component_base 
{

private:
  RTT::OutputPort<float> _laser_distance_output;

  std::string _port_interface;
  int compteuri=0;
  int sizebuff=512;
  // std::string nomPort;
  unsigned char buffer[512];
  char buffer2[512];
  int compteuri2=0;
  int compteuri3=0;
  float DLaser=0;
  int nbytesvalue=0;
  int erreur = 0 ;
  int nbrmesures=0;
  int nId,  nBytesWritten, nBytesRead;
  //mavlink_device_t portSerie;

  public:
    LaserJNPK(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void laserRecover();
    void laserDataSend();
};
}/*end namespace ISAE*/
#endif /*OROCOS_ISAE_LASERJNPK_COMP_HPP*/






