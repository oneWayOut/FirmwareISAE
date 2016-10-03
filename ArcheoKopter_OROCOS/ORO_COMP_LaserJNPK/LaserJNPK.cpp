#include "LaserJNPK.hpp"
#include <rtt/Component.hpp>

#include <iostream>

const std::string ISAE::component_base::name = "@Pkgname";
const std::string ISAE::component_base::version = "0.1";
const std::string ISAE::component_base::doc = "XXX To be documented\n";

namespace ISAE {

LaserJNPK::LaserJNPK(std::string const& name) : 
	ISAE::component_base(name)
{

    //nomPort="/dev/ttyUSB0";//1;//1; //3;
    this->ports()->addPort("_laser_distance_output",_laser_distance_output).doc("port sends data laser");

}

bool LaserJNPK::configureHook() {
  bool res = ISAE::component_base::configureHook();
  if (!res) return res;



  /* tentative d'ouverture */
 /*
 printf("Ouverture et configuration du port ...\r\n");
 if(!OpenCOM(nomPort))  printf("Port nom ouvert!!!\r\n");;
 printf("...OK\r\n");
 */

 printf("Opening and port configuration...\r\n");
        if(!OpenCOM("/dev/ttyUSB0")){
    //    printf("Port usb0 opening error!!!\r\n");
         if (!OpenCOM("/dev/ttyUSB1")){
     //        printf("Port usb1 opening error!!!\r\n");
        }
        else{
    //        printf("Port usb1 opens \n");
        }

    }
    else {
        printf("Port usb0 opens \n");

    }

 std::cout << "LASER configured !" <<std::endl;
 for(compteuri=0;compteuri<sizebuff;compteuri=compteuri+1)
    {
        buffer[compteuri]='#';

        buffer2[compteuri]='#';

    }

 //portSerie = make_mavlink_device(_port_interface);


  std::cout << "LaserJNPK configured !" <<std::endl;
  return true;
}

bool LaserJNPK::startHook() {
  bool res = ISAE::component_base::startHook();
  if (!res) return res;

  std::cout << "LaserJNPK started !" <<std::endl;
  return true;
}

void LaserJNPK::updateHook() {
  new_period();

  if (display_debug)
    std::cout << component_name() <<"-> updateHook()" << std::endl;

  //recover data of laser
  laserRecover();
  //send data laser
  laserDataSend();

  //std::cout << "LASER executes updateHook !" << std::endl;
  end_period();
}






//function which recover the  distance between the ground and the drone with the laser
void LaserJNPK::laserRecover(){


/*



    //init du buffer
      for(compteuri=0;compteuri<sizebuff;compteuri=compteuri+1)
        {
            buffer[compteuri]='#';
        }



  //recevoir des données
     nBytesRead=ReadCOM(buffer);
     if(nBytesRead)
     {
     //buffer[nBytesRead] = '\0';
     //nBytesRead=12;
     if (nBytesRead ||0)
     {
        nbytesvalue = nBytesRead;
         //on décode les informations
    //  	printf("\r\n");
     //   printf("%s",buffer);
      //  printf("\r\n");

        for (compteuri=0;compteuri<nBytesRead;compteuri=compteuri+1)
        {



                if (buffer[compteuri] =='D' && buffer[compteuri+1]!='E' )
                {
                    compteuri = compteuri+2;

                    compteuri3=0;

                    for(compteuri2=0;compteuri2<sizebuff;compteuri2=compteuri2+1)
                    {
                        //buffer[compteuri]='#';
                        buffer2[compteuri2]='#';
                    }

                }

                if (buffer[compteuri] =='D' && buffer[compteuri+1]=='E' )
                {
                    compteuri = compteuri+2;

                    compteuri3=0;

                    for(compteuri2=0;compteuri2<sizebuff;compteuri2=compteuri2+1)
                    {
                        buffer[compteuri]='#';
                        buffer2[compteuri2]='#';
                    }

                    printf ("erreur transmission");
                    printf ("\r\n");
                }
                    //buffer2[compteuri3]=buffer[compteuri];
                    //compteuri3=compteuri3+1;


                    //if (compteuri3 == 8)
                    if (buffer[compteuri] ==13 && buffer[compteuri+1]==10 )

                    {
                            if (compteuri3==8)
                            {
                                 char *sr;
                                buffer2[compteuri3]='\0';
                                DLaser= atof(buffer2);
                                //printf ("%f",DLaser);
                                //printf ("\r\n");
                                nbrmesures = nbrmesures+1;
                                for(compteuri=0;compteuri<sizebuff;compteuri=compteuri+1)
                                {
                                    buffer[compteuri]='#';
                                    buffer2[compteuri]='#';
                                }


                            }
                            else if (compteuri3!=8)
                            {
                                erreur=erreur+1;

                                printf("erreur %d / %d \r\n",erreur,nbrmesures);

                                for(compteuri=0;compteuri<sizebuff;compteuri=compteuri+1)
                                {
                                buffer2[compteuri]='#';

                                }
                            }

                            compteuri3=0;

                    }
                    else if (buffer[compteuri]!='#')
                    {
                    buffer2[compteuri3]=buffer[compteuri];
                    compteuri3=compteuri3+1;

                    }




                //buffer2[compteuri3-1]='\0';

                //DLaser = atof (buffer2);
                //gotoxy (8,2);
                //printf ("%f",DLaser);
                //
                //printf ("\r\n");



            // for(compteuri3=0;compteuri3<256;compteuri3=compteuri3+1)
                //    {
               //         buffer2[compteuri3]='\0';
                //    }



         }


         // si D




         //printf("%s",buffer);

            //gotoxy(0,10);
            //printf("                            ");
            //gotoxy(0,10);
            //printf("Nombre d'octets recu = %i",nBytesRead);


     }


     }
     else
     {
     printf("Erreur lors de la réception.\r\n");
     }


      //envoyer_data_MAGNETO("0SD\r\n");
      //minimum sleep de 78 pour le bon fonctionnement du magnetometre
    //88 ms ??!!
    //  Sleep(88);





     //fermeture du port COM et retour
     //CloseCOM();
     //return 0;*/


}

void LaserJNPK::laserDataSend(){
//printf("send\n");
    DLaser=20.12;
  // printf ("%f\n",DLaser);
_laser_distance_output.write(DLaser);
}





void LaserJNPK::stopHook() {
  ISAE::component_base::stopHook();
  std::cout << "LaserJNPK executes stopping !" << std::endl;
}

void LaserJNPK::cleanupHook() {
  ISAE::component_base::cleanupHook();
  std::cout << "LaserJNPK cleaning up !" << std::endl;
}

} /*end namespace ISAE*/

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(LaserJNPK)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ISAE::LaserJNPK)












