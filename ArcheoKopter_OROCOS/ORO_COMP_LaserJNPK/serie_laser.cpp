

#include "serie_laser.hpp"
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>



#include <iostream>
#include <system_error>

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>

int _fdlaser;

bool OpenCOM(std::string nSp)
    {

    // on ouvre le port
    _fdlaser = open (nSp.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if ( _fdlaser == -1 ) {
                perror("open");
            printf ("erreur open");
               return(false);
            }

        // on crée la structure
           struct termios termios_p;
               tcgetattr(_fdlaser,&termios_p);

               //paramétrage de la liaison

                   cfsetispeed(&termios_p, B115200);
                   cfsetospeed(&termios_p, B115200);
                   termios_p.c_iflag &= ~(/*IGNBRK | BRKINT | */ ICRNL/* | INLCR | PARMRK | INPCK | ISTRIP | IXON*/);
                   //termios_p.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);


                   termios_p.c_cflag &= ~(CS8 | CSTOPB);
                   termios_p.c_iflag &= ~(IXON | IXOFF | IXANY);
                   termios_p.c_cflag &= ~(PARENB | CSIZE);
                   //termios_p.c_cc[VMIN] = 1;
                  // termios_p.c_cc[VTIME] = 0;


                   //termios_p.c_iflag &= ~( IXON);
                   // on enregistre les paramétre
           tcsetattr(_fdlaser,TCSANOW,&termios_p);
           return(true);
}


int ReadCOM(void* buffer)
     {
    //printf("read buf\r\n");
        int n;
        n = read(_fdlaser, buffer, sizeof(buffer));


         if (n < 0) {
            printf("Reading failure\r\n");
             return -1 ;
            }
            return n;

     }

bool CloseCOM()
    {
    /* fermeture du port COM */
    close(_fdlaser);
    return true;
    }
