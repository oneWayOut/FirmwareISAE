#ifndef _Serie
#define _Serie


#include <vector>
#include <memory>


#include <netinet/in.h>


//int _fdlaser;


bool OpenCOM(std::string nSp);
int ReadCOM(void* buffer);
bool CloseCOM();
#endif /* _serie_ */
