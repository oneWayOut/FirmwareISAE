#ifndef _SMOOTHMODEVALUES_H
#define _SMOOTHMODEVALUES_H
//多个模态值的淡化处理

#ifdef WIN32
#define uint8_t  unsigned char
#endif

//mode = 1, 2, 3
//
typedef struct {
	bool    firstRun;
	uint8_t modeNum;      //2~3;
	uint8_t lastMode;     //last mode 
	uint8_t curMode;      //current mode 
	float   modeVal[3];   //val of mode1, mode2, mode3
	float   k_dt;         //time since entering NOTE: dt increment is 0.01s
}SMOOTH_MODE_VALUES;


void smoothInit(SMOOTH_MODE_VALUES * pSelf, uint8_t num);


float smoothUpdate(SMOOTH_MODE_VALUES * pSelf, uint8_t mode, float val);

#endif
