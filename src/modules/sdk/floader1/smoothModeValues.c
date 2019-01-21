#include "stdio.h"

#include "smoothModeValues.h"


//do this after armed
void smoothInit(SMOOTH_MODE_VALUES * pSelf, uint8_t num)
{
	pSelf->firstRun = true;

	if (num<2 || num>3)
	{
		printf("smoothInit: !!!mode error!!!\n");
		return;
	}
	pSelf->modeNum = num;
}



float smoothUpdate(SMOOTH_MODE_VALUES * pSelf, uint8_t mode, float val)
{
	if (mode<1 || mode > pSelf->modeNum)
	{
		printf("smoothUpdate: !!!mode error!!!\n");
		return 0;
	}

	if (pSelf->firstRun)
	{
		pSelf->firstRun = false;
	
		pSelf->curMode = pSelf->lastMode = mode;

		pSelf->modeVal[0] = val;
		pSelf->modeVal[1] = val;
		pSelf->modeVal[2] = val;
		pSelf->k_dt       = 2.0f;

		return val;
	}


	//modeChange Event
	if (pSelf->curMode != mode)
	{
		pSelf->k_dt = 0;
		pSelf->lastMode = pSelf->curMode;
		pSelf->curMode  = mode;
	}


	pSelf->k_dt += 0.01f;
	if (pSelf->k_dt>2.0f)
		pSelf->k_dt = 2.0f;



	pSelf->modeVal[mode-1] = (pSelf->modeVal[pSelf->lastMode-1] * (2.0f-pSelf->k_dt) + 
	                           val * pSelf->k_dt)/2.0f;


	return pSelf->modeVal[mode-1];
}

