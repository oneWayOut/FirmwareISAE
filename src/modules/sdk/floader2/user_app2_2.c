#include "../sdk.h"

int fun_user2_2(void)
{
	float value = 0.0f;
	if(api_param_update())
	{
		param_get(param_find("USER_FLOAT_1"),&value);
		printf("fun_user2_2 value=%5.2f\n",(double)value);
	}
	else
	{
		printf("fun_user2_2 value not change\n");
	}
	return 0;
}
                                                  