#include "../sdk.h"

static struct st_controls_s _output_info;
int fun_user1_2(void)
{
	static float base_value = -1.0f;
	static float delta_value = 0.1f;
	uint64_t time_us = 0;
	uint64_t hour = 0;
	uint64_t minute = 0;
	uint64_t second = 0;


	

	for(uint8_t i=0;i<14;i++){
		_output_info.control[i] = base_value;
	}
	api_send_controls_data(_output_info);
	printf("fun_user1_2:pwm output:%5.3f\n",(double)base_value);
	
	if(base_value < -0.95f){
		delta_value = 0.1f;
	}
	if(base_value > 0.95f){
		delta_value  = -0.1f;
	}
	base_value += delta_value;
	

	
	time_us = hrt_absolute_time();
	hour = time_us/3600000000;
	minute = (time_us/60000000)%60;
	second = (time_us/1000000)%60;
	
	printf("fun_user1_2: time = %llu %02llu:%02llu:%02llu\n",time_us,hour,minute,second);
	

	return 0;
}
                                                  
