#include "sdk.h"


PARAM_DEFINE_INT32(USER_INT32_1, 0);
PARAM_DEFINE_FLOAT(USER_FLOAT_1, 0.0f);



sem_t _user_sem_50hz;
sem_t _user_sem_100Hz;


__EXPORT int sl_app_main(int argc, char *argv[])
{
	uint16_t cnt = 0;
	sem_init(&_user_sem_50hz,0,0);/*信号量初始化为锁定状态*/
	sem_init(&_user_sem_100Hz,0,0);/*信号量初始化为锁定状态*/
	/*将新增的参数显示到地面站上*/
	(void)param_find("USER_INT32_1");
	(void)param_find("USER_FLOAT_1");

	px4_task_spawn_cmd("user1_1",
							 SCHED_DEFAULT,
							 SCHED_PRIORITY_MAX - 5,
							 3000,
							 user1_1_main,
							 (char * const *)argv);


	px4_task_spawn_cmd("fun_user2_1",
							 SCHED_DEFAULT,
							 SCHED_PRIORITY_DEFAULT,
							 2000,
							 fun_user2_1,
							 (char * const *)argv);

	while(1)
	{		
		sem_wait(&_sdk_sem_newData);/*200HZ*/
		if (cnt%2 == 0 && _user_sem_100Hz.semcount <= 0){
			sem_post(&_user_sem_100Hz);
		}
		//这个是200HZ的频率执行
		if(cnt >= 3){
			cnt = 0;
			if(_user_sem_50hz.semcount <= 0)
			{
	    		sem_post(&_user_sem_50hz);
			}	
		}
		else{
			cnt ++;
		}

		
#if 0
		static int cnt = 0;
		/*这个信号量以250HZ频率触发，每2s会运行括号中的程序*/
		sem_wait(&_sdk_sem_250hz);
		cnt++;
		if(cnt % 1000 == 0)
		{
			cnt = 0;
			fun_user1_2();/*每4s更新一次pwm*/
			//fun_user2_1();
			//fun_user2_2();

			/*每4s向地面站打印一条消息*//*切记:每秒不要打印超过三条，不然缓冲区可能会溢出*/
			mavlink_log_info(mavlink_log_fd, "hello world %d",123);
			printf("hello world\n");
		}


		if(cnt % 5 == 0)/*每四个4ms，也就是20ms触发一次信号量*/
		{
			if(_user_sem_50hz.semcount <= 0)
			{
	    		sem_post(&_user_sem_50hz);
			}	
		}
#else/*这里屏蔽了另一种固定时间运行程序的例子，这样子触发_user_sem_50hz也非常准确*/
		/*任务睡眠5s后运行程序*/
/*
		usleep(5000000);
		if(_user_sem_50hz.semcount <= 0)
		{
    		sem_post(&_user_sem_50hz);
		}
		fun_user1_2();
		fun_user2_1();
		fun_user2_2();
		printf("hello world\n");
		*/

#endif	

/*示例:读取遥控器通道的杆量(-1.0f-1.0f)，并打印出来*/
#if 0
	struct st_rc_s rcDataIn;
	api_get_rc_data(&rcDataIn);

	if(cnt%250==0)
		{
		int rcChannel=0;
		for(rcChannel=0;rcChannel<19;rcChannel++)
			printf("%5d-",rcChannel);
		printf("\n");
		
		for(rcChannel=0;rcChannel<19;rcChannel++)
			printf("%5.1f-",(double)rcDataIn.channels[rcChannel]);
		printf("\n\n\n");
		}
#endif

/*示例:读取所有航线的所有航点信息，并打印出来*/
#if 0
		struct st_waypoint_info_s waypointInfo;
		
		if(cnt==0)
		{

			for(airlineIndex=0;airlineIndex<MAX_Airline_Num;airlineIndex++)
			{
				int waypointCount = api_get_waypointCount(airlineIndex);
				for(wpIndex=0; wpIndex < waypointCount; wpIndex++)
				{
				
					while((api_get_us_time()-time_record_us)<1000*500)//0.5s
						{;}
				
					time_record_us=api_get_us_time();
					api_get_waypoint(airlineIndex,wpIndex,&waypointInfo);
					printf("\nwaypoint %d-%d\n",waypointInfo.airline_index,waypointInfo.waypoint_index);
					printf("lat:%9.6f lon:%9.6f alt:%6.1f cmd:%d ",(double)waypointInfo.lat,
																(double)waypointInfo.lon,
																(double)waypointInfo.alt,
																waypointInfo.waypoint_type);
					printf("param:%6.1f-%6.1f-%6.1f-%6.1f\n",(double)waypointInfo.param1,
															(double)waypointInfo.param2,
															(double)waypointInfo.param3,
															(double)waypointInfo.param4);
				}
			}
		}
#endif
	}

	return 0;
}
