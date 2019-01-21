#include "../sdk.h"

static struct mission_s _app_mission;
static struct mission_item_s _way_point;
static struct home_position_s _home_position;

int fun_user2_1(int argc, char *argv[])
{
	bool airline_updated = false;
	while(1){
		sleep(1);
		if(true == api_get_new_airline(&_app_mission,&airline_updated)){
			printf("new mission:seq=%d cnt=%d id=%d",_app_mission.current_seq,_app_mission.count,_app_mission.dataman_id);
			printf("points:\n");
			for(int32_t offset=0;offset<_app_mission.count;offset++){
				api_get_way_point(offset,&_way_point);
#if 0
			
				printf("%d:cmd=%d lon=%.7f lat=%.7f alt=%.2f yaw=%.1f tim=%.f head=%d rel=%d\n",i+1,_way_point.nav_cmd,_way_point.lon,_way_point.lat,(double)_way_point.altitude,\
					(double)_way_point.yaw,(double)_way_point.time_inside,_way_point.force_heading,_way_point.altitude_is_relative);
				printf("p1=%.2f p2=%.2f p3=%.2f p4=%.2f p5=%.2f p6=%.2f p7=%.2f\n",(double)_way_point.params[0],(double)_way_point.params[1],(double)_way_point.params[2],(double)_way_point.params[3],(double)_way_point.params[4],(double)_way_point.params[5],(double)_way_point.params[6]);
				printf(" 0x");
				for(uint16_t j=0;j<sizeof(_way_point);j++){	
					printf("%02x",((uint8_t *)&_way_point)[j]);
				}
				printf("\n");
#else
				if((_way_point.nav_cmd != NAV_CMD_TAKEOFF) && (_way_point.nav_cmd != NAV_CMD_WAYPOINT) && (_way_point.nav_cmd != NAV_CMD_DO_CHANGE_SPEED) && (_way_point.nav_cmd != NAV_CMD_LAND)){
					printf("No %d  :invalid way point\n");
				}
				else{
					if(_way_point.nav_cmd == NAV_CMD_DO_CHANGE_SPEED){
						printf("No %d  :type=change speed,newspeed = %.2f\n",offset+1,(double)_way_point.params[1]);
					}
					else{
						printf("No %d  :",offset+1);
						if(_way_point.nav_cmd == NAV_CMD_TAKEOFF){
							printf("type=takeoff,");
						}
						if(_way_point.nav_cmd == NAV_CMD_WAYPOINT){
							printf("type=navigator,");
						}
						if(_way_point.nav_cmd == NAV_CMD_DO_CHANGE_SPEED){
							printf("type=changeSpeed,");
						}
						if(_way_point.nav_cmd == NAV_CMD_LAND){
							printf("type=land,");
						}

						printf("lon=%.3f lat=%.3f alt=%.3f(%s) holdTime=%.2f\n",_way_point.lon,_way_point.lat,(double)_way_point.altitude,\
							_way_point.altitude_is_relative ? "relative" :"absolute",(double)_way_point.time_inside);
						printf("       acceptance_radius=%.2f  loiter_radius=%.2f ",(double)_way_point.acceptance_radius,(double)_way_point.loiter_radius);
						if(isfinite(_way_point.yaw)){
							printf("yaw=%.2f\n",(double)_way_point.yaw);
						}
						else{
							printf("yaw=invalid\n");
						}
					}
				}
#endif
			}
		}
		else{
			if(true == airline_updated){/*如果航线数据更新了，但是上一次没有成功读取到数据*/
				api_get_new_airline(&_app_mission,&airline_updated);
			}
		}

		if(api_get_home_position(&_home_position)){
			printf("home position:lat=%.6f lon=%.6f alt=%.2f yaw=%.2f manual_home=%d valid_alt=%d\n",_home_position.lon,_home_position.lat,(double)_home_position.alt,(double)_home_position.yaw,_home_position.manual_home,_home_position.valid_alt);
		}
	}
	return 0;
}
                                                  