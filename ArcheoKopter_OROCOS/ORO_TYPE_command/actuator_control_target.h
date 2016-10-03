#ifndef _ACTUATOR_CONTROL_TARGET_H_
#define _ACTUATOR_CONTROL_TARGET_H_

namespace ISAE {
	struct actuator_control_target {
		uint64_t time_usec; 
		float controls[8];
	};

}


#endif /* _ACTUATOR_CONTROL_TARGET_H_ */
