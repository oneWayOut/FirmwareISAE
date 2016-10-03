#ifndef _ISAE_ATTITUDE_H_
#define _ISAE_ATTITUDE_H_

#include "frame.h"

namespace ISAE {
	// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
	struct attitude {
		uint64_t timestamp;
		frame_t  frame;
		float qw, qx, qy, qz; 
		float rollspeed, pitchspeed, yawspeed;  // radian / s
	};

	struct attitude_euler {
		uint64_t timestamp;
		frame_t  frame;
		float roll, pitch, yaw;  // radian [-pi, pi]
		float rollspeed, pitchspeed, yawspeed;  // radian / s
	};
}


#endif /* _ISAE_ATTITUDE_H_ */
