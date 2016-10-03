#ifndef _ISAE_POSE_VEL_TYPE_H_
#define _ISAE_POSE_VEL_TYPE_H_

#include "frame.h"

namespace ISAE {

struct pos_vel {
	uint64_t timestamp; /* ns */
	frame_t frame;
	double x, y, z; /* translation (m) */
	double vx, vy, vz; /* linear velocity (m/s) */
};

}

#endif /*_ISAE_POSE_VEL_TYPE_H_ */
