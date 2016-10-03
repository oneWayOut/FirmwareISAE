#ifndef _ISAE_POSE_TYPE_H_
#define _ISAE_POSE_TYPE_H_

#include <math.h>
#include <stdint.h>
#include "frame.h"

namespace ISAE {
struct pose {
	frame_t frame;
	double x, y, z; /* translation */
	double qw, qx, qy, qz; /* quaternion */
};

struct euler_pose {
	frame_t frame;
	double x, y, z; /* translation */
	double yaw, pitch, roll; /* rotation euler ZYX */
};

struct pose_stamped {
	float sensor_timestamp; /* sec */
	uint64_t host_timestamp; /* nsec */
	pose pos;
};

struct euler_pose_stamped {
	float sensor_timestamp; /* sec */
	uint64_t host_timestamp; /* nsec */
	euler_pose pos;
};

}

#endif /*_ISAE_POSE_TYPE_H_ */
