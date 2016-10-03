#ifndef _ISAE_TWIST_H_
#define _ISAE_TWIST_H_

#include "frame.h"

namespace ISAE {
	struct twist {
		uint64_t timestamp; // ns
		frame_t frame;
		double vx, vy, vz; // linear velocity (m/s)
		double rx, ry, rz; // angular velociy (rad/s)
	};
}

#endif /* _ISAE_TWIST_H_ */
