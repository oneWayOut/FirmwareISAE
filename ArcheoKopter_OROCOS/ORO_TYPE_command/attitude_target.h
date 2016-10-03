#ifndef _ISAE_ATTITUDE_TARGET_H_
#define _ISAE_ATTITUDE_TARGET_H_

namespace ISAE {
	// Set the vehicle attitude and body angular rates.
	// XXX The mavlink structure use quaternion in input, but for consistency
	// with the rest of the system, use euler here.
	struct attitude_target {
		uint64_t timestamp;
		uint8_t type_mask; // Mappings: If any of these bits are set, the
						   // corresponding input should be ignored: bit 1: body roll rate, bit 2:
						   // body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, 
						   // bit 7: thrust, bit 8: attitude
						   // So For "RPY + thrust control", set type_mask to 7
		float roll, pitch, yaw;  // radian [-pi, pi]
		float body_roll_rate, body_pitch_rate, body_yaw_rate; // rad.s-1
		float thrust;  // normalized [ -1, 1 ]
	};
}


#endif /* _ISAE_ATTITUDE_TARGET_H_ */
