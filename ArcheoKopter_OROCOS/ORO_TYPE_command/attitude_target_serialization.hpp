#ifndef ATTITUDE_TARGET_SERIALIZATION_HPP_
#define ATTITUDE_TARGET_SERIALIZATION_HPP_

#include <boost/serialization/serialization.hpp>

#include "attitude_target.h"

namespace boost {
namespace serialization {


	template<class Archive>
	void serialize( Archive& a, ISAE::attitude_target& att, unsigned int) {
		using boost::serialization::make_nvp;
		using boost::serialization::make_array;

		a & make_nvp("timestamp", att.timestamp);
		a & make_nvp("type_mask", att.type_mask);
		a & make_nvp("roll", att.roll);
		a & make_nvp("pitch", att.pitch);
		a & make_nvp("yaw", att.yaw);
		a & make_nvp("body_roll_rate", att.body_roll_rate);
		a & make_nvp("body_pitch_rate", att.body_pitch_rate);
		a & make_nvp("body_yaw_rate", att.body_yaw_rate);
		a & make_nvp("thrust", att.thrust);
	}
}}


#endif /* ATTITUDE_TARGET_SERIALIZATION_HPP_ */
