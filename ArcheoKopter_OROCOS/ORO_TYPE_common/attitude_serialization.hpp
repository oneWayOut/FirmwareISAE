#ifndef ATTITUDE_SERIALIZATION_HPP_
#define ATTITUDE_SERIALIZATION_HPP_

#include <boost/serialization/serialization.hpp>

#include "attitude.h"

namespace boost {
namespace serialization {

	template<class Archive>
	void serialize( Archive& a, ISAE::attitude& att, unsigned int) {
		using boost::serialization::make_nvp;
		using boost::serialization::make_array;

		a & make_nvp("timestamp", att.timestamp);
		a & make_nvp("frame", att.frame);
		a & make_nvp("qw", att.qw);
		a & make_nvp("qx", att.qx);
		a & make_nvp("qy", att.qy);
		a & make_nvp("qz", att.qz);
		a & make_nvp("rollspeed", att.rollspeed);
		a & make_nvp("pitchspeed", att.pitchspeed);
		a & make_nvp("yawspeed", att.yawspeed);
	}

	template<class Archive>
	void serialize( Archive& a, ISAE::attitude_euler& att, unsigned int) {
		using boost::serialization::make_nvp;
		using boost::serialization::make_array;

		a & make_nvp("timestamp", att.timestamp);
		a & make_nvp("frame", att.frame);
		a & make_nvp("roll", att.roll);
		a & make_nvp("pitch", att.pitch);
		a & make_nvp("yaw", att.yaw);
		a & make_nvp("rollspeed", att.rollspeed);
		a & make_nvp("pitchspeed", att.pitchspeed);
		a & make_nvp("yawspeed", att.yawspeed);
	}
}}


#endif /* ATTITUDE_SERIALIZATION_HPP_ */
