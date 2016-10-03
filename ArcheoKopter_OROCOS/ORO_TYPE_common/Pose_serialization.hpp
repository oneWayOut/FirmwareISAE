#ifndef _ISAE_POSE_SERIALIZATION_H_
#define _ISAE_POSE_SERIALIZATION_H_

#include <boost/serialization/serialization.hpp>
#include <rtt/types/EnumTypeInfo.hpp>

#include "Pose.h"

namespace boost {
namespace serialization {
	
	template<class Archive>
	void serialize( Archive& a, ISAE::pose& p, unsigned int) {
		using boost::serialization::make_nvp;
		a & make_nvp("frame", p.frame);
		a & make_nvp("x", p.x);
		a & make_nvp("y", p.y);
		a & make_nvp("z", p.z);
		a & make_nvp("qw", p.qw);
		a & make_nvp("qx", p.qx);
		a & make_nvp("qy", p.qy);
		a & make_nvp("qz", p.qz);
	}

	template<class Archive>
	void serialize( Archive& a, ISAE::euler_pose& p, unsigned int) {
		using boost::serialization::make_nvp;
		a & make_nvp("frame", p.frame);
		a & make_nvp("x", p.x);
		a & make_nvp("y", p.y);
		a & make_nvp("z", p.z);
		a & make_nvp("yaw", p.yaw);
		a & make_nvp("pitch", p.pitch);
		a & make_nvp("roll", p.roll);
	}

	template<class Archive>
	void serialize( Archive& a, ISAE::pose_stamped& p, unsigned int) {
		using boost::serialization::make_nvp;
		a & make_nvp("sensor_timestamp", p.sensor_timestamp);
		a & make_nvp("host_timestamp", p.host_timestamp);
		a & make_nvp("pos", p.pos);
	}

	template<class Archive>
	void serialize( Archive& a, ISAE::euler_pose_stamped& p, unsigned int) {
		using boost::serialization::make_nvp;
		a & make_nvp("sensor_timestamp", p.sensor_timestamp);
		a & make_nvp("host_timestamp", p.host_timestamp);
		a & make_nvp("pos", p.pos);
	}
}}

#endif /*_ISAE_POSE_SERIALIZATION_H_*/
