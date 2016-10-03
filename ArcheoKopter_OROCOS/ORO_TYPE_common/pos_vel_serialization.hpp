#ifndef POS_VEL_SERIALIZATION_HPP_
#define POS_VEL_SERIALIZATION_HPP_

#include <boost/serialization/serialization.hpp>

#include "pos_vel.h"

namespace boost {
namespace serialization {

	template<class Archive>
	void serialize( Archive& a, ISAE::pos_vel& pv, unsigned int) {
		using boost::serialization::make_nvp;
		using boost::serialization::make_array;

		a & make_nvp("timestamp", pv.timestamp);
		a & make_nvp("frame", pv.frame);
		a & make_nvp("x", pv.x);
		a & make_nvp("y", pv.y);
		a & make_nvp("z", pv.z);
		a & make_nvp("vx", pv.vx);
		a & make_nvp("vy", pv.vy);
		a & make_nvp("vz", pv.vz);
	}
}}


#endif /* POS_VEL_SERIALIZATION_HPP_ */
