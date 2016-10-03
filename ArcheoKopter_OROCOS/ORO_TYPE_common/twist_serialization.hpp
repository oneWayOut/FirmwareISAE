#ifndef TWIST_SERIALIZATION_HPP_
#define TWIST_SERIALIZATION_HPP_

#include <boost/serialization/serialization.hpp>

#include "twist.h"

namespace boost {
namespace serialization {

	template<class Archive>
	void serialize( Archive& a, ISAE::twist& twi, unsigned int) {
		using boost::serialization::make_nvp;
		using boost::serialization::make_array;

		a & make_nvp("timestamp", twi.timestamp);
		a & make_nvp("frame", twi.frame);
		a & make_nvp("vx", twi.vx);
		a & make_nvp("vy", twi.vy);
		a & make_nvp("vz", twi.vz);
		a & make_nvp("rx", twi.rx);
		a & make_nvp("ry", twi.ry);
		a & make_nvp("rz", twi.rz);
	}
}}


#endif /* TWIST_SERIALIZATION_HPP_ */
