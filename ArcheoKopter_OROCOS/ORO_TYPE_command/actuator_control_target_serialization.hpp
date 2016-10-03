#ifndef ACTUATOR_CONTROL_TARGET_SERIALIZATION_HPP_
#define ACTUATOR_CONTROL_TARGET_SERIALIZATION_HPP_

#include <boost/serialization/serialization.hpp>

#include "actuator_control_target.h"

namespace boost {
namespace serialization {


	template<class Archive>
	void serialize( Archive& a, ISAE::actuator_control_target& s, unsigned int) {
		using boost::serialization::make_nvp;
		using boost::serialization::make_array;

		a & make_nvp("time_usec", s.time_usec);
		a & make_nvp("controls", make_array(s.controls, sizeof(s.controls)/sizeof(s.controls[0])));
	}
}}


#endif /* ACTUATOR_CONTROL_TARGET_SERIALIZATION_HPP_ */
