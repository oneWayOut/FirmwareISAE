#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

#include "Pose_serialization.hpp"
#include "attitude_serialization.hpp"
#include "twist_serialization.hpp"
#include "pos_vel_serialization.hpp"


namespace ISAE {
class TypekitCommon : public RTT::types::TypekitPlugin {
public:
    virtual std::string getName() { return "isae-common-typekit"; }

	virtual bool loadTypes() {
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<pose>("pose") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<pose_stamped>("pose_stamped") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<euler_pose>("euler_pose") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<euler_pose_stamped>("euler_pose_stamped") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<attitude>("attitude") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<attitude_euler>("attitude_euler") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<twist>("twist") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<pos_vel>("pos_vel") );

		return true;
	}

	virtual bool loadConstructors() { return true; }
	virtual bool loadOperators() { return true; }
};
}

/** Register the class as a plugin */
ORO_TYPEKIT_PLUGIN( ISAE::TypekitCommon );
