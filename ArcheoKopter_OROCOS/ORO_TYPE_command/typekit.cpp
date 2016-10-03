#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

#include "actuator_control_target_serialization.hpp"
#include "attitude_target_serialization.hpp"


namespace ISAE {
class TypekitCommand : public RTT::types::TypekitPlugin {
public:
	virtual std::string getName() { return "typekit-command"; }

	virtual bool loadTypes() {
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<ISAE::actuator_control_target>("actuator_control_target") );
		RTT::types::Types()->addType( new RTT::types::StructTypeInfo<ISAE::attitude_target>("attitude_target") );
		return true;
	}

	virtual bool loadConstructors() { return true; }
	virtual bool loadOperators() { return true; }
};
}

/** Register the class as a plugin */
ORO_TYPEKIT_PLUGIN( ISAE::TypekitCommand);



