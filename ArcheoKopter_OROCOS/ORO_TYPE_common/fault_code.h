#ifndef _ISAE_FAULT_CODE_H_
#define _ISAE_FAULT_CODE_H_

namespace ISAE {

	struct fault_code {
		uint64_t timestamp;
		uint8_t componentID;
		uint8_t faultCode;
	};
}


#endif /* _ISAE_FAULT_CODE_H_ */
