/*
 * ImuUKF.h
 *
 *  Created on: Apr 19, 2026
 *      Author: nclsr
 */

#ifndef INC_FILTERING_IMUUKF_H_
#define INC_FILTERING_IMUUKF_H_

#include "NLKalman.h"
template<unsigned int SS, unsigned int IS, unsigned int MS>
class ImuUKF : public NLKalman<SS, IS, MS> {
	public:
		ImuUKF(StaticVector<float, SS> initState) : NLKalman<SS, IS, MS>(initState) {

		}
		StaticVector<float, SS> predict(StaticVector<float, IS> inputMeasurements) {
			return {};
		}

		StaticVector<float, SS> update(StaticVector<float, SS> predictState, StaticVector<float, MS> refSensorMeasurements) {
			return {};
		}

		~ImuUKF() {

		}
};


#endif /* INC_FILTERING_IMUUKF_H_ */
