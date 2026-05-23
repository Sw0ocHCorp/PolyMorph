/*
 * ImuEKF.h
 *
 *  Created on: Apr 19, 2026
 *      Author: nclsr
 */

#ifndef INC_FILTERING_IMUEKF_H_
#define INC_FILTERING_IMUEKF_H_

#include "NLKalman.h"

class ImuEKF : public NLKalman {
public:
	ImuEKF(StaticVector<float> initState) : NLKalman(initState) {

	}

	StaticVector<float> predict() {
		return StaticVector<float>(0);
	}

	StaticVector<float> update(StaticVector<float> predictState,
			StaticVector<float> inputMeasurements, StaticVector<float> refSensorMeasurements) {
		return StaticVector<float>(0);
	}

	~ImuEKF() {

	}
};


#endif /* INC_FILTERING_IMUEKF_H_ */
