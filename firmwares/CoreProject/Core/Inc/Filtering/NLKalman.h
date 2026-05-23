/*
 * NLKalman.h
 *
 *  Created on: Apr 19, 2026
 *      Author: nclsr
 */

#ifndef INC_FILTERING_NLKALMAN_H_
#define INC_FILTERING_NLKALMAN_H_

#include "StaticVector.h"

template<unsigned int SS, unsigned int IS, unsigned int MS>
class NLKalman {
protected:
	StaticVector<float, SS> currentState;
	virtual StaticVector<float, SS> predict(StaticVector<float, IS> inputMeasurements)= 0;
	virtual StaticVector<float, SS> update(StaticVector<float, SS> predictState, StaticVector<float, MS> refSensorMeasurements)= 0;

public:
	NLKalman(StaticVector<float, SS> initState) : currentState(initState.capacity()) {
		this->currentState= initState;
	}

	void runFiltering(StaticVector<float, IS> inputMeasurements, StaticVector<float, MS> refSensorMeasurements) {
		StaticVector<float, SS> predictState= predict(inputMeasurements);
		this->currentState= update(predictState, refSensorMeasurements);

	}

	~NLKalman() {

	}
};


#endif /* INC_FILTERING_NLKALMAN_H_ */
