/*
 * AbstractSensor.h
 *
 *  Created on: Apr 17, 2026
 *      Author: nclsr
 */

#ifndef INC_SENSORS_ABSTRACTSENSOR_H_
#define INC_SENSORS_ABSTRACTSENSOR_H_

#include "main.h"
#include "Interfaces/AbstractInterface.h"
#include "MessageStructs/AbstractMessage.h"

template<unsigned int N, unsigned int FS>
class AbstractSensor: public Worker<1> {
protected:
	AbstractMessage<FS>* measurements;
	Publisher<AbstractMessage<FS>, N> measurementsPublisher;

public:
	AbstractSensor(int freq, bool isAsync, unsigned int id,
					unsigned int nNextWorkersObservers, unsigned int nMeasurementObservers) :
			Worker(freq, isAsync, id) {
	}

	virtual void execMainTask()= 0;

	void processMeasurements() {
	}

	void processFeedBack(uint8_t* feedBackData, uint16_t dataSize) {
		processMeasurements();
	}

	virtual void attachMeasurementsObservers(std::shared_ptr<Observer<AbstractMessage<FS>>> obs)= 0;
};


#endif /* INC_SENSORS_ABSTRACTSENSOR_H_ */
