/*
 * Rs232WmImu.h
 *
 *  Created on: Apr 14, 2026
 *      Author: nclsr
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "MessageStructs/ImuMeasurements.h"
#include "AbstractSensor.h"
#include "MessageStructs/AbstractMessage.h"
#include "EventsManagement.h"
#include "main.h"
template<unsigned int N, unsigned int FS>
class IMU : public AbstractSensor<N, FS> {
private:
public:
	IMU(int freq, bool isAsync, unsigned int id, unsigned int numMeasurementObservers) :
		AbstractSensor<N, FS>(freq, isAsync, id, numMeasurementObservers) {

	}

	virtual ~IMU() {

	}

	void execMainTask() {
		/*if (this->isAsync) {
			this->listeningQueryPublisher.trigger(nullptr);
			//launchAsyncRead(this->buffer, &this->sz);
		} else {

		}*/
	}

	/*void processMeasurements() {
		ImuMeasurements* measurements= (ImuMeasurements*)this->codec->decode(buffer, sz);
		this->measurementsPublisher.trigger(measurements);
		callNextWorker();
	}*/

	void launchAsyncRead(uint8_t* buffer, uint16_t* bufferSize) {
		//HAL_UART_Receive_IT(this->uartPort, buffer, *bufferSize);
	}

	void attachMeasurementsObservers(std::shared_ptr<Observer<AbstractMessage<FS>>> obs) {
		this->measurementsPublisher.addObserver(obs);
	}
};


#endif /* INC_IMU_H_ */
