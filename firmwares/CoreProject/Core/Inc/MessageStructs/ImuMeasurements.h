/*
 * ImuMeasurements.h
 *
 *  Created on: Apr 17, 2026
 *      Author: nclsr
 */

#ifndef INC_MESSAGESTRUCTS_IMUMEASUREMENTS_H_
#define INC_MESSAGESTRUCTS_IMUMEASUREMENTS_H_

#include "StaticVector.h"
#include "AbstractMessage.h"

class ImuMeasurements : public AbstractMessage<20> {
private:
	StaticVector<float, 3> gyroscope;
	StaticVector<float, 3>accelerometer;
	StaticVector<float, 3>magnetometer;
public:
	void setGyroscope(StaticVector<float, 3> gyroscopeData) {
		this->gyroscope= gyroscopeData;
	}

	StaticVector<float, 3> getGyroscope() {
		return this->gyroscope;
	}

	void setAccelerometer(StaticVector<float, 3> accelerometerData) {
		this->accelerometer= accelerometerData;
	}

	StaticVector<float, 3> getAccelerometer() {
		return this->accelerometer;
	}

	void setMagnetometer(StaticVector<float, 3> magnetometerData) {
		this->magnetometer= magnetometerData;
	}

	StaticVector<float, 3> getMagnetometer() {
		return this->magnetometer;
	}

};



#endif /* INC_MESSAGESTRUCTS_IMUMEASUREMENTS_H_ */
