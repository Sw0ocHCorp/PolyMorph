/*
 * MotorState.h
 *
 *  Created on: 12 mai 2026
 *      Author: nclsr
 */

#ifndef INC_MESSAGESTRUCTS_MOTORSTATE_H_
#define INC_MESSAGESTRUCTS_MOTORSTATE_H_

#include "MessageStructs/AbstractMessage.h"

#define SERVO_MOTOR 1	//Control in angle
#define BLDC_MOTOR  2	//Control in speed
#define FOC_MOTOR	3	//Control in torque / speed / angle

class MotorState : public AbstractMessage<25> {
private:
	float torque;
	float angle;
	float speed;
	unsigned int motorType;
public:
	MotorState() {

	}

	void setTorque(float trq) {
		torque= trq;
	}

	float getTorque() {
		return torque;
	}

	void setAngle(float angl) {
		angle= angl;
	}

	float getAngle() {
		return angle;
	}

	void setSpeed(float spd) {
		speed= spd;
	}

	float getSpeed() {
		return speed;
	}

	~MotorState() {

	}
};


#endif /* INC_MESSAGESTRUCTS_MOTORSTATE_H_ */
