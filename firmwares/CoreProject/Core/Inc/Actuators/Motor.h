/*
 * Motor.h
 *
 *  Created on: 12 mai 2026
 *      Author: nclsr
 */

#ifndef INC_ACTUATORS_MOTOR_H_
#define INC_ACTUATORS_MOTOR_H_

#include "MessageStructs/MotorState.h"
#include "Worker.h"

struct MotorConfig {
public:
	unsigned int motorType;
	bool isInClosedLoop;
	TIM_HandleTypeDef* pwmSignal;
	StaticVector<uint8_t, 3> pwmChannels;
	float acceleration;
	int angleRange;
	float minDutyCycle;
	float maxDutyCycle;
};

class Motor : public Worker<1> {
private:
	MotorState state;
	MotorState setpoint;
	MotorConfig config;

	virtual void updateTorque() {

	}

	virtual void updateAngle() {
		if (this->config.acceleration > 0 && (setpoint.getAngle() - state.getAngle() <= -this->config.acceleration || setpoint.getAngle() - state.getAngle() >= this->config.acceleration)) {
			float angleCmd= state.getAngle() + this->config.acceleration;
			if (this->config.isInClosedLoop == false) {
				state.setAngle(angleCmd);
			}
		}
		for (int i= 0; i < this->config.pwmChannels.size(); i++) {
			float dutyCycle= this->config.minDutyCycle + ((state.getAngle()) * (this->config.maxDutyCycle - this->config.minDutyCycle) / (this->config.angleRange));
			float ccr= dutyCycle * (float)this->config.pwmSignal->Init.Period;
			__HAL_TIM_SET_COMPARE(this->config.pwmSignal, this->config.pwmChannels[i], (int)ccr);
		}
	}

	virtual void updateSpeed() {
		if (this->config.acceleration > 0 && setpoint.getSpeed() - state.getSpeed() > -this->config.acceleration && setpoint.getSpeed() - state.getSpeed() < this->config.acceleration) {
			float speedCmd= state.getSpeed() + this->config.acceleration;
			if (this->config.isInClosedLoop == false) {
				state.setSpeed(speedCmd);
			}

		}
	}
public:
	Motor(MotorConfig config, int freq, unsigned int id) : Worker<1>(freq, true, id) {
		this->config= config;
	}

	virtual void init() {
		for (int i= 0; i < this->config.pwmChannels.size(); i++) {
			HAL_TIM_PWM_Start(this->config.pwmSignal, this->config.pwmChannels[i]);
		}
	}

	virtual void execMainTask() {
		if (this->config.motorType == SERVO_MOTOR) {
			updateAngle();
		} else if (this->config.motorType == BLDC_MOTOR) {
			updateAngle();
		} else {
			updateAngle();
			updateTorque();
			updateAngle();
		}


	}

	void processFeedBack(uint8_t* feedBackData, uint32_t dataSize) {

	}

	void applySetpoint(MotorState setpointState) {
		setpoint= setpointState;
	}

	~Motor() {

	}
};


#endif /* INC_ACTUATORS_MOTOR_H_ */
