/*
 * HardwareInterface.h
 *
 *  Created on: Apr 14, 2026
 *      Author: nclsr
 */

#ifndef INC_INTERFACES_ABSTRACTINTERFACE_H_
#define INC_INTERFACES_ABSTRACTINTERFACE_H_

#include "StaticVector.h"
#include <functional>
template<unsigned int FS, unsigned int NOBS>
class AbstractInterface {
	protected:
		bool isAsync= false;
		StaticVector<uint8_t, FS> buffer;
		std::shared_ptr<Observer<StaticVector<uint8_t, FS>>> workerFrameObserver;
		Publisher<StaticVector<uint8_t, FS>, NOBS> receivedFramePublisher;
	public:
		AbstractInterface() {
			this->workerFrameObserver= std::make_shared<Observer<StaticVector<uint8_t, FS>>>();
			//Using std::placeholders::_1 to be able to pass the frame to send by the async way to the event.trigger() function
			this->workerFrameObserver->setCallback(std::bind(&AbstractInterface::sendFrameAsync, this, std::placeholders::_1));
		};
		void connect() {

		}
		void sendBuffer(unsigned int* obsId) {
			this->receivedFramePublisher.trigger(&buffer, *obsId);
		}
		virtual void readFrameAsync()= 0;
		virtual void readFrame(unsigned int timeout)= 0;
		virtual void sendFrameAsync(StaticVector<uint8_t, FS>*  frame)= 0;
		virtual void sendFrame(StaticVector<uint8_t, FS>  frame, unsigned int timeout)= 0;
		virtual void processFrame()= 0;

		void attachFrameObserver(std::shared_ptr<Observer<StaticVector<uint8_t, FS>>> obs) {
			this->receivedFramePublisher.addObserver(obs);
		}

		std::shared_ptr<Observer<StaticVector<uint8_t, FS>>> getFrameObserver() {
			return this->workerFrameObserver;
		}

		bool getIsAsync() {
			return this->isAsync;
		}
		~AbstractInterface() {};
};


#endif /* INC_INTERFACES_ABSTRACTINTERFACE_H_ */
