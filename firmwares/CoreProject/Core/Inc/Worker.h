/*
 * Worker.h
 *
 *  Created on: Apr 13, 2026
 *      Author: nclsr
 */

#ifndef INC_WORKER_H_
#define INC_WORKER_H_

#include "main.h"
#include "EventsManagement.h"
#include <memory>

using namespace std;
template<unsigned int N>
class Worker {
	protected:
		Publisher<void, N> callNextWorkerPublisher;
		std::shared_ptr<Observer<void>> execTaskObserver;
		int freq;
		uint32_t startTime= 0;
		unsigned int ID= 0;
		bool isFirst= false;
		bool isAsync= false;

	public:
		Worker(int freq, bool isAsync, unsigned int id) {
			this->freq= freq;
			execTaskObserver = std::make_shared<Observer<void>>();
			execTaskObserver->setCallback(std::bind(&Worker::startWorkerTask, this));
			this->ID= id;
			this->isAsync= isAsync;
		}

		void setFirstInSchedule() {
			this->isFirst= true;
		}

		void callNextWorker() {
			this->callNextWorkerPublisher.trigger(nullptr);
		}

		void SetNextModule(Worker *nextWorker) {
			callNextWorkerPublisher.addObserver(nextWorker->execTaskObserver);
		}

		void startWorkerTask() {
			uint32_t now= HAL_GetTick();
			if (isFirst || freq < 0 || (int)(now - startTime) >= (1000 / freq) - 1) {
				startTime = now;
				execMainTask();
				callNextWorker();
			}
		}

		virtual void execMainTask()= 0;

		virtual void processFeedBack(uint8_t* feedBackData, uint32_t dataSize) {
		}

		bool getIsAsync() {
			return this->isAsync;
		}
		virtual ~Worker()= default;
};

#endif /* SRC_WORKER_H_ */
