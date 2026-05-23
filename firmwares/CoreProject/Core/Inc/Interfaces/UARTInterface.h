/*
 * UARTInterface.h
 *
 *  Created on: Apr 17, 2026
 *      Author: nclsr
 */

#ifndef INC_INTERFACES_UARTINTERFACE_H_
#define INC_INTERFACES_UARTINTERFACE_H_

#include "main.h"
#include "StaticVector.h"

template<unsigned int FS, unsigned int NOBS>
class UARTInterface : public AbstractInterface<FS, NOBS> {
	private:
		UART_HandleTypeDef* port;
		uint8_t asyncByte;

	public:
		UARTInterface(UART_HandleTypeDef* uartPort) :
				AbstractInterface<FS, NOBS>() {
			this->port= uartPort;
		}
		void connect() {

		}

		StaticVector<uint8_t, FS>& getBuffer() {
			return this->buffer;
		}

		void askPortListening() {
			HAL_UART_Receive_IT(this->port, &asyncByte, 1);
		}

		void readFrameAsync(){
			if (this->buffer.size() == this->buffer.capacity()) {
				this->buffer.removeAt(0);
			}
			this->buffer.push_back(asyncByte);
			HAL_UART_Receive_IT(this->port, &asyncByte, 1);
		}

		void storeReceivedAsyncByte() {
			if (this->buffer.size() < this->buffer.capacity()) {
				this->buffer.push_back(asyncByte);
			}
		}

		void readFrame(unsigned int timeout){
			uint8_t byte;
			while (true) {
				if (this->buffer.size() < this->buffer.capacity() && HAL_UART_Receive(this->port, &byte, 1, timeout) != HAL_OK) {
					this->buffer.push_back(byte);
				} else {
					break;
				}
			}
		}

		void sendFrameAsync(StaticVector<uint8_t, FS>*  frame) {
			HAL_UART_Transmit_IT(this->port, frame->data(), frame->size());
		}

		void sendFrame(StaticVector<uint8_t, FS>  frame, unsigned int timeout) {
			HAL_UART_Transmit(port, frame.data(), frame.size(), timeout);
		}
		void processFrame() {

		}
		bool getIsAsync() {
			return this->isAsync;
		}
		~UARTInterface() {

		}
};


#endif /* INC_INTERFACES_UARTINTERFACE_H_ */
