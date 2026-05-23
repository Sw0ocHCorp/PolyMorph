/*
 * AbstractMessage.h
 *
 *  Created on: Apr 15, 2026
 *      Author: nclsr
 */

#ifndef ABSTRACTMESSAGE_H_
#define ABSTRACTMESSAGE_H_

#include "StaticVector.h"
#include <cstdint>

#define SOF "abcd"

template<unsigned int N>
class AbstractMessage {
    public:
		AbstractMessage(){}
        ~AbstractMessage() {}
        void fillFromFrame(StaticVector<uint8_t, N>& frame) {

        }
        /*StaticVector<uint8_t, N> toFrame() {

        }*/
};

#endif /* ABSTRACTMESSAGE_H_ */
