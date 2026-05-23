/*
 * GNSSMeasurements.h
 *
 *  Created on: Apr 20, 2026
 *      Author: nclsr
 */

#ifndef INC_MESSAGESTRUCTS_GNSSMEASUREMENTS_H_
#define INC_MESSAGESTRUCTS_GNSSMEASUREMENTS_H_

#include "StaticVector.h"
#include "AbstractMessage.h"
template<unsigned int BS>
class GNSSMeasurements : public AbstractMessage<50> {
private:
	StaticVector<double, 3> location= {0.0, 0.0, 0.0};
	uint8_t signalQuality;
	float hdop;
	float speed;
	float bearing;
	bool isValid;
public:
	GNSSMeasurements() {

	}

	void fillFromFrame(StaticVector<uint8_t, BS>& frame) {
		if (frame.size() == frame.capacity()) {
			for (unsigned int i= 0; i < frame.size(); i++) {
				int idx1= frame.contains((uint8_t*)"$GNGGA", 6);
				int idx2= frame.contains((uint8_t*)"$GPGGA", 6);
				if ((idx1 >= 0 && idx1 + 50 <= frame.size() )|| (idx2 >= 0 && idx2 + 50 <= frame.size() )) {
					if (idx1 >= 0) {
						i= (unsigned int)idx1;
					} else {
						i= (unsigned int)idx2;
					}
					decodeGGA(frame.subVec(i+7, frame.size(), false).data(), frame.size() - (i+7));
					/*GNSSMeasurements m= decodeGGA(frame.subVec(i+7, frame.size(), false).data(), frame.size() - (i+7));//decodeGGA(frame.subVec(i+6, frame.size(), false).data(), frame.size() - i+6);
					measurements.setSignalQuality(m.getSignalQuality());
					if (measurements.getSignalQuality() > 0) {
						measurements.setLatitude(m.getLatitude());
						measurements.setLongitude(m.getLongitude());
						measurements.setHDop(m.getHDop());
					}*/
				}
				idx1= frame.contains((uint8_t*)"$GNRMC", 6);
				idx2= frame.contains((uint8_t*)"$GPRMC", 6);
				if ((idx1 >= 0 && idx1 + 50 <= frame.size() )|| (idx2 >= 0 && idx2 + 50 <= frame.size() )) {
					if (idx1 >= 0) {
						i= (unsigned int)idx1;
					} else {
						i= (unsigned int)idx2;
					}
					decodeRMC(frame.subVec(i+7, frame.size(), false).data(), frame.size() - (i+7));
					/*GNSSMeasurements m= decodeRMC(frame.subVec(i+7, frame.size(), false).data(), frame.size() - (i+7));
					measurements.setValidStatus(m.getValidStatus());
					if (measurements.getValidStatus()) {
						measurements.setLatitude(m.getLatitude());
						measurements.setLongitude(m.getLongitude());
						measurements.setSpeed(m.getSpeed());
						measurements.setBearing(m.getBearing());
					}*/
				}
			}
		}
	}

	StaticVector<uint8_t, 50> toFrame() {
		StaticVector<uint8_t, 50> frame;
		return frame;
	}

	double parseCoordinate(const uint8_t* frame, unsigned int frameSize) {
		int dotIdx = -1;
		for (unsigned int i = 0; i < frameSize; ++i) {
			if (frame[i] == '.') {
				dotIdx = (int)i;
				break;
			}
		}
		if (dotIdx < 2)
			return 0.0;  // need at least "dd." before the dot
		double deg = 0.0;
		for (int i = 0; i < dotIdx - 2; ++i) {
			if (frame[i] < '0' || frame[i] > '9')
				return 0.0;
			deg = deg * 10.0 + (frame[i] - '0');
		}

		double minInt = 0.0, frac = 0.0, scale = 1.0;
		bool seenDot = false;
		for (unsigned int i = dotIdx - 2; i < frameSize; ++i) {
			char c = frame[i];
			if (c == '.') {
				seenDot = true;
				continue;
			}
			if (c < '0' || c > '9')
				break;
			if (!seenDot) {
				minInt = minInt * 10.0 + (c - '0');
			} else {
				scale *= 10.0;
				frac += (c - '0') / scale;
			}
		}
		return deg + (minInt + frac) / 60.0;
	}

	int parseInt(const uint8_t* frame, unsigned int frameSize) {
		int value= 0;
		for (unsigned int i = 0; i < frameSize; ++i) {
			if (frame[i] < '0' || frame[i] > '9')
				return 0.0;
			value = value * 10.0 + (frame[i] - '0');
		}
		return value;
	}

	double parseDecimal(const uint8_t* frame, unsigned int frameSize) {
		bool isDecPart= false;
		double intPart= 0.0;
		double decPart= 0.0;
		unsigned int divider= 1;
		for (unsigned int i = 0; i < frameSize; ++i) {
			if (frame[i] == '.') {
				isDecPart= true;
			}
			else if (frame[i] < '0' || frame[i] > '9') {
				break;
			}
			else {
				if (isDecPart) {
					decPart = decPart * 10.0 + (frame[i] - '0');
					divider*=10;
				} else {
					intPart = intPart * 10.0 + (frame[i] - '0');
				}
			}
		}
		return intPart + (decPart / divider);
	}

	void decodeRMC(const uint8_t* frame, unsigned int frameSize) {
		unsigned int status= 0;
		unsigned int i= 0;
		StaticVector<uint8_t, 25> buffer;
		while (i < frameSize) {
			if (frame[i] == ',') {
				switch (status) {
					case 0:
						break;
					case 1:
						if (frame[i-1] == 'V') {
							isValid= false;
							i= frameSize;
						} else {
							isValid= true;
						}
						break;
					case 2:
						if (buffer.size()) {
							location[0]= parseCoordinate(buffer.data(), buffer.size());
						}
						break;
					case 3:  // N/S
						if (buffer.size() && buffer[0] == 'S')
							location[0] *= -1.0;
						break;
					case 4:
						if (buffer.size()) {
							location[1]= parseCoordinate(buffer.data(), buffer.size());
						}
						break;
					case 5:  // E/W
						if (buffer.size() && buffer[0] == 'W')
							location[1] *= -1.0;
						break;
					case 6:
						if (buffer.size()) {
							speed= (float)parseDecimal(buffer.data(), buffer.size()) * 0.514444;
						}
						break;
					case 7:
						if (buffer.size()) {
							bearing= (float)parseDecimal(buffer.data(), buffer.size());
						}
						break;
					default:
						break;
				}
				status++;
				buffer.clear();
			} else {
				buffer.push_back(frame[i]);
			}
			i++;
		}
	}

	void decodeGGA(const uint8_t* frame, unsigned int frameSize) {
		unsigned int status= 0;
		unsigned int i= 0;
		StaticVector<uint8_t, 25> buffer;
		while (i < frameSize) {
			if (frame[i] == ',') {
				switch (status) {
					case 0:
						break;
					case 1:
						if (buffer.size()) {
							location[0]= parseCoordinate(buffer.data(), buffer.size());
						}
						break;
					case 2:  // N/S — flip latitude if needed
						if (buffer.size() && buffer[0] == 'S')
							location[0] *= -1.0;
						break;
					case 3:
						if (buffer.size()) {
							location[1]= parseCoordinate(buffer.data(), buffer.size());
						}
						break;
					case 4:  // E/W — flip longitude if needed
						if (buffer.size() && buffer[0] == 'W')
							location[1] *= -1.0;
						break;
					case 5:
						//float test= (float)parseDecimal(buffer.data(), frameSize);
						if (buffer.size()) {
							signalQuality= parseInt(buffer.data(), buffer.size());
						}
						break;
					case 7:
						if (buffer.size()) {
							hdop= parseDecimal(buffer.data(), buffer.size());
						}
						break;
					default:
						break;
				}
				status++;
				buffer.clear();
			} else {
				buffer.push_back(frame[i]);
			}
			i++;
		}
	}
	~GNSSMeasurements() {

	}

};

#endif /* INC_MESSAGESTRUCTS_GNSSMEASUREMENTS_H_ */
