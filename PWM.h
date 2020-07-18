#pragma once

#ifndef PWM_H_
#define PWM_H_

#include <avr/io.h>
#include "BinaryOpr.h"

template <class Pin, const unsigned short int max = 0x3FF>
class PWM {
	public:
		static inline void on(void) {
			bitSet(TOCPMCOE, TOCC6OE);
		}

		static inline void off(void) {
			bitClear(TOCPMCOE, TOCC6OE);
		}

		static void pwm(const unsigned short int val) {
			if (val > max) {
				OCR2B = max;
				Pin::Set();
			}
			else {
				OCR2B = val;
				if (val == 0) {
					off();
				}
				else {
					on();
				}
				Pin::Clear();
			}
		}

		static inline unsigned short int pwm(void) {
			return OCR2B;
		}

		static inline void init(void) {
			TCCR2A = (TCCR2A & ~bit(COM2B0)) | bit(COM2B1) | bit(WGM21) | bit(WGM20);
			TCCR2B = (TCCR2B & ~(bit(WGM23) | bit(CS22) | bit(CS21))) | bit(WGM22) | bit(CS20);

			bitSet(TOCPMSA1, TOCC6S1);
			//bitSet(TOCPMCOE, TOCC6OE);
			//Pin::SetDirWrite(Pin::Port::Out);;
			Pin::SetDirWrite();
		}

		static inline void init(const unsigned short int val) {
			init();
			pwm(val);
		}

		inline PWM(void) {
			init();
		}

		inline PWM(const unsigned short int val) {
			init(val);
		}
};

#endif /* PWM_H_ */