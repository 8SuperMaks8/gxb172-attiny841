#pragma once

#ifndef MP3431_H_
#define MP3431_H_

#include "BinaryOpr.h"

template <class EN_Pin, class V_ctr_pwm>
class MP3431 {
	public:
		static inline void on(void) {
			EN_Pin::Set();
		}

		static inline void off(void) {
			EN_Pin::Clear();
		}

		static inline void toggle(void) {
			EN_Pin::Toggle();
		}

		static inline bool state(void) {
			return EN_Pin::Port::Read() & bit(EN_Pin::Number);
		}

		static inline void state(const bool st) {
			EN_Pin::Set(st);
		}

		static void voltage_lvl(const unsigned short int val) {
			if (val > 0) {
				on();
			}
			else {
				off();
			}
			V_ctr_pwm::pwm(val);
		}

		static inline const unsigned short int voltage_lvl(void) {
			return V_ctr_pwm::pwm();
		}

		static inline void init(void) {
			V_ctr_pwm::init();
			EN_Pin::SetDirWrite();
		}

		static inline void init(const unsigned short int val) {
			V_ctr_pwm::init(val);
			EN_Pin::SetDirWrite();
		}
	
		inline MP3431(void) {
			init();
		}

		inline MP3431(const unsigned short int val) {
			init(val);
		}
};

#endif