#pragma once

#ifndef Watchdog_H_h
#define Watchdog_H_h

#include <avr/io.h>
#include "BinaryOpr.h"
#include "atomic.h"

enum watchdog_prescaler : const unsigned char {
	ms16 = 0x0,
	ms32 = bit(WDP0),
	ms64 = bit(WDP1),
	ms125 = bit(WDP1) | bit(WDP0),
	ms250 = bit(WDP2),
	ms500 = bit(WDP2) | bit(WDP0),
	s1 = bit(WDP2) | bit(WDP1),
	s2 = bit(WDP2) | bit(WDP1) | bit(WDP0),
	s4 = bit(WDP3),
	s8 = bit(WDP3) | bit(WDP0)
};

class Watchdog {
	public:
		static inline void reset() {
			asm("wdr");
		}

	private:
		static void inline prep_protect_upd(void) {
			reset();
			bitClear(MCUCR, WDRF);
			if (bitRead(CCP, bit(0))) {
				bitmask_Set(CCP, 0xD8);
			}
		}

	public:
		static void set_conf(const unsigned char conf) {
			ATOMIC{
				prep_protect_upd();
				bitmask_Set(WDTCSR, conf);
			}
		}

		static void clear_conf(const unsigned char conf) {
			ATOMIC{
				prep_protect_upd();
				bitmask_Clear(WDTCSR, conf);
			}
		}

		static void init(void) {

		}

		static void enable(void) {
			set_conf(bit(WDE));
		}

		static void set_prescaler(watchdog_prescaler conf) {
			set_conf(conf);
		}

		static void disable(void) {
			clear_conf(bit(WDE));
		}

		static inline void en_interrupt(void) {
			bitSet(WDTCSR, WDIE);
		}

		static inline void dis_interrupt(void) {
			bitClear(WDTCSR, WDIE);
		}

		static inline void clr_int_flag(void) {
			bitClear(WDTCSR, WDIF);
		}
};

#endif