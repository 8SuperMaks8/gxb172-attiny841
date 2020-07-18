#pragma once

#ifndef LED_H_
#define LED_H_

//#include <util/delay.h>
#include "delay.h"
#include "Vector.h"
#include "Callback.h"
#include "MP3431.h"
#include "stdlib.h"

#ifdef WDT_vect
	ISRv(WDT_vect)
#endif
/*
template <class Driver, const unsigned char SmoothStep = 5>
class LED: public callback {
	protected:
		#ifdef WDT_vect
				friend ISRf(WDT_vect);
		#endif

		enum LED_states : const unsigned char {
			normal_mode,
			flash_mode
		};

		bool interrupt_flg;

		unsigned int target_brightness;
		unsigned char flash_counter;

		// Ramp param
		signed int ramp_step;
		unsigned int ramp_target;

		// Calculate new step
		void ramp_calc_step(void) {
			ramp_step = round((target_brightness - brightness()) / SmoothStep);
			ramp_target = target_brightness;
		}

		void ramp_stepping(void) {
			unsigned short int br = brightness();
			if (ramp_target - br > ramp_step) {
				force_brightness(br + ramp_step);
			}
			else {
				force_brightness(ramp_target);
			}
		}

		void operator()(void) override {
			if (flash_counter) {
				Driver::toggle();
				flash_counter--;
			}
			else {
				interrupt_flg = true;
				//ramp_callback();
			}
		}

	public:
		LED(void): target_brightness(0), flash_counter(0), ramp_step(0), ramp_target(0), interrupt_flg(false) {
			Driver::init();
		}

		static void force_brightness(const unsigned short int br) {
			Driver::voltage_lvl(br);
		}

		LED(const unsigned short int br): target_brightness(0), flash_counter(0), ramp_step(0), ramp_target(0), interrupt_flg(false) {
			Driver::init(br);
		}

		void ramp_callback(void) {
			if (interrupt_flg) {
				if (ramp_target == target_brightness) {
					ramp_stepping();
				}
				else {
					ramp_calc_step();
					ramp_stepping();
				}
				interrupt_flg = false;
			}
		}

		static const unsigned short int brightness(void) {
			return Driver::voltage_lvl();
		}
		
		void brightness(const unsigned short int br) {
			target_brightness = br;
			ramp_calc_step();
		}

		void flash(const unsigned char count) {
			flash_counter = count * 2;
			if (Driver::state()) {
				flash_counter += 2;
			}
		}

		void flash(const unsigned char count, const bool end_state) {
			flash_counter = count * 2;
			const bool state = Driver::state();
			if (state) {
				if (end_state) {
					flash_counter += 2;
				}
				else {
					flash_counter++;
				}
			}
			else {
				if (!end_state) {
					flash_counter++;
				}
			}
		}
};*/

template <class Driver, const unsigned char SmoothStep = 1u, const unsigned char SmoothDelay = 2u>
class LED {
	protected:
		#ifdef WDT_vect
			friend ISRf(WDT_vect);
		#endif

		//unsigned char flash_counter;

		//void operator()(void) override {
		//	if (flash_counter) {
		//		Driver::toggle();
		//		flash_counter--;
		//	}
		//	else {
		//		//interrupt_flg = true;
		//		//ramp_callback();
		//	}
		//}

	public:
		LED(void) /*: flash_counter(0)*/ {
			Driver::init();
		}

		LED(const unsigned short int br) /*: flash_counter(0)*/ {
			Driver::init(br);
		}

		static inline const unsigned short int brightness(void) {
			return Driver::voltage_lvl();
		}

		static void brightness(const unsigned short int br) {
			Driver::voltage_lvl(br);
		}

		static void Dbrightness(const signed short int diff) {
			brightness(brightness() + diff);
		}

		static void brightness_ramp(const unsigned short int br) {
			signed char k = SmoothStep;
			if (brightness() > br) {
				k = -SmoothStep;
			}
			while (abs(brightness() - br) >= SmoothStep) {
				Dbrightness(k);
				Mcucpp::delay_ms<SmoothDelay, F_CPU>();
			}
		}

		static inline void toogle(void) {
			Driver::toggle();
		}

		static inline void on(void) {
			Driver::on();
		}

		static inline void off(void) {
			Driver::off();
		}

		static inline const bool state(void) {
			return Driver::state();
		}

		static inline void state(const bool st) {
			Driver::state(st);
		}

		/*void flash(const unsigned char count) {
			flash_counter = count * 2;
			if (Driver::state()) {
				flash_counter += 2;
			}
		}

		void flash(const unsigned char count, const bool end_state) {
			flash_counter = count * 2;
			const bool state = Driver::state();
			if (state) {
				if (end_state) {
					flash_counter += 2;
				}
				else {
					flash_counter++;
				}
			}
			else {
				if (!end_state) {
					flash_counter++;
				}
			}
		}*/
};

#endif /* LED_H_ */