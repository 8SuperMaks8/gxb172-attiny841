/*
 * ADC_avg.h
 *
 * Created: 29.05.2019 12:20:29
 *  Author: Maks
 */ 

#pragma once

#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <avr/sleep.h>
#include "BinaryOpr.h"
#include "Common.h"

class ADC_main {
	public:
		enum ADC_ref : const unsigned char {
			Vcc = 0x00,
			int_1v1 = bit(REFS0),
			int_2v2 = bit(REFS1),
			int_4v096 = bit(REFS1) | bit(REFS0),
			Aref = bit(REFS2),
			int_1v1_ext_bypass = bit(REFS2) | bit(REFS0),
			int_2v2_ext_bypass = bit(REFS2) | bit(REFS1),
			int_4v096_ext_bypass = bit(REFS2) | bit(REFS1) | bit(REFS0),
		};

		static inline unsigned short int mux(void) {
			return ADMUXA;
		}

		static inline void mux(const unsigned char conf) {
			ADMUXA = conf;
		}

		static inline void ref_voltage(const ADC_ref ref) {
			// static_cast<char>(ref);
			bitmask_Set(ADMUXB, ref);
		}

		static inline ADC_ref ref_voltage(void) {
			return static_cast<ADC_ref>(ADMUXB);
		}

		static inline unsigned short int result(void) {
			return ADC;
		}

		static inline void en_interrupt(void) {
			bitSet(ADCSRA, ADIE);
		}

		static inline void dis_interrupt(void) {
			bitClear(ADCSRA, ADIE);
		}

		static inline void start_conversion(void) {
			bitSet(ADCSRA, ADSC);
		}

		static inline void stop_conversion(void) {
			bitClear(ADCSRA, ADSC);
		}

		static void wait_finish(void) {
			while (bitRead(ADCSRA, ADSC));
		}

		static void wait_finish_sleep(void) {
			//set_sleep_mode(SLEEP_MODE_ADC);
			//en_interrupt();
			EN_AND_RESTORE_INT{
				sleep_enable();
				do {
					sleep_cpu();
				} while (bitRead(ADCSRA, ADSC));
				sleep_disable();
			}
		}

		static unsigned short int read(void) {
			//start_conversion();
			wait_finish();
			return result();
		}

		static unsigned short int read(unsigned char MUX) {
			mux(MUX);
			return read();
		}


		static inline void prepare_sleep(void) {
			set_sleep_mode(SLEEP_MODE_IDLE);
			en_interrupt();
		}

		static unsigned short int read_sleep(void) {
			//start_conversion();
			wait_finish_sleep();
			return result();
		}

		static unsigned short int read_sleep(unsigned char MUX) {
			mux(MUX);
			return read_sleep();
		}
};

class ADC_val {
	protected:
		const unsigned char mux;

		virtual unsigned short int adc_read(void) {
			return ADC_main::read_sleep();
		}

	public:
		ADC_val(const unsigned char Mux) : mux(Mux) {}

		template<typename buffT, const unsigned char bits>
		unsigned short int operator()(void) {
			ADC_main::prepare_sleep();
			ADC_main::mux(mux);
			constexpr auto smpl = 2 * bits;
			buffT avg_buf = 0;
			for (unsigned int i = 0; i < (1 << smpl); i++) {
				avg_buf += adc_read();
			}
			ADC_main::dis_interrupt();
			return avg_buf >> bits;
		}

};


#endif /* ADC_H_ */