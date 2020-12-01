/*
 * GXB172.cpp
 *
 * Created: 28.05.2019 19:18:53
 * Author : Maks
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "iopins.h"
#include "pinlist.h"

#include "PWM.h"
#include "MP3431.h"
#include "I2C.h"
#include "TMP103.h"
#include "ADC.h"
#include "LED.h"
#include "PID.h"
#include "math.h"
#include "BinaryOpr.h"
#include "MedianFilter.h"
#include "Common.h"
#include "EEPPROM.h"
#include "uart.h"
#include "Smooth.h"
#include "Watchdog.h"
#include "ESW.h"
#include "delay.h"
//#include "uart.h"

#pragma region Global settings
FUSES = {
	.low = FUSE_SUT_CKSEL0 & FUSE_SUT_CKSEL2 & FUSE_SUT_CKSEL3 & FUSE_SUT_CKSEL4, // 0xE2,
	.high = HFUSE_DEFAULT & FUSE_EESAVE & FUSE_BODLEVEL0, // 0xD6,
	.extended = EFUSE_DEFAULT & FUSE_BODACT1 // 0xFB
};

#define TMP103_sensor
//#define USE_JUMPERS
//#define CLICK_EXT_BT
//#define MEM_LAST_MODE
constexpr unsigned char DEFAULT_MODE = 0u;

template<typename T>
constexpr auto RES2COUNTS(T res) {
	return (1 << res) - 1;
}

constexpr auto ADC_REF_V = 1.1;
const unsigned char ADC_RESOLUTION = 10; // 10-bit
const unsigned char PWM_RESOLUTION = 10; // 10-bit
constexpr unsigned short int ADC_COUNTS = RES2COUNTS(ADC_RESOLUTION);
constexpr unsigned short int PWM_COUNTS = RES2COUNTS(PWM_RESOLUTION);
constexpr auto CAP_SHORT = V2ADC(0.61, ADC_REF_V, ADC_COUNTS);	// Short Press  (modify depending on OTC cap)

constexpr auto R4 = 1000u; // 1 kOm;
constexpr auto R2 = 4700u; // 4.7kOm;

constexpr unsigned char BATT_OVERSAMPLING_BITS = 3;

template<typename T>
constexpr auto BATT2ADC(T v) {
	return V2ADC(VOLT_DIVIDER(v, R2, R4), ADC_REF_V, ADC_COUNTS << BATT_OVERSAMPLING_BITS);
}

enum BatteryStates : const unsigned short int {
	BATT_CRIT = BATT2ADC(2.51),
	BATT_SHELF = BATT2ADC(2.6),
	BATT_LOW = BATT2ADC(2.71),
	BATT_FULL = BATT2ADC(4.2),
};

enum TempStates : const signed char {
	TEMP_SHELF = 50,
	TEMP_CRIT = 70,
};

constexpr auto LED_I2PWM_K = (double)5500u / 938u;	// Coff between led current and driver PWM signal;

template<typename T>
constexpr auto LED_I2PWM(T i) {
	return (const unsigned short int) simple_round(i / LED_I2PWM_K);
}

constexpr unsigned short int modes[] {
	1,//LED_I2PWM(25u),
	LED_I2PWM(50u),
	LED_I2PWM(250u),
	LED_I2PWM(1000u),
	LED_I2PWM(5000u)
};

#pragma endregion

namespace pins {
	using namespace Mcucpp::IO;

	using debug = Pa3; // not used
	using uart = Pa5;
	using MP3431_en = Pa0;
	using MP3431_PWM = Pa7;
	using batt = Pa1; // full analog mode ADC1
	using OTC = Pa2; // full analog mode ADC2
	using ESW = Pb2; // PCINT10
	using J2 = Pb1;
	using J1 = Pb0;
	using SCL = Pa4;
	using SDA = Pa6;

	using jumpers = PinList<J1, J2>;
}

unsigned char mode __attribute__((section(".noinit")));

class Brightness_mngr {
	public:
		//unsigned short int modes[];

	protected:
		//static unsigned char mode;

		constexpr unsigned char last_id(void) {
			return array_count(modes) - 1;
			//return 4;
		}

		bool mode_is_proper(unsigned char br) {
			return br >= 0 && br <= last_id();
		}

	public:
		bool init(const unsigned char Default_mode = 0u) {
			if (mode_is_proper(mode)) {
				return true;
			}
			else {
				mode = Default_mode;
			}
			return false;
		}

		inline unsigned short int operator()(void) {
			return modes[mode];
		}

		unsigned short int first(void) {
			mode = 0;
			return modes[mode];
		}

		constexpr inline unsigned short int first_def(void) {
			return modes[0];
		}

		unsigned short int last(void) {
			mode = last_id();
			return modes[mode];
		}

		constexpr inline unsigned short int last_def(void) {
			return modes[last_id()];
		}

		unsigned short int next(void) {
			if (mode >= last_id()) {
				first();
			}
			else {
				mode++;
			}
			return modes[mode];
		}

		unsigned short int prev(void) {
			if (mode == 0) {
				last();
			}
			else {
				mode--;
			}
			return modes[mode];
		}
};

class Brightness_mngr_eep : public Brightness_mngr {
	private:
		struct strg_conv {
			static unsigned char encode(unsigned char val) {
				unsigned char res = 0xff;
				if (val) {
					do {
						val--;
						bitClear(res, val);
					} while (val);
				}
				return res;
			}

			static unsigned char decode(unsigned char val) {
				val = ~val;

				// Count the bits
				unsigned char res = 0;
				while (val) {
					res += val & 0b1;
					val >>= 1;
				}
				return res;
			}
		};

		EEPROM_range_val_buf<unsigned char, unsigned short int, E2START, E2SIZE - 1, E2PAGESIZE> eep_mode_strg;

	public:
		bool init(const unsigned char Default_mode = 0u) {
			if (mode_is_proper(mode)) {
				return true;
			}
			else {
				mode = Default_mode;

				unsigned char br_mode = strg_conv::decode(eep_mode_strg.read());
				if (mode_is_proper(br_mode)) {
					mode = br_mode;
				}
			}
			return false;
		}

		void save(void) {
			eep_mode_strg.update(strg_conv::encode(mode));
		}
};

template<const unsigned char Mux>
class Battery : public ADC_val {
	private:
		MedianFilter<unsigned short int, 32u> pre_filter;

		unsigned short int adc_read(void) override {
			return pre_filter(ADC_val::adc_read());
		}

	public:
		Battery(void) : ADC_val(Mux) {}

		unsigned short int fast_read(void) {
			constexpr unsigned char bits = 1;
			return ADC_val::operator() < unsigned short int, bits > () << (BATT_OVERSAMPLING_BITS - bits);
		}

		unsigned short int operator()(void) {
			return ADC_val::operator() < unsigned short int, BATT_OVERSAMPLING_BITS > ();
		}
};

class TempSensor {
	public:
		virtual signed char read_temp(void) = 0;
		bool operator==(TempSensor& other) const {
			 return this == &other;
		}
};

template<const unsigned char Mux>
class InternalTempSensor : public TempSensor, public ADC_val {
	private:
		MedianFilter<unsigned short int, 32u> filter;

		signed char to_temp(unsigned short int val) {
			return val - 275u;
		}

		unsigned short int adc_read(void) override {
			return filter(ADC_val::adc_read());
		}

	public:
		InternalTempSensor(void) : ADC_val(Mux) {}

		signed char read_temp(void) override {
			constexpr unsigned char avg = 2;
			return to_temp(ADC_val::operator() < unsigned short int, avg > () >> avg);
		}
};

using i2c = I2C<pins::SCL, pins::SDA, 6u>;
using tmp103 = TMP103<i2c, TMP103G>;

class ExternalTempSensor : public TempSensor {
	public:
		signed char read_temp(void) override {
			return tmp103::read_temp();
		}
};

#if defined MEM_LAST_MODE && ! defined CLICK_EXT_BT
	Brightness_mngr_eep br;
#else
	Brightness_mngr br;
#endif // MEM_LAST_MODE

using Led = LED<MP3431<pins::MP3431_en, PWM<pins::MP3431_PWM, PWM_COUNTS>>, 1u, 2u>;
Led led;

#ifdef CLICK_EXT_BT
	class wdt {
		public:
			static void init(void) {
				Watchdog::set_prescaler(watchdog_prescaler::ms500);
			}

			static inline void reset(void) {
				Watchdog::reset();
			}

			static inline void start(void) {
				//Watchdog::en_interrupt();
				WDTCSR = (WDTCSR & ~bit(WDIF)) | bit(WDIE);
				reset();
			}

			static inline void stop(void) {
				bitmask_Clear(WDTCSR, bit(WDIF) | bit(WDIE));
			}
	};

	using Esw = ESW<pins::ESW, wdt, Led, 2>;

	template<>
	void Esw::event_mngr(void) {
		switch (state) {
			case one_click:
				led.toogle();
				new_event = false;
				break;

			case double_click:
				br.last();
				new_event = false;
				break;

			case hold_on:
				if (led.state()) {
					br.next();
				}
				new_event = false;
				break;
		}
	}

	Esw esw;
#endif // CLICK_EXT_BT

namespace fb {
	Battery<bit(MUX0)> batt;
	namespace priv {
		InternalTempSensor<bit(MUX3) | bit(MUX2)> int_temp;
		#ifdef TMP103_sensor
			ExternalTempSensor ext_temp;
		#endif // TMP103_sensor
	}
	
	TempSensor * temp;
}

namespace pid {
	template<typename T>
	double saturation(T val) {
		if (val > br()) {
			return br();
		}
		else if (val < 1) {
			return 1u;
		}
		return val;
	}

	template<typename T>
	double filter_res(T val) {
		return saturation(round(val));
	}

	template<class T, class B>
	constexpr signed long int calcC(T k, B man) {
		return simple_round(k * man);
	}

	namespace coff {
		namespace temp {
			constexpr const unsigned short int man = 1000;

			// Calculate PID original coff
			constexpr auto K = 60;
			constexpr auto Kp_base = K;
			constexpr auto Ki_base = 0.008 /*1.3*//*((double)br.last_def() - Kp_base) / I_cache_max*/;
			constexpr auto Kd_base = K >> 1;

			constexpr auto I_cache_min = 0/*-(signed short int)simple_round(Kp_base)*/;
			constexpr const unsigned long int I_cache_max = ((double)br.last_def() - Kp_base) / Ki_base;

			typedef const signed long int KT;

			// Convert coff to acceptable by PID class
			constexpr KT Kp = calcC(Kp_base, man);
			constexpr KT Ki = calcC(Ki_base, man);
			constexpr KT Kd = calcC(Kd_base, man);
		}

		namespace batt {
			constexpr const unsigned short int man = SHRT_MAX;
			//constexpr const short int I_cache_max = SHRT_MAX;

			// Calculate PID original coff
			constexpr auto K = 6;
			constexpr auto Kp_base = (double)K / (1 << BATT_OVERSAMPLING_BITS);
			constexpr auto Ki_base = 0.02/*0.03*//*((double)br.last_def() - Kp_base) / I_cache_max*/;
			constexpr auto Kd_base = (double)K / (1 << BATT_OVERSAMPLING_BITS + 1);

			constexpr auto I_cache_min = 0/*-(signed short int)simple_round(Kp_base)*/;
			constexpr const unsigned short int I_cache_max = ((double)br.last_def() - Kp_base) / Ki_base;

			typedef const signed short int KT;

			// Convert coff to acceptable by PID class
			constexpr KT Kp = calcC(Kp_base, man);
			constexpr KT Ki = calcC(Ki_base, man);
			constexpr KT Kd = calcC(Kd_base, man);
		}
	}

	PID<signed char, double, unsigned long int, coff::temp::KT, 
		coff::temp::Kp,
		coff::temp::Ki,
		coff::temp::Kd,
		coff::temp::man,
		coff::temp::I_cache_min,
		coff::temp::I_cache_max> temp;

	PID<signed short int, double, unsigned short int, coff::batt::KT,
		coff::batt::Kp,
		coff::batt::Ki,
		coff::batt::Kd,
		coff::batt::man,
		coff::batt::I_cache_min,
		coff::batt::I_cache_max> batt;
}

void inline power_down(void) {
	led.brightness(0);
	bitClear(ADCSRA, ADEN);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	DIS_AND_RESTORE_PRR{
		sleep_mode();
	}
}

void sleep(void) {
	sei();
	power_down();
	bitSet(ADCSRA, ADEN);
}

void __attribute__((used, naked, section(".fini0")))
terminate(void) {
	cli();
	power_down();
}

void short_Butt_clk_handler(void) {
	br.next();
	#ifdef MEM_LAST_MODE
	br.save();
	#endif // MEM_LAST_MODE
}

void jumpers_handler(unsigned char conf) {
	// True table is inverted
	enum jumpers_states : const unsigned char {
		//AllLow = 0b11,
		J1 = 0b01,
		J2 = 0b10,
		J1J2 = 0x00,
	};

	switch (conf) {
		case J1:
			break;

		case J2:
			break;

		case J1J2:
			break;
	}
}

void __attribute__((used, naked, section(".init1")))
pre_init(void) {
	MCUSR = 0x00;

	#pragma region Pull-up unused pins
	bitmask_Set(PUEA, 0x28);
	#ifdef CLICK_EXT_BT
		bitmask_Set(PUEB, 0xF);
	#else
		bitmask_Set(PUEB, 0xB);
	#endif
	
	#pragma endregion

	#pragma region Disable unused functions
	bitmask_Set(PRR, bit(PRTWI) | bit(PRUSART1) | bit(PRUSART0) | bit(PRSPI) | bit(PRTIM1) | bit(PRTIM0));
	bitSet(ACSR0A, ACD0);
	bitSet(ACSR1A, ACD1);
	#pragma endregion

	#pragma region ADC
	// Disable input mode for pins with ADC
	bitSet(DIDR0, ADC1D);
	ADMUXB = bit(REFS0); // ADC Ref 1.1V
	ADCSRA = bit(ADEN) | bit(ADPS2) | bit(ADPS1)/* | bit(ADPS0)*/; // enable ADC + prescaler 128
	#pragma endregion

	#ifdef CLICK_EXT_BT
	bitSet(PCMSK1, PCINT10);
	bitSet(GIMSK, PCIE1);
	#endif

	//set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void __attribute__((used, naked, section(".init8")))
init(void) {
	#pragma region Jumpers
	#ifdef USE_JUMPERS
		// All pins on input by default and pullups is set before
		//pins::jumpers::SetConfiguration<pins::jumpers::In>();
		//pins::jumpers::SetPullUp(pins::jumpers::PullUp);
		jumpers_handler(pins::jumpers::PinRead());
		// Disable pullup for connected jumper
		pins::jumpers::SetPullUp(pins::jumpers::NoPullUp);
	#endif
	#pragma endregion

	#pragma region Power button
	#ifndef CLICK_EXT_BT
	if (!br.init(DEFAULT_MODE)) {
		//pins::OTC::SetDirRead();
		if (pins::OTC::IsSet()) {
			short_Butt_clk_handler();
		}
		else {
			bitSet(DIDR0, ADC2D);
			if (ADC_main::read(bit(MUX1)) > CAP_SHORT) {
				short_Butt_clk_handler();
			}
		}
	}
	else {
		short_Butt_clk_handler();
	}
	pins::OTC::SetDirWrite();
	pins::OTC::Set();
	#else
	br.init(DEFAULT_MODE);
	#endif // !CLICK_EXT_SWH
	#pragma endregion

	#pragma region Temp sensor
	#ifdef TMP103_sensor
		if (i2c::init()) {
			fb::temp = &fb::priv::ext_temp;
		}
		else {
			// Fallback to internal MCU thermal sensor
			fb::temp = &fb::priv::int_temp;
		}
	#else
		fb::temp = &fb::priv::int_temp;
	#endif // TMP103_sensor
	#pragma endregion

	//ADMUXA = bit(MUX0); // ADC1 Pa1 batt

	//ADC_async_handler::start_free_run();
	//bitSet(ADCSRA, ADIE);

	#ifdef CLICK_EXT_BT
	// Interrupt enable
	sei();
	#endif // CLICK_EXT_BT
}

//Soft_uart_tx<pins::uart> uart;

int main() {
	#ifndef CLICK_EXT_BT
		// Terminate start if flashlight in critical state
		if (fb::batt.fast_read() < BATT_LOW ||
			fb::temp->read_temp() > TEMP_SHELF) {
			exit(EXIT_SUCCESS);
		}
	#endif
	Smooth_dyn<double> smooth(br.first_def());
	constexpr auto smooth_steps_limit = 256;
	signed short int batt = 0;
	signed char tmp = SCHAR_MIN;

	double batt_pid_res = 0, tmp_pid_res = br();
	constexpr unsigned short int sample_time = 5112 >> 3;
	const double tmp_sample_time = (fb::temp == &fb::priv::int_temp ? (19243 >> 3) / sample_time : (3233 >> 3) / sample_time);
	double batt_time_sample, tmp_time_sample = 0;
	bool stage = true;
	//uart.init(bit(CS01), /*F_CPU / 8 / 9600*/104);
	for (;;) {
		#ifdef CLICK_EXT_BT
		if (led.state()) {
			switch (0) {
				default:
		#endif

					#pragma region MainControl
					// For better response pid control is sepparated in two steps
					if (stage) {
						auto Dctrl = fabs(smooth() - batt_pid_res);
						if (Dctrl < (1 << BATT_OVERSAMPLING_BITS + 2)) {
							// More precisely and most of the long operation
							batt = fb::batt();
							batt_time_sample = (70409u >> 3) / sample_time;
						}
						else {
							// Significant fastest operation
							batt = fb::batt.fast_read();
							batt_time_sample = (5112u >> 3) / sample_time;
						}

						// Overdischarge protection
						if (batt < BATT_CRIT && Dctrl < 3) {
							#ifndef CLICK_EXT_BT
								exit(EXIT_SUCCESS);
							#endif
							break;
						}

						// Err correction for dynamic sample time

						signed short err = batt - BATT_SHELF;
						batt_pid_res = pid::filter_res(pid::batt(err, err * (batt_time_sample + tmp_time_sample), err));
					}
					else {
						tmp = fb::temp->read_temp();

						// Overheat protection
						if (tmp > TEMP_CRIT) {
							#ifndef CLICK_EXT_BT
								exit(EXIT_SUCCESS);
							#endif
							break;
						}

						// Err correction for dynamic sample time
						tmp_time_sample = tmp_sample_time;

						signed char err = TEMP_SHELF - tmp;
						tmp_pid_res = pid::filter_res(pid::temp(err, err * (batt_time_sample + tmp_time_sample), err));
					}

					// Apply action
					led.brightness(smooth(br() > smooth_steps_limit ? smooth_steps_limit : br(), // limit smooth delay
								  (double)min(batt_pid_res,
											  tmp_pid_res)));
					/*uart(static_cast<unsigned short int>(0x5555 - static_cast<unsigned short int>(stage)));
					uart(static_cast<unsigned short int>(TEMP_SHELF - tmp));
					uart(static_cast<unsigned short int>(pid::temp.I_cache));*/
					/*uart(static_cast<unsigned short int>(0x5555 - static_cast<unsigned short int>(stage)));
					uart(static_cast<unsigned short int>(batt - BATT_SHELF));
					uart(static_cast<unsigned short int>(pid::batt.I_cache));*/
					stage = !stage;
					#pragma endregion


		#ifdef CLICK_EXT_BT
			}
		}
		else {
			sleep();
		}
		#endif
	}
}

//ISRf(TIMER0_COMPA_vect) {
//	uart.timer_callback();
//}

ISR(ADC_vect) {}

#ifdef CLICK_EXT_BT
ISRf(WDT_vect) {
	esw.watchdog_int_callback();
}

ISRf(PCINT1_vect) {
	esw.pin_change_int_callback();
}
#endif // CLICK_EXT_BT

extern "C" void __cxa_pure_virtual(void);

void __cxa_pure_virtual(void) {};