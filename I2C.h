/*
 * Author : Maks
 */

#pragma once

#ifndef I2C_H_
#define I2C_H_

#include <pinlist.h>
#include "atomic.h"
#include "delay.h"

// Constants for reading & writing
constexpr unsigned char I2C_READ = 1u;
constexpr unsigned char I2C_WRITE = 0u;

template <class SCL, class SDA, const unsigned char SCL_low_delay = 4u, const bool PullUp = false, const unsigned short int wait = 5000u>
class I2C {
	private:
		template <class Pin, const bool PullU = false, const bool atomic = false>
		class I2Cpin : public Pin {
			private:
				static inline void _set(bool val) {
					if (val) {
						if (PullU) {
							Pin::SetPullUp(Pin::Port::PullUp);
						}
						Pin::SetConfiguration(Pin::Port::In);
					}
					else {
						if (PullU) {
							Pin::Clear();
						}
						Pin::SetConfiguration(Pin::Port::Out);
					}
				}

			public:
				static inline void set(bool val) {
					if (atomic) {
						ATOMIC{
							_set(val);
						}
					}
					else {
						_set(val);
					}
				}

			static inline void setH(void) {
				set(true);
			}

			static inline void setL(void) {
				set(false);
			}
		};

		typedef I2Cpin<SCL, PullUp> scl;
		typedef I2Cpin<SDA, PullUp> sda;

		typedef Mcucpp::IO::PinList<SCL, SDA> I2CPort;

		static void delay(void) {
			Mcucpp::delay_us<SCL_low_delay, F_CPU>();
			//_delay_us(SCL_low_delay);
		}

		static void half_delay(void) {
			Mcucpp::delay_us<SCL_low_delay / 2, F_CPU>();
			//_delay_us(SCL_low_delay / 2);
		}

		static void qdr_delay(void) {
			Mcucpp::delay_ns<(SCL_low_delay * 1000) / 4, F_CPU>();
			//_delay_us(SCL_low_delay / 2);
		}

		static inline void port_prepare(void) {
			/*scl::SetConfiguration(scl::Port::Out);
			sda::SetConfiguration(sda::Port::Out);
			sda::Clear();
			scl::Clear();*/
			I2CPort::template SetConfiguration<I2CPort::In>();
			I2CPort::template Clear<0xff>();
		}

		static void start(void) {
			sda::setL();
			half_delay();
			scl::setL();
			half_delay();
			//delay();
		}

		static void ack(const bool state) {
			if (state) {
				sda::setL();
			}
			else {
				sda::setH();
			}
			half_delay();
			scl::setH();
			half_delay();
			scl::setL();
			qdr_delay();
			sda::setL();
			half_delay();
		}

		static bool ack(void) {
			sda::setH();
			half_delay();
			scl::setH();
			qdr_delay();
			const bool res = sda::IsSet();
			qdr_delay();
			scl::setL();
			half_delay();
			//sda::setL();
			//half_delay();
			return res;
		}

	public:
		I2C(void) {
			port_prepare();
		}

		static bool is_idle(void) {
			return sda::IsSet() && scl::IsSet();
		}

		static bool init(void) {
			port_prepare();
			half_delay();
			/*sda::setH();
			scl::setH();*/
			return is_idle();
		}

		static bool start(const unsigned char addr) {
			start();
			return write(addr);
		}

		static bool start_wait(const unsigned char addr) {
			for (unsigned short int retry = wait; retry > 0; retry--) {
				if (start(addr)) {
					return true;
				}
			}
			return false;
		}

		static bool start_rep(const unsigned char addr) {
			sda::setH();
			half_delay();
			scl::setH();
			half_delay();
			return start(addr);
		}

		static void stop(void) {
			sda::setL();
			half_delay();
			scl::setH();
			half_delay();
			sda::setH();
			delay();
		}

		static bool write(const unsigned char data) {
			for (unsigned char i = 0x80; i != 0; i >>= 1) {
				sda::set(i & data);
				half_delay();
				scl::setH();
				half_delay();
				scl::setL();
				half_delay();
			}
			//delay();
			//sda::setH();
			return !ack();
		}

		static unsigned char read(const bool ask = true) {
			sda::setH();
			unsigned char data = 0;
			for (unsigned char i = 0x80; i != 0; i >>= 1) {
				half_delay();
				scl::setH();
				qdr_delay();
				data <<= 1;
				if (sda::IsSet()) {
					data |= 0x1;
				}
				qdr_delay();
				scl::setL();
				half_delay();
			}
			ack(ask);
			return data;
		}
};

#endif