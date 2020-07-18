/*
 * TMP103.h
 *
 * Created: 29.05.2019 10:42:29
 *  Author: Maks
 */

#pragma once


#ifndef TMP103_H_
#define TMP103_H_

#include "I2C.h"

enum TMP103_dev : const unsigned char {
	TMP103A = 0b1110000,
	TMP103B = 0b1110001,
	TMP103C = 0b1110010,
	TMP103D = 0b1110011,
	TMP103E = 0b1110100,
	TMP103F = 0b1110101,
	TMP103G = 0b1110110,
	TMP103H = 0b1110111
};

template<class I2C, const TMP103_dev addr>
class TMP103 {
	public:
		enum reg : const unsigned char {
			temp = 0x00,
			conf = 0b1,
			Tl = 0b10,
			Th = 0b11
		};

		static const unsigned char read_reg(const reg reg) {
			I2C::start_wait((addr << 1) | I2C_WRITE);
			I2C::write(reg);
			I2C::stop();

			I2C::start((addr << 1) | I2C_READ);
			const unsigned char returned_data = I2C::read(true);
			I2C::stop();
			return returned_data;
		};

		static void write_reg(const reg reg) {};
	
		static signed char read_temp(void) {
			return read_reg(reg::temp);
		};
};

#endif /* TMP103_H_ */