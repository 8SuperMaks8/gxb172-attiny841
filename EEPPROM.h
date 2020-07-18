/*
 * EEPPROM_range.h
 *
 * Created: 28.05.2019 22:55:24
 *  Author: Maks
 */ 

#pragma once

#ifndef EEPPROM_H_
#define EEPPROM_H_

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "BinaryOpr.h"
#include "atomic.h"

template<typename addr_type = const unsigned short int>
struct EEPROM {
	private:
		enum prog_mode : const unsigned char {
			atomic_mode = 0x00,
			erase_mode = bit(EEPM0),
			write_mode = bit(EEPM1)
		};

		static void program(prog_mode mode, addr_type addr, unsigned char val) {
			/* Wait for completion of previous write */
			wait();
			ATOMIC{
				/* Set Programming mode */
				EECR = mode;
				//EECR = (1 << EEPM1) | (0 << EEPM0);
				/* Set up address and data registers */
				if (E2END > 0xff) {
					EEAR = addr;
				}
				else {
					EEARL = addr;
				}
				EEDR = val;
				/* Write logical one to EEMPE */
				//EECR |= (1 << EEMPE);
				/* Start eeprom write by setting EEPE */
				//EECR |= (1 << EEPE);
				__asm__ __volatile__(
					"sbi    %[__eecr], %[__eemwe]   \n\t"
					"sbi    %[__eecr], %[__eewe]    \n\t"
					:
				: [__eecr] "i" (_SFR_IO_ADDR(EECR)),
				  [__eemwe] "i" (EEMPE),
				  [__eewe]  "i" (EEPE):);
			}
		}

	public:
		static bool is_busy(void) {
			return bitRead(EECR, EEPE);
		}

		static void wait(void) {
			while (is_busy());
		}

		static void erase_write(addr_type addr, unsigned char val) {
			program(atomic_mode, addr, val);
		}

		static void erase(addr_type addr) {
			program(erase_mode, addr, 0x00);
		}

		static void write(addr_type addr, unsigned char val) {
			program(write_mode, addr, val);
		}

		static const unsigned char read(addr_type addr) {
			/* Wait for completion of previous write */
			wait();
			/* Set up address register */
			if (E2END > 0xff) {
				EEAR = addr;
			}
			else {
				EEARL = addr;
			}
			/* Start eeprom read by writing EERE */
			EECR |= (1 << EERE);
			/* Return data from data register */
			return EEDR;
		}

		static void update(addr_type addr, unsigned char val) {
			const unsigned char curr = read(addr);
			if (curr != val) {
				if (~curr & val) {
					if (0xff & ~val) {
						erase_write(addr, val);
					}
					else {
						erase(addr);
					}
				}
				else {
					write(addr, val);
				}
			}
		}

		static bool is_empty(addr_type addr) {
			return read(addr) == 0xff;
		}
};

template <typename type, typename addr_type, addr_type addr_, const bool atomic = false>
struct EEPROM_cell: private EEPROM<type> {
	static void wait(void) {
		while (EECR & (1 << EEPE));
	}

	static type read(void) override {
		return EEPROM<type>::read(addr_);
	};

	static void write(type val) override {
		EEPROM<type>::write(addr_, val);
	}

	static void update(type val) override {
		EEPROM<type>::update(addr_, val);
	}
};

template<typename type, typename addr_type, addr_type start, addr_type end, const unsigned char step = 1>
class EEPROM_range_val {
	private:
		typedef EEPROM<addr_type> eeprom;

		//type saved_val;
		addr_type eepos;

	public:
		EEPROM_range_val(type val = 0) : eepos(start) {};

		bool is_same(type val) {
			return eeprom::read(eepos) == val;
		}

		type read(void) {
			unsigned char res = 0xff;
			for (addr_type p = start; p < end; p++) {
				res = eeprom::read(p);
				if (res != 0xff) {
					eepos = p;
					//saved_val = eep;
					return res;
					//break;
				}
			}
			eepos = start;
			return res;
		}

		void update(type val) {
			if (!is_same(val)) {
			
				#pragma region next addr
				addr_type oldpos = eepos;
				eepos += step;

				if (eepos > end) {
					unsigned char diff = eepos - end;
					if (diff >= step) {
						if (oldpos == end) {
							diff = 0;
						}
						else {
							diff = 1;
						}
					}
					eepos = start + diff;
				}
				#pragma endregion

				eeprom::update(eepos, val);
				eeprom::update(oldpos, 0xff);
			}
		};
};

template<typename type, typename addr_type, addr_type start, addr_type end, const unsigned char step = 1>
class EEPROM_range_val_buf : public EEPROM_range_val<type, addr_type, start, end, step> {
	private:
		type saved_val;

	public:
		bool is_same(type val) {
			return saved_val == val;
		}

		type read(void) {
			saved_val = EEPROM_range_val<type, addr_type, start, end, step>::read();
			return saved_val;
		}

		void update(type val) {
			EEPROM_range_val<type, addr_type, start, end, step>::update(val);
			saved_val = val;
		}
};

#endif /* EEPPROM_H_ */