#pragma once

#ifndef ESW_H_h
#define ESW_H_h

#include "Vector.h"

#ifdef WDT_vect
ISRv(WDT_vect)
#endif

#ifdef PCINT1_vect
ISRv(PCINT1_vect)
#endif

template<class Pin, class t, class Led, const unsigned char time_for_click>
class ESW {
	public:
		enum ESW_state : const unsigned char {
			stand_by = 0,
			one_click = 1,
			double_click = 2,
			hold_on = 3,
			locked
		};

	private:
		#ifdef WDT_vect
			friend ISRf(WDT_vect);
		#endif
		#ifdef PCINT1_vect
			friend ISRf(PCINT1_vect);
		#endif
		//unsigned char clk_counts;
		unsigned char timer;
		ESW_state state;
		bool new_event;

		void set_timer(const unsigned char time) {
			timer = time;
			t::reset();
		}

		void start_timer(const unsigned char time) {
			set_timer(time);
			t::start();
		}

		void upd_state(ESW_state st) {
			state = st;
			new_event = true;
			event_mngr();
		}

	public:
		ESW(void) {
			upd_state(stand_by);
			//Pin::SetDirRead();
			Pin::SetPullUp(Pin::Port::PullUp);
			t::init();
		};

		ESW_state operator()(void) {
			new_event = false;
			return state;
		}

		void lock(void) {
			upd_state(locked);
		}

		void unlock(void) {
			upd_state(stand_by);
		}

		bool is_new(void) {
			return new_event;
		}

	private:

		void event_mngr(void);

		inline void pin_change_int_callback(void) {
			if (!Pin::IsSet()) {
				start_timer(time_for_click);
			}
			else {
				// If short click
				if (timer > 0) {
					if (state < double_click) {
						upd_state(static_cast<ESW_state>(static_cast<unsigned char>(state) + 1));
					}
				} // Or long
				else if (state != locked) {
					upd_state(stand_by);
					t::stop();
				}
			}
		}

		inline void watchdog_int_callback(void) {
			if (timer > 0) {
				timer--;
			}
			else {
				if (state < hold_on) {
					if (!Pin::IsSet()) {
						upd_state(hold_on);
						if (Led::state()) {
							set_timer(time_for_click);
						}
						else {
							set_timer(time_for_click * 4);
						}
					}
					else {
						upd_state(stand_by);
						t::stop();
					}
				}
				else if (state == hold_on) {
					if (Led::state()) {
						if (!Pin::IsSet()) {
							upd_state(hold_on);
							set_timer(time_for_click);
						}
						else {
							upd_state(stand_by);
							t::stop();
						}
					}
					else {
						lock();
						t::stop();
					}
				}
				else {
					upd_state(stand_by);
					t::stop();
				}
			}
		}
};

#endif