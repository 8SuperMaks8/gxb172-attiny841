#pragma once

#ifndef Smooth_H_h
#define Smooth_H_h

template<typename T>
class Smooth_dyn {
	private:
		T val;

	public:
		Smooth_dyn(void) : val(0) {};
		Smooth_dyn(T init_val) : val(init_val) {};

		template<class A>
		T operator()(unsigned short int smpl, A v) {
			return val = ((A)val * (smpl - 1) + v) / smpl;
		}

		T operator()(void) {
			return val;
		}
};

template<typename T, unsigned short int samples>
class Smooth: public Smooth_dyn<T> {
	private:
		typedef Smooth_dyn<T> base;
	public:
		Smooth(void): base() {};
		Smooth(T init_val): base(init_val) {};

		template<class A>
		T operator()(A v) {
			return base::val = ((A)base::val * (samples - 1) + v) / base::samples;
		}

		T operator()(void) {
			return base::val;
		}
};

#endif