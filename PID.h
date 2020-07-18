#pragma once

#ifndef PID_h
#define PID_h

#include "math.h"
#include "Common.h"

template<typename S, typename K, typename Err>
S Kcalc(S s, K k, Err e) {
	return k * e + s;
}

template <typename Err>
double Kcalc(double s, double k, Err e) {
	return fma(k, e, s);
}

template<typename InpType, typename CalcType, typename I_cacheType, typename K_type, K_type Kp, K_type Ki, K_type Kd, auto sc, I_cacheType Imin_limit = 0, I_cacheType Imax_limit = 0>
class PID {
public:
		I_cacheType I_cache;
		InpType D_cache;

		static constexpr CalcType scale(K_type k) {
			return (CalcType)k / sc;
		}

		static CalcType kp(CalcType out, InpType err) {
			if (Kp) {
				return Kcalc(out, scale(Kp), err);
			}
			return out;
		}

		CalcType ki(CalcType out, InpType err) {
			if (Ki) {
				return Kcalc(
					out, 
					scale(Ki),
					Imin_limit == 0 && Imax_limit == 0 ? 
						overflow_limit_sum(I_cache, err) :
						overflow_limit_sum(I_cache, err, Imin_limit, Imax_limit)
				);
			}
			return out;
		}


		CalcType kd(CalcType out, InpType fb) {
			if (Kd) {
				CalcType res = Kcalc(out, -scale(Kd), fb - D_cache);
				D_cache = fb;
				return res;
			}
			return out;
		}

	public:
		CalcType operator()(InpType P_err, InpType I_err, InpType D_err) {
			return kd(ki(kp(0, P_err), I_err), D_err);
		}

		// Sepparate input for d err
		CalcType operator()(InpType PI_err, InpType D_err) {
			return kd(ki(kp(0, PI_err), PI_err), D_err);
		}

		CalcType operator()(InpType err) {
			return kd(ki(kp(0, err), err), err);
		}

};

template<signed short int Kp, signed short int Ki, signed short int Kd, unsigned char sc = 7u>
class PID_int: PID<signed short int, signed long int, signed short int, signed short int, Kp, Ki, Kd, sc> {
	protected:
		typedef PID<signed short int, signed long int, signed short int, signed short int, Kp, Ki, Kd, sc> base;

		constexpr signed long int scale(signed short int k) {
			return k * sc;
		}
	
	public:
		template<typename InpType>
		signed short int operator()(InpType error, InpType fb) {
			return base::operator()(error, fb) / sc;
		}

		template<typename InpType>
		signed short int operator()(InpType error) {
			return base::operator()(error) / sc;
		}
};

//template<typename InpType, typename CalcType, const signed long int Kp, const signed long int Ki, const signed long int Kd, const short int man, const bool use_fma = false>
//class PID {
//	protected:
//
//		bool kp(InpType err) {
//			if (Kp) {
//				if (use_fma) {
//					out = fma(mk_float(Kp), err, out);
//				}
//				else {
//					out += mk_float(Kp) * err;
//				}
//				return true;
//			}
//			return false;
//		}
//
//		bool ki(InpType err) {
//			if (Ki) {
//				//I_cache += err;
//				overflow_limit_sum(I_cache, err);
//				if (use_fma) {
//					out = fma(mk_float(Ki), I_cache, out);
//				}
//				else {
//					out += mk_float(Ki) * I_cache;
//				}
//				return true;
//			}
//			return false;
//		}
//
//
//		bool kd(InpType fb) {
//			if (Kd) {
//				if (use_fma) {
//					out = fma(-mk_float(Kd), (fb - D_cache), out);
//				}
//				else {
//					out -= mk_float(Kd) * (fb - D_cache);
//				}
//				D_cache = fb;
//				return true;
//			}
//			return false;
//		}
//
//	public:
//		PID(void) : out(0), I_cache(0), D_cache(0) {};
//		PID(CalcType val) : out(val), I_cache(0), D_cache(0) {};
//
//		CalcType operator()(InpType error, InpType fb) {
//			out = 0;
//			kp(error);
//			ki(error);
//			kd(fb);
//			return out;
//		}
//
//		CalcType operator()(InpType error) {
//			out = 0;
//			kp(error);
//			ki(error);
//			kd(error);
//			return out;
//		}
//};

template<typename InpType, typename CalcType, const signed long int Kp, const signed long int Ki, const signed long int Kd, const short int man>
class PID_stepped : public PID<InpType, CalcType, InpType, const signed long int, Kp, Ki, Kd, man> {
	public:
		enum steps : const unsigned char {
			calcI,
			calcP,
			calcD,
			done
		};

	private:
		typedef PID<InpType, CalcType, InpType, const signed long int, Kp, Ki, Kd, man> base;
		CalcType out, err;
		steps step;

	public:
		PID_stepped(void) : base(), step(done) {};
		PID_stepped(CalcType val) : base(), out(val), step(done) {};

		const bool operator()(void) {
			switch (step) {
				case calcP:
					if (Kp) {
						out = base::kp(out, err);
						step = calcD;
						break;
					}

				case calcI:
					if (Ki) {
						out = base::ki(out, err);
						step = calcP;
						break;
					}


				case calcD:
					out = base::kd(out, err);
					step = done;
					break;
				}
			return step == done;
		}

		steps status(void) {
			return step;
		}

		void release(void) {
			if (step == done) {
				terminate();
			}
		}

		void new_iteration(CalcType error, InpType fb) {
			if (step == done) {
				err = error;
				terminate();
			}

		}

		void terminate(void) {
			out = 0;
			step = calcI;
		}

		CalcType result(void) {
			return out;
		}
};

#endif
