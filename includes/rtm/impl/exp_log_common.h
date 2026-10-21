#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2026 Nicholas Frechette & Realtime Math contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "rtm/math.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/macros.vector4.impl.h"

#include <cstdint>
#include <cstring>
#include <limits>

RTM_IMPL_FILE_PRAGMA_PUSH

// The natural exponential and logarithm functions.
//
// The scalar and the vector functions use the same algorithm with the same order of operations.
// Thus for the same input, scalar_exp(x) and each lane of vector_exp(x) give the same result.
// The results do not depend on the C runtime library. The results can differ in the last bit
// between ARM and the other code paths because the code uses FMA on ARM, as the rest of RTM does.
//
// We use the polynomial and rational approximations of the Cephes Math Library (Stephen L. Moshier).
//
// exp(x):
//     e^x = 2^n * e^r with n = round(x / ln(2)) and r = x - (n * ln(2)) in the range [-ln(2)/2, ln(2)/2]
//     We find n with the addition and the subtraction of 1.5 * 2^23 (float32) or 1.5 * 2^52 (float64).
//     The integer value of n is in the low bits of the sum.
//     We multiply by 2^n as 2^n0 * 2^n1 with n = n0 + n1. Each power of two is a normal value
//     and the result can be denormal.
//
// log(x):
//     log(x) = log(m * 2^e) + (e * ln(2)) with the mantissa m in the range [sqrt(0.5), sqrt(2)]
//     We multiply a denormal input by 2^23 (float32) or 2^54 (float64) to make it a normal value.
//
// In the scalar code, each multiplication and each addition is a separate statement:
// the compiler does not fuse them into an FMA.

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	namespace rtm_impl
	{
		// Constants of the float32 functions

		// Below -104.0, e^x is zero. Above 89.0, e^x is +inf.
		// The clamp keeps n in the range [-150, 128] that the two powers of two support.
		constexpr float k_exp_f32_min_input = -104.0F;
		constexpr float k_exp_f32_max_input = 89.0F;
		constexpr float k_exp_f32_log2e = 1.44269504088896341F;
		constexpr float k_exp_f32_round_bias = 12582912.0F;			// 1.5 * 2^23
		constexpr int32_t k_exp_f32_round_bias_bits = 0x4B400000;
		constexpr float k_f32_ln2_hi = 0.693359375F;					// ln(2) in two parts, for precision
		constexpr float k_f32_ln2_lo = -2.12194440E-4F;
		constexpr float k_exp_f32_c0 = 1.9875691500E-4F;
		constexpr float k_exp_f32_c1 = 1.3981999507E-3F;
		constexpr float k_exp_f32_c2 = 8.3334519073E-3F;
		constexpr float k_exp_f32_c3 = 4.1665795894E-2F;
		constexpr float k_exp_f32_c4 = 1.6666665459E-1F;
		constexpr float k_exp_f32_c5 = 5.0000001201E-1F;

		constexpr float k_log_f32_denormal_scale = 8388608.0F;		// 2^23
		constexpr float k_log_f32_sqrt_half = 0.707106781186547524F;
		constexpr float k_log_f32_c0 = 7.0376836292E-2F;
		constexpr float k_log_f32_c1 = -1.1514610310E-1F;
		constexpr float k_log_f32_c2 = 1.1676998740E-1F;
		constexpr float k_log_f32_c3 = -1.2420140846E-1F;
		constexpr float k_log_f32_c4 = 1.4249322787E-1F;
		constexpr float k_log_f32_c5 = -1.6668057665E-1F;
		constexpr float k_log_f32_c6 = 2.0000714765E-1F;
		constexpr float k_log_f32_c7 = -2.4999993993E-1F;
		constexpr float k_log_f32_c8 = 3.3333331174E-1F;

		// Constants of the float64 functions

		// Below -746.0, e^x is zero. Above 710.0, e^x is +inf.
		// The clamp keeps n in the range [-1076, 1024] that the two powers of two support.
		constexpr double k_exp_f64_min_input = -746.0;
		constexpr double k_exp_f64_max_input = 710.0;
		constexpr double k_exp_f64_log2e = 1.4426950408889634073599;
		constexpr double k_exp_f64_round_bias = 6755399441055744.0;	// 1.5 * 2^52
		constexpr int64_t k_exp_f64_round_bias_bits = 0x4338000000000000LL;
		constexpr double k_exp_f64_ln2_hi = 6.93145751953125E-1;		// ln(2) in two parts, for precision
		constexpr double k_exp_f64_ln2_lo = 1.42860682030941723212E-6;
		constexpr double k_exp_f64_p0 = 1.26177193074810590878E-4;
		constexpr double k_exp_f64_p1 = 3.02994407707441961300E-2;
		constexpr double k_exp_f64_p2 = 9.99999999999999999910E-1;
		constexpr double k_exp_f64_q0 = 3.00198505138664455042E-6;
		constexpr double k_exp_f64_q1 = 2.52448340349684104192E-3;
		constexpr double k_exp_f64_q2 = 2.27265548208155028766E-1;
		constexpr double k_exp_f64_q3 = 2.00000000000000000009E0;

		constexpr double k_log_f64_denormal_scale = 18014398509481984.0;	// 2^54
		constexpr double k_log_f64_sqrt_half = 0.70710678118654752440;
		constexpr double k_log_f64_ln2_hi = 0.693359375;					// ln(2) in two parts, for precision
		constexpr double k_log_f64_ln2_lo = -2.121944400546905827679E-4;
		constexpr double k_log_f64_exponent_bias = 4503599627370496.0;		// 2^52
		constexpr double k_log_f64_r0 = -7.89580278884799154124E-1;
		constexpr double k_log_f64_r1 = 1.63866645699558079767E1;
		constexpr double k_log_f64_r2 = -6.41409952958715622951E1;
		constexpr double k_log_f64_s0 = -3.56722798256324312549E1;
		constexpr double k_log_f64_s1 = 3.12093766372244180303E2;
		constexpr double k_log_f64_s2 = -7.69691943550460008604E2;

		// Bit conversions for the scalar code

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32_t float_to_bits(float input) RTM_NO_EXCEPT
		{
			uint32_t result;
			std::memcpy(&result, &input, sizeof(result));
			return result;
		}

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE float bits_to_float(uint32_t input) RTM_NO_EXCEPT
		{
			float result;
			std::memcpy(&result, &input, sizeof(result));
			return result;
		}

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint64_t double_to_bits(double input) RTM_NO_EXCEPT
		{
			uint64_t result;
			std::memcpy(&result, &input, sizeof(result));
			return result;
		}

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE double bits_to_double(uint64_t input) RTM_NO_EXCEPT
		{
			double result;
			std::memcpy(&result, &input, sizeof(result));
			return result;
		}

		// Returns e^input for a float32 value.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float exp_f32(float input) RTM_NO_EXCEPT
		{
			if (input != input)
				return input;	// NaN

			float x = input < k_exp_f32_min_input ? k_exp_f32_min_input : input;
			x = x > k_exp_f32_max_input ? k_exp_f32_max_input : x;

			float biased_n = x * k_exp_f32_log2e;
			biased_n = biased_n + k_exp_f32_round_bias;
			const float n = biased_n - k_exp_f32_round_bias;
			const int32_t n_i = int32_t(float_to_bits(biased_n)) - k_exp_f32_round_bias_bits;

			float t = n * k_f32_ln2_hi;
			float r = x - t;
			t = n * k_f32_ln2_lo;
			r = r - t;

			const float r2 = r * r;
			float result = r * k_exp_f32_c0;
			result = result + k_exp_f32_c1;
			result = result * r;
			result = result + k_exp_f32_c2;
			result = result * r;
			result = result + k_exp_f32_c3;
			result = result * r;
			result = result + k_exp_f32_c4;
			result = result * r;
			result = result + k_exp_f32_c5;
			result = result * r2;
			result = result + r;
			result = result + 1.0F;

			const int32_t n0 = n_i >> 1;
			const int32_t n1 = n_i - n0;
			const float pow2_n0 = bits_to_float(uint32_t(n0 + 127) << 23);
			const float pow2_n1 = bits_to_float(uint32_t(n1 + 127) << 23);
			result = result * pow2_n0;
			result = result * pow2_n1;
			return result;
		}

		// Returns the natural logarithm of a float32 value.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float log_f32(float input) RTM_NO_EXCEPT
		{
			if (input != input)
				return input;	// NaN
			if (input < 0.0F)
				return std::numeric_limits<float>::quiet_NaN();
			if (input == 0.0F)
				return -std::numeric_limits<float>::infinity();
			if (input == std::numeric_limits<float>::infinity())
				return input;

			const bool is_denormal = input < (std::numeric_limits<float>::min)();
			const float x = is_denormal ? (input * k_log_f32_denormal_scale) : input;
			const float exponent_offset = is_denormal ? 23.0F : 0.0F;

			// The exponent bits minus 126 give a mantissa in the range [0.5, 1.0)
			const uint32_t x_bits = float_to_bits(x);
			float exponent = float(int32_t(x_bits >> 23) - 126);
			exponent = exponent - exponent_offset;
			const float mantissa = bits_to_float((x_bits & 0x007FFFFF) | 0x3F000000);

			// If the mantissa is below sqrt(0.5), we use 2 * mantissa and exponent - 1
			float m = mantissa - 1.0F;
			if (mantissa < k_log_f32_sqrt_half)
			{
				exponent = exponent - 1.0F;
				m = m + mantissa;
			}

			const float m2 = m * m;
			float result = m * k_log_f32_c0;
			result = result + k_log_f32_c1;
			result = result * m;
			result = result + k_log_f32_c2;
			result = result * m;
			result = result + k_log_f32_c3;
			result = result * m;
			result = result + k_log_f32_c4;
			result = result * m;
			result = result + k_log_f32_c5;
			result = result * m;
			result = result + k_log_f32_c6;
			result = result * m;
			result = result + k_log_f32_c7;
			result = result * m;
			result = result + k_log_f32_c8;
			result = result * m;
			result = result * m2;

			float t = exponent * k_f32_ln2_lo;
			result = result + t;
			t = m2 * 0.5F;
			result = result - t;
			result = result + m;
			t = exponent * k_f32_ln2_hi;
			result = result + t;
			return result;
		}

		// Returns e^input for a float64 value.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline double exp_f64(double input) RTM_NO_EXCEPT
		{
			if (input != input)
				return input;	// NaN

			double x = input < k_exp_f64_min_input ? k_exp_f64_min_input : input;
			x = x > k_exp_f64_max_input ? k_exp_f64_max_input : x;

			double biased_n = x * k_exp_f64_log2e;
			biased_n = biased_n + k_exp_f64_round_bias;
			const double n = biased_n - k_exp_f64_round_bias;
			const int64_t n_i = int64_t(double_to_bits(biased_n)) - k_exp_f64_round_bias_bits;

			double t = n * k_exp_f64_ln2_hi;
			double r = x - t;
			t = n * k_exp_f64_ln2_lo;
			r = r - t;

			// e^r = 1 + 2 * P(r) / (Q(r^2) - P(r)) with P(r) = r * P(r^2)
			const double r2 = r * r;
			double p = r2 * k_exp_f64_p0;
			p = p + k_exp_f64_p1;
			p = p * r2;
			p = p + k_exp_f64_p2;
			p = p * r;
			double q = r2 * k_exp_f64_q0;
			q = q + k_exp_f64_q1;
			q = q * r2;
			q = q + k_exp_f64_q2;
			q = q * r2;
			q = q + k_exp_f64_q3;
			q = q - p;
			double result = p / q;
			result = result * 2.0;
			result = result + 1.0;

			const int64_t n0 = n_i >> 1;
			const int64_t n1 = n_i - n0;
			const double pow2_n0 = bits_to_double(uint64_t(n0 + 1023) << 52);
			const double pow2_n1 = bits_to_double(uint64_t(n1 + 1023) << 52);
			result = result * pow2_n0;
			result = result * pow2_n1;
			return result;
		}

		// Returns the natural logarithm of a float64 value.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline double log_f64(double input) RTM_NO_EXCEPT
		{
			if (input != input)
				return input;	// NaN
			if (input < 0.0)
				return std::numeric_limits<double>::quiet_NaN();
			if (input == 0.0)
				return -std::numeric_limits<double>::infinity();
			if (input == std::numeric_limits<double>::infinity())
				return input;

			const bool is_denormal = input < (std::numeric_limits<double>::min)();
			const double x = is_denormal ? (input * k_log_f64_denormal_scale) : input;
			const double exponent_offset = is_denormal ? 54.0 : 0.0;

			// The exponent bits minus 1022 give a mantissa in the range [0.5, 1.0)
			const uint64_t x_bits = double_to_bits(x);
			double exponent = bits_to_double((x_bits >> 52) | 0x4330000000000000ULL);
			exponent = exponent - k_log_f64_exponent_bias;
			exponent = exponent - 1022.0;
			exponent = exponent - exponent_offset;
			const double mantissa = bits_to_double((x_bits & 0x000FFFFFFFFFFFFFULL) | 0x3FE0000000000000ULL);

			// s = 2 * (m - 1) / (m + 1)
			// If the mantissa is below sqrt(0.5), we use m = 2 * mantissa and exponent - 1
			double z;
			double y;
			if (mantissa < k_log_f64_sqrt_half)
			{
				exponent = exponent - 1.0;
				z = mantissa - 0.5;
				y = z * 0.5;
				y = y + 0.5;
			}
			else
			{
				z = mantissa - 0.5;
				z = z - 0.5;
				y = mantissa * 0.5;
				y = y + 0.5;
			}

			// log(m) = s + s * s^2 * R(s^2) / S(s^2)
			const double s = z / y;
			const double s2 = s * s;
			double r = s2 * k_log_f64_r0;
			r = r + k_log_f64_r1;
			r = r * s2;
			r = r + k_log_f64_r2;
			double q = s2 + k_log_f64_s0;
			q = q * s2;
			q = q + k_log_f64_s1;
			q = q * s2;
			q = q + k_log_f64_s2;
			double result = s2 * r;
			result = result / q;
			result = result * s;

			double t = exponent * k_log_f64_ln2_lo;
			result = result + t;
			result = result + s;
			t = exponent * k_log_f64_ln2_hi;
			result = result + t;
			return result;
		}

#if defined(RTM_SSE2_INTRINSICS)
		// Returns e^input for each float32 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline __m128 RTM_SIMD_CALL exp_f32x4(__m128 input) RTM_NO_EXCEPT
		{
			const __m128 x = _mm_min_ps(_mm_max_ps(input, _mm_set_ps1(k_exp_f32_min_input)), _mm_set_ps1(k_exp_f32_max_input));

			const __m128 round_bias = _mm_set_ps1(k_exp_f32_round_bias);
			const __m128 biased_n = _mm_add_ps(_mm_mul_ps(x, _mm_set_ps1(k_exp_f32_log2e)), round_bias);
			const __m128 n = _mm_sub_ps(biased_n, round_bias);
			const __m128i n_i = _mm_sub_epi32(_mm_castps_si128(biased_n), _mm_set1_epi32(k_exp_f32_round_bias_bits));

			__m128 r = _mm_sub_ps(x, _mm_mul_ps(n, _mm_set_ps1(k_f32_ln2_hi)));
			r = _mm_sub_ps(r, _mm_mul_ps(n, _mm_set_ps1(k_f32_ln2_lo)));

			const __m128 r2 = _mm_mul_ps(r, r);
			__m128 result = _mm_add_ps(_mm_mul_ps(r, _mm_set_ps1(k_exp_f32_c0)), _mm_set_ps1(k_exp_f32_c1));
			result = _mm_add_ps(_mm_mul_ps(result, r), _mm_set_ps1(k_exp_f32_c2));
			result = _mm_add_ps(_mm_mul_ps(result, r), _mm_set_ps1(k_exp_f32_c3));
			result = _mm_add_ps(_mm_mul_ps(result, r), _mm_set_ps1(k_exp_f32_c4));
			result = _mm_add_ps(_mm_mul_ps(result, r), _mm_set_ps1(k_exp_f32_c5));
			result = _mm_add_ps(_mm_add_ps(_mm_mul_ps(result, r2), r), _mm_set_ps1(1.0F));

			const __m128i n0 = _mm_srai_epi32(n_i, 1);
			const __m128i n1 = _mm_sub_epi32(n_i, n0);
			const __m128i exponent_bias = _mm_set1_epi32(127);
			const __m128 pow2_n0 = _mm_castsi128_ps(_mm_slli_epi32(_mm_add_epi32(n0, exponent_bias), 23));
			const __m128 pow2_n1 = _mm_castsi128_ps(_mm_slli_epi32(_mm_add_epi32(n1, exponent_bias), 23));
			result = _mm_mul_ps(_mm_mul_ps(result, pow2_n0), pow2_n1);

			// The result is NaN when the input is NaN
			return RTM_VECTOR4F_SELECT(_mm_cmpunord_ps(input, input), input, result);
		}

		// Returns the natural logarithm of each float32 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline __m128 RTM_SIMD_CALL log_f32x4(__m128 input) RTM_NO_EXCEPT
		{
			const __m128 is_denormal = _mm_cmplt_ps(input, _mm_set_ps1((std::numeric_limits<float>::min)()));
			const __m128 x = RTM_VECTOR4F_SELECT(is_denormal, _mm_mul_ps(input, _mm_set_ps1(k_log_f32_denormal_scale)), input);
			const __m128 exponent_offset = _mm_and_ps(is_denormal, _mm_set_ps1(23.0F));

			// The exponent bits minus 126 give a mantissa in the range [0.5, 1.0)
			const __m128i x_i = _mm_castps_si128(x);
			__m128 exponent = _mm_cvtepi32_ps(_mm_sub_epi32(_mm_srli_epi32(x_i, 23), _mm_set1_epi32(126)));
			exponent = _mm_sub_ps(exponent, exponent_offset);
			const __m128 mantissa = _mm_castsi128_ps(_mm_or_si128(_mm_and_si128(x_i, _mm_set1_epi32(0x007FFFFF)), _mm_set1_epi32(0x3F000000)));

			// If the mantissa is below sqrt(0.5), we use 2 * mantissa and exponent - 1
			const __m128 is_mantissa_small = _mm_cmplt_ps(mantissa, _mm_set_ps1(k_log_f32_sqrt_half));
			exponent = _mm_sub_ps(exponent, _mm_and_ps(is_mantissa_small, _mm_set_ps1(1.0F)));
			const __m128 m = _mm_add_ps(_mm_sub_ps(mantissa, _mm_set_ps1(1.0F)), _mm_and_ps(is_mantissa_small, mantissa));

			const __m128 m2 = _mm_mul_ps(m, m);
			__m128 result = _mm_add_ps(_mm_mul_ps(m, _mm_set_ps1(k_log_f32_c0)), _mm_set_ps1(k_log_f32_c1));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c2));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c3));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c4));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c5));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c6));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c7));
			result = _mm_add_ps(_mm_mul_ps(result, m), _mm_set_ps1(k_log_f32_c8));
			result = _mm_mul_ps(_mm_mul_ps(result, m), m2);

			result = _mm_add_ps(result, _mm_mul_ps(exponent, _mm_set_ps1(k_f32_ln2_lo)));
			result = _mm_sub_ps(result, _mm_mul_ps(m2, _mm_set_ps1(0.5F)));
			result = _mm_add_ps(result, m);
			result = _mm_add_ps(result, _mm_mul_ps(exponent, _mm_set_ps1(k_f32_ln2_hi)));

			// Special values: log(+inf) = +inf, log(0) = -inf, log(negative) = NaN, log(NaN) = NaN
			const __m128 infinity = _mm_set_ps1(std::numeric_limits<float>::infinity());
			const __m128 zero = _mm_setzero_ps();
			result = RTM_VECTOR4F_SELECT(_mm_cmpeq_ps(input, infinity), infinity, result);
			result = RTM_VECTOR4F_SELECT(_mm_cmpeq_ps(input, zero), _mm_set_ps1(-std::numeric_limits<float>::infinity()), result);
			result = RTM_VECTOR4F_SELECT(_mm_cmplt_ps(input, zero), _mm_set_ps1(std::numeric_limits<float>::quiet_NaN()), result);
			return RTM_VECTOR4F_SELECT(_mm_cmpunord_ps(input, input), input, result);
		}

		// Returns e^input for each float64 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline __m128d RTM_SIMD_CALL exp_f64x2(__m128d input) RTM_NO_EXCEPT
		{
			const __m128d x = _mm_min_pd(_mm_max_pd(input, _mm_set1_pd(k_exp_f64_min_input)), _mm_set1_pd(k_exp_f64_max_input));

			const __m128d round_bias = _mm_set1_pd(k_exp_f64_round_bias);
			const __m128d biased_n = _mm_add_pd(_mm_mul_pd(x, _mm_set1_pd(k_exp_f64_log2e)), round_bias);
			const __m128d n = _mm_sub_pd(biased_n, round_bias);
			const __m128i n_i = _mm_sub_epi64(_mm_castpd_si128(biased_n), _mm_castpd_si128(round_bias));

			__m128d r = _mm_sub_pd(x, _mm_mul_pd(n, _mm_set1_pd(k_exp_f64_ln2_hi)));
			r = _mm_sub_pd(r, _mm_mul_pd(n, _mm_set1_pd(k_exp_f64_ln2_lo)));

			const __m128d r2 = _mm_mul_pd(r, r);
			__m128d p = _mm_add_pd(_mm_mul_pd(r2, _mm_set1_pd(k_exp_f64_p0)), _mm_set1_pd(k_exp_f64_p1));
			p = _mm_add_pd(_mm_mul_pd(p, r2), _mm_set1_pd(k_exp_f64_p2));
			p = _mm_mul_pd(p, r);
			__m128d q = _mm_add_pd(_mm_mul_pd(r2, _mm_set1_pd(k_exp_f64_q0)), _mm_set1_pd(k_exp_f64_q1));
			q = _mm_add_pd(_mm_mul_pd(q, r2), _mm_set1_pd(k_exp_f64_q2));
			q = _mm_add_pd(_mm_mul_pd(q, r2), _mm_set1_pd(k_exp_f64_q3));
			q = _mm_sub_pd(q, p);
			__m128d result = _mm_div_pd(p, q);
			result = _mm_add_pd(_mm_mul_pd(result, _mm_set1_pd(2.0)), _mm_set1_pd(1.0));

			// SSE2 has no 64-bit arithmetic shift. n is in the range of an int32: the 32-bit shift of the low
			// half gives n0, and the high half contains the sign bits for the 64-bit value.
			const __m128i n0 = _mm_srai_epi32(n_i, 1);
			const __m128i n1 = _mm_sub_epi64(n_i, n0);
			const __m128i exponent_bias = _mm_set_epi32(0, 1023, 0, 1023);
			const __m128d pow2_n0 = _mm_castsi128_pd(_mm_slli_epi64(_mm_add_epi64(n0, exponent_bias), 52));
			const __m128d pow2_n1 = _mm_castsi128_pd(_mm_slli_epi64(_mm_add_epi64(n1, exponent_bias), 52));
			result = _mm_mul_pd(_mm_mul_pd(result, pow2_n0), pow2_n1);

			// The result is NaN when the input is NaN
			return RTM_VECTOR2D_SELECT(_mm_cmpunord_pd(input, input), input, result);
		}

		// Returns the natural logarithm of each float64 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline __m128d RTM_SIMD_CALL log_f64x2(__m128d input) RTM_NO_EXCEPT
		{
			const __m128d is_denormal = _mm_cmplt_pd(input, _mm_set1_pd((std::numeric_limits<double>::min)()));
			const __m128d x = RTM_VECTOR2D_SELECT(is_denormal, _mm_mul_pd(input, _mm_set1_pd(k_log_f64_denormal_scale)), input);
			const __m128d exponent_offset = _mm_and_pd(is_denormal, _mm_set1_pd(54.0));

			// The exponent bits minus 1022 give a mantissa in the range [0.5, 1.0)
			const __m128i x_i = _mm_castpd_si128(x);
			const __m128i exponent_bits = _mm_or_si128(_mm_srli_epi64(x_i, 52), _mm_set_epi32(0x43300000, 0, 0x43300000, 0));
			__m128d exponent = _mm_sub_pd(_mm_castsi128_pd(exponent_bits), _mm_set1_pd(k_log_f64_exponent_bias));
			exponent = _mm_sub_pd(exponent, _mm_set1_pd(1022.0));
			exponent = _mm_sub_pd(exponent, exponent_offset);
			const __m128i mantissa_mask = _mm_set_epi32(0x000FFFFF, -1, 0x000FFFFF, -1);
			const __m128i mantissa_exponent = _mm_set_epi32(0x3FE00000, 0, 0x3FE00000, 0);
			const __m128d mantissa = _mm_castsi128_pd(_mm_or_si128(_mm_and_si128(x_i, mantissa_mask), mantissa_exponent));

			// s = 2 * (m - 1) / (m + 1)
			// If the mantissa is below sqrt(0.5), we use m = 2 * mantissa and exponent - 1
			const __m128d half = _mm_set1_pd(0.5);
			const __m128d is_mantissa_small = _mm_cmplt_pd(mantissa, _mm_set1_pd(k_log_f64_sqrt_half));
			exponent = _mm_sub_pd(exponent, _mm_and_pd(is_mantissa_small, _mm_set1_pd(1.0)));
			const __m128d z_small = _mm_sub_pd(mantissa, half);
			const __m128d y_small = _mm_add_pd(_mm_mul_pd(z_small, half), half);
			const __m128d z_large = _mm_sub_pd(_mm_sub_pd(mantissa, half), half);
			const __m128d y_large = _mm_add_pd(_mm_mul_pd(mantissa, half), half);
			const __m128d z = RTM_VECTOR2D_SELECT(is_mantissa_small, z_small, z_large);
			const __m128d y = RTM_VECTOR2D_SELECT(is_mantissa_small, y_small, y_large);

			const __m128d s = _mm_div_pd(z, y);
			const __m128d s2 = _mm_mul_pd(s, s);
			__m128d r = _mm_add_pd(_mm_mul_pd(s2, _mm_set1_pd(k_log_f64_r0)), _mm_set1_pd(k_log_f64_r1));
			r = _mm_add_pd(_mm_mul_pd(r, s2), _mm_set1_pd(k_log_f64_r2));
			__m128d q = _mm_add_pd(s2, _mm_set1_pd(k_log_f64_s0));
			q = _mm_add_pd(_mm_mul_pd(q, s2), _mm_set1_pd(k_log_f64_s1));
			q = _mm_add_pd(_mm_mul_pd(q, s2), _mm_set1_pd(k_log_f64_s2));
			__m128d result = _mm_mul_pd(_mm_div_pd(_mm_mul_pd(s2, r), q), s);

			result = _mm_add_pd(result, _mm_mul_pd(exponent, _mm_set1_pd(k_log_f64_ln2_lo)));
			result = _mm_add_pd(result, s);
			result = _mm_add_pd(result, _mm_mul_pd(exponent, _mm_set1_pd(k_log_f64_ln2_hi)));

			// Special values: log(+inf) = +inf, log(0) = -inf, log(negative) = NaN, log(NaN) = NaN
			const __m128d infinity = _mm_set1_pd(std::numeric_limits<double>::infinity());
			const __m128d zero = _mm_setzero_pd();
			result = RTM_VECTOR2D_SELECT(_mm_cmpeq_pd(input, infinity), infinity, result);
			result = RTM_VECTOR2D_SELECT(_mm_cmpeq_pd(input, zero), _mm_set1_pd(-std::numeric_limits<double>::infinity()), result);
			result = RTM_VECTOR2D_SELECT(_mm_cmplt_pd(input, zero), _mm_set1_pd(std::numeric_limits<double>::quiet_NaN()), result);
			return RTM_VECTOR2D_SELECT(_mm_cmpunord_pd(input, input), input, result);
		}
#elif defined(RTM_WASM128_INTRINSICS)
		// Returns e^input for each float32 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline v128_t RTM_SIMD_CALL exp_f32x4(v128_t input) RTM_NO_EXCEPT
		{
			const v128_t x = wasm_f32x4_pmin(wasm_f32x4_pmax(input, wasm_f32x4_splat(k_exp_f32_min_input)), wasm_f32x4_splat(k_exp_f32_max_input));

			const v128_t round_bias = wasm_f32x4_splat(k_exp_f32_round_bias);
			const v128_t biased_n = wasm_f32x4_add(wasm_f32x4_mul(x, wasm_f32x4_splat(k_exp_f32_log2e)), round_bias);
			const v128_t n = wasm_f32x4_sub(biased_n, round_bias);
			const v128_t n_i = wasm_i32x4_sub(biased_n, wasm_i32x4_splat(k_exp_f32_round_bias_bits));

			v128_t r = wasm_f32x4_sub(x, wasm_f32x4_mul(n, wasm_f32x4_splat(k_f32_ln2_hi)));
			r = wasm_f32x4_sub(r, wasm_f32x4_mul(n, wasm_f32x4_splat(k_f32_ln2_lo)));

			const v128_t r2 = wasm_f32x4_mul(r, r);
			v128_t result = wasm_f32x4_add(wasm_f32x4_mul(r, wasm_f32x4_splat(k_exp_f32_c0)), wasm_f32x4_splat(k_exp_f32_c1));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, r), wasm_f32x4_splat(k_exp_f32_c2));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, r), wasm_f32x4_splat(k_exp_f32_c3));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, r), wasm_f32x4_splat(k_exp_f32_c4));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, r), wasm_f32x4_splat(k_exp_f32_c5));
			result = wasm_f32x4_add(wasm_f32x4_add(wasm_f32x4_mul(result, r2), r), wasm_f32x4_splat(1.0F));

			const v128_t n0 = wasm_i32x4_shr(n_i, 1);
			const v128_t n1 = wasm_i32x4_sub(n_i, n0);
			const v128_t exponent_bias = wasm_i32x4_splat(127);
			const v128_t pow2_n0 = wasm_i32x4_shl(wasm_i32x4_add(n0, exponent_bias), 23);
			const v128_t pow2_n1 = wasm_i32x4_shl(wasm_i32x4_add(n1, exponent_bias), 23);
			result = wasm_f32x4_mul(wasm_f32x4_mul(result, pow2_n0), pow2_n1);

			// The result is NaN when the input is NaN
			return RTM_VECTOR4F_SELECT(wasm_f32x4_ne(input, input), input, result);
		}

		// Returns the natural logarithm of each float32 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline v128_t RTM_SIMD_CALL log_f32x4(v128_t input) RTM_NO_EXCEPT
		{
			const v128_t is_denormal = wasm_f32x4_lt(input, wasm_f32x4_splat((std::numeric_limits<float>::min)()));
			const v128_t x = RTM_VECTOR4F_SELECT(is_denormal, wasm_f32x4_mul(input, wasm_f32x4_splat(k_log_f32_denormal_scale)), input);
			const v128_t exponent_offset = wasm_v128_and(is_denormal, wasm_f32x4_splat(23.0F));

			// The exponent bits minus 126 give a mantissa in the range [0.5, 1.0)
			v128_t exponent = wasm_f32x4_convert_i32x4(wasm_i32x4_sub(wasm_u32x4_shr(x, 23), wasm_i32x4_splat(126)));
			exponent = wasm_f32x4_sub(exponent, exponent_offset);
			const v128_t mantissa = wasm_v128_or(wasm_v128_and(x, wasm_i32x4_splat(0x007FFFFF)), wasm_i32x4_splat(0x3F000000));

			// If the mantissa is below sqrt(0.5), we use 2 * mantissa and exponent - 1
			const v128_t is_mantissa_small = wasm_f32x4_lt(mantissa, wasm_f32x4_splat(k_log_f32_sqrt_half));
			exponent = wasm_f32x4_sub(exponent, wasm_v128_and(is_mantissa_small, wasm_f32x4_splat(1.0F)));
			const v128_t m = wasm_f32x4_add(wasm_f32x4_sub(mantissa, wasm_f32x4_splat(1.0F)), wasm_v128_and(is_mantissa_small, mantissa));

			const v128_t m2 = wasm_f32x4_mul(m, m);
			v128_t result = wasm_f32x4_add(wasm_f32x4_mul(m, wasm_f32x4_splat(k_log_f32_c0)), wasm_f32x4_splat(k_log_f32_c1));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c2));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c3));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c4));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c5));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c6));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c7));
			result = wasm_f32x4_add(wasm_f32x4_mul(result, m), wasm_f32x4_splat(k_log_f32_c8));
			result = wasm_f32x4_mul(wasm_f32x4_mul(result, m), m2);

			result = wasm_f32x4_add(result, wasm_f32x4_mul(exponent, wasm_f32x4_splat(k_f32_ln2_lo)));
			result = wasm_f32x4_sub(result, wasm_f32x4_mul(m2, wasm_f32x4_splat(0.5F)));
			result = wasm_f32x4_add(result, m);
			result = wasm_f32x4_add(result, wasm_f32x4_mul(exponent, wasm_f32x4_splat(k_f32_ln2_hi)));

			// Special values: log(+inf) = +inf, log(0) = -inf, log(negative) = NaN, log(NaN) = NaN
			const v128_t infinity = wasm_f32x4_splat(std::numeric_limits<float>::infinity());
			const v128_t zero = wasm_f32x4_splat(0.0F);
			result = RTM_VECTOR4F_SELECT(wasm_f32x4_eq(input, infinity), infinity, result);
			result = RTM_VECTOR4F_SELECT(wasm_f32x4_eq(input, zero), wasm_f32x4_splat(-std::numeric_limits<float>::infinity()), result);
			result = RTM_VECTOR4F_SELECT(wasm_f32x4_lt(input, zero), wasm_f32x4_splat(std::numeric_limits<float>::quiet_NaN()), result);
			return RTM_VECTOR4F_SELECT(wasm_f32x4_ne(input, input), input, result);
		}

		// Returns e^input for each float64 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline v128_t RTM_SIMD_CALL exp_f64x2(v128_t input) RTM_NO_EXCEPT
		{
			const v128_t x = wasm_f64x2_pmin(wasm_f64x2_pmax(input, wasm_f64x2_splat(k_exp_f64_min_input)), wasm_f64x2_splat(k_exp_f64_max_input));

			const v128_t round_bias = wasm_f64x2_splat(k_exp_f64_round_bias);
			const v128_t biased_n = wasm_f64x2_add(wasm_f64x2_mul(x, wasm_f64x2_splat(k_exp_f64_log2e)), round_bias);
			const v128_t n = wasm_f64x2_sub(biased_n, round_bias);
			const v128_t n_i = wasm_i64x2_sub(biased_n, wasm_i64x2_splat(k_exp_f64_round_bias_bits));

			v128_t r = wasm_f64x2_sub(x, wasm_f64x2_mul(n, wasm_f64x2_splat(k_exp_f64_ln2_hi)));
			r = wasm_f64x2_sub(r, wasm_f64x2_mul(n, wasm_f64x2_splat(k_exp_f64_ln2_lo)));

			const v128_t r2 = wasm_f64x2_mul(r, r);
			v128_t p = wasm_f64x2_add(wasm_f64x2_mul(r2, wasm_f64x2_splat(k_exp_f64_p0)), wasm_f64x2_splat(k_exp_f64_p1));
			p = wasm_f64x2_add(wasm_f64x2_mul(p, r2), wasm_f64x2_splat(k_exp_f64_p2));
			p = wasm_f64x2_mul(p, r);
			v128_t q = wasm_f64x2_add(wasm_f64x2_mul(r2, wasm_f64x2_splat(k_exp_f64_q0)), wasm_f64x2_splat(k_exp_f64_q1));
			q = wasm_f64x2_add(wasm_f64x2_mul(q, r2), wasm_f64x2_splat(k_exp_f64_q2));
			q = wasm_f64x2_add(wasm_f64x2_mul(q, r2), wasm_f64x2_splat(k_exp_f64_q3));
			q = wasm_f64x2_sub(q, p);
			v128_t result = wasm_f64x2_div(p, q);
			result = wasm_f64x2_add(wasm_f64x2_mul(result, wasm_f64x2_splat(2.0)), wasm_f64x2_splat(1.0));

			const v128_t n0 = wasm_i64x2_shr(n_i, 1);
			const v128_t n1 = wasm_i64x2_sub(n_i, n0);
			const v128_t exponent_bias = wasm_i64x2_splat(1023);
			const v128_t pow2_n0 = wasm_i64x2_shl(wasm_i64x2_add(n0, exponent_bias), 52);
			const v128_t pow2_n1 = wasm_i64x2_shl(wasm_i64x2_add(n1, exponent_bias), 52);
			result = wasm_f64x2_mul(wasm_f64x2_mul(result, pow2_n0), pow2_n1);

			// The result is NaN when the input is NaN
			return RTM_VECTOR2D_SELECT(wasm_f64x2_ne(input, input), input, result);
		}

		// Returns the natural logarithm of each float64 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline v128_t RTM_SIMD_CALL log_f64x2(v128_t input) RTM_NO_EXCEPT
		{
			const v128_t is_denormal = wasm_f64x2_lt(input, wasm_f64x2_splat((std::numeric_limits<double>::min)()));
			const v128_t x = RTM_VECTOR2D_SELECT(is_denormal, wasm_f64x2_mul(input, wasm_f64x2_splat(k_log_f64_denormal_scale)), input);
			const v128_t exponent_offset = wasm_v128_and(is_denormal, wasm_f64x2_splat(54.0));

			// The exponent bits minus 1022 give a mantissa in the range [0.5, 1.0)
			const v128_t exponent_bits = wasm_v128_or(wasm_u64x2_shr(x, 52), wasm_i64x2_splat(0x4330000000000000LL));
			v128_t exponent = wasm_f64x2_sub(exponent_bits, wasm_f64x2_splat(k_log_f64_exponent_bias));
			exponent = wasm_f64x2_sub(exponent, wasm_f64x2_splat(1022.0));
			exponent = wasm_f64x2_sub(exponent, exponent_offset);
			const v128_t mantissa = wasm_v128_or(wasm_v128_and(x, wasm_i64x2_splat(0x000FFFFFFFFFFFFFLL)), wasm_i64x2_splat(0x3FE0000000000000LL));

			// s = 2 * (m - 1) / (m + 1)
			// If the mantissa is below sqrt(0.5), we use m = 2 * mantissa and exponent - 1
			const v128_t half = wasm_f64x2_splat(0.5);
			const v128_t is_mantissa_small = wasm_f64x2_lt(mantissa, wasm_f64x2_splat(k_log_f64_sqrt_half));
			exponent = wasm_f64x2_sub(exponent, wasm_v128_and(is_mantissa_small, wasm_f64x2_splat(1.0)));
			const v128_t z_small = wasm_f64x2_sub(mantissa, half);
			const v128_t y_small = wasm_f64x2_add(wasm_f64x2_mul(z_small, half), half);
			const v128_t z_large = wasm_f64x2_sub(wasm_f64x2_sub(mantissa, half), half);
			const v128_t y_large = wasm_f64x2_add(wasm_f64x2_mul(mantissa, half), half);
			const v128_t z = RTM_VECTOR2D_SELECT(is_mantissa_small, z_small, z_large);
			const v128_t y = RTM_VECTOR2D_SELECT(is_mantissa_small, y_small, y_large);

			const v128_t s = wasm_f64x2_div(z, y);
			const v128_t s2 = wasm_f64x2_mul(s, s);
			v128_t r = wasm_f64x2_add(wasm_f64x2_mul(s2, wasm_f64x2_splat(k_log_f64_r0)), wasm_f64x2_splat(k_log_f64_r1));
			r = wasm_f64x2_add(wasm_f64x2_mul(r, s2), wasm_f64x2_splat(k_log_f64_r2));
			v128_t q = wasm_f64x2_add(s2, wasm_f64x2_splat(k_log_f64_s0));
			q = wasm_f64x2_add(wasm_f64x2_mul(q, s2), wasm_f64x2_splat(k_log_f64_s1));
			q = wasm_f64x2_add(wasm_f64x2_mul(q, s2), wasm_f64x2_splat(k_log_f64_s2));
			v128_t result = wasm_f64x2_mul(wasm_f64x2_div(wasm_f64x2_mul(s2, r), q), s);

			result = wasm_f64x2_add(result, wasm_f64x2_mul(exponent, wasm_f64x2_splat(k_log_f64_ln2_lo)));
			result = wasm_f64x2_add(result, s);
			result = wasm_f64x2_add(result, wasm_f64x2_mul(exponent, wasm_f64x2_splat(k_log_f64_ln2_hi)));

			// Special values: log(+inf) = +inf, log(0) = -inf, log(negative) = NaN, log(NaN) = NaN
			const v128_t infinity = wasm_f64x2_splat(std::numeric_limits<double>::infinity());
			const v128_t zero = wasm_f64x2_splat(0.0);
			result = RTM_VECTOR2D_SELECT(wasm_f64x2_eq(input, infinity), infinity, result);
			result = RTM_VECTOR2D_SELECT(wasm_f64x2_eq(input, zero), wasm_f64x2_splat(-std::numeric_limits<double>::infinity()), result);
			result = RTM_VECTOR2D_SELECT(wasm_f64x2_lt(input, zero), wasm_f64x2_splat(std::numeric_limits<double>::quiet_NaN()), result);
			return RTM_VECTOR2D_SELECT(wasm_f64x2_ne(input, input), input, result);
		}
#elif defined(RTM_NEON_INTRINSICS)
		// Returns e^input for each float32 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float32x4_t RTM_SIMD_CALL exp_f32x4(float32x4_t input) RTM_NO_EXCEPT
		{
			const float32x4_t x = vminq_f32(vmaxq_f32(input, vdupq_n_f32(k_exp_f32_min_input)), vdupq_n_f32(k_exp_f32_max_input));

			const float32x4_t round_bias = vdupq_n_f32(k_exp_f32_round_bias);
			const float32x4_t biased_n = vaddq_f32(vmulq_n_f32(x, k_exp_f32_log2e), round_bias);
			const float32x4_t n = vsubq_f32(biased_n, round_bias);
			const int32x4_t n_i = vsubq_s32(vreinterpretq_s32_f32(biased_n), vdupq_n_s32(k_exp_f32_round_bias_bits));

			float32x4_t r = vsubq_f32(x, vmulq_n_f32(n, k_f32_ln2_hi));
			r = vsubq_f32(r, vmulq_n_f32(n, k_f32_ln2_lo));

			const float32x4_t r2 = vmulq_f32(r, r);
			float32x4_t result = RTM_VECTOR4F_MULS_ADD(r, k_exp_f32_c0, vdupq_n_f32(k_exp_f32_c1));
			result = RTM_VECTOR4F_MULV_ADD(result, r, vdupq_n_f32(k_exp_f32_c2));
			result = RTM_VECTOR4F_MULV_ADD(result, r, vdupq_n_f32(k_exp_f32_c3));
			result = RTM_VECTOR4F_MULV_ADD(result, r, vdupq_n_f32(k_exp_f32_c4));
			result = RTM_VECTOR4F_MULV_ADD(result, r, vdupq_n_f32(k_exp_f32_c5));
			result = vaddq_f32(RTM_VECTOR4F_MULV_ADD(result, r2, r), vdupq_n_f32(1.0F));

			const int32x4_t n0 = vshrq_n_s32(n_i, 1);
			const int32x4_t n1 = vsubq_s32(n_i, n0);
			const int32x4_t exponent_bias = vdupq_n_s32(127);
			const float32x4_t pow2_n0 = vreinterpretq_f32_s32(vshlq_n_s32(vaddq_s32(n0, exponent_bias), 23));
			const float32x4_t pow2_n1 = vreinterpretq_f32_s32(vshlq_n_s32(vaddq_s32(n1, exponent_bias), 23));
			result = vmulq_f32(vmulq_f32(result, pow2_n0), pow2_n1);

			// The result is NaN when the input is NaN
			return vbslq_f32(vceqq_f32(input, input), result, input);
		}

		// Returns the natural logarithm of each float32 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float32x4_t RTM_SIMD_CALL log_f32x4(float32x4_t input) RTM_NO_EXCEPT
		{
			const uint32x4_t is_denormal = vcltq_f32(input, vdupq_n_f32((std::numeric_limits<float>::min)()));
			const float32x4_t x = vbslq_f32(is_denormal, vmulq_n_f32(input, k_log_f32_denormal_scale), input);
			const float32x4_t exponent_offset = vreinterpretq_f32_u32(vandq_u32(is_denormal, vreinterpretq_u32_f32(vdupq_n_f32(23.0F))));

			// The exponent bits minus 126 give a mantissa in the range [0.5, 1.0)
			const uint32x4_t x_u = vreinterpretq_u32_f32(x);
			float32x4_t exponent = vcvtq_f32_s32(vsubq_s32(vreinterpretq_s32_u32(vshrq_n_u32(x_u, 23)), vdupq_n_s32(126)));
			exponent = vsubq_f32(exponent, exponent_offset);
			const float32x4_t mantissa = vreinterpretq_f32_u32(vorrq_u32(vandq_u32(x_u, vdupq_n_u32(0x007FFFFF)), vdupq_n_u32(0x3F000000)));

			// If the mantissa is below sqrt(0.5), we use 2 * mantissa and exponent - 1
			const uint32x4_t is_mantissa_small = vcltq_f32(mantissa, vdupq_n_f32(k_log_f32_sqrt_half));
			exponent = vsubq_f32(exponent, vreinterpretq_f32_u32(vandq_u32(is_mantissa_small, vreinterpretq_u32_f32(vdupq_n_f32(1.0F)))));
			const float32x4_t m = vaddq_f32(vsubq_f32(mantissa, vdupq_n_f32(1.0F)), vreinterpretq_f32_u32(vandq_u32(is_mantissa_small, vreinterpretq_u32_f32(mantissa))));

			const float32x4_t m2 = vmulq_f32(m, m);
			float32x4_t result = RTM_VECTOR4F_MULS_ADD(m, k_log_f32_c0, vdupq_n_f32(k_log_f32_c1));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c2));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c3));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c4));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c5));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c6));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c7));
			result = RTM_VECTOR4F_MULV_ADD(result, m, vdupq_n_f32(k_log_f32_c8));
			result = vmulq_f32(vmulq_f32(result, m), m2);

			result = RTM_VECTOR4F_MULS_ADD(exponent, k_f32_ln2_lo, result);
			result = RTM_VECTOR4F_MULS_ADD(m2, -0.5F, result);
			result = vaddq_f32(result, m);
			result = RTM_VECTOR4F_MULS_ADD(exponent, k_f32_ln2_hi, result);

			// Special values: log(+inf) = +inf, log(0) = -inf, log(negative) = NaN, log(NaN) = NaN
			const float32x4_t infinity = vdupq_n_f32(std::numeric_limits<float>::infinity());
			const float32x4_t zero = vdupq_n_f32(0.0F);
			result = vbslq_f32(vceqq_f32(input, infinity), infinity, result);
			result = vbslq_f32(vceqq_f32(input, zero), vdupq_n_f32(-std::numeric_limits<float>::infinity()), result);
			result = vbslq_f32(vcltq_f32(input, zero), vdupq_n_f32(std::numeric_limits<float>::quiet_NaN()), result);
			return vbslq_f32(vceqq_f32(input, input), result, input);
		}

#if defined(RTM_NEON64_INTRINSICS)
		// Returns e^input for each float64 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float64x2_t RTM_SIMD_CALL exp_f64x2(float64x2_t input) RTM_NO_EXCEPT
		{
			const float64x2_t x = vminq_f64(vmaxq_f64(input, vdupq_n_f64(k_exp_f64_min_input)), vdupq_n_f64(k_exp_f64_max_input));

			const float64x2_t round_bias = vdupq_n_f64(k_exp_f64_round_bias);
			const float64x2_t biased_n = vaddq_f64(vmulq_n_f64(x, k_exp_f64_log2e), round_bias);
			const float64x2_t n = vsubq_f64(biased_n, round_bias);
			const int64x2_t n_i = vsubq_s64(vreinterpretq_s64_f64(biased_n), vdupq_n_s64(k_exp_f64_round_bias_bits));

			float64x2_t r = vsubq_f64(x, vmulq_n_f64(n, k_exp_f64_ln2_hi));
			r = vsubq_f64(r, vmulq_n_f64(n, k_exp_f64_ln2_lo));

			const float64x2_t r2 = vmulq_f64(r, r);
			float64x2_t p = vfmaq_f64(vdupq_n_f64(k_exp_f64_p1), r2, vdupq_n_f64(k_exp_f64_p0));
			p = vfmaq_f64(vdupq_n_f64(k_exp_f64_p2), p, r2);
			p = vmulq_f64(p, r);
			float64x2_t q = vfmaq_f64(vdupq_n_f64(k_exp_f64_q1), r2, vdupq_n_f64(k_exp_f64_q0));
			q = vfmaq_f64(vdupq_n_f64(k_exp_f64_q2), q, r2);
			q = vfmaq_f64(vdupq_n_f64(k_exp_f64_q3), q, r2);
			q = vsubq_f64(q, p);
			float64x2_t result = vdivq_f64(p, q);
			result = vfmaq_f64(vdupq_n_f64(1.0), result, vdupq_n_f64(2.0));

			const int64x2_t n0 = vshrq_n_s64(n_i, 1);
			const int64x2_t n1 = vsubq_s64(n_i, n0);
			const int64x2_t exponent_bias = vdupq_n_s64(1023);
			const float64x2_t pow2_n0 = vreinterpretq_f64_s64(vshlq_n_s64(vaddq_s64(n0, exponent_bias), 52));
			const float64x2_t pow2_n1 = vreinterpretq_f64_s64(vshlq_n_s64(vaddq_s64(n1, exponent_bias), 52));
			result = vmulq_f64(vmulq_f64(result, pow2_n0), pow2_n1);

			// The result is NaN when the input is NaN
			return vbslq_f64(vceqq_f64(input, input), result, input);
		}

		// Returns the natural logarithm of each float64 lane.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float64x2_t RTM_SIMD_CALL log_f64x2(float64x2_t input) RTM_NO_EXCEPT
		{
			const uint64x2_t is_denormal = vcltq_f64(input, vdupq_n_f64((std::numeric_limits<double>::min)()));
			const float64x2_t x = vbslq_f64(is_denormal, vmulq_n_f64(input, k_log_f64_denormal_scale), input);
			const float64x2_t exponent_offset = vreinterpretq_f64_u64(vandq_u64(is_denormal, vreinterpretq_u64_f64(vdupq_n_f64(54.0))));

			// The exponent bits minus 1022 give a mantissa in the range [0.5, 1.0)
			const uint64x2_t x_u = vreinterpretq_u64_f64(x);
			const uint64x2_t exponent_bits = vorrq_u64(vshrq_n_u64(x_u, 52), vdupq_n_u64(0x4330000000000000ULL));
			float64x2_t exponent = vsubq_f64(vreinterpretq_f64_u64(exponent_bits), vdupq_n_f64(k_log_f64_exponent_bias));
			exponent = vsubq_f64(exponent, vdupq_n_f64(1022.0));
			exponent = vsubq_f64(exponent, exponent_offset);
			const float64x2_t mantissa = vreinterpretq_f64_u64(vorrq_u64(vandq_u64(x_u, vdupq_n_u64(0x000FFFFFFFFFFFFFULL)), vdupq_n_u64(0x3FE0000000000000ULL)));

			// s = 2 * (m - 1) / (m + 1)
			// If the mantissa is below sqrt(0.5), we use m = 2 * mantissa and exponent - 1
			const float64x2_t half = vdupq_n_f64(0.5);
			const uint64x2_t is_mantissa_small = vcltq_f64(mantissa, vdupq_n_f64(k_log_f64_sqrt_half));
			exponent = vsubq_f64(exponent, vreinterpretq_f64_u64(vandq_u64(is_mantissa_small, vreinterpretq_u64_f64(vdupq_n_f64(1.0)))));
			const float64x2_t z_small = vsubq_f64(mantissa, half);
			const float64x2_t y_small = vfmaq_f64(half, z_small, half);
			const float64x2_t z_large = vsubq_f64(vsubq_f64(mantissa, half), half);
			const float64x2_t y_large = vfmaq_f64(half, mantissa, half);
			const float64x2_t z = vbslq_f64(is_mantissa_small, z_small, z_large);
			const float64x2_t y = vbslq_f64(is_mantissa_small, y_small, y_large);

			const float64x2_t s = vdivq_f64(z, y);
			const float64x2_t s2 = vmulq_f64(s, s);
			float64x2_t r = vfmaq_f64(vdupq_n_f64(k_log_f64_r1), s2, vdupq_n_f64(k_log_f64_r0));
			r = vfmaq_f64(vdupq_n_f64(k_log_f64_r2), r, s2);
			float64x2_t q = vaddq_f64(s2, vdupq_n_f64(k_log_f64_s0));
			q = vfmaq_f64(vdupq_n_f64(k_log_f64_s1), q, s2);
			q = vfmaq_f64(vdupq_n_f64(k_log_f64_s2), q, s2);
			float64x2_t result = vmulq_f64(vdivq_f64(vmulq_f64(s2, r), q), s);

			result = vfmaq_f64(result, exponent, vdupq_n_f64(k_log_f64_ln2_lo));
			result = vaddq_f64(result, s);
			result = vfmaq_f64(result, exponent, vdupq_n_f64(k_log_f64_ln2_hi));

			// Special values: log(+inf) = +inf, log(0) = -inf, log(negative) = NaN, log(NaN) = NaN
			const float64x2_t infinity = vdupq_n_f64(std::numeric_limits<double>::infinity());
			const float64x2_t zero = vdupq_n_f64(0.0);
			result = vbslq_f64(vceqq_f64(input, infinity), infinity, result);
			result = vbslq_f64(vceqq_f64(input, zero), vdupq_n_f64(-std::numeric_limits<double>::infinity()), result);
			result = vbslq_f64(vcltq_f64(input, zero), vdupq_n_f64(std::numeric_limits<double>::quiet_NaN()), result);
			return vbslq_f64(vceqq_f64(input, input), result, input);
		}
#endif
#endif
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
