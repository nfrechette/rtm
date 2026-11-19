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
#include "rtm/vector4f.h"
#include "rtm/version.h"
#include "rtm/impl/bit_cast.impl.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/detect_features.h"

#include <cstdint>
#include <cstring>

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	namespace rtm_impl
	{
		// Bit layout of a half-precision (IEEE 754 binary16) value:
		// [15] sign, [14:10] exponent with a bias of 15, [9:0] mantissa
		//
		// The SIMD code paths without a native conversion use the same steps as these scalar functions,
		// but they compute all the cases and then select the result of each lane.

		// Converts a float32 value to a float16 value. The function rounds to the nearest value, ties to even.
		// It uses only integer operations, thus the result does not depend on the floating point environment.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline uint16_t half_from_float(float input) RTM_NO_EXCEPT
		{
			uint32_t input_u32;
			std::memcpy(&input_u32, &input, sizeof(float));

			const uint32_t sign = (input_u32 >> 16) & 0x8000U;
			const uint32_t abs_u32 = input_u32 & 0x7FFFFFFFU;

			uint32_t result;
			if (abs_u32 >= 0x47800000U)
			{
				// The value is 65536.0 or more, an infinity, or a NaN
				// A NaN keeps the high bits of its payload and becomes a quiet NaN
				// A value that is too large becomes an infinity
				result = abs_u32 > 0x7F800000U ? (0x7E00U | ((abs_u32 >> 13) & 0x03FFU)) : 0x7C00U;
			}
			else if (abs_u32 < 0x38800000U)
			{
				// The value is less than 2^-14, the result is a subnormal half or zero
				const uint32_t exponent = abs_u32 >> 23;
				if (exponent < 102U)
				{
					// The value is less than 2^-25, half of the smallest subnormal half
					result = 0;
				}
				else
				{
					// Shift the mantissa with its implicit bit to align it with the subnormal mantissa
					// The exponent is in [102, 112], thus the shift is in [14, 24]
					const uint32_t mantissa = (abs_u32 & 0x007FFFFFU) | 0x00800000U;
					const uint32_t shift = 126U - exponent;
					const uint32_t remainder = mantissa & ((1U << shift) - 1U);
					const uint32_t halfway = 1U << (shift - 1U);

					result = mantissa >> shift;
					if (remainder > halfway || (remainder == halfway && (result & 1U) != 0))
						result++;
				}
			}
			else
			{
				// The result is a normal half or an infinity
				// Change the exponent bias from 127 to 15 and add the rounding bias: 0xFFF, plus 1 when the result mantissa is odd
				// When the rounding moves the mantissa into the exponent, the result is correct, also for an infinity
				const uint32_t mantissa_odd = (abs_u32 >> 13) & 1U;
				result = (abs_u32 + 0xC8000FFFU + mantissa_odd) >> 13;
			}

			return static_cast<uint16_t>(sign | result);
		}

		// Converts a float16 value to a float32 value. The result is exact.
		// A signaling NaN becomes a quiet NaN, as it does with F16C and NEON.
		RTM_DISABLE_SECURITY_COOKIE_CHECK inline float half_to_float(uint16_t input) RTM_NO_EXCEPT
		{
			const uint32_t input_u32 = input;
			const uint32_t sign = (input_u32 & 0x8000U) << 16;
			const uint32_t exponent = (input_u32 >> 10) & 0x1FU;
			const uint32_t mantissa = input_u32 & 0x03FFU;

			uint32_t result_u32;
			if (exponent == 0x1FU)
			{
				// Infinity or NaN
				const uint32_t quiet_bit = mantissa != 0 ? 0x00400000U : 0;
				result_u32 = sign | 0x7F800000U | quiet_bit | (mantissa << 13);
			}
			else if (exponent == 0)
			{
				// Zero or subnormal: the value is mantissa * 2^-24, the multiplication is exact
				const float abs_result = static_cast<float>(mantissa) * 5.9604644775390625E-8F;
				std::memcpy(&result_u32, &abs_result, sizeof(float));
				result_u32 |= sign;
			}
			else
			{
				// Normal: change the exponent bias from 15 to 127
				result_u32 = sign | ((exponent + 112U) << 23) | (mantissa << 13);
			}

			float result;
			std::memcpy(&result, &result_u32, sizeof(float));
			return result;
		}
	}

	/// Converts the four components of a vector4 to half-precision (IEEE 754 binary16) values
	/// and writes them to memory. The output must point to four values. It needs only the alignment of uint16_t.
	/// The function rounds to the nearest value, ties to even. On ARM, the hardware uses the rounding mode
	/// of the FPCR register, and the default mode is also to the nearest value, ties to even.
	/// A value that is too large for a half becomes an infinity with the same sign.
	/// A NaN stays a NaN, but its payload can change.
	/// x64 and x86 use F16C when it is enabled (for example with -mf16c, or with /arch:AVX2 on MSVC).
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE void RTM_SIMD_CALL vector_pack_half(vector4f_arg0 input, uint16_t* output) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(RTM_IMPL_F16C_SUPPORTED)
		// The immediate value 0 selects round to nearest even, and the MXCSR rounding mode does not apply
		const __m128i result = _mm_cvtps_ph(input, 0);
		_mm_storel_epi64(rtm_impl::bit_cast<__m128i*>(output), result);
#else
		const __m128i input_u32 = _mm_castps_si128(input);
		const __m128i sign = _mm_and_si128(input_u32, _mm_set1_epi32(static_cast<int32_t>(0x80000000U)));
		const __m128i abs_u32 = _mm_xor_si128(input_u32, sign);

		// Normal: change the exponent bias and round to nearest even, see rtm_impl::half_from_float
		const __m128i mantissa_odd = _mm_and_si128(_mm_srli_epi32(abs_u32, 13), _mm_set1_epi32(1));
		const __m128i normal = _mm_srli_epi32(_mm_add_epi32(_mm_add_epi32(abs_u32, _mm_set1_epi32(static_cast<int32_t>(0xC8000FFFU))), mantissa_odd), 13);

		// Subnormal: the addition of 0.5 aligns the subnormal mantissa with the low bits and rounds to nearest even
		// The sum is a normal value, thus the flush-to-zero and denormals-are-zero modes do not change the result
		const __m128i subnormal = _mm_sub_epi32(_mm_castps_si128(_mm_add_ps(_mm_castsi128_ps(abs_u32), _mm_set1_ps(0.5F))), _mm_set1_epi32(0x3F000000));

		// Infinity and NaN
		const __m128i is_nan = _mm_cmpgt_epi32(abs_u32, _mm_set1_epi32(0x7F800000));
		const __m128i nan_payload = _mm_or_si128(_mm_set1_epi32(0x0200), _mm_and_si128(_mm_srli_epi32(abs_u32, 13), _mm_set1_epi32(0x03FF)));
		const __m128i inf_nan = _mm_or_si128(_mm_set1_epi32(0x7C00), _mm_and_si128(is_nan, nan_payload));

		// The sign bit is clear, thus the signed comparisons are correct
		const __m128i is_inf_nan = _mm_cmpgt_epi32(abs_u32, _mm_set1_epi32(0x477FFFFF));
		const __m128i is_subnormal = _mm_cmplt_epi32(abs_u32, _mm_set1_epi32(0x38800000));

		__m128i result = _mm_or_si128(_mm_and_si128(is_subnormal, subnormal), _mm_andnot_si128(is_subnormal, normal));
		result = _mm_or_si128(_mm_and_si128(is_inf_nan, inf_nan), _mm_andnot_si128(is_inf_nan, result));
		result = _mm_or_si128(result, _mm_srli_epi32(sign, 16));

		// Sign extend the low 16 bits of each lane, then the signed saturation does not change the values
		result = _mm_srai_epi32(_mm_slli_epi32(result, 16), 16);
		result = _mm_packs_epi32(result, result);
		_mm_storel_epi64(rtm_impl::bit_cast<__m128i*>(output), result);
#endif
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t sign = wasm_v128_and(input, wasm_u32x4_splat(0x80000000U));
		const v128_t abs_u32 = wasm_v128_xor(input, sign);

		// Normal: change the exponent bias and round to nearest even, see rtm_impl::half_from_float
		const v128_t mantissa_odd = wasm_v128_and(wasm_u32x4_shr(abs_u32, 13), wasm_u32x4_splat(1));
		const v128_t normal = wasm_u32x4_shr(wasm_i32x4_add(wasm_i32x4_add(abs_u32, wasm_u32x4_splat(0xC8000FFFU)), mantissa_odd), 13);

		// Subnormal: the addition of 0.5 aligns the subnormal mantissa with the low bits and rounds to nearest even
		const v128_t subnormal = wasm_i32x4_sub(wasm_f32x4_add(abs_u32, wasm_f32x4_splat(0.5F)), wasm_u32x4_splat(0x3F000000U));

		// Infinity and NaN
		const v128_t is_nan = wasm_u32x4_gt(abs_u32, wasm_u32x4_splat(0x7F800000U));
		const v128_t nan_payload = wasm_v128_or(wasm_u32x4_splat(0x0200U), wasm_v128_and(wasm_u32x4_shr(abs_u32, 13), wasm_u32x4_splat(0x03FFU)));
		const v128_t inf_nan = wasm_v128_or(wasm_u32x4_splat(0x7C00U), wasm_v128_and(is_nan, nan_payload));

		const v128_t is_inf_nan = wasm_u32x4_ge(abs_u32, wasm_u32x4_splat(0x47800000U));
		const v128_t is_subnormal = wasm_u32x4_lt(abs_u32, wasm_u32x4_splat(0x38800000U));

		v128_t result = wasm_v128_bitselect(subnormal, normal, is_subnormal);
		result = wasm_v128_bitselect(inf_nan, result, is_inf_nan);
		result = wasm_v128_or(result, wasm_u32x4_shr(sign, 16));

		result = wasm_i16x8_shuffle(result, result, 0, 2, 4, 6, 0, 2, 4, 6);
		wasm_v128_store64_lane(output, result, 0);
#elif defined(RTM_NEON_INTRINSICS) && defined(RTM_IMPL_NEON_FP16_CONVERSION_SUPPORTED)
		vst1_u16(output, vreinterpret_u16_f16(vcvt_f16_f32(input)));
#else
		output[0] = rtm_impl::half_from_float(vector_get_x(input));
		output[1] = rtm_impl::half_from_float(vector_get_y(input));
		output[2] = rtm_impl::half_from_float(vector_get_z(input));
		output[3] = rtm_impl::half_from_float(vector_get_w(input));
#endif
	}

	/// Reads four half-precision (IEEE 754 binary16) values from memory and converts them to a vector4.
	/// The input must point to four values. It needs only the alignment of uint16_t.
	/// The conversion is exact. A NaN stays a NaN, but its payload can change.
	/// x64 and x86 use F16C when it is enabled (for example with -mf16c, or with /arch:AVX2 on MSVC).
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL vector_unpack_half(const uint16_t* input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(RTM_IMPL_F16C_SUPPORTED)
		return _mm_cvtph_ps(_mm_loadl_epi64(rtm_impl::bit_cast<const __m128i*>(input)));
#else
		const __m128i input_u32 = _mm_unpacklo_epi16(_mm_loadl_epi64(rtm_impl::bit_cast<const __m128i*>(input)), _mm_setzero_si128());
		const __m128i sign = _mm_slli_epi32(_mm_and_si128(input_u32, _mm_set1_epi32(0x8000)), 16);
		const __m128i exponent_mantissa = _mm_slli_epi32(_mm_and_si128(input_u32, _mm_set1_epi32(0x7FFF)), 13);
		const __m128i exponent = _mm_and_si128(exponent_mantissa, _mm_set1_epi32(0x0F800000));

		const __m128i is_inf_nan = _mm_cmpeq_epi32(exponent, _mm_set1_epi32(0x0F800000));
		const __m128i is_nan = _mm_cmpgt_epi32(exponent_mantissa, _mm_set1_epi32(0x0F800000));
		const __m128i is_subnormal = _mm_cmpeq_epi32(exponent, _mm_setzero_si128());

		// Normal: change the exponent bias from 15 to 127
		// Infinity and NaN: add the bias again to set all the exponent bits, and set the quiet bit of a NaN
		__m128i normal = _mm_add_epi32(exponent_mantissa, _mm_set1_epi32(0x38000000));
		normal = _mm_add_epi32(normal, _mm_and_si128(is_inf_nan, _mm_set1_epi32(0x38000000)));
		normal = _mm_or_si128(normal, _mm_and_si128(is_nan, _mm_set1_epi32(0x00400000)));

		// Subnormal: (2^-14 + mantissa * 2^-24) - 2^-14
		// All the values are normal, thus the flush-to-zero and denormals-are-zero modes do not change the result
		const __m128 subnormal_bias = _mm_castsi128_ps(_mm_set1_epi32(0x38800000));
		const __m128 subnormal = _mm_sub_ps(_mm_castsi128_ps(_mm_add_epi32(exponent_mantissa, _mm_castps_si128(subnormal_bias))), subnormal_bias);

		const __m128i result = _mm_or_si128(_mm_and_si128(is_subnormal, _mm_castps_si128(subnormal)), _mm_andnot_si128(is_subnormal, normal));
		return _mm_castsi128_ps(_mm_or_si128(result, sign));
#endif
#elif defined(RTM_WASM128_INTRINSICS)
		const v128_t input_u32 = wasm_u32x4_extend_low_u16x8(wasm_v128_load64_zero(input));
		const v128_t sign = wasm_i32x4_shl(wasm_v128_and(input_u32, wasm_u32x4_splat(0x8000U)), 16);
		const v128_t exponent_mantissa = wasm_i32x4_shl(wasm_v128_and(input_u32, wasm_u32x4_splat(0x7FFFU)), 13);
		const v128_t exponent = wasm_v128_and(exponent_mantissa, wasm_u32x4_splat(0x0F800000U));

		const v128_t is_inf_nan = wasm_i32x4_eq(exponent, wasm_u32x4_splat(0x0F800000U));
		const v128_t is_nan = wasm_u32x4_gt(exponent_mantissa, wasm_u32x4_splat(0x0F800000U));
		const v128_t is_subnormal = wasm_i32x4_eq(exponent, wasm_u32x4_splat(0));

		// Normal: change the exponent bias from 15 to 127
		// Infinity and NaN: add the bias again to set all the exponent bits, and set the quiet bit of a NaN
		v128_t normal = wasm_i32x4_add(exponent_mantissa, wasm_u32x4_splat(0x38000000U));
		normal = wasm_i32x4_add(normal, wasm_v128_and(is_inf_nan, wasm_u32x4_splat(0x38000000U)));
		normal = wasm_v128_or(normal, wasm_v128_and(is_nan, wasm_u32x4_splat(0x00400000U)));

		// Subnormal: (2^-14 + mantissa * 2^-24) - 2^-14
		const v128_t subnormal_bias = wasm_u32x4_splat(0x38800000U);
		const v128_t subnormal = wasm_f32x4_sub(wasm_i32x4_add(exponent_mantissa, subnormal_bias), subnormal_bias);

		const v128_t result = wasm_v128_bitselect(subnormal, normal, is_subnormal);
		return wasm_v128_or(result, sign);
#elif defined(RTM_NEON_INTRINSICS) && defined(RTM_IMPL_NEON_FP16_CONVERSION_SUPPORTED)
		return vcvt_f32_f16(vreinterpret_f16_u16(vld1_u16(input)));
#else
		return vector_set(rtm_impl::half_to_float(input[0]), rtm_impl::half_to_float(input[1]), rtm_impl::half_to_float(input[2]), rtm_impl::half_to_float(input[3]));
#endif
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
