#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2020 Nicholas Frechette & Realtime Math contributors
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
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/mask_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4f [x] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32_t RTM_SIMD_CALL mask_get_x(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(input));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(vreinterpretq_u32_f32(input), 0);
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4f [y] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32_t RTM_SIMD_CALL mask_get_y(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(_mm_shuffle_ps(input, input, _MM_SHUFFLE(1, 1, 1, 1))));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(vreinterpretq_u32_f32(input), 1);
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4f [z] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32_t RTM_SIMD_CALL mask_get_z(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(_mm_shuffle_ps(input, input, _MM_SHUFFLE(2, 2, 2, 2))));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(vreinterpretq_u32_f32(input), 2);
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4f [w] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32_t RTM_SIMD_CALL mask_get_w(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 3, 3, 3))));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(vreinterpretq_u32_f32(input), 3);
#else
		return input.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are true, otherwise false: all(input.xyzw != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_true(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(input) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vreinterpretq_u32_f32(input);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFU;
#else
		return input.x != 0 && input.y != 0 && input.z != 0 && input.w != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are true, otherwise false: all(input.xy != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_true2(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x3) == 0x3;
#elif defined(RTM_NEON_INTRINSICS)
		return vget_lane_u64(vget_low_u32(vreinterpretq_u32_f32(input)), 0) == 0xFFFFFFFFFFFFFFFFULL;
#else
		return input.x != 0 && input.y != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are true, otherwise false: all(input.xyz != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_true3(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vreinterpretq_u32_f32(input);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) == 0x00FFFFFFU;
#else
		return input.x != 0 && input.y != 0 && input.z != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are true, otherwise false: any(input.xyzw != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_true(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(input) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vreinterpretq_u32_f32(input);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) != 0;
#else
		return input.x != 0 || input.y != 0 || input.z != 0 || input.w != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are true, otherwise false: any(input.xy != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_true2(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x3) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		return vget_lane_u64(vget_low_u32(vreinterpretq_u32_f32(input)), 0) != 0;
#else
		return input.x != 0 || input.y != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are true, otherwise false: any(input.xyz != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_true3(mask4f_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vreinterpretq_u32_f32(input);
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) != 0;
#else
		return input.x != 0 || input.y != 0 || input.z != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are equal, otherwise false: all(lhs.xyzw == rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_equal(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmpeq_ps(lhs, rhs)) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vceqq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFU;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 4) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are equal, otherwise false: all(lhs.xy == rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_equal2(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpeq_ps(lhs, rhs)) & 0x3) == 0x3;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vceq_u32(vget_low_u32(vreinterpretq_u32_f32(lhs)), vget_low_u32(vreinterpretq_u32_f32(rhs)));
		return vget_lane_u64(mask, 0) == 0xFFFFFFFFFFFFFFFFu;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 2) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are equal, otherwise false: all(lhs.xyz == rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_equal3(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpeq_ps(lhs, rhs)) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vceqq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) == 0x00FFFFFFU;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 3) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are equal, otherwise false: any(lhs.xyzw == rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_equal(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(_mm_cmpeq_ps(lhs, rhs)) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vceqq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) != 0;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 4) != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are equal, otherwise false: any(lhs.xy == rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_equal2(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpeq_ps(lhs, rhs)) & 0x3) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x2_t mask = vceq_u32(vget_low_u32(vreinterpretq_u32_f32(lhs)), vget_low_u32(vreinterpretq_u32_f32(rhs)));
		return vget_lane_u64(mask, 0) != 0;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 2) != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are equal, otherwise false: any(lhs.xyz == rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_equal3(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(_mm_cmpeq_ps(lhs, rhs)) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint32x4_t mask = vceqq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs));
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask), vget_high_u8(mask));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) != 0;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint32_t) * 3) != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical AND between the inputs: lhs & rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4f RTM_SIMD_CALL mask_and(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_and_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs)));
#else
		const uint32_t* lhs_ = reinterpret_cast<const uint32_t*>(&lhs);
		const uint32_t* rhs_ = reinterpret_cast<const uint32_t*>(&rhs);

		union
		{
			mask4f vector;
			uint32_t scalar[4];
		} result;

		result.scalar[0] = lhs_[0] & rhs_[0];
		result.scalar[1] = lhs_[1] & rhs_[1];
		result.scalar[2] = lhs_[2] & rhs_[2];
		result.scalar[3] = lhs_[3] & rhs_[3];

		return result.vector;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical OR between the inputs: lhs | rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4f RTM_SIMD_CALL mask_or(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_or_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs)));
#else
		const uint32_t* lhs_ = reinterpret_cast<const uint32_t*>(&lhs);
		const uint32_t* rhs_ = reinterpret_cast<const uint32_t*>(&rhs);

		union
		{
			mask4f vector;
			uint32_t scalar[4];
		} result;

		result.scalar[0] = lhs_[0] | rhs_[0];
		result.scalar[1] = lhs_[1] | rhs_[1];
		result.scalar[2] = lhs_[2] | rhs_[2];
		result.scalar[3] = lhs_[3] | rhs_[3];

		return result.vector;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical XOR between the inputs: lhs ^ rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4f RTM_SIMD_CALL mask_xor(mask4f_arg0 lhs, mask4f_arg1 rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_xor_ps(lhs, rhs);
#elif defined(RTM_NEON_INTRINSICS)
		return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(lhs), vreinterpretq_u32_f32(rhs)));
#else
		const uint32_t* lhs_ = reinterpret_cast<const uint32_t*>(&lhs);
		const uint32_t* rhs_ = reinterpret_cast<const uint32_t*>(&rhs);

		union
		{
			mask4f vector;
			uint32_t scalar[4];
		} result;

		result.scalar[0] = lhs_[0] ^ rhs_[0];
		result.scalar[1] = lhs_[1] ^ rhs_[1];
		result.scalar[2] = lhs_[2] ^ rhs_[2];
		result.scalar[3] = lhs_[3] ^ rhs_[3];

		return result.vector;
#endif
	}
}

RTM_IMPL_FILE_PRAGMA_POP
