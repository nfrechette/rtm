#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2022 Nicholas Frechette & Realtime Math contributors
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

// Included only once from macros.h

#include "rtm/math.h"
#include "rtm/impl/compiler_utils.h"

RTM_IMPL_FILE_PRAGMA_PUSH

//////////////////////////////////////////////////////////////////////////
// This file contains helper macros to help improve code generation where required.
//////////////////////////////////////////////////////////////////////////

namespace rtm
{
	namespace rtm_impl
	{
#if defined(RTM_NEON_INTRINSICS)
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32x4_t RTM_SIMD_CALL cast_to_u32(uint32x4_t value) RTM_NO_EXCEPT
		{
			return value;
		}

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32x2_t RTM_SIMD_CALL cast_to_u32(uint32x2_t value) RTM_NO_EXCEPT
		{
			return value;
		}

		// MSVC has uint32x4_t and float32x4_t as aliases, we cannot have an override
#if !defined(RTM_COMPILER_MSVC)
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32x4_t RTM_SIMD_CALL cast_to_u32(float32x4_t value) RTM_NO_EXCEPT
		{
			return vreinterpretq_u32_f32(value);
		}

		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint32x2_t RTM_SIMD_CALL cast_to_u32(float32x2_t value) RTM_NO_EXCEPT
		{
			return vreinterpret_u32_f32(value);
		}
#endif
#endif
	}
}

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyzw] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f or a like type: rtm::vector4f, rtm::quatf, _m128, float32x4_t, uint32x4_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE(input_mask, output) \
		do { \
			uint8x16_t mask_rtm_impl = vreinterpretq_u8_u32(rtm::rtm_impl::cast_to_u32(input_mask)); \
			uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask_rtm_impl), vget_high_u8(mask_rtm_impl)); \
			uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0]), vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1])); \
			output = vget_lane_u32(vreinterpret_u32_u16(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0]), 0) == 0xFFFFFFFFU; \
		} while (0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyzw] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE(input_mask, output) \
		output = _mm_movemask_ps(input_mask) == 0xF
#else
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyzw] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE(input_mask, output) \
		output = input_mask.x != 0 && input_mask.y != 0 && input_mask.z != 0 && input_mask.w != 0
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f or a like type: rtm::vector4f, rtm::quatf, _m128, float32x4_t, uint32x4_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE2(input_mask, output) \
		output = vgetq_lane_u64(vreinterpretq_u64_u32(rtm::rtm_impl::cast_to_u32(input_mask)), 0) == 0xFFFFFFFFFFFFFFFFULL

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] SIMD lanes are true (aka ~0).
	// Input must be a float32x2_t or uint32x2_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK2F_ALL_TRUE(input_mask, output) \
		output = vget_lane_u64(vreinterpret_u64_u32(rtm::rtm_impl::cast_to_u32(input_mask)), 0) == 0xFFFFFFFFFFFFFFFFULL
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE2(input_mask, output) \
		output = (_mm_movemask_ps(input_mask) & 0x3) == 0x3
#else
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE2(input_mask, output) \
		output = input_mask.x != 0 && input_mask.y != 0
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f or a like type: rtm::vector4f, rtm::quatf, _m128, float32x4_t, uint32x4_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE3(input_mask, output) \
		do { \
			uint8x16_t mask_rtm_impl = vreinterpretq_u8_u32(rtm::rtm_impl::cast_to_u32(input_mask)); \
			uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask_rtm_impl), vget_high_u8(mask_rtm_impl)); \
			uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0]), vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1])); \
			output = (vget_lane_u32(vreinterpret_u32_u16(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0]), 0) & 0x00FFFFFFU) == 0x00FFFFFFU; \
		} while (0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE3(input_mask, output) \
		output = (_mm_movemask_ps(input_mask) & 0x7) == 0x7
#else
	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] SIMD lanes are true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ALL_TRUE3(input_mask, output) \
		output = input_mask.x != 0 && input_mask.y != 0 && input_mask.z != 0
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyzw] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f or a like type: rtm::vector4f, rtm::quatf, _m128, float32x4_t, uint32x4_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE(input_mask, output) \
		do { \
			uint8x16_t mask_rtm_impl = vreinterpretq_u8_u32(rtm::rtm_impl::cast_to_u32(input_mask)); \
			uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask_rtm_impl), vget_high_u8(mask_rtm_impl)); \
			uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0]), vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1])); \
			output = vget_lane_u32(vreinterpret_u32_u16(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0]), 0) != 0; \
		} while (0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyzw] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE(input_mask, output) \
		output = _mm_movemask_ps(input_mask) != 0
#else
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyzw] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE(input_mask, output) \
		output = input_mask.x != 0 || input_mask.y != 0 || input_mask.z != 0 || input_mask.w != 0
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f or a like type: rtm::vector4f, rtm::quatf, _m128, float32x4_t, uint32x4_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE2(input_mask, output) \
		output = vgetq_lane_u64(vreinterpretq_u64_u32(rtm::rtm_impl::cast_to_u32(input_mask)), 0) != 0

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] SIMD lanes is true (aka ~0).
	// Input must be a float32x2_t or uint32x2_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK2F_ANY_TRUE(input_mask, output) \
		output = vget_lane_u64(vreinterpret_u64_u32(rtm::rtm_impl::cast_to_u32(input_mask)), 0) != 0
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE2(input_mask, output) \
		output = (_mm_movemask_ps(input_mask) & 0x3) != 0
#else
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE2(input_mask, output) \
		output = input_mask.x != 0 || input_mask.y != 0
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f or a like type: rtm::vector4f, rtm::quatf, _m128, float32x4_t, uint32x4_t.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE3(input_mask, output) \
		do { \
			uint8x16_t mask_rtm_impl = vreinterpretq_u8_u32(rtm::rtm_impl::cast_to_u32(input_mask)); \
			uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(mask_rtm_impl), vget_high_u8(mask_rtm_impl)); \
			uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0]), vreinterpret_u16_u8(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1])); \
			output = (vget_lane_u32(vreinterpret_u32_u16(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0]), 0) & 0x00FFFFFFU) != 0; \
		} while (0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE3(input_mask, output) \
		output = (_mm_movemask_ps(input_mask) & 0x7) != 0
#else
	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] SIMD lanes is true (aka ~0).
	// Input must be a rtm::mask4f.
	// Output must be a bool.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MASK4F_ANY_TRUE3(input_mask, output) \
		output = input_mask.x != 0 || input_mask.y != 0 || input_mask.z != 0
#endif

RTM_IMPL_FILE_PRAGMA_POP
