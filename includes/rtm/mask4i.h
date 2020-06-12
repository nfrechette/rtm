#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Realtime Math contributors
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
	// Creates a mask4i from all 4 integer components.
	//////////////////////////////////////////////////////////////////////////
	inline mask4i RTM_SIMD_CALL mask_set(uint32_t x, uint32_t y, uint32_t z, uint32_t w) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_castsi128_ps(_mm_set_epi32(w, z, y, x));
#elif defined(RTM_NEON_INTRINSICS)
		float32x2_t V0 = vcreate_f32(((uint64_t)x) | ((uint64_t)(y) << 32));
		float32x2_t V1 = vcreate_f32(((uint64_t)z) | ((uint64_t)(w) << 32));
		return vcombine_f32(V0, V1);
#else
		return mask4i{ x, y, z, w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4i [x] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint32_t RTM_SIMD_CALL mask_get_x(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(input));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(input, 0);
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4i [y] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint32_t RTM_SIMD_CALL mask_get_y(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(_mm_shuffle_ps(input, input, _MM_SHUFFLE(1, 1, 1, 1))));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(input, 1);
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4i [z] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint32_t RTM_SIMD_CALL mask_get_z(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(_mm_shuffle_ps(input, input, _MM_SHUFFLE(2, 2, 2, 2))));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(input, 2);
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4i [w] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint32_t RTM_SIMD_CALL mask_get_w(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_cvtsi128_si32(_mm_castps_si128(_mm_shuffle_ps(input, input, _MM_SHUFFLE(3, 3, 3, 3))));
#elif defined(RTM_NEON_INTRINSICS)
		return vgetq_lane_u32(input, 3);
#else
		return input.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are true, otherwise false: all(input != 0)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL mask_all_true(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(input) == 0xF;
#elif defined(RTM_NEON_INTRINSICS)
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(input), vget_high_u8(input));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) == 0xFFFFFFFFU;
#else
		return input.x != 0 && input.y != 0 && input.z != 0 && input.w != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are true, otherwise false: all(input != 0)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL mask_all_true2(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x3) == 0x3;
#elif defined(RTM_NEON_INTRINSICS)
		return vget_lane_u64(input, 0) == 0xFFFFFFFFFFFFFFFFULL;
#else
		return input.x != 0 && input.y != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are true, otherwise false: all(input != 0)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL mask_all_true3(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x7) == 0x7;
#elif defined(RTM_NEON_INTRINSICS)
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(input), vget_high_u8(input));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) == 0x00FFFFFFU;
#else
		return input.x != 0 && input.y != 0 && input.z != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are true, otherwise false: any(input != 0)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL mask_any_true(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_ps(input) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(input), vget_high_u8(input));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) != 0;
#else
		return input.x != 0 || input.y != 0 || input.z != 0 || input.w != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are true, otherwise false: any(input != 0)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL mask_any_true2(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x3) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		return vget_lane_u64(input, 0) != 0;
#else
		return input.x != 0 || input.y != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are true, otherwise false: any(input != 0)
	//////////////////////////////////////////////////////////////////////////
	inline bool RTM_SIMD_CALL mask_any_true3(mask4i_arg0 input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_ps(input) & 0x7) != 0;
#elif defined(RTM_NEON_INTRINSICS)
		uint8x8x2_t mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15 = vzip_u8(vget_low_u8(input), vget_high_u8(input));
		uint16x4x2_t mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15 = vzip_u16(mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[0], mask_0_8_1_9_2_10_3_11_4_12_5_13_6_14_7_15.val[1]);
		return (vget_lane_u32(mask_0_8_4_12_1_9_5_13_2_10_6_14_3_11_7_15.val[0], 0) & 0x00FFFFFFU) != 0;
#else
		return input.x != 0 || input.y != 0 || input.z != 0;
#endif
	}
}

RTM_IMPL_FILE_PRAGMA_POP
