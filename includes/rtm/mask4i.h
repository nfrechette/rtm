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
}

RTM_IMPL_FILE_PRAGMA_POP
