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
	// Creates a mask4q from all 4 integer components.
	//////////////////////////////////////////////////////////////////////////
	inline mask4q RTM_SIMD_CALL mask_set(uint64_t x, uint64_t y, uint64_t z, uint64_t w) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return mask4q{ _mm_castsi128_pd(_mm_set_epi64x(y, x)), _mm_castsi128_pd(_mm_set_epi64x(w, z)) };
#else
		return mask4q{ x, y, z, w };
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4q [x] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint64_t RTM_SIMD_CALL mask_get_x(const mask4q& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(_M_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(input.xy));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(input.xy));
#endif
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4q [y] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint64_t RTM_SIMD_CALL mask_get_y(const mask4q& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(_M_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(_mm_shuffle_pd(input.xy, input.xy, 1)));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(_mm_shuffle_pd(input.xy, input.xy, 1)));
#endif
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4q [z] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint64_t RTM_SIMD_CALL mask_get_z(const mask4q& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(_M_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(input.zw));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(input.zw));
#endif
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4q [w] component.
	//////////////////////////////////////////////////////////////////////////
	inline uint64_t RTM_SIMD_CALL mask_get_w(const mask4q& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(_M_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(_mm_shuffle_pd(input.zw, input.zw, 1)));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(_mm_shuffle_pd(input.zw, input.zw, 1)));
#endif
#else
		return input.w;
#endif
	}
}

RTM_IMPL_FILE_PRAGMA_POP
