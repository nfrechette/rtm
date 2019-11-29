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

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// This is a helper struct to allow a single consistent API between
		// various vector types when the semantics are identical but the return
		// type differs. Implicit coercion is used to return the desired value
		// at the call site.
		//////////////////////////////////////////////////////////////////////////
		struct mask4_bool_set
		{
			inline RTM_SIMD_CALL operator mask4q() const RTM_NO_EXCEPT
			{
#if defined(RTM_SSE2_INTRINSICS)
				const uint64_t x_mask = x ? 0xFFFFFFFFFFFFFFFFULL : 0;
				const uint64_t y_mask = y ? 0xFFFFFFFFFFFFFFFFULL : 0;
				const uint64_t z_mask = z ? 0xFFFFFFFFFFFFFFFFULL : 0;
				const uint64_t w_mask = w ? 0xFFFFFFFFFFFFFFFFULL : 0;

				return mask4q{ _mm_castsi128_pd(_mm_set_epi64x(y_mask, x_mask)), _mm_castsi128_pd(_mm_set_epi64x(w_mask, z_mask)) };
#else
				const uint64_t x_mask = x ? 0xFFFFFFFFFFFFFFFFULL : 0;
				const uint64_t y_mask = y ? 0xFFFFFFFFFFFFFFFFULL : 0;
				const uint64_t z_mask = z ? 0xFFFFFFFFFFFFFFFFULL : 0;
				const uint64_t w_mask = w ? 0xFFFFFFFFFFFFFFFFULL : 0;

				return mask4q{ x_mask, y_mask, z_mask, w_mask };
#endif
			}

			inline RTM_SIMD_CALL operator mask4i() const RTM_NO_EXCEPT
			{
				const uint32_t x_mask = x ? 0xFFFFFFFFU : 0;
				const uint32_t y_mask = y ? 0xFFFFFFFFU : 0;
				const uint32_t z_mask = z ? 0xFFFFFFFFU : 0;
				const uint32_t w_mask = w ? 0xFFFFFFFFU : 0;

#if defined(RTM_SSE2_INTRINSICS)
				return _mm_castsi128_ps(_mm_set_epi32(w_mask, z_mask, y_mask, x_mask));
#elif defined(RTM_NEON_INTRINSICS)
				float32x2_t V0 = vcreate_f32(((uint64_t)x_mask) | ((uint64_t)(y_mask) << 32));
				float32x2_t V1 = vcreate_f32(((uint64_t)z_mask) | ((uint64_t)(w_mask) << 32));
				return vcombine_f32(V0, V1);
#else
				return mask4i{ x_mask, y_mask, z_mask, w_mask };
#endif
			}

			bool x;
			bool y;
			bool z;
			bool w;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Creates a mask4 from all 4 bool components.
	//////////////////////////////////////////////////////////////////////////
	constexpr rtm_impl::mask4_bool_set RTM_SIMD_CALL mask_set(bool x, bool y, bool z, bool w) RTM_NO_EXCEPT
	{
		return rtm_impl::mask4_bool_set{x, y, z, w};
	}
}

RTM_IMPL_FILE_PRAGMA_POP
