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

#include "catch2.impl.h"

#include <rtm/macros.h>
#include <rtm/mask4d.h>
#include <rtm/mask4f.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>

TEST_CASE("macros vector4f", "[math][macros][vector4]")
{
	const float threshold = 0.0F;	// Result must be binary exact!

	// The products and the sums of these values are exact in float32
	const rtm::vector4f v0 = rtm::vector_set(1.0F, -2.0F, 3.5F, 4.0F);
	const rtm::vector4f v1 = rtm::vector_set(-0.5F, 3.0F, 2.0F, 0.25F);
	const rtm::vector4f v2 = rtm::vector_set(10.0F, 20.0F, -30.0F, 40.0F);
	const float s1 = 1.5F;

	{
		const rtm::vector4f result = RTM_VECTOR4F_MULV_ADD(v0, v1, v2);
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(9.5F, 14.0F, -23.0F, 41.0F), result, threshold));
	}

	{
		const rtm::vector4f result = RTM_VECTOR4F_MULS_ADD(v0, s1, v2);
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(11.5F, 17.0F, -24.75F, 46.0F), result, threshold));
	}

	{
		const rtm::vector4f result = RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2);
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(10.5F, 26.0F, -37.0F, 39.0F), result, threshold));
	}

	{
		const rtm::vector4f result = RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2);
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(8.5F, 23.0F, -35.25F, 34.0F), result, threshold));
	}

#if defined(RTM_NEON_INTRINSICS)
	{
		// RTM_VECTOR2F_MULV_ADD is only available with NEON
		const float32x2_t result = RTM_VECTOR2F_MULV_ADD(vget_low_f32(v0), vget_low_f32(v1), vget_low_f32(v2));
		CHECK(vget_lane_f32(result, 0) == 9.5F);
		CHECK(vget_lane_f32(result, 1) == 14.0F);
	}
#endif

#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS)
	{
		// RTM_VECTOR4F_SELECT is not available with the scalar code path
		const rtm::mask4f mask = rtm::mask_set(true, false, false, true);
		const rtm::vector4f result = RTM_VECTOR4F_SELECT(mask, v0, v2);
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(1.0F, 20.0F, -30.0F, 4.0F), result, threshold));
	}
#endif

#if defined(RTM_SSE2_INTRINSICS)
	{
		// RTM_VECTOR4F_MAKE is only available with SSE2
		constexpr __m128 result = RTM_VECTOR4F_MAKE(1.0F, -2.0F, 3.5F, 4.0F);
		CHECK(rtm::vector_all_near_equal(v0, result, threshold));
	}
#endif
}

TEST_CASE("macros vector4d", "[math][macros][vector4]")
{
#if defined(RTM_SSE2_INTRINSICS)
	const double threshold = 0.0;	// Result must be binary exact!

	const rtm::vector4d v0 = rtm::vector_set(1.0, -2.0, 3.5, 4.0);
	const rtm::vector4d v2 = rtm::vector_set(10.0, 20.0, -30.0, 40.0);

	{
		// RTM_VECTOR2D_MAKE is only available with SSE2
		constexpr __m128d xy = RTM_VECTOR2D_MAKE(1.0, -2.0);
		constexpr __m128d zw = RTM_VECTOR2D_MAKE(3.5, 4.0);
		const rtm::vector4d result = rtm::vector4d{ xy, zw };
		CHECK(rtm::vector_all_near_equal(v0, result, threshold));
	}

	{
		// RTM_VECTOR2D_SELECT is only available with SSE2
		const rtm::mask4d mask = rtm::mask_set(true, false, false, true);
		const __m128d xy = RTM_VECTOR2D_SELECT(mask.xy, v0.xy, v2.xy);
		const __m128d zw = RTM_VECTOR2D_SELECT(mask.zw, v0.zw, v2.zw);
		const rtm::vector4d result = rtm::vector4d{ xy, zw };
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(1.0, 20.0, -30.0, 4.0), result, threshold));
	}
#else
	// The float64 vector macros are only available with SSE2
	SUCCEED();
#endif
}
