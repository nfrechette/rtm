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
#include <rtm/mask4f.h>

#include <cstdint>

TEST_CASE("macros mask4f", "[math][macros][mask]")
{
	// Test all the combinations of true and false lanes
	for (uint32_t lane_bits = 0; lane_bits < 16; ++lane_bits)
	{
		const bool x = (lane_bits & 0x1) != 0;
		const bool y = (lane_bits & 0x2) != 0;
		const bool z = (lane_bits & 0x4) != 0;
		const bool w = (lane_bits & 0x8) != 0;

		const rtm::mask4f mask = rtm::mask_set(x, y, z, w);
		bool result;

		RTM_MASK4F_ALL_TRUE(mask, result);
		CHECK(result == (x && y && z && w));

		RTM_MASK4F_ALL_TRUE2(mask, result);
		CHECK(result == (x && y));

		RTM_MASK4F_ALL_TRUE3(mask, result);
		CHECK(result == (x && y && z));

		RTM_MASK4F_ANY_TRUE(mask, result);
		CHECK(result == (x || y || z || w));

		RTM_MASK4F_ANY_TRUE2(mask, result);
		CHECK(result == (x || y));

		RTM_MASK4F_ANY_TRUE3(mask, result);
		CHECK(result == (x || y || z));

#if defined(RTM_NEON_INTRINSICS)
		// With NEON, the macros also accept a float32x4_t input
		const float32x4_t mask_f32 = vreinterpretq_f32_u32(mask);

		RTM_MASK4F_ALL_TRUE(mask_f32, result);
		CHECK(result == (x && y && z && w));

		RTM_MASK4F_ANY_TRUE(mask_f32, result);
		CHECK(result == (x || y || z || w));

		// With NEON, the RTM_MASK2F macros use the [xy] lanes of a 64 bit input
		const uint32x2_t mask_xy = vget_low_u32(mask);

		RTM_MASK2F_ALL_TRUE(mask_xy, result);
		CHECK(result == (x && y));

		RTM_MASK2F_ANY_TRUE(mask_xy, result);
		CHECK(result == (x || y));

		const float32x2_t mask_xy_f32 = vreinterpret_f32_u32(mask_xy);

		RTM_MASK2F_ALL_TRUE(mask_xy_f32, result);
		CHECK(result == (x && y));

		RTM_MASK2F_ANY_TRUE(mask_xy_f32, result);
		CHECK(result == (x || y));
#endif
	}
}
