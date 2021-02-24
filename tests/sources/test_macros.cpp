////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2021 Nicholas Frechette & Realtime Math contributors
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

#include <catch2/catch.hpp>

#include <rtm/macros.h>
#include <rtm/vector4f.h>

TEST_CASE("macros", "[math][macros]")
{
	const float threshold = 0.0F;	// Result must be binary exact!

	{
		rtm::vector4f xyz0 = rtm::vector_set(1.0F, 2.0F, 3.0F);
		rtm::vector4f xyz1 = rtm::vector_set(4.0F, 5.0F, 6.0F);
		rtm::vector4f xyz2 = rtm::vector_set(7.0F, 8.0F, 9.0F);

		rtm::vector4f xxx;
		rtm::vector4f yyy;
		rtm::vector4f zzz;
		RTM_MATRIXF_TRANSPOSE_3X3(xyz0, xyz1, xyz2, xxx, yyy, zzz);

		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(1.0F, 4.0F, 7.0F), xxx, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(2.0F, 5.0F, 8.0F), yyy, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(3.0F, 6.0F, 9.0F), zzz, threshold));

		// Test when input == output
		RTM_MATRIXF_TRANSPOSE_3X3(xyz0, xyz1, xyz2, xyz0, xyz1, xyz2);

		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(1.0F, 4.0F, 7.0F), xyz0, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(2.0F, 5.0F, 8.0F), xyz1, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(3.0F, 6.0F, 9.0F), xyz2, threshold));
	}

	{
		rtm::vector4f xyzw0 = rtm::vector_set(1.0F, 2.0F, 3.0F, 20.0F);
		rtm::vector4f xyzw1 = rtm::vector_set(4.0F, 5.0F, 6.0F, 21.0F);
		rtm::vector4f xyzw2 = rtm::vector_set(7.0F, 8.0F, 9.0F, 22.0F);
		rtm::vector4f xyzw3 = rtm::vector_set(10.0F, 11.0F, 12.0F, 23.0F);

		rtm::vector4f xxxx;
		rtm::vector4f yyyy;
		rtm::vector4f zzzz;
		rtm::vector4f wwww;
		RTM_MATRIXF_TRANSPOSE_4X4(xyzw0, xyzw1, xyzw2, xyzw3, xxxx, yyyy, zzzz, wwww);

		CHECK(rtm::vector_all_near_equal(rtm::vector_set(1.0F, 4.0F, 7.0F, 10.0F), xxxx, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(2.0F, 5.0F, 8.0F, 11.0F), yyyy, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(3.0F, 6.0F, 9.0F, 12.0F), zzzz, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(20.0F, 21.0F, 22.0F, 23.0F), wwww, threshold));

		// Test when input == output
		RTM_MATRIXF_TRANSPOSE_4X4(xyzw0, xyzw1, xyzw2, xyzw3, xyzw0, xyzw1, xyzw2, xyzw3);

		CHECK(rtm::vector_all_near_equal(rtm::vector_set(1.0F, 4.0F, 7.0F, 10.0F), xyzw0, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(2.0F, 5.0F, 8.0F, 11.0F), xyzw1, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(3.0F, 6.0F, 9.0F, 12.0F), xyzw2, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(20.0F, 21.0F, 22.0F, 23.0F), xyzw3, threshold));
	}

	{
		rtm::vector4f xyz0 = rtm::vector_set(1.0F, 2.0F, 3.0F, 20.0F);
		rtm::vector4f xyz1 = rtm::vector_set(4.0F, 5.0F, 6.0F, 21.0F);
		rtm::vector4f xyz2 = rtm::vector_set(7.0F, 8.0F, 9.0F, 22.0F);
		rtm::vector4f xyz3 = rtm::vector_set(10.0F, 11.0F, 12.0F, 23.0F);

		rtm::vector4f xxxx;
		rtm::vector4f yyyy;
		rtm::vector4f zzzz;
		RTM_MATRIXF_TRANSPOSE_4X3(xyz0, xyz1, xyz2, xyz3, xxxx, yyyy, zzzz);

		CHECK(rtm::vector_all_near_equal(rtm::vector_set(1.0F, 4.0F, 7.0F, 10.0F), xxxx, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(2.0F, 5.0F, 8.0F, 11.0F), yyyy, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(3.0F, 6.0F, 9.0F, 12.0F), zzzz, threshold));

		// Test when input == output
		RTM_MATRIXF_TRANSPOSE_4X3(xyz0, xyz1, xyz2, xyz3, xyz0, xyz1, xyz2);

		CHECK(rtm::vector_all_near_equal(rtm::vector_set(1.0F, 4.0F, 7.0F, 10.0F), xyz0, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(2.0F, 5.0F, 8.0F, 11.0F), xyz1, threshold));
		CHECK(rtm::vector_all_near_equal(rtm::vector_set(3.0F, 6.0F, 9.0F, 12.0F), xyz2, threshold));
	}

	{
		rtm::vector4f xyzw0 = rtm::vector_set(1.0F, 2.0F, 3.0F, 20.0F);
		rtm::vector4f xyzw1 = rtm::vector_set(4.0F, 5.0F, 6.0F, 21.0F);
		rtm::vector4f xyzw2 = rtm::vector_set(7.0F, 8.0F, 9.0F, 22.0F);

		rtm::vector4f xxx;
		rtm::vector4f yyy;
		rtm::vector4f zzz;
		rtm::vector4f www;
		RTM_MATRIXF_TRANSPOSE_3X4(xyzw0, xyzw1, xyzw2, xxx, yyy, zzz, www);

		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(1.0F, 4.0F, 7.0F), xxx, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(2.0F, 5.0F, 8.0F), yyy, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(3.0F, 6.0F, 9.0F), zzz, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(20.0F, 21.0F, 22.0F), www, threshold));

		// Test when input == output
		RTM_MATRIXF_TRANSPOSE_3X4(xyzw0, xyzw1, xyzw2, xyzw0, xyzw1, xyzw2, xxx);

		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(1.0F, 4.0F, 7.0F), xyzw0, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(2.0F, 5.0F, 8.0F), xyzw1, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(3.0F, 6.0F, 9.0F), xyzw2, threshold));
		CHECK(rtm::vector_all_near_equal3(rtm::vector_set(20.0F, 21.0F, 22.0F), xxx, threshold));
	}
}
