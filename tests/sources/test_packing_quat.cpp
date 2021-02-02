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

#include <catch2/catch.hpp>

#include <rtm/type_traits.h>
#include <rtm/packing/quatf.h>
#include <rtm/packing/quatd.h>

using namespace rtm;

template<typename FloatType>
static void test_quat_impl(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;

	{
		QuatType quat0 = quat_set(FloatType(0.39564531008956383), FloatType(0.044254239301713752), FloatType(0.22768840967675355), FloatType(-0.88863059760894492));
		QuatType quat1 = quat_ensure_positive_w(quat0);
		QuatType quat2 = quat_ensure_positive_w(quat1);

		CHECK(FloatType(quat_get_x(quat0)) == -FloatType(quat_get_x(quat1)));
		CHECK(FloatType(quat_get_y(quat0)) == -FloatType(quat_get_y(quat1)));
		CHECK(FloatType(quat_get_z(quat0)) == -FloatType(quat_get_z(quat1)));
		CHECK(FloatType(quat_get_w(quat0)) == -FloatType(quat_get_w(quat1)));

		CHECK(FloatType(quat_get_x(quat2)) == FloatType(quat_get_x(quat1)));
		CHECK(FloatType(quat_get_y(quat2)) == FloatType(quat_get_y(quat1)));
		CHECK(FloatType(quat_get_z(quat2)) == FloatType(quat_get_z(quat1)));
		CHECK(FloatType(quat_get_w(quat2)) == FloatType(quat_get_w(quat1)));

		Vector4Type vec1 = quat_to_vector(quat1);
		QuatType quat3 = quat_from_positive_w(vec1);
		CHECK(FloatType(quat_get_x(quat1)) == FloatType(quat_get_x(quat3)));
		CHECK(FloatType(quat_get_y(quat1)) == FloatType(quat_get_y(quat3)));
		CHECK(FloatType(quat_get_z(quat1)) == FloatType(quat_get_z(quat3)));
		CHECK(scalar_near_equal(FloatType(quat_get_w(quat1)), FloatType(quat_get_w(quat3)), threshold));
	}
}

TEST_CASE("quatf packing math", "[math][quat][packing]")
{
	test_quat_impl<float>(1.0E-4F);
}

TEST_CASE("quatd packing math", "[math][quat][packing]")
{
	test_quat_impl<double>(1.0E-6);
}
