////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2018 Nicholas Frechette & Animation Compression Library contributors
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
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

#include "test_vector4_impl.h"

TEST_CASE("vector4f math get/set", "[math][vector4]")
{
	test_vector4_getset_impl<float>();
}

TEST_CASE("vector4f math arithmetic", "[math][vector4]")
{
#if defined(RTM_NO_INTRINSICS)
	const float threshold = 1.0E-4F;
#else
	const float threshold = 1.0E-5F;
#endif

	test_vector4_arithmetic_impl<float>(threshold);
}

TEST_CASE("vector4f math relational", "[math][vector4]")
{
#if defined(RTM_NO_INTRINSICS)
	const float threshold = 1.0E-4F;
#else
	const float threshold = 1.0E-5F;
#endif

	test_vector4_relational_impl<float>(threshold);
}

TEST_CASE("vector4f math logical", "[math][vector4]")
{
	test_vector4_logical_impl<float>();
}

TEST_CASE("vector4f math misc", "[math][vector4]")
{
#if defined(RTM_NO_INTRINSICS)
	const float threshold = 1.0E-4F;
#else
	const float threshold = 1.0E-5F;
#endif

	test_vector4_impl<float>(threshold);

	const vector4f src = vector_set(-2.65F, 2.996113F, 0.68123521F, -5.9182F);
	const vector4d dst = vector_cast(src);
	CHECK(scalar_near_equal(vector_get_x(dst), -2.65, 1.0E-6));
	CHECK(scalar_near_equal(vector_get_y(dst), 2.996113, 1.0E-6));
	CHECK(scalar_near_equal(vector_get_z(dst), 0.68123521, 1.0E-6));
	CHECK(scalar_near_equal(vector_get_w(dst), -5.9182, 1.0E-6));

	const vector4f large_values = vector_set(1073741824.5F, 1073741824.5F, -1073741824.5F, -1073741824.5F);
	CHECK(float(vector_get_x(vector_floor(large_values))) == scalar_floor(float(vector_get_x(large_values))));
	CHECK(float(vector_get_y(vector_floor(large_values))) == scalar_floor(float(vector_get_y(large_values))));
	CHECK(float(vector_get_z(vector_floor(large_values))) == scalar_floor(float(vector_get_z(large_values))));
	CHECK(float(vector_get_w(vector_floor(large_values))) == scalar_floor(float(vector_get_w(large_values))));

	CHECK(float(vector_get_x(vector_ceil(large_values))) == scalar_ceil(float(vector_get_x(large_values))));
	CHECK(float(vector_get_y(vector_ceil(large_values))) == scalar_ceil(float(vector_get_y(large_values))));
	CHECK(float(vector_get_z(vector_ceil(large_values))) == scalar_ceil(float(vector_get_z(large_values))));
	CHECK(float(vector_get_w(vector_ceil(large_values))) == scalar_ceil(float(vector_get_w(large_values))));
}
