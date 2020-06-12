////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2020 Nicholas Frechette & Realtime Math contributors
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

#include "test_matrix3x3_impl.h"

TEST_CASE("matrix3x3f math get/set", "[math][matrix3x3]")
{
	test_matrix3x3_setters<float>(1.0E-4F);
}

TEST_CASE("matrix3x3f math arithmetic", "[math][matrix3x3]")
{
	test_matrix3x3_arithmetic<float>(1.0E-4F);
}

TEST_CASE("matrix3x3f math transformations", "[math][matrix3x3]")
{
	test_matrix3x3_transformations<float>(1.0E-4F);
}

TEST_CASE("matrix3x3f math misc", "[math][matrix3x3]")
{
	test_matrix3x3_misc<float>(1.0E-4F);

	{
		quatf rotation_around_z = quat_from_euler(scalar_deg_to_rad(0.0F), scalar_deg_to_rad(90.0F), scalar_deg_to_rad(0.0F));
		matrix3x3f src = matrix_from_quat(rotation_around_z);

		// Work around clang5 compiler crash by marking this variable as static
		static matrix3x3d dst = matrix_cast(src);
		CHECK(vector_all_near_equal3(vector_cast(src.x_axis), dst.x_axis, 1.0E-4));
		CHECK(vector_all_near_equal3(vector_cast(src.y_axis), dst.y_axis, 1.0E-4));
		CHECK(vector_all_near_equal3(vector_cast(src.z_axis), dst.z_axis, 1.0E-4));
	}
}
