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

// Deprecated in 2.5, to be removed in 2.6
// This file tests the deprecated vqv_inverse functions. Remove it with these functions.

#include "catch2.impl.h"

#include <rtm/vqvf.h>
#include <rtm/vqvd.h>

// This test uses the deprecated functions on purpose. Disable the deprecation warning.
// Do not define RTM_NO_DEPRECATION: the test must compile the deprecation attributes.
#if defined(RTM_COMPILER_MSVC)
	#pragma warning(disable : 4996)
#elif defined(RTM_COMPILER_CLANG)
	#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(RTM_COMPILER_GCC)
	#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace rtm;

template<typename TransformType, typename FloatType>
static void test_vqv_inverse_deprecated_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);

	Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
	Vector4Type test_scale1 = vector_set(FloatType(1.2));
	Vector4Type test_scale2 = vector_set(FloatType(-1.2));
	Vector4Type test_scale3 = vector_set(FloatType(1.2), FloatType(0.0), FloatType(-1.2));

	QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
	TransformType transform_a = vqv_set(rotation_around_z, x_axis, test_scale1);
	TransformType transform_b = vqv_inverse(transform_a);
	TransformType transform_ab = vqv_mul(transform_a, transform_b);
	CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
	CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
	CHECK(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));

	// With a uniform scale, the inverse is correct: it is equal to identity * inverse(transform_a)
	const TransformType transform_ref_uniform = vqv_mul_inverse(identity, transform_a);
	CHECK(quat_near_equal(transform_ref_uniform.rotation, transform_b.rotation, threshold));
	CHECK(vector_all_near_equal3(transform_ref_uniform.translation, transform_b.translation, threshold));
	CHECK(vector_all_near_equal3(transform_ref_uniform.scale, transform_b.scale, threshold));

	transform_a = vqv_set(rotation_around_z, x_axis, test_scale2);
	transform_b = vqv_inverse(transform_a);
	transform_ab = vqv_mul(transform_a, transform_b);
	CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
	CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));
	CHECK(vector_all_near_equal3(identity.scale, transform_ab.scale, threshold));

	// The function replaces each scale lane that is zero with the fallback lane
	transform_a = vqv_set(rotation_around_z, x_axis, test_scale3);
	const Vector4Type fallback_scale = vector_set(FloatType(2.0), FloatType(4.0), FloatType(0.5));
	transform_b = vqv_inverse(transform_a, fallback_scale);
	CHECK(vqv_is_finite(transform_b));

	const TransformType transform_ref = vqv_inverse(vqv_set(rotation_around_z, x_axis, vector_set(FloatType(1.2), FloatType(4.0), FloatType(-1.2))));
	CHECK(quat_near_equal(transform_ref.rotation, transform_b.rotation, threshold));
	CHECK(vector_all_near_equal3(transform_ref.translation, transform_b.translation, threshold));
	CHECK(vector_all_near_equal3(transform_ref.scale, transform_b.scale, threshold));
}

TEST_CASE("vqv deprecated API", "[math][vqv]")
{
	test_vqv_inverse_deprecated_impl<vqvf, float>(vqv_identity(), 1.0E-4F);
	test_vqv_inverse_deprecated_impl<vqvd, double>(vqv_identity(), 1.0E-6);
}
