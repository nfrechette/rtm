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
// This file tests the deprecated vqv_inverse and vqv_*_no_scale functions. Remove it with these functions.

#include "catch2.impl.h"

#include <rtm/type_traits.h>
#include <rtm/vqf.h>
#include <rtm/vqd.h>
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

template<typename TransformType, typename FloatType>
static void test_vqv_no_scale_deprecated_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = decltype(TransformType::rotation);
	using Vector4Type = decltype(TransformType::translation);
	using ScalarType = typename related_types<FloatType>::scalar;

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		// The scale is not 1.0, thus a function that applies the scale fails
		const Vector4Type ignored_scale_a = vector_set(FloatType(2.5), FloatType(0.5), FloatType(-1.5));
		const Vector4Type ignored_scale_b = vector_set(FloatType(-0.75), FloatType(3.0), FloatType(1.25));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = vqv_set(rotation_around_z, x_axis, ignored_scale_a);
		Vector4Type result = vqv_mul_point3_no_scale(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = vqv_mul_point3_no_scale(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = vqv_set(rotation_around_x, y_axis, ignored_scale_b);
		result = vqv_mul_point3_no_scale(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = vqv_mul_point3_no_scale(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = vqv_mul_no_scale(transform_a, transform_b);
		TransformType transform_ba = vqv_mul_no_scale(transform_b, transform_a);

		// The result has the scale of the left input
		CHECK(vector_all_near_equal3(ignored_scale_a, transform_ab.scale, threshold));
		CHECK(vector_all_near_equal3(ignored_scale_b, transform_ba.scale, threshold));

		result = vqv_mul_point3_no_scale(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3_no_scale(vqv_mul_point3_no_scale(x_axis, transform_a), transform_b), threshold));
		result = vqv_mul_point3_no_scale(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3_no_scale(vqv_mul_point3_no_scale(y_axis, transform_a), transform_b), threshold));
		result = vqv_mul_point3_no_scale(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3_no_scale(vqv_mul_point3_no_scale(x_axis, transform_b), transform_a), threshold));
		result = vqv_mul_point3_no_scale(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqv_mul_point3_no_scale(vqv_mul_point3_no_scale(y_axis, transform_b), transform_a), threshold));

		// The VQ functions give the same rotation and translation
		const auto vq_a = vq_set(transform_a.rotation, transform_a.translation);
		const auto vq_b = vq_set(transform_b.rotation, transform_b.translation);
		const auto vq_ab = vq_mul(vq_a, vq_b);
		CHECK(quat_near_equal(vq_ab.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_ab.translation, transform_ab.translation, threshold));
		CHECK(vector_all_near_equal3(vq_mul_point3(x_axis, vq_ab), vqv_mul_point3_no_scale(x_axis, transform_ab), threshold));
		CHECK(vector_all_near_equal3(vq_mul_point3(y_axis, vq_ab), vqv_mul_point3_no_scale(y_axis, transform_ab), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		// The scale is not 1.0, thus a function that applies the scale fails
		const Vector4Type ignored_scale = vector_set(FloatType(2.5), FloatType(0.5), FloatType(-1.5));
		TransformType transform_a = vqv_set(rotation_around_z, x_axis, ignored_scale);
		TransformType transform_b = vqv_inverse_no_scale(transform_a);
		TransformType transform_ab = vqv_mul_no_scale(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(identity.translation, transform_ab.translation, threshold));

		// The results have the scale of the input
		CHECK(vector_all_near_equal3(ignored_scale, transform_b.scale, threshold));
		CHECK(vector_all_near_equal3(ignored_scale, transform_ab.scale, threshold));

		// The VQ function gives the same rotation and translation
		const auto vq_inv = vq_inverse(vq_set(transform_a.rotation, transform_a.translation));
		CHECK(quat_near_equal(vq_inv.rotation, transform_b.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_inv.translation, transform_b.translation, threshold));
	}

	{
		const FloatType alpha = FloatType(0.33);
		const ScalarType alpha_s = scalar_set(alpha);
		const QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		const QuatType quat1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));
		const Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		const Vector4Type trans1 = vector_set(FloatType(2.5), FloatType(-1.75), FloatType(0.5));
		const Vector4Type scale0 = vector_set(FloatType(-1.915), FloatType(0.23656), FloatType(-3.7811));
		const Vector4Type scale1 = vector_set(FloatType(-0.2113), FloatType(12.22335), FloatType(-1.7261));

		const TransformType transform0 = vqv_set(quat0, trans0, scale0);
		const TransformType transform1 = vqv_set(quat1, trans1, scale1);

		const Vector4Type trans_ref = vector_lerp(trans0, trans1, alpha);
		const Vector4Type trans_ref_s = vector_lerp(trans0, trans1, alpha_s);

		const TransformType transform_lerp_no_scale = vqv_lerp_no_scale(transform0, transform1, alpha);
		const TransformType transform_lerp_no_scale_s = vqv_lerp_no_scale(transform0, transform1, alpha_s);
		const TransformType transform_slerp_no_scale = vqv_slerp_no_scale(transform0, transform1, alpha);
		const TransformType transform_slerp_no_scale_s = vqv_slerp_no_scale(transform0, transform1, alpha_s);

		CHECK(quat_near_equal(transform_lerp_no_scale.rotation, quat_lerp(quat0, quat1, alpha), threshold));
		CHECK(quat_near_equal(transform_lerp_no_scale_s.rotation, quat_lerp(quat0, quat1, alpha_s), threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale.translation, trans_ref, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale_s.translation, trans_ref_s, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale.scale, scale0, threshold));
		CHECK(vector_all_near_equal3(transform_lerp_no_scale_s.scale, scale0, threshold));

		CHECK(quat_near_equal(transform_slerp_no_scale.rotation, quat_slerp(quat0, quat1, alpha), threshold));
		CHECK(quat_near_equal(transform_slerp_no_scale_s.rotation, quat_slerp(quat0, quat1, alpha_s), threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale.translation, trans_ref, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale_s.translation, trans_ref_s, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale.scale, scale0, threshold));
		CHECK(vector_all_near_equal3(transform_slerp_no_scale_s.scale, scale0, threshold));

		// The VQ functions give the same rotation and translation
		const auto vq0 = vq_set(quat0, trans0);
		const auto vq1 = vq_set(quat1, trans1);
		const auto vq_lerp_result = vq_lerp(vq0, vq1, alpha);
		const auto vq_slerp_result = vq_slerp(vq0, vq1, alpha);
		CHECK(quat_near_equal(vq_lerp_result.rotation, transform_lerp_no_scale.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_lerp_result.translation, transform_lerp_no_scale.translation, threshold));
		CHECK(quat_near_equal(vq_slerp_result.rotation, transform_slerp_no_scale.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_slerp_result.translation, transform_slerp_no_scale.translation, threshold));
	}
}

TEST_CASE("vqv deprecated API", "[math][vqv]")
{
	test_vqv_inverse_deprecated_impl<vqvf, float>(vqv_identity(), 1.0E-4F);
	test_vqv_inverse_deprecated_impl<vqvd, double>(vqv_identity(), 1.0E-6);

	test_vqv_no_scale_deprecated_impl<vqvf, float>(vqv_identity(), 1.0E-4F);
	test_vqv_no_scale_deprecated_impl<vqvd, double>(vqv_identity(), 1.0E-6);
}
