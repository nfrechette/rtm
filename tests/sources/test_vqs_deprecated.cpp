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
// This file tests the deprecated vqs_*_no_scale functions. Remove it with these functions.

#include "catch2.impl.h"

#include <rtm/type_traits.h>
#include <rtm/vqf.h>
#include <rtm/vqd.h>
#include <rtm/vqsf.h>
#include <rtm/vqsd.h>

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
static void test_vqs_no_scale_deprecated_impl(const TransformType& identity, const FloatType threshold)
{
	using QuatType = typename related_types<FloatType>::quat;
	using Vector4Type = typename related_types<FloatType>::vector4;
	using ScalarType = typename related_types<FloatType>::scalar;

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));
		Vector4Type y_axis = vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0));

		// The scale is not 1.0, thus a function that applies the scale fails
		const FloatType ignored_scale_a = FloatType(2.5);
		const FloatType ignored_scale_b = FloatType(-0.75);

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		TransformType transform_a = vqs_set(rotation_around_z, x_axis, ignored_scale_a);
		Vector4Type result = vqs_mul_point3_no_scale(x_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = vqs_mul_point3_no_scale(y_axis, transform_a);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0)), threshold));

		QuatType rotation_around_x = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)));
		TransformType transform_b = vqs_set(rotation_around_x, y_axis, ignored_scale_b);
		result = vqs_mul_point3_no_scale(x_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(0.0)), threshold));
		result = vqs_mul_point3_no_scale(y_axis, transform_b);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(-1.0)), threshold));

		TransformType transform_ab = vqs_mul_no_scale(transform_a, transform_b);
		TransformType transform_ba = vqs_mul_no_scale(transform_b, transform_a);

		// The result has the scale of the left input
		CHECK(scalar_near_equal(ignored_scale_a, vqs_get_scale(transform_ab), threshold));
		CHECK(scalar_near_equal(ignored_scale_b, vqs_get_scale(transform_ba), threshold));

		result = vqs_mul_point3_no_scale(x_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(1.0), FloatType(1.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqs_mul_point3_no_scale(vqs_mul_point3_no_scale(x_axis, transform_a), transform_b), threshold));
		result = vqs_mul_point3_no_scale(y_axis, transform_ab);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqs_mul_point3_no_scale(vqs_mul_point3_no_scale(y_axis, transform_a), transform_b), threshold));
		result = vqs_mul_point3_no_scale(x_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqs_mul_point3_no_scale(vqs_mul_point3_no_scale(x_axis, transform_b), transform_a), threshold));
		result = vqs_mul_point3_no_scale(y_axis, transform_ba);
		CHECK(vector_all_near_equal3(result, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0)), threshold));
		CHECK(vector_all_near_equal3(result, vqs_mul_point3_no_scale(vqs_mul_point3_no_scale(y_axis, transform_b), transform_a), threshold));

		// The VQ functions give the same rotation and translation
		const auto vq_a = vq_set(transform_a.rotation, vqs_get_translation(transform_a));
		const auto vq_b = vq_set(transform_b.rotation, vqs_get_translation(transform_b));
		const auto vq_ab = vq_mul(vq_a, vq_b);
		CHECK(quat_near_equal(vq_ab.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_ab.translation, vqs_get_translation(transform_ab), threshold));
		CHECK(vector_all_near_equal3(vq_mul_point3(x_axis, vq_ab), vqs_mul_point3_no_scale(x_axis, transform_ab), threshold));
		CHECK(vector_all_near_equal3(vq_mul_point3(y_axis, vq_ab), vqs_mul_point3_no_scale(y_axis, transform_ab), threshold));
	}

	{
		Vector4Type x_axis = vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0));

		QuatType rotation_around_z = quat_from_euler(scalar_deg_to_rad(FloatType(0.0)), scalar_deg_to_rad(FloatType(90.0)), scalar_deg_to_rad(FloatType(0.0)));
		// The scale is not 1.0, thus a function that applies the scale fails
		const FloatType ignored_scale = FloatType(2.5);
		TransformType transform_a = vqs_set(rotation_around_z, x_axis, ignored_scale);
		TransformType transform_b = vqs_inverse_no_scale(transform_a);
		TransformType transform_ab = vqs_mul_no_scale(transform_a, transform_b);
		CHECK(quat_near_equal(identity.rotation, transform_ab.rotation, threshold));
		CHECK(vector_all_near_equal3(vqs_get_translation(identity), vqs_get_translation(transform_ab), threshold));

		// The results have the scale of the input
		CHECK(scalar_near_equal(ignored_scale, vqs_get_scale(transform_b), threshold));
		CHECK(scalar_near_equal(ignored_scale, vqs_get_scale(transform_ab), threshold));

		// The VQ function gives the same rotation and translation
		const auto vq_inv = vq_inverse(vq_set(transform_a.rotation, vqs_get_translation(transform_a)));
		CHECK(quat_near_equal(vq_inv.rotation, transform_b.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_inv.translation, vqs_get_translation(transform_b), threshold));
	}

	{
		const FloatType alpha = FloatType(0.33);
		const ScalarType alpha_s = scalar_set(alpha);
		const QuatType quat0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
		const QuatType quat1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));
		const Vector4Type trans0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
		const Vector4Type trans1 = vector_set(FloatType(2.5), FloatType(-1.75), FloatType(0.5));
		const FloatType scale0 = FloatType(-1.915);
		const FloatType scale1 = FloatType(-0.2113);

		const TransformType transform0 = vqs_set(quat0, trans0, scale0);
		const TransformType transform1 = vqs_set(quat1, trans1, scale1);

		const Vector4Type trans_ref = vector_lerp(trans0, trans1, alpha);
		const Vector4Type trans_ref_s = vector_lerp(trans0, trans1, alpha_s);

		const TransformType transform_lerp_no_scale = vqs_lerp_no_scale(transform0, transform1, alpha);
		const TransformType transform_lerp_no_scale_s = vqs_lerp_no_scale(transform0, transform1, alpha_s);
		const TransformType transform_slerp_no_scale = vqs_slerp_no_scale(transform0, transform1, alpha);
		const TransformType transform_slerp_no_scale_s = vqs_slerp_no_scale(transform0, transform1, alpha_s);

		CHECK(quat_near_equal(transform_lerp_no_scale.rotation, quat_lerp(quat0, quat1, alpha), threshold));
		CHECK(quat_near_equal(transform_lerp_no_scale_s.rotation, quat_lerp(quat0, quat1, alpha_s), threshold));
		CHECK(vector_all_near_equal3(vqs_get_translation(transform_lerp_no_scale), trans_ref, threshold));
		CHECK(vector_all_near_equal3(vqs_get_translation(transform_lerp_no_scale_s), trans_ref_s, threshold));
		CHECK(scalar_near_equal(vqs_get_scale(transform_lerp_no_scale), scale0, threshold));
		CHECK(scalar_near_equal(vqs_get_scale(transform_lerp_no_scale_s), scale0, threshold));

		CHECK(quat_near_equal(transform_slerp_no_scale.rotation, quat_slerp(quat0, quat1, alpha), threshold));
		CHECK(quat_near_equal(transform_slerp_no_scale_s.rotation, quat_slerp(quat0, quat1, alpha_s), threshold));
		CHECK(vector_all_near_equal3(vqs_get_translation(transform_slerp_no_scale), trans_ref, threshold));
		CHECK(vector_all_near_equal3(vqs_get_translation(transform_slerp_no_scale_s), trans_ref_s, threshold));
		CHECK(scalar_near_equal(vqs_get_scale(transform_slerp_no_scale), scale0, threshold));
		CHECK(scalar_near_equal(vqs_get_scale(transform_slerp_no_scale_s), scale0, threshold));

		// The VQ functions give the same rotation and translation
		const auto vq0 = vq_set(quat0, trans0);
		const auto vq1 = vq_set(quat1, trans1);
		const auto vq_lerp_result = vq_lerp(vq0, vq1, alpha);
		const auto vq_slerp_result = vq_slerp(vq0, vq1, alpha);
		CHECK(quat_near_equal(vq_lerp_result.rotation, transform_lerp_no_scale.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_lerp_result.translation, vqs_get_translation(transform_lerp_no_scale), threshold));
		CHECK(quat_near_equal(vq_slerp_result.rotation, transform_slerp_no_scale.rotation, threshold));
		CHECK(vector_all_near_equal3(vq_slerp_result.translation, vqs_get_translation(transform_slerp_no_scale), threshold));
	}
}

TEST_CASE("vqs deprecated API", "[math][vqs]")
{
	test_vqs_no_scale_deprecated_impl<vqsf, float>(vqs_identity(), 1.0E-4F);
	test_vqs_no_scale_deprecated_impl<vqsd, double>(vqs_identity(), 1.0E-6);
}
