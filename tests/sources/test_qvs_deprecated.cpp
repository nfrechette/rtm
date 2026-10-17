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

// Deprecated in 2.4, to be removed in 2.5
// This file tests the deprecated qvs names. Remove it with the qvs names.

#include "catch2.impl.h"

#include <rtm/qvsf.h>
#include <rtm/qvsd.h>
#include <rtm/matrix3x4f.h>
#include <rtm/matrix3x4d.h>
#include <rtm/type_traits.h>

#include <type_traits>

// This test uses the deprecated names on purpose. Disable the deprecation warning.
// Do not define RTM_NO_DEPRECATION: the test must compile the deprecation attributes.
#if defined(RTM_COMPILER_MSVC)
	#pragma warning(disable : 4996)
#elif defined(RTM_COMPILER_CLANG)
	#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(RTM_COMPILER_GCC)
	#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace rtm;

static_assert(std::is_same<qvsf, vqsf>::value, "qvsf must be an alias of vqsf");
static_assert(std::is_same<qvsd, vqsd>::value, "qvsd must be an alias of vqsd");

// These helpers use the deprecated argument types in their signatures
static bool is_near_equal(qvsf_arg0 lhs, qvsf_arg1 rhs, float threshold)
{
	return quat_near_equal(lhs.rotation, rhs.rotation, threshold)
		&& vector_all_near_equal(lhs.translation_scale, rhs.translation_scale, threshold);
}

static bool is_near_equal(qvsd_arg0 lhs, qvsd_arg1 rhs, double threshold)
{
	return quat_near_equal(lhs.rotation, rhs.rotation, threshold)
		&& vector_all_near_equal(lhs.translation_scale, rhs.translation_scale, threshold);
}

static qvsf copy_transform(qvsf_argn input)
{
	return input;
}

static qvsd copy_transform(qvsd_argn input)
{
	return input;
}

template<typename Matrix3x4Type, typename FloatType>
static bool is_near_equal_matrix(const Matrix3x4Type& lhs, const Matrix3x4Type& rhs, FloatType threshold)
{
	return vector_all_near_equal3(lhs.x_axis, rhs.x_axis, threshold)
		&& vector_all_near_equal3(lhs.y_axis, rhs.y_axis, threshold)
		&& vector_all_near_equal3(lhs.z_axis, rhs.z_axis, threshold)
		&& vector_all_near_equal3(lhs.w_axis, rhs.w_axis, threshold);
}

template<typename FloatType>
static void test_qvs_deprecated_impl(const FloatType threshold)
{
	using TransformType = typename related_types<FloatType>::qvs;
	using QuatType = typename related_types<FloatType>::quat;
	using Vector4Type = typename related_types<FloatType>::vector4;
	using ScalarType = typename related_types<FloatType>::scalar;
	using Matrix3x3Type = typename related_types<FloatType>::matrix3x3;
	using Matrix3x4Type = typename related_types<FloatType>::matrix3x4;

	static_assert(std::is_same<TransformType, typename related_types<FloatType>::vqs>::value, "related_types::qvs must be an alias of related_types::vqs");

	const QuatType rotation0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
	const QuatType rotation1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));
	const Vector4Type translation0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
	const Vector4Type translation1 = vector_set(FloatType(2.5), FloatType(-1.75), FloatType(0.5));
	const FloatType scale0 = FloatType(1.915);
	const FloatType scale1 = FloatType(-0.2113);
	const Vector4Type point = vector_set(FloatType(1.25), FloatType(-0.5), FloatType(2.0));
	const FloatType alpha = FloatType(0.33);
	const ScalarType alpha_s = scalar_set(alpha);

	const TransformType transform0 = qvs_set(rotation0, translation0, scale0);
	const TransformType transform1 = copy_transform(vqs_set(rotation1, translation1, scale1));
	CHECK(is_near_equal(transform0, vqs_set(rotation0, translation0, scale0), threshold));

	// With SSE2, the scalar scale selects the overload that takes scalarf or scalard
	CHECK(is_near_equal(qvs_set(rotation0, translation0, scalar_set(scale0)), vqs_set(rotation0, translation0, scalar_set(scale0)), threshold));

	{
		const TransformType identity = qvs_identity();
		const TransformType identity_ref = vqs_identity();
		CHECK(is_near_equal(identity, identity_ref, threshold));
	}

	{
		// The cast to the other precision and back uses both qvs_cast overloads
		const auto dst = qvs_cast(transform0);
		const auto dst_ref = vqs_cast(transform0);
		CHECK(is_near_equal(qvs_cast(dst), vqs_cast(dst_ref), threshold));
	}

	{
		const Matrix3x3Type mtx3x3 = matrix_from_quat(rotation0);
		CHECK(is_near_equal(qvs_from_matrix(mtx3x3), vqs_from_matrix(mtx3x3), threshold));

		const Matrix3x4Type mtx3x4 = matrix_from_vqs(transform0);
		CHECK(is_near_equal(qvs_from_matrix(mtx3x4), vqs_from_matrix(mtx3x4), threshold));

		CHECK(is_near_equal_matrix(matrix_from_qvs(rotation0, translation0, scale0), matrix_from_vqs(rotation0, translation0, scale0), threshold));
		CHECK(is_near_equal_matrix(matrix_from_qvs(transform0), matrix_from_vqs(transform0), threshold));
	}

	CHECK(quat_near_equal(qvs_get_rotation(transform0), vqs_get_rotation(transform0), threshold));
	CHECK(vector_all_near_equal3(qvs_get_translation(transform0), vqs_get_translation(transform0), threshold));
	CHECK(scalar_near_equal(FloatType(qvs_get_scale(transform0)), FloatType(vqs_get_scale(transform0)), threshold));

	CHECK(is_near_equal(qvs_mul(transform0, transform1), vqs_mul(transform0, transform1), threshold));
	CHECK(is_near_equal(qvs_mul_no_scale(transform0, transform1), vqs_mul_no_scale(transform0, transform1), threshold));
	CHECK(vector_all_near_equal3(qvs_mul_point3(point, transform0), vqs_mul_point3(point, transform0), threshold));
	CHECK(vector_all_near_equal3(qvs_mul_point3_no_scale(point, transform0), vqs_mul_point3_no_scale(point, transform0), threshold));

	CHECK(is_near_equal(qvs_inverse(transform0), vqs_inverse(transform0), threshold));
	CHECK(is_near_equal(qvs_inverse(transform1, FloatType(1.0)), vqs_inverse(transform1, FloatType(1.0)), threshold));
	CHECK(is_near_equal(qvs_inverse_no_scale(transform0), vqs_inverse_no_scale(transform0), threshold));
	CHECK(is_near_equal(qvs_normalize(transform0), vqs_normalize(transform0), threshold));

	// With SSE2, the scalar alpha selects the overloads that take scalarf or scalard
	CHECK(is_near_equal(qvs_lerp(transform0, transform1, alpha), vqs_lerp(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvs_lerp(transform0, transform1, alpha_s), vqs_lerp(transform0, transform1, alpha_s), threshold));
	CHECK(is_near_equal(qvs_lerp_no_scale(transform0, transform1, alpha), vqs_lerp_no_scale(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvs_lerp_no_scale(transform0, transform1, alpha_s), vqs_lerp_no_scale(transform0, transform1, alpha_s), threshold));
	CHECK(is_near_equal(qvs_slerp(transform0, transform1, alpha), vqs_slerp(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvs_slerp(transform0, transform1, alpha_s), vqs_slerp(transform0, transform1, alpha_s), threshold));
	CHECK(is_near_equal(qvs_slerp_no_scale(transform0, transform1, alpha), vqs_slerp_no_scale(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvs_slerp_no_scale(transform0, transform1, alpha_s), vqs_slerp_no_scale(transform0, transform1, alpha_s), threshold));

	CHECK(qvs_is_finite(transform0) == vqs_is_finite(transform0));
	CHECK(qvs_is_finite(transform0));
}

TEST_CASE("qvs deprecated API", "[math][qvs]")
{
	test_qvs_deprecated_impl<float>(1.0E-5F);
	test_qvs_deprecated_impl<double>(1.0E-10);

	// The deprecated type names
	const qvsf transform_f = qvs_set(quat_identity(), vector_zero(), 1.0F);
	const qvsd transform_d = qvs_set(quat_identity(), vector_zero(), 1.0);
	CHECK(is_near_equal(transform_f, vqs_cast(transform_d), 0.0F));
	CHECK(is_near_equal(transform_d, vqs_cast(transform_f), 0.0));
}
