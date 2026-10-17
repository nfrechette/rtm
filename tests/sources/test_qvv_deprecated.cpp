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
// This file tests the deprecated qvv names. Remove it with the qvv names.

#include "catch2.impl.h"

#include <rtm/qvvf.h>
#include <rtm/qvvd.h>
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

static_assert(std::is_same<qvvf, vqvf>::value, "qvvf must be an alias of vqvf");
static_assert(std::is_same<qvvd, vqvd>::value, "qvvd must be an alias of vqvd");

// These helpers use the deprecated argument types in their signatures
static bool is_near_equal(qvvf_arg0 lhs, qvvf_arg1 rhs, float threshold)
{
	return quat_near_equal(lhs.rotation, rhs.rotation, threshold)
		&& vector_all_near_equal3(lhs.translation, rhs.translation, threshold)
		&& vector_all_near_equal3(lhs.scale, rhs.scale, threshold);
}

static bool is_near_equal(qvvd_arg0 lhs, qvvd_arg1 rhs, double threshold)
{
	return quat_near_equal(lhs.rotation, rhs.rotation, threshold)
		&& vector_all_near_equal3(lhs.translation, rhs.translation, threshold)
		&& vector_all_near_equal3(lhs.scale, rhs.scale, threshold);
}

static qvvf copy_transform(qvvf_argn input)
{
	return input;
}

static qvvd copy_transform(qvvd_argn input)
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
static void test_qvv_deprecated_impl(const FloatType threshold)
{
	using TransformType = typename related_types<FloatType>::qvv;
	using QuatType = typename related_types<FloatType>::quat;
	using Vector4Type = typename related_types<FloatType>::vector4;
	using ScalarType = typename related_types<FloatType>::scalar;
	using Matrix3x3Type = typename related_types<FloatType>::matrix3x3;
	using Matrix3x4Type = typename related_types<FloatType>::matrix3x4;

	static_assert(std::is_same<TransformType, typename related_types<FloatType>::vqv>::value, "related_types::qvv must be an alias of related_types::vqv");

	const QuatType rotation0 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(30.0)), scalar_deg_to_rad(FloatType(-45.0)), scalar_deg_to_rad(FloatType(90.0))));
	const QuatType rotation1 = quat_normalize(quat_from_euler(scalar_deg_to_rad(FloatType(45.0)), scalar_deg_to_rad(FloatType(60.0)), scalar_deg_to_rad(FloatType(120.0))));
	const Vector4Type translation0 = vector_set(FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598));
	const Vector4Type translation1 = vector_set(FloatType(2.5), FloatType(-1.75), FloatType(0.5));
	const Vector4Type scale0 = vector_set(FloatType(1.915), FloatType(0.23656), FloatType(3.7811));
	const Vector4Type scale1 = vector_set(FloatType(-0.2113), FloatType(12.22335), FloatType(-1.7261));
	const Vector4Type point = vector_set(FloatType(1.25), FloatType(-0.5), FloatType(2.0));
	const FloatType alpha = FloatType(0.33);
	const ScalarType alpha_s = scalar_set(alpha);

	const TransformType transform0 = qvv_set(rotation0, translation0, scale0);
	const TransformType transform1 = copy_transform(vqv_set(rotation1, translation1, scale1));
	CHECK(is_near_equal(transform0, vqv_set(rotation0, translation0, scale0), threshold));

	{
		const TransformType identity = qvv_identity();
		const TransformType identity_ref = vqv_identity();
		CHECK(is_near_equal(identity, identity_ref, threshold));
	}

	{
		// The cast to the other precision and back uses both qvv_cast overloads
		const auto dst = qvv_cast(transform0);
		const auto dst_ref = vqv_cast(transform0);
		CHECK(is_near_equal(qvv_cast(dst), vqv_cast(dst_ref), threshold));
	}

	{
		const Matrix3x3Type mtx3x3 = matrix_from_quat(rotation0);
		CHECK(is_near_equal(qvv_from_matrix(mtx3x3), vqv_from_matrix(mtx3x3), threshold));

		const Matrix3x4Type mtx3x4 = matrix_from_vqv(transform0);
		CHECK(is_near_equal(qvv_from_matrix(mtx3x4), vqv_from_matrix(mtx3x4), threshold));

		CHECK(is_near_equal_matrix(matrix_from_qvv(rotation0, translation0, scale0), matrix_from_vqv(rotation0, translation0, scale0), threshold));
		CHECK(is_near_equal_matrix(matrix_from_qvv(transform0), matrix_from_vqv(transform0), threshold));
	}

	CHECK(is_near_equal(qvv_mul(transform0, transform1), vqv_mul(transform0, transform1), threshold));
	CHECK(is_near_equal(qvv_mul_no_scale(transform0, transform1), vqv_mul_no_scale(transform0, transform1), threshold));
	CHECK(vector_all_near_equal3(qvv_mul_point3(point, transform0), vqv_mul_point3(point, transform0), threshold));
	CHECK(vector_all_near_equal3(qvv_mul_point3_no_scale(point, transform0), vqv_mul_point3_no_scale(point, transform0), threshold));

	CHECK(is_near_equal(qvv_inverse(transform0), vqv_inverse(transform0), threshold));
	CHECK(is_near_equal(qvv_inverse(transform1, vector_set(FloatType(1.0))), vqv_inverse(transform1, vector_set(FloatType(1.0))), threshold));
	CHECK(is_near_equal(qvv_inverse_no_scale(transform0), vqv_inverse_no_scale(transform0), threshold));
	CHECK(is_near_equal(qvv_normalize(transform0), vqv_normalize(transform0), threshold));

	// With SSE2, the scalar alpha selects the overloads that take scalarf or scalard
	CHECK(is_near_equal(qvv_lerp(transform0, transform1, alpha), vqv_lerp(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvv_lerp(transform0, transform1, alpha_s), vqv_lerp(transform0, transform1, alpha_s), threshold));
	CHECK(is_near_equal(qvv_lerp_no_scale(transform0, transform1, alpha), vqv_lerp_no_scale(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvv_lerp_no_scale(transform0, transform1, alpha_s), vqv_lerp_no_scale(transform0, transform1, alpha_s), threshold));
	CHECK(is_near_equal(qvv_slerp(transform0, transform1, alpha), vqv_slerp(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvv_slerp(transform0, transform1, alpha_s), vqv_slerp(transform0, transform1, alpha_s), threshold));
	CHECK(is_near_equal(qvv_slerp_no_scale(transform0, transform1, alpha), vqv_slerp_no_scale(transform0, transform1, alpha), threshold));
	CHECK(is_near_equal(qvv_slerp_no_scale(transform0, transform1, alpha_s), vqv_slerp_no_scale(transform0, transform1, alpha_s), threshold));

	CHECK(qvv_is_finite(transform0) == vqv_is_finite(transform0));
	CHECK(qvv_is_finite(transform0));
}

TEST_CASE("qvv deprecated API", "[math][qvv]")
{
	test_qvv_deprecated_impl<float>(1.0E-5F);
	test_qvv_deprecated_impl<double>(1.0E-10);

	// The deprecated type names
	const qvvf transform_f = qvv_set(quat_identity(), vector_zero(), vector_set(1.0F));
	const qvvd transform_d = qvv_set(quat_identity(), vector_zero(), vector_set(1.0));
	CHECK(is_near_equal(transform_f, vqv_cast(transform_d), 0.0F));
	CHECK(is_near_equal(transform_d, vqv_cast(transform_f), 0.0));
}
