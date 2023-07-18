#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2023 Nicholas Frechette & Realtime Math contributors
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

#include "rtm/math.h"
#include "rtm/quatd.h"
#include "rtm/vector4d.h"
#include "rtm/matrix3x4d.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/qvvs_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Casts a QVVS transform float32 variant to a float64 variant.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd RTM_SIMD_CALL qvvs_cast(qvvsf_arg0 input) RTM_NO_EXCEPT
	{
		return qvvsd{ quat_cast(input.rotation), vector_cast(input.translation_uniform_scale), vector_cast(input.non_uniform_scale) };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the rotation part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE quatd qvvs_get_rotation(const qvvsd& input) RTM_NO_EXCEPT
	{
		return input.rotation;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the translation part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d qvvs_get_translation(const qvvsd& input) RTM_NO_EXCEPT
	{
		return input.translation_uniform_scale;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the uniform scale part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_w qvvs_get_uniform_scale(const qvvsd& input) RTM_NO_EXCEPT
	{
		return rtm_impl::vector4d_vector_get_w{ input.translation_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the non-uniform scale part of a QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d qvvs_get_non_uniform_scale(const qvvsd& input) RTM_NO_EXCEPT
	{
		return input.non_uniform_scale;
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two QVVS transforms.
	// Multiplication order is as follow: local_to_world = qvvs_mul(local_to_object, object_to_world)
	// NOTE: The uniform scale portion will combine through multiplication but the non-uniform part does not.
	// The resulting QVVS transform will obtain its non-uniform scale part from the left hand side value, as is.
	// By design, QVVS transforms do not propagate the parent transform non-uniform scale. This means that
	// qvvs_mul_point3(point, local_to_world) != qvvs_mul_point3(qvvs_mul_point3(point, local_to_object), object_to_world)
	// However, the multiplication operation can be reversed by multiplying with an inverse transform as expected:
	// local_to_object = qvvs_mul(local_to_world, qvvs_inverse(object_to_world))
	// local_to_object = qvvs_mul(local_to_world, world_to_object)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsd qvvs_mul(const qvvsd& lhs, const qvvsd& rhs) RTM_NO_EXCEPT
	{
		const vector4d rhs_uniform_scale = vector_dup_w(rhs.translation_uniform_scale);

		const quatd rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4d translation = vector_add(quat_mul_vector3(vector_mul(lhs.translation_uniform_scale, rhs_uniform_scale), rhs.rotation), rhs.translation_uniform_scale);
		const vector4d uniform_scale_w = vector_mul(lhs.translation_uniform_scale, rhs.translation_uniform_scale);
		const vector4d translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation, uniform_scale_w);

		return qvvsd{ rotation, translation_uniform_scale, lhs.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two QVVS transforms ignoring 3D scale.
	// The resulting QVVS transform will have the LHS scale.
	// Multiplication order is as follow: local_to_world = qvvs_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsd qvvs_mul_no_scale(const qvvsd& lhs, const qvvsd& rhs) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_mul(lhs.rotation, rhs.rotation);
		const vector4d translation = vector_add(quat_mul_vector3(lhs.translation_uniform_scale, rhs.rotation), rhs.translation_uniform_scale);
		const vector4d translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation, lhs.translation_uniform_scale);

		return qvvsd{ rotation, translation_uniform_scale, lhs.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a QVVS transform and a 3D point.
	// Multiplication order is as follow: world_position = qvvs_mul_point3(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d qvvs_mul_point3(const vector4d& point, const qvvsd& qvvs) RTM_NO_EXCEPT
	{
		const vector4d uniform_scale = vector_dup_w(qvvs.translation_uniform_scale);
		const vector4d scale = vector_mul(uniform_scale, qvvs.non_uniform_scale);
		return vector_add(quat_mul_vector3(vector_mul(scale, point), qvvs.rotation), qvvs.translation_uniform_scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a QVVS transform and a 3D point ignoring 3D scale.
	// Multiplication order is as follow: world_position = qvvs_mul_point3_no_scale(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d qvvs_mul_point3_no_scale(const vector4d& point, const qvvsd& qvvs) RTM_NO_EXCEPT
	{
		return vector_add(quat_mul_vector3(point, qvvs.rotation), qvvs.translation_uniform_scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input QVVS transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsd qvvs_inverse(const qvvsd& input) RTM_NO_EXCEPT
	{
		const quatd inv_rotation = quat_conjugate(input.rotation);
		const vector4d inv_uniform_scale_w = vector_reciprocal(input.translation_uniform_scale);
		const vector4d inv_uniform_scale = vector_dup_w(inv_uniform_scale_w);
		const vector4d inv_non_uniform_scale = vector_reciprocal(input.non_uniform_scale);
		const vector4d inv_scale = vector_mul(inv_uniform_scale, inv_non_uniform_scale);
		const vector4d inv_translation = vector_neg(quat_mul_vector3(vector_mul(input.translation_uniform_scale, inv_scale), inv_rotation));
		const vector4d inv_translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(inv_translation, inv_uniform_scale_w);

		return qvvsd{ inv_rotation, inv_translation_uniform_scale, inv_non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the inverse of the input QVVS transform ignoring 3D scale.
	// The resulting QVVS transform will have the input scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline qvvsd qvvs_inverse_no_scale(const qvvsd& input) RTM_NO_EXCEPT
	{
		const quatd inv_rotation = quat_conjugate(input.rotation);
		const vector4d inv_translation = vector_neg(quat_mul_vector3(input.translation_uniform_scale, inv_rotation));
		const vector4d inv_translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(inv_translation, input.translation_uniform_scale);

		return qvvsd{ inv_rotation, inv_translation_uniform_scale, input.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a QVVS transforms with the rotation part normalized.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_normalize(const qvvsd& input) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_normalize(input.rotation);
		return qvvsd{ rotation, input.translation_uniform_scale, input.non_uniform_scale };
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_lerp(const qvvsd& start, const qvvsd& end, double alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsd{ rotation, translation_uniform_scale, non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_lerp(const qvvsd& start, const qvvsd& end, const scalard& alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsd{ rotation, translation_uniform_scale, non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	// The resulting QVV transform will have the start scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_lerp_no_scale(const qvvsd& start, const qvvsd& end, double alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsd{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component linear interpolation of the two inputs at the specified alpha.
	// The formula used is: ((1.0 - alpha) * start) + (alpha * end).
	// Interpolation is stable and will return 'start' when alpha is 0.0 and 'end' when it is 1.0.
	// This is the same instruction count when FMA is present but it might be slightly slower
	// due to the extra multiplication compared to: start + (alpha * (end - start)).
	// The resulting QVV transform will have the start scale.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_lerp_no_scale(const qvvsd& start, const qvvsd& end, const scalard& alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_lerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsd{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_slerp(const qvvsd& start, const qvvsd& end, double alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsd{ rotation, translation_uniform_scale, non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_slerp(const qvvsd& start, const qvvsd& end, const scalard& alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d non_uniform_scale = vector_lerp(start.non_uniform_scale, end.non_uniform_scale, alpha);
		return qvvsd{ rotation, translation_uniform_scale, non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_slerp_no_scale(const qvvsd& start, const qvvsd& end, double alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsd{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component spherical interpolation of the two inputs at the specified alpha.
	// See quat_slerp(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE qvvsd qvvs_slerp_no_scale(const qvvsd& start, const qvvsd& end, const scalard& alpha) RTM_NO_EXCEPT
	{
		const quatd rotation = quat_slerp(start.rotation, end.rotation, alpha);
		const vector4d translation_uniform_scale_lerp = vector_lerp(start.translation_uniform_scale, end.translation_uniform_scale, alpha);
		const vector4d translation_uniform_scale = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(translation_uniform_scale_lerp, start.translation_uniform_scale);
		return qvvsd{ rotation, translation_uniform_scale, start.non_uniform_scale };
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Returns true if the input QVVS does not contain any NaN or Inf, otherwise false.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool qvvs_is_finite(const qvvsd& input) RTM_NO_EXCEPT
	{
		return quat_is_finite(input.rotation) && vector_is_finite(input.translation_uniform_scale) && vector_is_finite3(input.non_uniform_scale);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
