#pragma once

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

#include "rtm/math.h"
#include "rtm/version.h"
#include "rtm/vqsd.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"
#include "rtm/impl/qvs_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

// Deprecated in 2.4, to be removed in 2.5
// This header keeps the previous qvs names. Use rtm/vqsd.h instead.

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	/// Casts a VQS transform float32 variant to a float64 variant.
	RTM_DEPRECATED("Use 'vqs_cast' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_cast(vqsf_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_cast(input);
	}

	/// Converts a rotation 3x3 matrix into a VQS transform.
	RTM_DEPRECATED("Use 'vqs_from_matrix' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_from_matrix(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_from_matrix(input);
	}

	/// Converts a rotation 3x4 affine matrix into a VQS transform.
	RTM_DEPRECATED("Use 'vqs_from_matrix' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_from_matrix(matrix3x4d_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_from_matrix(input);
	}

	/// Returns the rotation part of a VQS transform.
	RTM_DEPRECATED("Use 'vqs_get_rotation' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE quatd RTM_SIMD_CALL qvs_get_rotation(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_get_rotation(input);
	}

	/// Returns the translation part of a VQS transform.
	RTM_DEPRECATED("Use 'vqs_get_translation' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL qvs_get_translation(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_get_translation(input);
	}

	/// Returns the scale part of a VQS transform.
	RTM_DEPRECATED("Use 'vqs_get_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vector4d_vector_get_w RTM_SIMD_CALL qvs_get_scale(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_get_scale(input);
	}

	/// Multiplies two VQS transforms.
	RTM_DEPRECATED("Use 'vqs_mul' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_mul(vqsd_arg0 lhs, vqsd_arg1 rhs) RTM_NO_EXCEPT
	{
		return vqs_mul(lhs, rhs);
	}

	/// Multiplies two VQS transforms ignoring scale.
	RTM_DEPRECATED("Use 'vqs_mul_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_mul_no_scale(vqsd_arg0 lhs, vqsd_arg1 rhs) RTM_NO_EXCEPT
	{
		return vqs_mul_no_scale(lhs, rhs);
	}

	/// Multiplies a VQS transform and a 3D point.
	RTM_DEPRECATED("Use 'vqs_mul_point3' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL qvs_mul_point3(vector4d_arg0 point, vqsd_arg1 vqs) RTM_NO_EXCEPT
	{
		return vqs_mul_point3(point, vqs);
	}

	/// Multiplies a VQS transform and a 3D point ignoring 3D scale.
	RTM_DEPRECATED("Use 'vqs_mul_point3_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL qvs_mul_point3_no_scale(vector4d_arg0 point, vqsd_arg1 vqs) RTM_NO_EXCEPT
	{
		return vqs_mul_point3_no_scale(point, vqs);
	}

	/// Returns the inverse of the input VQS transform.
	RTM_DEPRECATED("Use 'vqs_inverse' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_inverse(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_inverse(input);
	}

	/// Returns the inverse of the input VQS transform with a fallback scale.
	RTM_DEPRECATED("Use 'vqs_inverse' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_inverse(vqsd_arg0 input, double fallback_scale, double threshold = 1.0E-8) RTM_NO_EXCEPT
	{
		return vqs_inverse(input, fallback_scale, threshold);
	}

	/// Returns the inverse of the input VQS transform ignoring 3D scale.
	RTM_DEPRECATED("Use 'vqs_inverse_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqsd RTM_SIMD_CALL qvs_inverse_no_scale(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_inverse_no_scale(input);
	}

	/// Returns a VQS transform with the rotation part normalized.
	RTM_DEPRECATED("Use 'vqs_normalize' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_normalize(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_normalize(input);
	}

	/// Per component linear interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqs_lerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_lerp(vqsd_arg0 start, vqsd_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		return vqs_lerp(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component linear interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqs_lerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_lerp(vqsd_arg0 start, vqsd_arg1 end, scalard_argn alpha) RTM_NO_EXCEPT
	{
		return vqs_lerp(start, end, alpha);
	}
#endif

	/// Per component linear interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqs_lerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_lerp_no_scale(vqsd_arg0 start, vqsd_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		return vqs_lerp_no_scale(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component linear interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqs_lerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_lerp_no_scale(vqsd_arg0 start, vqsd_arg1 end, scalard_argn alpha) RTM_NO_EXCEPT
	{
		return vqs_lerp_no_scale(start, end, alpha);
	}
#endif

	/// Per component spherical interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqs_slerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_slerp(vqsd_arg0 start, vqsd_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		return vqs_slerp(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component spherical interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vqs_slerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_slerp(vqsd_arg0 start, vqsd_arg1 end, scalard_argn alpha) RTM_NO_EXCEPT
	{
		return vqs_slerp(start, end, alpha);
	}
#endif

	/// Per component spherical interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqs_slerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_slerp_no_scale(vqsd_arg0 start, vqsd_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		return vqs_slerp_no_scale(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component spherical interpolation of the two inputs at the specified alpha, without the scale.
	RTM_DEPRECATED("Use 'vqs_slerp_no_scale' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqsd RTM_SIMD_CALL qvs_slerp_no_scale(vqsd_arg0 start, vqsd_arg1 end, scalard_argn alpha) RTM_NO_EXCEPT
	{
		return vqs_slerp_no_scale(start, end, alpha);
	}
#endif

	/// Returns true if the input VQS does not contain any NaN or Inf, otherwise false.
	RTM_DEPRECATED("Use 'vqs_is_finite' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL qvs_is_finite(vqsd_arg0 input) RTM_NO_EXCEPT
	{
		return vqs_is_finite(input);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
