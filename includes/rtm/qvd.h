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
#include "rtm/vqd.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"
#include "rtm/impl/qv_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

// Deprecated in 2.4, to be removed in 2.5
// This header keeps the previous qv names. Use rtm/vqd.h instead.

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	/// Casts a VQ transform float32 variant to a float64 variant.
	RTM_DEPRECATED("Use 'vq_cast' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqd RTM_SIMD_CALL qv_cast(vqf_arg0 input) RTM_NO_EXCEPT
	{
		return vq_cast(input);
	}

	/// Converts a rotation 3x3 matrix into a VQ transform.
	RTM_DEPRECATED("Use 'vq_from_matrix' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqd RTM_SIMD_CALL qv_from_matrix(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return vq_from_matrix(input);
	}

	/// Converts a rotation 3x4 affine matrix into a VQ transform.
	RTM_DEPRECATED("Use 'vq_from_matrix' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqd RTM_SIMD_CALL qv_from_matrix(matrix3x4d_arg0 input) RTM_NO_EXCEPT
	{
		return vq_from_matrix(input);
	}

	/// Multiplies two VQ transforms.
	RTM_DEPRECATED("Use 'vq_mul' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqd RTM_SIMD_CALL qv_mul(vqd_arg0 lhs, vqd_arg1 rhs) RTM_NO_EXCEPT
	{
		return vq_mul(lhs, rhs);
	}

	/// Multiplies a VQ transform and a 3D point.
	RTM_DEPRECATED("Use 'vq_mul_point3' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vector4d RTM_SIMD_CALL qv_mul_point3(vector4d_arg0 point, vqd_arg1 vq) RTM_NO_EXCEPT
	{
		return vq_mul_point3(point, vq);
	}

	/// Returns the inverse of the input VQ transform.
	RTM_DEPRECATED("Use 'vq_inverse' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline vqd RTM_SIMD_CALL qv_inverse(vqd_arg0 input) RTM_NO_EXCEPT
	{
		return vq_inverse(input);
	}

	/// Returns a VQ transform with the rotation part normalized.
	RTM_DEPRECATED("Use 'vq_normalize' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqd RTM_SIMD_CALL qv_normalize(vqd_arg0 input) RTM_NO_EXCEPT
	{
		return vq_normalize(input);
	}

	/// Per component linear interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vq_lerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqd RTM_SIMD_CALL qv_lerp(vqd_arg0 start, vqd_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		return vq_lerp(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component linear interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vq_lerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqd RTM_SIMD_CALL qv_lerp(vqd_arg0 start, vqd_arg1 end, scalard_argn alpha) RTM_NO_EXCEPT
	{
		return vq_lerp(start, end, alpha);
	}
#endif

	/// Per component spherical interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vq_slerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqd RTM_SIMD_CALL qv_slerp(vqd_arg0 start, vqd_arg1 end, double alpha) RTM_NO_EXCEPT
	{
		return vq_slerp(start, end, alpha);
	}

#if defined(RTM_SSE2_INTRINSICS)
	/// Per component spherical interpolation of the two inputs at the specified alpha.
	RTM_DEPRECATED("Use 'vq_slerp' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vqd RTM_SIMD_CALL qv_slerp(vqd_arg0 start, vqd_arg1 end, scalard_argn alpha) RTM_NO_EXCEPT
	{
		return vq_slerp(start, end, alpha);
	}
#endif

	/// Returns true if the input VQ does not contain any NaN or Inf, otherwise false.
	RTM_DEPRECATED("Use 'vq_is_finite' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL qv_is_finite(vqd_arg0 input) RTM_NO_EXCEPT
	{
		return vq_is_finite(input);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
