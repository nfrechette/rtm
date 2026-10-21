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
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"
#include "rtm/impl/vq_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

// Deprecated in 2.4, to be removed in 2.5
// This header keeps the previous qv names. Use rtm/impl/vq_common.h instead.

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	/// Creates a VQ transform from a rotation quaternion and a translation.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use 'vq_set' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vqf RTM_SIMD_CALL qv_set(quatf_arg0 rotation, vector4f_arg1 translation) RTM_NO_EXCEPT
	{
		return vq_set(rotation, translation);
	}

	//////////////////////////////////////////////////////////////////////////
	/// Creates a VQ transform from a rotation quaternion and a translation.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use 'vq_set' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vqd RTM_SIMD_CALL qv_set(quatd_arg0 rotation, vector4d_arg1 translation) RTM_NO_EXCEPT
	{
		return vq_set(rotation, translation);
	}

	//////////////////////////////////////////////////////////////////////////
	/// Returns the identity VQ transform.
	//////////////////////////////////////////////////////////////////////////
	RTM_DEPRECATED("Use 'vq_identity' instead. To be removed in 2.5.")
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr rtm_impl::vq_identity_impl RTM_SIMD_CALL qv_identity() RTM_NO_EXCEPT
	{
		return vq_identity();
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
