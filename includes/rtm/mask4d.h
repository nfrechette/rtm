#pragma once

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

#include "rtm/math.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/mask_common.h"

#if !defined(RTM_SSE2_INTRINSICS)
	#include <cstring>
#endif

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4d [x] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint64_t RTM_SIMD_CALL mask_get_x(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(RTM_ARCH_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(input.xy));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(input.xy));
#endif
#else
		return input.x;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4d [y] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint64_t RTM_SIMD_CALL mask_get_y(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(RTM_ARCH_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(_mm_shuffle_pd(input.xy, input.xy, 1)));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(_mm_shuffle_pd(input.xy, input.xy, 1)));
#endif
#else
		return input.y;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4d [z] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint64_t RTM_SIMD_CALL mask_get_z(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(RTM_ARCH_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(input.zw));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(input.zw));
#endif
#else
		return input.z;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the mask4d [w] component.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE uint64_t RTM_SIMD_CALL mask_get_w(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
#if defined(RTM_ARCH_X64)
		return _mm_cvtsi128_si64(_mm_castpd_si128(_mm_shuffle_pd(input.zw, input.zw, 1)));
#else
		// Just sign extend on 32bit systems
		return (uint64_t)_mm_cvtsi128_si32(_mm_castpd_si128(_mm_shuffle_pd(input.zw, input.zw, 1)));
#endif
#else
		return input.w;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are true, otherwise false: all(input.xyzw != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_true(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_pd(input.xy) & _mm_movemask_pd(input.zw)) == 3;
#else
		return input.x != 0 && input.y != 0 && input.z != 0 && input.w != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are true, otherwise false: all(input.xy != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_true2(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_pd(input.xy) == 3;
#else
		return input.x != 0 && input.y != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are true, otherwise false: all(input.xyz != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_true3(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_pd(input.xy) == 3 && (_mm_movemask_pd(input.zw) & 1) != 0;
#else
		return input.x != 0 && input.y != 0 && input.z != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are true, otherwise false: any(input.xyzw != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_true(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return (_mm_movemask_pd(input.xy) | _mm_movemask_pd(input.zw)) != 0;
#else
		return input.x != 0 || input.y != 0 || input.z != 0 || input.w != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are true, otherwise false: any(input.xy != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_true2(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_pd(input.xy) != 0;
#else
		return input.x != 0 || input.y != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are true, otherwise false: any(input.xyz != 0)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_true3(const mask4d& input) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_pd(input.xy) != 0 || (_mm_movemask_pd(input.zw) & 1) != 0;
#else
		return input.x != 0 || input.y != 0 || input.z != 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all 4 components are equal, otherwise false: all(lhs.xyzw == rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_equal(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_eq_pd) & _mm_movemask_pd(zw_eq_pd)) == 3;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint64_t) * 4) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xy] components are equal, otherwise false: all(lhs.xy == rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_equal2(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_pd(_mm_cmpeq_pd(lhs.xy, rhs.xy)) == 3;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint64_t) * 2) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if all [xyz] components are equal, otherwise false: all(lhs.xyz == rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_all_equal3(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_eq_pd) == 3 && (_mm_movemask_pd(zw_eq_pd) & 1) == 1;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs, &rhs, sizeof(uint64_t) * 3) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any 4 components are equal, otherwise false: any(lhs.xyzw == rhs.xyzw)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_equal(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return (_mm_movemask_pd(xy_eq_pd) & _mm_movemask_pd(zw_eq_pd)) != 0;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs.x, &rhs.x, sizeof(uint64_t)) == 0
			|| std::memcmp(&lhs.y, &rhs.y, sizeof(uint64_t)) == 0
			|| std::memcmp(&lhs.z, &rhs.z, sizeof(uint64_t)) == 0
			|| std::memcmp(&lhs.w, &rhs.w, sizeof(uint64_t)) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xy] components are equal, otherwise false: any(lhs.xy == rhs.xy)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_equal2(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		return _mm_movemask_pd(_mm_cmpeq_pd(lhs.xy, rhs.xy)) != 0;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs.x, &rhs.x, sizeof(uint64_t)) == 0
			|| std::memcmp(&lhs.y, &rhs.y, sizeof(uint64_t)) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns true if any [xyz] components are equal, otherwise false: any(lhs.xyz == rhs.xyz)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE bool RTM_SIMD_CALL mask_any_equal3(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy_eq_pd = _mm_cmpeq_pd(lhs.xy, rhs.xy);
		__m128d zw_eq_pd = _mm_cmpeq_pd(lhs.zw, rhs.zw);
		return _mm_movemask_pd(xy_eq_pd) != 0 && (_mm_movemask_pd(zw_eq_pd) & 1) != 0;
#else
		// Cannot use == and != with NaN floats
		return std::memcmp(&lhs.x, &rhs.x, sizeof(uint64_t)) == 0
			|| std::memcmp(&lhs.y, &rhs.y, sizeof(uint64_t)) == 0
			|| std::memcmp(&lhs.z, &rhs.z, sizeof(uint64_t)) == 0;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical AND between the inputs: lhs & rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d mask_and(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_and_pd(lhs.xy, rhs.xy);
		__m128d zw = _mm_and_pd(lhs.zw, rhs.zw);
		return mask4d{ xy, zw };
#else
		const uint64_t* lhs_ = reinterpret_cast<const uint64_t*>(&lhs);
		const uint64_t* rhs_ = reinterpret_cast<const uint64_t*>(&rhs);

		union
		{
			mask4d vector;
			uint64_t scalar[4];
		} result;

		result.scalar[0] = lhs_[0] & rhs_[0];
		result.scalar[1] = lhs_[1] & rhs_[1];
		result.scalar[2] = lhs_[2] & rhs_[2];
		result.scalar[3] = lhs_[3] & rhs_[3];

		return result.vector;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical OR between the inputs: lhs | rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d mask_or(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_or_pd(lhs.xy, rhs.xy);
		__m128d zw = _mm_or_pd(lhs.zw, rhs.zw);
		return mask4d{ xy, zw };
#else
		const uint64_t* lhs_ = reinterpret_cast<const uint64_t*>(&lhs);
		const uint64_t* rhs_ = reinterpret_cast<const uint64_t*>(&rhs);

		union
		{
			mask4d vector;
			uint64_t scalar[4];
		} result;

		result.scalar[0] = lhs_[0] | rhs_[0];
		result.scalar[1] = lhs_[1] | rhs_[1];
		result.scalar[2] = lhs_[2] | rhs_[2];
		result.scalar[3] = lhs_[3] | rhs_[3];

		return result.vector;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Per component logical XOR between the inputs: lhs ^ rhs
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE mask4d mask_xor(const mask4d& lhs, const mask4d& rhs) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		__m128d xy = _mm_xor_pd(lhs.xy, rhs.xy);
		__m128d zw = _mm_xor_pd(lhs.zw, rhs.zw);
		return mask4d{ xy, zw };
#else
		const uint64_t* lhs_ = reinterpret_cast<const uint64_t*>(&lhs);
		const uint64_t* rhs_ = reinterpret_cast<const uint64_t*>(&rhs);

		union
		{
			mask4d vector;
			uint64_t scalar[4];
		} result;

		result.scalar[0] = lhs_[0] ^ rhs_[0];
		result.scalar[1] = lhs_[1] ^ rhs_[1];
		result.scalar[2] = lhs_[2] ^ rhs_[2];
		result.scalar[3] = lhs_[3] ^ rhs_[3];

		return result.vector;
#endif
	}
}

RTM_IMPL_FILE_PRAGMA_POP
