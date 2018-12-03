#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
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

#include "rtm/error.h"
#include "rtm/math.h"

namespace rtm
{
	constexpr qvvf RTM_SIMD_CALL qvv_set(quatf_arg0 rotation, vector4f_arg1 translation, vector4f_arg2 scale) RTM_NO_EXCEPT
	{
		return qvvf{ rotation, translation, scale };
	}

	constexpr qvvd RTM_SIMD_CALL qvv_set(const quatd& rotation, const vector4d& translation, const vector4d& scale) RTM_NO_EXCEPT
	{
		return qvvd{ rotation, translation, scale };
	}

	namespace rtm_impl
	{
		enum class qvv_constants
		{
			identity
		};

		template<qvv_constants constant>
		struct qvv_constant
		{
			inline RTM_SIMD_CALL operator qvvd() RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case qvv_constants::identity:
				default:
					return qvv_set(quat_identity(), vector_zero(), vector_set(1.0));
				}
			}

			inline RTM_SIMD_CALL operator qvvf() RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case qvv_constants::identity:
				default:
					return qvv_set(quat_identity(), vector_zero(), vector_set(1.0f));
				}
			}
		};
	}

	inline rtm_impl::qvv_constant<rtm_impl::qvv_constants::identity> RTM_SIMD_CALL qvv_identity() RTM_NO_EXCEPT
	{
		return rtm_impl::qvv_constant<rtm_impl::qvv_constants::identity>();
	}
}
