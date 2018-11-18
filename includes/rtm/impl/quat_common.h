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
	namespace rtm_impl
	{
		enum class quat_constants
		{
			identity
		};

		template<quat_constants constant>
		struct quat_constant
		{
			inline RTM_SIMD_CALL operator quatd() RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case quat_constants::identity:
				default:
					return quat_set(0.0, 0.0, 0.0, 1.0);
				}
			}

			inline RTM_SIMD_CALL operator quatf() RTM_NO_EXCEPT
			{
				switch (constant)
				{
				case quat_constants::identity:
				default:
					return quat_set(0.0f, 0.0f, 0.0f, 1.0f);
				}
			}
		};
	}

	inline rtm_impl::quat_constant<rtm_impl::quat_constants::identity> quat_identity() RTM_NO_EXCEPT
	{
		return rtm_impl::quat_constant<rtm_impl::quat_constants::identity>();
	}
}
