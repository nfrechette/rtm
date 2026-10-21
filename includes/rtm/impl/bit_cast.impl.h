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

#include "rtm/config.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/detect_cpp_version.h"

// Detect if the current compilation environment supports bit operations.

#if !defined(RTM_NO_BIT_CAST)
	#if !defined(__cpp_lib_bitops)
		#define RTM_NO_BIT_CAST
	#endif
#endif

// See config.h for details on how to configure std::bit_cast for your project

// Use RTM_NO_BIT_CAST to disable std::bit_cast
#if RTM_CPP_VERSION >= RTM_CPP_VERSION_20 && !defined(RTM_NO_BIT_CAST)
	#include <bit>
#endif

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	namespace rtm_impl
	{
		// C++20 introduced std::bit_cast which is safer than reinterpret_cast

	#if RTM_CPP_VERSION >= RTM_CPP_VERSION_20 && !defined(RTM_NO_BIT_CAST)
		using std::bit_cast;
	#else
		template<class dest_type_t, class src_type_t>
		RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr dest_type_t bit_cast(src_type_t input) noexcept
		{
			return reinterpret_cast<dest_type_t>(input);
		}
	#endif
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
