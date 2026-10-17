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

#include "rtm/types.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"

//////////////////////////////////////////////////////////////////////////
// Register passing typedefs
//////////////////////////////////////////////////////////////////////////

#if defined(RTM_USE_VECTORCALL)
	#include "rtm/impl/type_args.vectorcall.impl.h"
#elif defined(RTM_NEON64_INTRINSICS)
	#include "rtm/impl/type_args.neon64.impl.h"
#elif defined(RTM_NEON_INTRINSICS)
	#include "rtm/impl/type_args.neon.impl.h"
#elif defined(RTM_WASM128_INTRINSICS)
	#include "rtm/impl/type_args.wasm.impl.h"
#elif defined(RTM_ARCH_X64) && defined(RTM_COMPILER_GCC)
	#include "rtm/impl/type_args.x64_gcc.impl.h"
#elif defined(RTM_ARCH_X64) && defined(RTM_COMPILER_CLANG)
	#include "rtm/impl/type_args.x64_clang.impl.h"
#else
	#include "rtm/impl/type_args.other.impl.h"
#endif

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// The previous names of the vqsf_arg0, vqsf_arg1, and vqsf_argn types.
	//////////////////////////////////////////////////////////////////////////
	using qvsf_arg0 RTM_DEPRECATED("Use 'vqsf_arg0' instead. To be removed in 2.5.") = vqsf_arg0;
	using qvsf_arg1 RTM_DEPRECATED("Use 'vqsf_arg1' instead. To be removed in 2.5.") = vqsf_arg1;
	using qvsf_argn RTM_DEPRECATED("Use 'vqsf_argn' instead. To be removed in 2.5.") = vqsf_argn;

	//////////////////////////////////////////////////////////////////////////
	// The previous names of the vqsd_arg0, vqsd_arg1, and vqsd_argn types.
	//////////////////////////////////////////////////////////////////////////
	using qvsd_arg0 RTM_DEPRECATED("Use 'vqsd_arg0' instead. To be removed in 2.5.") = vqsd_arg0;
	using qvsd_arg1 RTM_DEPRECATED("Use 'vqsd_arg1' instead. To be removed in 2.5.") = vqsd_arg1;
	using qvsd_argn RTM_DEPRECATED("Use 'vqsd_argn' instead. To be removed in 2.5.") = vqsd_argn;

	//////////////////////////////////////////////////////////////////////////
	// The previous names of the vqvf_arg0, vqvf_arg1, and vqvf_argn types.
	//////////////////////////////////////////////////////////////////////////
	using qvvf_arg0 RTM_DEPRECATED("Use 'vqvf_arg0' instead. To be removed in 2.5.") = vqvf_arg0;
	using qvvf_arg1 RTM_DEPRECATED("Use 'vqvf_arg1' instead. To be removed in 2.5.") = vqvf_arg1;
	using qvvf_argn RTM_DEPRECATED("Use 'vqvf_argn' instead. To be removed in 2.5.") = vqvf_argn;

	//////////////////////////////////////////////////////////////////////////
	// The previous names of the vqvd_arg0, vqvd_arg1, and vqvd_argn types.
	//////////////////////////////////////////////////////////////////////////
	using qvvd_arg0 RTM_DEPRECATED("Use 'vqvd_arg0' instead. To be removed in 2.5.") = vqvd_arg0;
	using qvvd_arg1 RTM_DEPRECATED("Use 'vqvd_arg1' instead. To be removed in 2.5.") = vqvd_arg1;
	using qvvd_argn RTM_DEPRECATED("Use 'vqvd_argn' instead. To be removed in 2.5.") = vqvd_argn;

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
