#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2024 Nicholas Frechette & Realtime Math contributors
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

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Register passing typedefs
	//////////////////////////////////////////////////////////////////////////

#if defined(RTM_NEON64_INTRINSICS)
	// On ARM64 NEON, the first 8x vector4f/quatf arguments can be passed by value in a register,
	// everything else afterwards is passed by const&. They can also be returned by register.

	using vector4f_arg0 = const vector4f;
	using vector4f_arg1 = const vector4f;
	using vector4f_arg2 = const vector4f;
	using vector4f_arg3 = const vector4f;
	using vector4f_arg4 = const vector4f;
	using vector4f_arg5 = const vector4f;
	using vector4f_arg6 = const vector4f;
	using vector4f_arg7 = const vector4f;
	using vector4f_argn = const vector4f&;

	using quatf_arg0 = const quatf;
	using quatf_arg1 = const quatf;
	using quatf_arg2 = const quatf;
	using quatf_arg3 = const quatf;
	using quatf_arg4 = const quatf;
	using quatf_arg5 = const quatf;
	using quatf_arg6 = const quatf;
	using quatf_arg7 = const quatf;
	using quatf_argn = const quatf&;

	using scalarf_arg0 = const scalarf;
	using scalarf_arg1 = const scalarf;
	using scalarf_arg2 = const scalarf;
	using scalarf_arg3 = const scalarf;
	using scalarf_arg4 = const scalarf;
	using scalarf_arg5 = const scalarf;
	using scalarf_arg6 = const scalarf;
	using scalarf_arg7 = const scalarf;
	using scalarf_argn = const scalarf;

	using scalard_arg0 = const scalard;
	using scalard_arg1 = const scalard;
	using scalard_arg2 = const scalard;
	using scalard_arg3 = const scalard;
	using scalard_arg4 = const scalard;
	using scalard_arg5 = const scalard;
	using scalard_arg6 = const scalard;
	using scalard_arg7 = const scalard;
	using scalard_argn = const scalard;

	using mask4f_arg0 = const mask4f;
	using mask4f_arg1 = const mask4f;
	using mask4f_arg2 = const mask4f;
	using mask4f_arg3 = const mask4f;
	using mask4f_arg4 = const mask4f;
	using mask4f_arg5 = const mask4f;
	using mask4f_arg6 = const mask4f;
	using mask4f_arg7 = const mask4f;
	using mask4f_argn = const mask4f&;

	using mask4i_arg0 = const mask4i;
	using mask4i_arg1 = const mask4i;
	using mask4i_arg2 = const mask4i;
	using mask4i_arg3 = const mask4i;
	using mask4i_arg4 = const mask4i;
	using mask4i_arg5 = const mask4i;
	using mask4i_arg6 = const mask4i;
	using mask4i_arg7 = const mask4i;
	using mask4i_argn = const mask4i&;

	// With ARM64 NEON, an aggregate of up to 4 vector registers is also passed by register,
	// but the whole aggregate must fit in the registers that remain (e.g. we can pass 2x vqvf but not 3x).
	// A vqvf can also be returned by register.
	// The non-scalar types that use doubles are aggregates of 2x float64x2_t or 2x uint64x2_t.
	// Thus, the first 4x vector4d/quatd/mask4d/mask4q arguments are passed by register.
	// A qvd and a vqsd use 4 registers. Thus, the first 2x qvd/vqsd arguments are passed by register.
	// A vqvd and the double matrices use more than 4 registers. Thus, they are passed in memory.
	// When such an argument is passed by value, the caller copies it to the stack and passes a pointer.
	// Thus, we pass them by const& to remove the copy.

	using vector4d_arg0 = const vector4d;
	using vector4d_arg1 = const vector4d;
	using vector4d_arg2 = const vector4d;
	using vector4d_arg3 = const vector4d;
	using vector4d_arg4 = const vector4d&;
	using vector4d_arg5 = const vector4d&;
	using vector4d_arg6 = const vector4d&;
	using vector4d_arg7 = const vector4d&;
	using vector4d_argn = const vector4d&;

	using quatd_arg0 = const quatd;
	using quatd_arg1 = const quatd;
	using quatd_arg2 = const quatd;
	using quatd_arg3 = const quatd;
	using quatd_arg4 = const quatd&;
	using quatd_arg5 = const quatd&;
	using quatd_arg6 = const quatd&;
	using quatd_arg7 = const quatd&;
	using quatd_argn = const quatd&;

	using mask4d_arg0 = const mask4d;
	using mask4d_arg1 = const mask4d;
	using mask4d_arg2 = const mask4d;
	using mask4d_arg3 = const mask4d;
	using mask4d_arg4 = const mask4d&;
	using mask4d_arg5 = const mask4d&;
	using mask4d_arg6 = const mask4d&;
	using mask4d_arg7 = const mask4d&;
	using mask4d_argn = const mask4d&;

	using mask4q_arg0 = const mask4q;
	using mask4q_arg1 = const mask4q;
	using mask4q_arg2 = const mask4q;
	using mask4q_arg3 = const mask4q;
	using mask4q_arg4 = const mask4q&;
	using mask4q_arg5 = const mask4q&;
	using mask4q_arg6 = const mask4q&;
	using mask4q_arg7 = const mask4q&;
	using mask4q_argn = const mask4q&;

	using qvf_arg0 = const qvf;
	using qvf_arg1 = const qvf;
	using qvf_argn = const qvf&;

	using qvd_arg0 = const qvd;
	using qvd_arg1 = const qvd;
	using qvd_argn = const qvd&;

	using vqsf_arg0 = const vqsf;
	using vqsf_arg1 = const vqsf;
	using vqsf_argn = const vqsf&;

	using vqsd_arg0 = const vqsd;
	using vqsd_arg1 = const vqsd;
	using vqsd_argn = const vqsd&;

	using vqvf_arg0 = const vqvf;
	using vqvf_arg1 = const vqvf;
	using vqvf_argn = const vqvf&;

	using vqvd_arg0 = const vqvd&;
	using vqvd_arg1 = const vqvd&;
	using vqvd_argn = const vqvd&;

	using matrix3x3f_arg0 = const matrix3x3f;
	using matrix3x3f_arg1 = const matrix3x3f;
	using matrix3x3f_argn = const matrix3x3f&;

	// TODO: Pass by const& as well. With const&, GCC 15.2 stops with an internal compiler error
	// in simplify-rtx.cc when it compiles test_matrix4x4.cpp.
	using matrix3x3d_arg0 = const matrix3x3d;
	using matrix3x3d_arg1 = const matrix3x3d&;
	using matrix3x3d_argn = const matrix3x3d&;

	using matrix3x4f_arg0 = const matrix3x4f;
	using matrix3x4f_arg1 = const matrix3x4f;
	using matrix3x4f_argn = const matrix3x4f&;

	using matrix3x4d_arg0 = const matrix3x4d&;
	using matrix3x4d_arg1 = const matrix3x4d&;
	using matrix3x4d_argn = const matrix3x4d&;

	using matrix4x4f_arg0 = const matrix4x4f;
	using matrix4x4f_arg1 = const matrix4x4f;
	using matrix4x4f_argn = const matrix4x4f&;

	using matrix4x4d_arg0 = const matrix4x4d&;
	using matrix4x4d_arg1 = const matrix4x4d&;
	using matrix4x4d_argn = const matrix4x4d&;
#endif

	RTM_IMPL_VERSION_NAMESPACE_END
}
