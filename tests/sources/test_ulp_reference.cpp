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

// This file compiles the scalar code path of RTM in the namespace rtm_ref.
// The other test files compile the code path of the build (SSE2, AVX, NEON, WASM, or scalar) in the namespace rtm.
// The two copies can be in the same executable because the RTM headers declare nothing outside of the namespace rtm.
// If a header declares a function or a type outside of the namespace rtm, the two copies have the same name
// with different definitions. The linker keeps one of them and does not give an error.
//
// Do not include catch2.impl.h or another test header that includes RTM: it would use the namespace rtm_ref.

#if !defined(RTM_NO_INTRINSICS)
	#define RTM_NO_INTRINSICS
#endif

#define rtm rtm_ref

#include <rtm/mask4d.h>
#include <rtm/mask4f.h>
#include <rtm/matrix3x3d.h>
#include <rtm/matrix3x3f.h>
#include <rtm/matrix3x4d.h>
#include <rtm/matrix3x4f.h>
#include <rtm/matrix4x4d.h>
#include <rtm/matrix4x4f.h>
#include <rtm/quatd.h>
#include <rtm/quatf.h>
#include <rtm/scalard.h>
#include <rtm/scalarf.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>
#include <rtm/vqd.h>
#include <rtm/vqf.h>
#include <rtm/vqsd.h>
#include <rtm/vqsf.h>
#include <rtm/vqvd.h>
#include <rtm/vqvf.h>
#include <rtm/packing/quatf.h>

#include <cfloat>
#include <cstring>

#include "test_ulp_functions.h"

#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS) || defined(RTM_WASM128_INTRINSICS)
	#error "The reference code path must not use SIMD intrinsics"
#endif

namespace rtm_ulp_ref
{
	void compute_vector_function(ulp_function function, const float* input0, const float* input1, const float* input2, float* output)
	{
		compute_vector_function_impl(function, input0, input1, input2, output);
	}

	void compute_vector_function(ulp_function function, const double* input0, const double* input1, const double* input2, double* output)
	{
		compute_vector_function_impl(function, input0, input1, input2, output);
	}

	void compute_bit_exact_function(bit_exact_function function, const float* input0, const float* input1, const float* input2, float* output)
	{
		compute_bit_exact_function_impl(function, input0, input1, input2, output);
	}

	void compute_bit_exact_function(bit_exact_function function, const double* input0, const double* input1, const double* input2, double* output)
	{
		compute_bit_exact_function_impl(function, input0, input1, input2, output);
	}

	void compute_transform_function(transform_function function, const float* input, float* output)
	{
		compute_transform_function_impl(function, input, output);
	}

	void compute_transform_function(transform_function function, const double* input, double* output)
	{
		compute_transform_function_impl(function, input, output);
	}

	void quat_normalize_deterministic(const float* input, float* output)
	{
		rtm::quat_store(rtm::quat_normalize_deterministic(rtm::quat_load(input)), output);
	}

	void quat_normalize_deterministic(const double* input, double* output)
	{
		rtm::quat_store(rtm::quat_normalize_deterministic(rtm::quat_load(input)), output);
	}

	bool has_excess_precision()
	{
#if defined(FLT_EVAL_METHOD) && FLT_EVAL_METHOD != 0
		return true;
#else
		return false;
#endif
	}
}
