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

#include "test_ulp_reference.h"

#include <rtm/quatd.h>
#include <rtm/quatf.h>
#include <rtm/vector4d.h>
#include <rtm/vector4f.h>

#include <cfloat>

#if defined(RTM_SSE2_INTRINSICS) || defined(RTM_NEON_INTRINSICS) || defined(RTM_WASM128_INTRINSICS)
	#error "The reference code path must not use SIMD intrinsics"
#endif

namespace rtm_ulp_ref
{
	void vector_exp(const float* input, float* output)
	{
		rtm::vector_store(rtm::vector_exp(rtm::vector_load(input)), output);
	}

	void vector_exp(const double* input, double* output)
	{
		rtm::vector_store(rtm::vector_exp(rtm::vector_load(input)), output);
	}

	void vector_log(const float* input, float* output)
	{
		rtm::vector_store(rtm::vector_log(rtm::vector_load(input)), output);
	}

	void vector_log(const double* input, double* output)
	{
		rtm::vector_store(rtm::vector_log(rtm::vector_load(input)), output);
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
