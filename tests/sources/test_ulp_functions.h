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

// test_ulp.cpp and test_ulp_reference.cpp both include this file after the RTM headers.
// In test_ulp_reference.cpp, the macro "rtm" changes the namespace to rtm_ref: thus the same code
// calls the scalar code path there and the code path of the build in test_ulp.cpp.
// The function is in an unnamed namespace: each file has its own copy, and the linker does not merge them.

#include "test_ulp_reference.h"

namespace
{
	// Computes the vector function on 4 values.
	// atan2 reads y from input0 and x from input1. The other functions do not read input1.
	template<typename FloatType>
	void compute_vector_function_impl(rtm_ulp_ref::ulp_function function, const FloatType* input0, const FloatType* input1, FloatType* output)
	{
		using rtm_ulp_ref::ulp_function;

		const auto input = rtm::vector_load(input0);

		switch (function)
		{
		case ulp_function::exp:				rtm::vector_store(rtm::vector_exp(input), output); break;
		case ulp_function::log:				rtm::vector_store(rtm::vector_log(input), output); break;
		case ulp_function::sqrt:			rtm::vector_store(rtm::vector_sqrt(input), output); break;
		case ulp_function::reciprocal:		rtm::vector_store(rtm::vector_reciprocal(input), output); break;
		case ulp_function::sqrt_reciprocal:	rtm::vector_store(rtm::vector_sqrt_reciprocal(input), output); break;
		case ulp_function::sin:				rtm::vector_store(rtm::vector_sin(input), output); break;
		case ulp_function::cos:				rtm::vector_store(rtm::vector_cos(input), output); break;
		case ulp_function::tan:				rtm::vector_store(rtm::vector_tan(input), output); break;
		case ulp_function::asin:			rtm::vector_store(rtm::vector_asin(input), output); break;
		case ulp_function::acos:			rtm::vector_store(rtm::vector_acos(input), output); break;
		case ulp_function::atan:			rtm::vector_store(rtm::vector_atan(input), output); break;
		case ulp_function::atan2:			rtm::vector_store(rtm::vector_atan2(input, rtm::vector_load(input1)), output); break;
		}
	}
}
