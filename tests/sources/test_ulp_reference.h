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

// These functions run the scalar code path of RTM (RTM_NO_INTRINSICS).
// test_ulp_reference.cpp defines them. test_ulp.cpp compares the code path of the build with them.
//
// The two files see different RTM types (for example, vector4f is __m128 in one file and a struct
// in the other). Thus the functions use only arrays of float and double.
// Each function reads 4 values from each input and writes 4 values to the output.

namespace rtm_ulp_ref
{
	// The functions that the tests measure
	enum class ulp_function
	{
		exp,
		log,
		sqrt,
		reciprocal,
		sqrt_reciprocal,
		sin,
		cos,
		tan,
		asin,
		acos,
		atan,
		atan2,
	};

	// Computes the vector function with the scalar code path.
	// atan2 reads y from input0 and x from input1. The other functions do not read input1.
	void compute_vector_function(ulp_function function, const float* input0, const float* input1, float* output);
	void compute_vector_function(ulp_function function, const double* input0, const double* input1, double* output);

	void quat_normalize_deterministic(const float* input, float* output);
	void quat_normalize_deterministic(const double* input, double* output);

	// Returns true if the scalar code path computes with more precision than the float type (FLT_EVAL_METHOD is not 0).
	// For example, GCC uses the x87 registers for 32-bit x86 and does not round each operation to float32 or float64.
	bool has_excess_precision();
}
