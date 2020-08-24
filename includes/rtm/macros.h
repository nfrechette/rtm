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

RTM_IMPL_FILE_PRAGMA_PUSH

//////////////////////////////////////////////////////////////////////////
// This file contains helper macros to help improve code generation where required.
//////////////////////////////////////////////////////////////////////////

#if defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULV_ADD(v0, v1, v2) vfmaq_f32((v2), (v0), (v1))
#elif defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULV_ADD(v0, v1, v2) vmlaq_f32((v2), (v0), (v1))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULV_ADD(v0, v1, v2) rtm::vector_add(rtm::vector_mul((v0), (v1)), (v2))
#endif

#if defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULS_ADD(v0, s1, v2) vfmaq_n_f32((v2), (v0), (s1))
#elif defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULS_ADD(v0, s1, v2) vmlaq_n_f32((v2), (v0), (s1))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULS_ADD(v0, s1, v2) rtm::vector_add(rtm::vector_mul((v0), (s1)), (v2))
#endif

#if defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2) vfmsq_f32((v2), (v0), (v1))
#elif defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2) vmlsq_f32((v2), (v0), (v1))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2) rtm::vector_sub((v2), rtm::vector_mul((v0), (v1)))
#endif

#if defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2) vfmsq_n_f32((v2), (v0), (s1))
#elif defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2) vmlsq_n_f32((v2), (v0), (s1))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2) rtm::vector_sub((v2), rtm::vector_mul((v0), (s1)))
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input0, input1, input2, input3, output0, output1, output2, output3) \
		do { \
			const float32x4x2_t tmp0 = vzipq_f32(input0, input2); \
			const float32x4x2_t tmp1 = vzipq_f32(input1, input3); \
			const float32x4x2_t tmp2 = vzipq_f32(tmp0.val[0], tmp1.val[0]); \
			const float32x4x2_t tmp3 = vzipq_f32(tmp0.val[1], tmp1.val[1]); \
			(output0) = tmp2.val[0]; \
			(output1) = tmp2.val[1]; \
			(output2) = tmp3.val[0]; \
			(output3) = tmp3.val[1]; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input0, input1, input2, input3, output0, output1, output2, output3) \
		do { \
			const rtm::vector4f tmp0 = rtm::vector_mix<rtm::mix4::x, rtm::mix4::y, rtm::mix4::a, rtm::mix4::b>((input0), (input1)); \
			const rtm::vector4f tmp1 = rtm::vector_mix<rtm::mix4::z, rtm::mix4::w, rtm::mix4::c, rtm::mix4::d>((input0), (input1)); \
			const rtm::vector4f tmp2 = rtm::vector_mix<rtm::mix4::x, rtm::mix4::y, rtm::mix4::a, rtm::mix4::b>((input2), (input3)); \
			const rtm::vector4f tmp3 = rtm::vector_mix<rtm::mix4::z, rtm::mix4::w, rtm::mix4::c, rtm::mix4::d>((input2), (input3)); \
			(output0) = rtm::vector_mix<rtm::mix4::x, rtm::mix4::z, rtm::mix4::a, rtm::mix4::c>(tmp0, tmp2); \
			(output1) = rtm::vector_mix<rtm::mix4::y, rtm::mix4::w, rtm::mix4::b, rtm::mix4::d>(tmp0, tmp2); \
			(output2) = rtm::vector_mix<rtm::mix4::x, rtm::mix4::z, rtm::mix4::a, rtm::mix4::c>(tmp1, tmp3); \
			(output3) = rtm::vector_mix<rtm::mix4::y, rtm::mix4::w, rtm::mix4::b, rtm::mix4::d>(tmp1, tmp3); \
		} while(0)
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input0, input1, input2, output0, output1, output2) \
		do { \
			const float32x4x2_t tmp0 = vzipq_f32(input0, input2); \
			const float32x4x2_t tmp1 = vzipq_f32(input1, input1); \
			const float32x4x2_t tmp2 = vzipq_f32(tmp0.val[0], tmp1.val[0]); \
			const float32x4x2_t tmp3 = vzipq_f32(tmp0.val[1], tmp1.val[1]); \
			(output0) = tmp2.val[0]; \
			(output1) = tmp2.val[1]; \
			(output2) = tmp3.val[0]; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input0, input1, input2, output0, output1, output2) \
		do { \
			const rtm::vector4f tmp0 = rtm::vector_mix<rtm::mix4::x, rtm::mix4::y, rtm::mix4::a, rtm::mix4::b>((input0), (input1)); \
			const rtm::vector4f tmp1 = rtm::vector_mix<rtm::mix4::z, rtm::mix4::w, rtm::mix4::c, rtm::mix4::d>((input0), (input1)); \
			(output0) = rtm::vector_mix<rtm::mix4::x, rtm::mix4::z, rtm::mix4::a, rtm::mix4::c>(tmp0, (input2)); \
			(output1) = rtm::vector_mix<rtm::mix4::y, rtm::mix4::w, rtm::mix4::b, rtm::mix4::d>(tmp0, (input2)); \
			(output2) = rtm::vector_mix<rtm::mix4::x, rtm::mix4::z, rtm::mix4::c, rtm::mix4::c>(tmp1, (input2)); \
		} while(0)
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input0, input1, input2, input3, output0, output1, output2) \
		do { \
			const float32x4x2_t tmp0 = vzipq_f32(input0, input2); \
			const float32x4x2_t tmp1 = vzipq_f32(input1, input3); \
			const float32x4x2_t tmp2 = vzipq_f32(tmp0.val[0], tmp1.val[0]); \
			const float32x4x2_t tmp3 = vzipq_f32(tmp0.val[1], tmp1.val[1]); \
			(output0) = tmp2.val[0]; \
			(output1) = tmp2.val[1]; \
			(output2) = tmp3.val[0]; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input0, input1, input2, input3, output0, output1, output2) \
		do { \
			const rtm::vector4f tmp0 = rtm::vector_mix<rtm::mix4::x, rtm::mix4::y, rtm::mix4::a, rtm::mix4::b>((input0), (input1)); \
			const rtm::vector4f tmp1 = rtm::vector_mix<rtm::mix4::z, rtm::mix4::w, rtm::mix4::c, rtm::mix4::d>((input0), (input1)); \
			const rtm::vector4f tmp2 = rtm::vector_mix<rtm::mix4::x, rtm::mix4::y, rtm::mix4::a, rtm::mix4::b>((input2), (input3)); \
			const rtm::vector4f tmp3 = rtm::vector_mix<rtm::mix4::z, rtm::mix4::w, rtm::mix4::c, rtm::mix4::d>((input2), (input3)); \
			(output0) = rtm::vector_mix<rtm::mix4::x, rtm::mix4::z, rtm::mix4::a, rtm::mix4::c>(tmp0, tmp2); \
			(output1) = rtm::vector_mix<rtm::mix4::y, rtm::mix4::w, rtm::mix4::b, rtm::mix4::d>(tmp0, tmp2); \
			(output2) = rtm::vector_mix<rtm::mix4::x, rtm::mix4::z, rtm::mix4::a, rtm::mix4::c>(tmp1, tmp3); \
		} while(0)
#endif

RTM_IMPL_FILE_PRAGMA_POP
