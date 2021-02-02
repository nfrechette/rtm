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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULV_ADD(v0, v1, v2) _mm_add_ps(_mm_mul_ps((v0), (v1)), (v2))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULV_ADD(v0, v1, v2) rtm::vector4f { (v2).x + ((v0).x * (v1).x), (v2).y + ((v0).y * (v1).y), (v2).z + ((v0).z * (v1).z), (v2).w + ((v0).w * (v1).w) }
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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULS_ADD(v0, s1, v2) _mm_add_ps(_mm_mul_ps((v0), _mm_set_ps1((s1))), (v2))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component multiplication/addition of the three inputs: v2 + (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_MULS_ADD(v0, s1, v2) rtm::vector4f { (v2).x + ((v0).x * (s1)), (v2).y + ((v0).y * (s1)), (v2).z + ((v0).z * (s1)), (v2).w + ((v0).w * (s1)) }
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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2) _mm_sub_ps((v2), _mm_mul_ps((v0), (v1)))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * v1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * v1)
	// All three inputs must be an rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2) rtm::vector4f { (v2).x - ((v0).x * (v1).x), (v2).y - ((v0).y * (v1).y), (v2).z - ((v0).z * (v1).z), (v2).w - ((v0).w * (v1).w) }
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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2) _mm_sub_ps((v2), _mm_mul_ps((v0), _mm_set_ps1((s1))))
#else
	//////////////////////////////////////////////////////////////////////////
	// Per component negative multiplication/subtraction of the three inputs: -((v0 * s1) - v2)
	// This is mathematically equivalent to: v2 - (v0 * s1)
	// The v0 and v2 inputs must be a rtm::vector4f and s1 must be a float.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2) rtm::vector4f { (v2).x - ((v0).x * (s1)), (v2).y - ((v0).y * (s1)), (v2).z - ((v0).z * (s1)), (v2).w - ((v0).w * (s1)) }
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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input0, input1, input2, input3, output0, output1, output2, output3) \
		do { \
			const __m128 tmp0 = _mm_shuffle_ps((input0), (input1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 tmp1 = _mm_shuffle_ps((input0), (input1), _MM_SHUFFLE(3, 2, 3, 2)); \
			const __m128 tmp2 = _mm_shuffle_ps((input2), (input3), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 tmp3 = _mm_shuffle_ps((input2), (input3), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output0) = _mm_shuffle_ps(tmp0, tmp2, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output1) = _mm_shuffle_ps(tmp0, tmp2, _MM_SHUFFLE(3, 1, 3, 1)); \
			(output2) = _mm_shuffle_ps(tmp1, tmp3, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output3) = _mm_shuffle_ps(tmp1, tmp3, _MM_SHUFFLE(3, 1, 3, 1)); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input0, input1, input2, input3, output0, output1, output2, output3) \
		do { \
			(output0) = rtm::vector4f { (input0).x, (input1).x, (input2).x, (input3).x }; \
			(output1) = rtm::vector4f { (input0).y, (input1).y, (input2).y, (input3).y }; \
			(output2) = rtm::vector4f { (input0).z, (input1).z, (input2).z, (input3).z }; \
			(output3) = rtm::vector4f { (input0).w, (input1).w, (input2).w, (input3).w }; \
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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input0, input1, input2, output0, output1, output2) \
		do { \
			const __m128 tmp0 = _mm_shuffle_ps((input0), (input1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 tmp1 = _mm_shuffle_ps((input0), (input1), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output0) = _mm_shuffle_ps(tmp0, (input2), _MM_SHUFFLE(2, 0, 2, 0)); \
			(output1) = _mm_shuffle_ps(tmp0, (input2), _MM_SHUFFLE(3, 1, 3, 1)); \
			(output2) = _mm_shuffle_ps(tmp1, (input2), _MM_SHUFFLE(2, 2, 2, 0)); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input0, input1, input2, output0, output1, output2) \
		do { \
			(output0) = rtm::vector4f { (input0).x, (input1).x, (input2).x, (input2).x }; \
			(output1) = rtm::vector4f { (input0).y, (input1).y, (input2).y, (input2).y }; \
			(output2) = rtm::vector4f { (input0).z, (input1).z, (input2).z, (input2).z }; \
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
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input0, input1, input2, input3, output0, output1, output2) \
		do { \
			const __m128 tmp0 = _mm_shuffle_ps((input0), (input1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 tmp1 = _mm_shuffle_ps((input0), (input1), _MM_SHUFFLE(3, 2, 3, 2)); \
			const __m128 tmp2 = _mm_shuffle_ps((input2), (input3), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 tmp3 = _mm_shuffle_ps((input2), (input3), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output0) = _mm_shuffle_ps(tmp0, tmp2, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output1) = _mm_shuffle_ps(tmp0, tmp2, _MM_SHUFFLE(3, 1, 3, 1)); \
			(output2) = _mm_shuffle_ps(tmp1, tmp3, _MM_SHUFFLE(2, 0, 2, 0)); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input0, input1, input2, input3, output0, output1, output2) \
		do { \
			(output0) = rtm::vector4f { (input0).x, (input1).x, (input2).x, (input3).x }; \
			(output1) = rtm::vector4f { (input0).y, (input1).y, (input2).y, (input3).y }; \
			(output2) = rtm::vector4f { (input0).z, (input1).z, (input2).z, (input3).z }; \
		} while(0)
#endif

RTM_IMPL_FILE_PRAGMA_POP
