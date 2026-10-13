#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2022 Nicholas Frechette & Realtime Math contributors
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

// Included only once from macros.h

#include "rtm/math.h"
#include "rtm/impl/compiler_utils.h"

RTM_IMPL_FILE_PRAGMA_PUSH

//////////////////////////////////////////////////////////////////////////
// This file contains helper macros to help improve code generation where required.
//////////////////////////////////////////////////////////////////////////

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const float32x4x2_t x0x2y0y2_z0z2w0w2 = vzipq_f32((input_xyzw0), (input_xyzw2)); \
			const float32x4x2_t x1x3y1y3_z1z3w1w3 = vzipq_f32((input_xyzw1), (input_xyzw3)); \
			const float32x4x2_t x0x1x2x3_y0y1y2y3 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[0], x1x3y1y3_z1z3w1w3.val[0]); \
			const float32x4x2_t z0z1z2z3_w0w1w2w3 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[1], x1x3y1y3_z1z3w1w3.val[1]); \
			(output_xxxx) = x0x1x2x3_y0y1y2y3.val[0]; \
			(output_yyyy) = x0x1x2x3_y0y1y2y3.val[1]; \
			(output_zzzz) = z0z1z2z3_w0w1w2w3.val[0]; \
			(output_wwww) = z0z1z2z3_w0w1w2w3.val[1]; \
		} while(0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const __m128 x0y0x1y1 = _mm_shuffle_ps((input_xyzw0), (input_xyzw1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 z0w0z1w1 = _mm_shuffle_ps((input_xyzw0), (input_xyzw1), _MM_SHUFFLE(3, 2, 3, 2)); \
			const __m128 x2y2x3y3 = _mm_shuffle_ps((input_xyzw2), (input_xyzw3), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 z2w2z3w3 = _mm_shuffle_ps((input_xyzw2), (input_xyzw3), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output_xxxx) = _mm_shuffle_ps(x0y0x1y1, x2y2x3y3, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output_yyyy) = _mm_shuffle_ps(x0y0x1y1, x2y2x3y3, _MM_SHUFFLE(3, 1, 3, 1)); \
			(output_zzzz) = _mm_shuffle_ps(z0w0z1w1, z2w2z3w3, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output_wwww) = _mm_shuffle_ps(z0w0z1w1, z2w2z3w3, _MM_SHUFFLE(3, 1, 3, 1)); \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const v128_t x0y0x1y1 = wasm_i32x4_shuffle((input_xyzw0), (input_xyzw1), 0, 1, 4, 5); \
			const v128_t z0w0z1w1 = wasm_i32x4_shuffle((input_xyzw0), (input_xyzw1), 2, 3, 6, 7); \
			const v128_t x2y2x3y3 = wasm_i32x4_shuffle((input_xyzw2), (input_xyzw3), 0, 1, 4, 5); \
			const v128_t z2w2z3w3 = wasm_i32x4_shuffle((input_xyzw2), (input_xyzw3), 2, 3, 6, 7); \
			(output_xxxx) = wasm_i32x4_shuffle(x0y0x1y1, x2y2x3y3, 0, 2, 4, 6); \
			(output_yyyy) = wasm_i32x4_shuffle(x0y0x1y1, x2y2x3y3, 1, 3, 5, 7); \
			(output_zzzz) = wasm_i32x4_shuffle(z0w0z1w1, z2w2z3w3, 0, 2, 4, 6); \
			(output_wwww) = wasm_i32x4_shuffle(z0w0z1w1, z2w2z3w3, 1, 3, 5, 7); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const float input_x0 = (input_xyzw0).x; \
			const float input_y0 = (input_xyzw0).y; \
			const float input_z0 = (input_xyzw0).z; \
			const float input_w0 = (input_xyzw0).w; \
			const float input_x1 = (input_xyzw1).x; \
			const float input_y1 = (input_xyzw1).y; \
			const float input_z1 = (input_xyzw1).z; \
			const float input_w1 = (input_xyzw1).w; \
			const float input_x2 = (input_xyzw2).x; \
			const float input_y2 = (input_xyzw2).y; \
			const float input_z2 = (input_xyzw2).z; \
			const float input_w2 = (input_xyzw2).w; \
			const float input_x3 = (input_xyzw3).x; \
			const float input_y3 = (input_xyzw3).y; \
			const float input_z3 = (input_xyzw3).z; \
			const float input_w3 = (input_xyzw3).w; \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4f { input_x0, input_x1, input_x2, input_x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4f { input_y0, input_y1, input_y2, input_y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4f { input_z0, input_z1, input_z2, input_z3 }; \
			(output_wwww) = RTM_IMPL_NAMESPACE::vector4f { input_w0, input_w1, input_w2, input_w3 }; \
		} while(0)
#endif

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const __m128d x0x1 = _mm_unpacklo_pd(input_xyzw0.xy, input_xyzw1.xy); \
			const __m128d y0y1 = _mm_unpackhi_pd(input_xyzw0.xy, input_xyzw1.xy); \
			const __m128d x2x3 = _mm_unpacklo_pd(input_xyzw2.xy, input_xyzw3.xy); \
			const __m128d y2y3 = _mm_unpackhi_pd(input_xyzw2.xy, input_xyzw3.xy); \
			const __m128d z0z1 = _mm_unpacklo_pd(input_xyzw0.zw, input_xyzw1.zw); \
			const __m128d w0w1 = _mm_unpackhi_pd(input_xyzw0.zw, input_xyzw1.zw); \
			const __m128d z2z3 = _mm_unpacklo_pd(input_xyzw2.zw, input_xyzw3.zw); \
			const __m128d w2w3 = _mm_unpackhi_pd(input_xyzw2.zw, input_xyzw3.zw); \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x2x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, z2z3 }; \
			(output_wwww) = RTM_IMPL_NAMESPACE::vector4d { w0w1, w2w3 }; \
		} while(0)
#elif defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const float64x2_t x0x1 = vzip1q_f64(input_xyzw0.xy, input_xyzw1.xy); \
			const float64x2_t y0y1 = vzip2q_f64(input_xyzw0.xy, input_xyzw1.xy); \
			const float64x2_t x2x3 = vzip1q_f64(input_xyzw2.xy, input_xyzw3.xy); \
			const float64x2_t y2y3 = vzip2q_f64(input_xyzw2.xy, input_xyzw3.xy); \
			const float64x2_t z0z1 = vzip1q_f64(input_xyzw0.zw, input_xyzw1.zw); \
			const float64x2_t w0w1 = vzip2q_f64(input_xyzw0.zw, input_xyzw1.zw); \
			const float64x2_t z2z3 = vzip1q_f64(input_xyzw2.zw, input_xyzw3.zw); \
			const float64x2_t w2w3 = vzip2q_f64(input_xyzw2.zw, input_xyzw3.zw); \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x2x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, z2z3 }; \
			(output_wwww) = RTM_IMPL_NAMESPACE::vector4d { w0w1, w2w3 }; \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const v128_t x0x1 = wasm_i64x2_shuffle(input_xyzw0.xy, input_xyzw1.xy, 0, 2); \
			const v128_t y0y1 = wasm_i64x2_shuffle(input_xyzw0.xy, input_xyzw1.xy, 1, 3); \
			const v128_t x2x3 = wasm_i64x2_shuffle(input_xyzw2.xy, input_xyzw3.xy, 0, 2); \
			const v128_t y2y3 = wasm_i64x2_shuffle(input_xyzw2.xy, input_xyzw3.xy, 1, 3); \
			const v128_t z0z1 = wasm_i64x2_shuffle(input_xyzw0.zw, input_xyzw1.zw, 0, 2); \
			const v128_t w0w1 = wasm_i64x2_shuffle(input_xyzw0.zw, input_xyzw1.zw, 1, 3); \
			const v128_t z2z3 = wasm_i64x2_shuffle(input_xyzw2.zw, input_xyzw3.zw, 0, 2); \
			const v128_t w2w3 = wasm_i64x2_shuffle(input_xyzw2.zw, input_xyzw3.zw, 1, 3); \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x2x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, z2z3 }; \
			(output_wwww) = RTM_IMPL_NAMESPACE::vector4d { w0w1, w2w3 }; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X4(input_xyzw0, input_xyzw1, input_xyzw2, input_xyzw3, output_xxxx, output_yyyy, output_zzzz, output_wwww) \
		do { \
			const double input_x0 = (input_xyzw0).x; \
			const double input_y0 = (input_xyzw0).y; \
			const double input_z0 = (input_xyzw0).z; \
			const double input_w0 = (input_xyzw0).w; \
			const double input_x1 = (input_xyzw1).x; \
			const double input_y1 = (input_xyzw1).y; \
			const double input_z1 = (input_xyzw1).z; \
			const double input_w1 = (input_xyzw1).w; \
			const double input_x2 = (input_xyzw2).x; \
			const double input_y2 = (input_xyzw2).y; \
			const double input_z2 = (input_xyzw2).z; \
			const double input_w2 = (input_xyzw2).w; \
			const double input_x3 = (input_xyzw3).x; \
			const double input_y3 = (input_xyzw3).y; \
			const double input_z3 = (input_xyzw3).z; \
			const double input_w3 = (input_xyzw3).w; \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { input_x0, input_x1, input_x2, input_x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { input_y0, input_y1, input_y2, input_y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { input_z0, input_z1, input_z2, input_z3 }; \
			(output_wwww) = RTM_IMPL_NAMESPACE::vector4d { input_w0, input_w1, input_w2, input_w3 }; \
		} while(0)
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const float32x4x2_t x0x2y0y2_z0z2w0w2 = vzipq_f32((input_xyz0), (input_xyz2)); \
			const float32x4x2_t x1x1y1y1_z1z1w1w1 = vzipq_f32((input_xyz1), (input_xyz1)); \
			const float32x4x2_t x0x1x2x1_y0y1y2y1 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[0], x1x1y1y1_z1z1w1w1.val[0]); \
			const float32x4x2_t z0z1z2z1_w0w1w2w1 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[1], x1x1y1y1_z1z1w1w1.val[1]); \
			(output_xxx) = x0x1x2x1_y0y1y2y1.val[0]; \
			(output_yyy) = x0x1x2x1_y0y1y2y1.val[1]; \
			(output_zzz) = z0z1z2z1_w0w1w2w1.val[0]; \
		} while(0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const __m128 input_xyz2_ = (input_xyz2); \
			const __m128 x0y0x1y1 = _mm_shuffle_ps((input_xyz0), (input_xyz1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 z0w0z1w1 = _mm_shuffle_ps((input_xyz0), (input_xyz1), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output_xxx) = _mm_shuffle_ps(x0y0x1y1, input_xyz2_, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output_yyy) = _mm_shuffle_ps(x0y0x1y1, input_xyz2_, _MM_SHUFFLE(3, 1, 3, 1)); \
			(output_zzz) = _mm_shuffle_ps(z0w0z1w1, input_xyz2_, _MM_SHUFFLE(2, 2, 2, 0)); \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const v128_t input_xyz2_ = (input_xyz2); \
			const v128_t x0y0x1y1 = wasm_i32x4_shuffle((input_xyz0), (input_xyz1), 0, 1, 4, 5); \
			const v128_t z0w0z1w1 = wasm_i32x4_shuffle((input_xyz0), (input_xyz1), 2, 3, 6, 7); \
			(output_xxx) = wasm_i32x4_shuffle(x0y0x1y1, input_xyz2_, 0, 2, 4, 6); \
			(output_yyy) = wasm_i32x4_shuffle(x0y0x1y1, input_xyz2_, 1, 3, 5, 7); \
			(output_zzz) = wasm_i32x4_shuffle(z0w0z1w1, input_xyz2_, 0, 2, 6, 6); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const float input_x0 = (input_xyz0).x; \
			const float input_y0 = (input_xyz0).y; \
			const float input_z0 = (input_xyz0).z; \
			const float input_x1 = (input_xyz1).x; \
			const float input_y1 = (input_xyz1).y; \
			const float input_z1 = (input_xyz1).z; \
			const float input_x2 = (input_xyz2).x; \
			const float input_y2 = (input_xyz2).y; \
			const float input_z2 = (input_xyz2).z; \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4f { input_x0, input_x1, input_x2, input_x2 }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4f { input_y0, input_y1, input_y2, input_y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4f { input_z0, input_z1, input_z2, input_z2 }; \
		} while(0)
#endif

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const __m128d x0x1 = _mm_shuffle_pd(input_xyz0.xy, input_xyz1.xy, _MM_SHUFFLE2(0, 0)); \
			const __m128d y0y1 = _mm_shuffle_pd(input_xyz0.xy, input_xyz1.xy, _MM_SHUFFLE2(1, 1)); \
			const __m128d y2 = _mm_shuffle_pd(input_xyz2.xy, input_xyz2.xy, _MM_SHUFFLE2(1, 1)); \
			const __m128d z0z1 = _mm_shuffle_pd(input_xyz0.zw, input_xyz1.zw, _MM_SHUFFLE2(0, 0)); \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, input_xyz2.xy }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, input_xyz2.zw }; \
		} while(0)
#elif defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const float64x2_t x0x1 = vzip1q_f64(input_xyz0.xy, input_xyz1.xy); \
			const float64x2_t y0y1 = vzip2q_f64(input_xyz0.xy, input_xyz1.xy); \
			const float64x2_t y2 = vzip2q_f64(input_xyz2.xy, input_xyz2.xy); \
			const float64x2_t z0z1 = vzip1q_f64(input_xyz0.zw, input_xyz1.zw); \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, input_xyz2.xy }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, input_xyz2.zw }; \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const v128_t x0x1 = wasm_i64x2_shuffle(input_xyz0.xy, input_xyz1.xy, 0, 2); \
			const v128_t y0y1 = wasm_i64x2_shuffle(input_xyz0.xy, input_xyz1.xy, 1, 3); \
			const v128_t y2 = wasm_i64x2_shuffle(input_xyz2.xy, input_xyz2.xy, 1, 3); \
			const v128_t z0z1 = wasm_i64x2_shuffle(input_xyz0.zw, input_xyz1.zw, 0, 2); \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, input_xyz2.xy }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, input_xyz2.zw }; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X3(input_xyz0, input_xyz1, input_xyz2, output_xxx, output_yyy, output_zzz) \
		do { \
			const double input_x0 = (input_xyz0).x; \
			const double input_y0 = (input_xyz0).y; \
			const double input_z0 = (input_xyz0).z; \
			const double input_x1 = (input_xyz1).x; \
			const double input_y1 = (input_xyz1).y; \
			const double input_z1 = (input_xyz1).z; \
			const double input_x2 = (input_xyz2).x; \
			const double input_y2 = (input_xyz2).y; \
			const double input_z2 = (input_xyz2).z; \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { input_x0, input_x1, input_x2, input_x2 }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { input_y0, input_y1, input_y2, input_y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { input_z0, input_z1, input_z2, input_z2 }; \
		} while(0)
#endif

#if defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const float32x4_t x0x1y0y1 = vzip1q_f32((input_xy0), (input_xy1)); \
			(output_xx) = x0x1y0y1; \
			(output_yy) = vextq_f32(x0x1y0y1, x0x1y0y1, 2); \
		} while(0)
#elif defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4f.
	// vzip1q_f32 is only available with ARM64, the first half of vzipq_f32 has the same lanes.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const float32x4_t x0x1y0y1 = vzipq_f32((input_xy0), (input_xy1)).val[0]; \
			(output_xx) = x0x1y0y1; \
			(output_yy) = vextq_f32(x0x1y0y1, x0x1y0y1, 2); \
		} while(0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const __m128 x0x1y0y1 = _mm_unpacklo_ps((input_xy0), (input_xy1)); \
			(output_xx) = x0x1y0y1; \
			(output_yy) = _mm_movehl_ps(x0x1y0y1, x0x1y0y1); \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const v128_t x0x1y0y1 = wasm_i32x4_shuffle((input_xy0), (input_xy1), 0, 4, 1, 5); \
			(output_xx) = x0x1y0y1; \
			(output_yy) = wasm_i32x4_shuffle(x0x1y0y1, x0x1y0y1, 6, 7, 2, 3); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const float input_x0 = (input_xy0).x; \
			const float input_y0 = (input_xy0).y; \
			const float input_x1 = (input_xy1).x; \
			const float input_y1 = (input_xy1).y; \
			(output_xx) = RTM_IMPL_NAMESPACE::vector4f { input_x0, input_x1, input_x0, input_x1 }; \
			(output_yy) = RTM_IMPL_NAMESPACE::vector4f { input_y0, input_y1, input_y0, input_y1 }; \
		} while(0)
#endif

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const __m128d x0x1 = _mm_unpacklo_pd((input_xy0).xy, (input_xy1).xy); \
			const __m128d y0y1 = _mm_unpackhi_pd((input_xy0).xy, (input_xy1).xy); \
			(output_xx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x0x1 }; \
			(output_yy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y0y1 }; \
		} while(0)
#elif defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const float64x2_t x0x1 = vzip1q_f64((input_xy0).xy, (input_xy1).xy); \
			const float64x2_t y0y1 = vzip2q_f64((input_xy0).xy, (input_xy1).xy); \
			(output_xx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x0x1 }; \
			(output_yy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y0y1 }; \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const v128_t x0x1 = wasm_i64x2_shuffle((input_xy0).xy, (input_xy1).xy, 0, 2); \
			const v128_t y0y1 = wasm_i64x2_shuffle((input_xy0).xy, (input_xy1).xy, 1, 3); \
			(output_xx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x0x1 }; \
			(output_yy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y0y1 }; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 2x2 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_2X2(input_xy0, input_xy1, output_xx, output_yy) \
		do { \
			const double input_x0 = (input_xy0).x; \
			const double input_y0 = (input_xy0).y; \
			const double input_x1 = (input_xy1).x; \
			const double input_y1 = (input_xy1).y; \
			(output_xx) = RTM_IMPL_NAMESPACE::vector4d { input_x0, input_x1, input_x0, input_x1 }; \
			(output_yy) = RTM_IMPL_NAMESPACE::vector4d { input_y0, input_y1, input_y0, input_y1 }; \
		} while(0)
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const float32x4x2_t x0x2y0y2_z0z2w0w2 = vzipq_f32((input_xyz0), (input_xyz2)); \
			const float32x4x2_t x1x3y1y3_z1z3w1w3 = vzipq_f32((input_xyz1), (input_xyz3)); \
			const float32x4x2_t x0x1x2x3_y0y1y2y3 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[0], x1x3y1y3_z1z3w1w3.val[0]); \
			const float32x4x2_t z0z1z2z3_w0w1w2w3 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[1], x1x3y1y3_z1z3w1w3.val[1]); \
			(output_xxxx) = x0x1x2x3_y0y1y2y3.val[0]; \
			(output_yyyy) = x0x1x2x3_y0y1y2y3.val[1]; \
			(output_zzzz) = z0z1z2z3_w0w1w2w3.val[0]; \
		} while(0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const __m128 x0y0x1y1 = _mm_shuffle_ps((input_xyz0), (input_xyz1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 z0w0z1w1 = _mm_shuffle_ps((input_xyz0), (input_xyz1), _MM_SHUFFLE(3, 2, 3, 2)); \
			const __m128 x2y2x3y3 = _mm_shuffle_ps((input_xyz2), (input_xyz3), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 z2w2z3w3 = _mm_shuffle_ps((input_xyz2), (input_xyz3), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output_xxxx) = _mm_shuffle_ps(x0y0x1y1, x2y2x3y3, _MM_SHUFFLE(2, 0, 2, 0)); \
			(output_yyyy) = _mm_shuffle_ps(x0y0x1y1, x2y2x3y3, _MM_SHUFFLE(3, 1, 3, 1)); \
			(output_zzzz) = _mm_shuffle_ps(z0w0z1w1, z2w2z3w3, _MM_SHUFFLE(2, 0, 2, 0)); \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const v128_t x0y0x1y1 = wasm_i32x4_shuffle((input_xyz0), (input_xyz1), 0, 1, 4, 5); \
			const v128_t z0w0z1w1 = wasm_i32x4_shuffle((input_xyz0), (input_xyz1), 2, 3, 6, 7); \
			const v128_t x2y2x3y3 = wasm_i32x4_shuffle((input_xyz2), (input_xyz3), 0, 1, 4, 5); \
			const v128_t z2w2z3w3 = wasm_i32x4_shuffle((input_xyz2), (input_xyz3), 2, 3, 6, 7); \
			(output_xxxx) = wasm_i32x4_shuffle(x0y0x1y1, x2y2x3y3, 0, 2, 4, 6); \
			(output_yyyy) = wasm_i32x4_shuffle(x0y0x1y1, x2y2x3y3, 1, 3, 5, 7); \
			(output_zzzz) = wasm_i32x4_shuffle(z0w0z1w1, z2w2z3w3, 0, 2, 4, 6); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const float input_x0 = (input_xyz0).x; \
			const float input_y0 = (input_xyz0).y; \
			const float input_z0 = (input_xyz0).z; \
			const float input_x1 = (input_xyz1).x; \
			const float input_y1 = (input_xyz1).y; \
			const float input_z1 = (input_xyz1).z; \
			const float input_x2 = (input_xyz2).x; \
			const float input_y2 = (input_xyz2).y; \
			const float input_z2 = (input_xyz2).z; \
			const float input_x3 = (input_xyz3).x; \
			const float input_y3 = (input_xyz3).y; \
			const float input_z3 = (input_xyz3).z; \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4f { input_x0, input_x1, input_x2, input_x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4f { input_y0, input_y1, input_y2, input_y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4f { input_z0, input_z1, input_z2, input_z3 }; \
		} while(0)
#endif

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const __m128d x0x1 = _mm_unpacklo_pd(input_xyz0.xy, input_xyz1.xy); \
			const __m128d y0y1 = _mm_unpackhi_pd(input_xyz0.xy, input_xyz1.xy); \
			const __m128d x2x3 = _mm_unpacklo_pd(input_xyz2.xy, input_xyz3.xy); \
			const __m128d y2y3 = _mm_unpackhi_pd(input_xyz2.xy, input_xyz3.xy); \
			const __m128d z0z1 = _mm_unpacklo_pd(input_xyz0.zw, input_xyz1.zw); \
			const __m128d z2z3 = _mm_unpacklo_pd(input_xyz2.zw, input_xyz3.zw); \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x2x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, z2z3 }; \
		} while(0)
#elif defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const float64x2_t x0x1 = vzip1q_f64(input_xyz0.xy, input_xyz1.xy); \
			const float64x2_t y0y1 = vzip2q_f64(input_xyz0.xy, input_xyz1.xy); \
			const float64x2_t x2x3 = vzip1q_f64(input_xyz2.xy, input_xyz3.xy); \
			const float64x2_t y2y3 = vzip2q_f64(input_xyz2.xy, input_xyz3.xy); \
			const float64x2_t z0z1 = vzip1q_f64(input_xyz0.zw, input_xyz1.zw); \
			const float64x2_t z2z3 = vzip1q_f64(input_xyz2.zw, input_xyz3.zw); \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x2x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, z2z3 }; \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const v128_t x0x1 = wasm_i64x2_shuffle(input_xyz0.xy, input_xyz1.xy, 0, 2); \
			const v128_t y0y1 = wasm_i64x2_shuffle(input_xyz0.xy, input_xyz1.xy, 1, 3); \
			const v128_t x2x3 = wasm_i64x2_shuffle(input_xyz2.xy, input_xyz3.xy, 0, 2); \
			const v128_t y2y3 = wasm_i64x2_shuffle(input_xyz2.xy, input_xyz3.xy, 1, 3); \
			const v128_t z0z1 = wasm_i64x2_shuffle(input_xyz0.zw, input_xyz1.zw, 0, 2); \
			const v128_t z2z3 = wasm_i64x2_shuffle(input_xyz2.zw, input_xyz3.zw, 0, 2); \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, x2x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, z2z3 }; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x3 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_4X3(input_xyz0, input_xyz1, input_xyz2, input_xyz3, output_xxxx, output_yyyy, output_zzzz) \
		do { \
			const double input_x0 = (input_xyz0).x; \
			const double input_y0 = (input_xyz0).y; \
			const double input_z0 = (input_xyz0).z; \
			const double input_x1 = (input_xyz1).x; \
			const double input_y1 = (input_xyz1).y; \
			const double input_z1 = (input_xyz1).z; \
			const double input_x2 = (input_xyz2).x; \
			const double input_y2 = (input_xyz2).y; \
			const double input_z2 = (input_xyz2).z; \
			const double input_x3 = (input_xyz3).x; \
			const double input_y3 = (input_xyz3).y; \
			const double input_z3 = (input_xyz3).z; \
			(output_xxxx) = RTM_IMPL_NAMESPACE::vector4d { input_x0, input_x1, input_x2, input_x3 }; \
			(output_yyyy) = RTM_IMPL_NAMESPACE::vector4d { input_y0, input_y1, input_y2, input_y3 }; \
			(output_zzzz) = RTM_IMPL_NAMESPACE::vector4d { input_z0, input_z1, input_z2, input_z3 }; \
		} while(0)
#endif

#if defined(RTM_NEON_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const float32x4x2_t x0x2y0y2_z0z2w0w2 = vzipq_f32((input_xyzw0), (input_xyzw2)); \
			const float32x4x2_t x1x1y1y1_z1z1w1w1 = vzipq_f32((input_xyzw1), (input_xyzw1)); \
			const float32x4x2_t x0x1x2x1_y0y1y2y1 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[0], x1x1y1y1_z1z1w1w1.val[0]); \
			const float32x4x2_t z0z1z2z1_w0w1w2w1 = vzipq_f32(x0x2y0y2_z0z2w0w2.val[1], x1x1y1y1_z1z1w1w1.val[1]); \
			(output_xxx) = x0x1x2x1_y0y1y2y1.val[0]; \
			(output_yyy) = x0x1x2x1_y0y1y2y1.val[1]; \
			(output_zzz) = z0z1z2z1_w0w1w2w1.val[0]; \
			(output_www) = z0z1z2z1_w0w1w2w1.val[1]; \
		} while(0)
#elif defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const __m128 input_xyzw2_ = (input_xyzw2); \
			const __m128 x0y0x1y1 = _mm_shuffle_ps((input_xyzw0), (input_xyzw1), _MM_SHUFFLE(1, 0, 1, 0)); \
			const __m128 z0w0z1w1 = _mm_shuffle_ps((input_xyzw0), (input_xyzw1), _MM_SHUFFLE(3, 2, 3, 2)); \
			(output_xxx) = _mm_shuffle_ps(x0y0x1y1, input_xyzw2_, _MM_SHUFFLE(0, 0, 2, 0)); \
			(output_yyy) = _mm_shuffle_ps(x0y0x1y1, input_xyzw2_, _MM_SHUFFLE(1, 1, 3, 1)); \
			(output_zzz) = _mm_shuffle_ps(z0w0z1w1, input_xyzw2_, _MM_SHUFFLE(2, 2, 2, 0)); \
			(output_www) = _mm_shuffle_ps(z0w0z1w1, input_xyzw2_, _MM_SHUFFLE(3, 3, 3, 1)); \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const v128_t input_xyzw2_ = (input_xyzw2); \
			const v128_t x0y0x1y1 = wasm_i32x4_shuffle((input_xyzw0), (input_xyzw1), 0, 1, 4, 5); \
			const v128_t z0w0z1w1 = wasm_i32x4_shuffle((input_xyzw0), (input_xyzw1), 2, 3, 6, 7); \
			(output_xxx) = wasm_i32x4_shuffle(x0y0x1y1, input_xyzw2_, 0, 2, 4, 4); \
			(output_yyy) = wasm_i32x4_shuffle(x0y0x1y1, input_xyzw2_, 1, 3, 5, 5); \
			(output_zzz) = wasm_i32x4_shuffle(z0w0z1w1, input_xyzw2_, 0, 2, 6, 6); \
			(output_www) = wasm_i32x4_shuffle(z0w0z1w1, input_xyzw2_, 1, 3, 7, 7); \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4f.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXF_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const float input_x0 = (input_xyzw0).x; \
			const float input_y0 = (input_xyzw0).y; \
			const float input_z0 = (input_xyzw0).z; \
			const float input_w0 = (input_xyzw0).w; \
			const float input_x1 = (input_xyzw1).x; \
			const float input_y1 = (input_xyzw1).y; \
			const float input_z1 = (input_xyzw1).z; \
			const float input_w1 = (input_xyzw1).w; \
			const float input_x2 = (input_xyzw2).x; \
			const float input_y2 = (input_xyzw2).y; \
			const float input_z2 = (input_xyzw2).z; \
			const float input_w2 = (input_xyzw2).w; \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4f { input_x0, input_x1, input_x2, input_x2 }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4f { input_y0, input_y1, input_y2, input_y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4f { input_z0, input_z1, input_z2, input_z2 }; \
			(output_www) = RTM_IMPL_NAMESPACE::vector4f { input_w0, input_w1, input_w2, input_w2 }; \
		} while(0)
#endif

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const __m128d x0x1 = _mm_unpacklo_pd(input_xyzw0.xy, input_xyzw1.xy); \
			const __m128d y0y1 = _mm_unpackhi_pd(input_xyzw0.xy, input_xyzw1.xy); \
			const __m128d y2y2 = _mm_unpackhi_pd(input_xyzw2.xy, input_xyzw2.xy); \
			const __m128d z0z1 = _mm_unpacklo_pd(input_xyzw0.zw, input_xyzw1.zw); \
			const __m128d w0w1 = _mm_unpackhi_pd(input_xyzw0.zw, input_xyzw1.zw); \
			const __m128d w2w2 = _mm_unpackhi_pd(input_xyzw2.zw, input_xyzw2.zw); \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, input_xyzw2.xy }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, input_xyzw2.zw }; \
			(output_www) = RTM_IMPL_NAMESPACE::vector4d { w0w1, w2w2 }; \
		} while(0)
#elif defined(RTM_NEON64_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const float64x2_t x0x1 = vzip1q_f64(input_xyzw0.xy, input_xyzw1.xy); \
			const float64x2_t y0y1 = vzip2q_f64(input_xyzw0.xy, input_xyzw1.xy); \
			const float64x2_t y2y2 = vzip2q_f64(input_xyzw2.xy, input_xyzw2.xy); \
			const float64x2_t z0z1 = vzip1q_f64(input_xyzw0.zw, input_xyzw1.zw); \
			const float64x2_t w0w1 = vzip2q_f64(input_xyzw0.zw, input_xyzw1.zw); \
			const float64x2_t w2w2 = vzip2q_f64(input_xyzw2.zw, input_xyzw2.zw); \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, input_xyzw2.xy }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, input_xyzw2.zw }; \
			(output_www) = RTM_IMPL_NAMESPACE::vector4d { w0w1, w2w2 }; \
		} while(0)
#elif defined(RTM_WASM128_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const v128_t x0x1 = wasm_i64x2_shuffle(input_xyzw0.xy, input_xyzw1.xy, 0, 2); \
			const v128_t y0y1 = wasm_i64x2_shuffle(input_xyzw0.xy, input_xyzw1.xy, 1, 3); \
			const v128_t y2y2 = wasm_i64x2_shuffle(input_xyzw2.xy, input_xyzw2.xy, 1, 3); \
			const v128_t z0z1 = wasm_i64x2_shuffle(input_xyzw0.zw, input_xyzw1.zw, 0, 2); \
			const v128_t w0w1 = wasm_i64x2_shuffle(input_xyzw0.zw, input_xyzw1.zw, 1, 3); \
			const v128_t w2w2 = wasm_i64x2_shuffle(input_xyzw2.zw, input_xyzw2.zw, 1, 3); \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { x0x1, input_xyzw2.xy }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { y0y1, y2y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { z0z1, input_xyzw2.zw }; \
			(output_www) = RTM_IMPL_NAMESPACE::vector4d { w0w1, w2w2 }; \
		} while(0)
#else
	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 matrix.
	// All inputs and outputs must be rtm::vector4d.
	//////////////////////////////////////////////////////////////////////////
	#define RTM_MATRIXD_TRANSPOSE_3X4(input_xyzw0, input_xyzw1, input_xyzw2, output_xxx, output_yyy, output_zzz, output_www) \
		do { \
			const double input_x0 = (input_xyzw0).x; \
			const double input_y0 = (input_xyzw0).y; \
			const double input_z0 = (input_xyzw0).z; \
			const double input_w0 = (input_xyzw0).w; \
			const double input_x1 = (input_xyzw1).x; \
			const double input_y1 = (input_xyzw1).y; \
			const double input_z1 = (input_xyzw1).z; \
			const double input_w1 = (input_xyzw1).w; \
			const double input_x2 = (input_xyzw2).x; \
			const double input_y2 = (input_xyzw2).y; \
			const double input_z2 = (input_xyzw2).z; \
			const double input_w2 = (input_xyzw2).w; \
			(output_xxx) = RTM_IMPL_NAMESPACE::vector4d { input_x0, input_x1, input_x2, input_x2 }; \
			(output_yyy) = RTM_IMPL_NAMESPACE::vector4d { input_y0, input_y1, input_y2, input_y2 }; \
			(output_zzz) = RTM_IMPL_NAMESPACE::vector4d { input_z0, input_z1, input_z2, input_z2 }; \
			(output_www) = RTM_IMPL_NAMESPACE::vector4d { input_w0, input_w1, input_w2, input_w2 }; \
		} while(0)
#endif

RTM_IMPL_FILE_PRAGMA_POP
