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
#include "rtm/impl/error.h"

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

#if defined(RTM_COMPILER_GCC)
	#pragma GCC diagnostic push
#if __GNUC__ > 5
	// GCC complains that the alignment attribute is ignored, we don't care with type traits
	#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif
#endif

	//////////////////////////////////////////////////////////////////////////
	/// Specifies the related types given a source type.
	//////////////////////////////////////////////////////////////////////////
	template<typename source_type>
	struct related_types {};

	//////////////////////////////////////////////////////////////////////////
	/// The related types of float32 values.
	//////////////////////////////////////////////////////////////////////////
	template<>
	struct related_types<float>
	{
		using mask4 = mask4f; ///< The 4 lane mask type.

		using scalar = scalarf; ///< The SIMD scalar type.
		using vector4 = vector4f; ///< The 4D vector type.
		using quat = quatf; ///< The quaternion type.

		using vq = vqf; ///< The VQ transform type.
		using vqs = vqsf; ///< The VQS transform type.
		using vqv = vqvf; ///< The VQV transform type.
		using vqm = vqmf; ///< The VQM transform type.

		using qv RTM_DEPRECATED("Use 'vq' instead. To be removed in 2.5.") = vqf; ///< The previous name of vq.
		using qvs RTM_DEPRECATED("Use 'vqs' instead. To be removed in 2.5.") = vqsf; ///< The previous name of vqs.
		using qvv RTM_DEPRECATED("Use 'vqv' instead. To be removed in 2.5.") = vqvf; ///< The previous name of vqv.

		using matrix3x3 = matrix3x3f; ///< The 3x3 matrix type.
		using matrix3x4 = matrix3x4f; ///< The 3x4 affine matrix type.
		using matrix4x4 = matrix4x4f; ///< The 4x4 matrix type.

		using float1 = float; ///< The scalar type for memory storage.
		using float2 = float2f; ///< The 2D vector type for memory storage.
		using float3 = float3f; ///< The 3D vector type for memory storage.
		using float4 = float4f; ///< The 4D vector type for memory storage.

		using int1 = uint32_t; ///< The unsigned integer type with the same size as the float type.
	};

	//////////////////////////////////////////////////////////////////////////
	/// The related types of float64 values.
	//////////////////////////////////////////////////////////////////////////
	template<>
	struct related_types<double>
	{
		using mask4 = mask4d; ///< The 4 lane mask type.

		using scalar = scalard; ///< The SIMD scalar type.
		using vector4 = vector4d; ///< The 4D vector type.
		using quat = quatd; ///< The quaternion type.

		using vq = vqd; ///< The VQ transform type.
		using vqs = vqsd; ///< The VQS transform type.
		using vqv = vqvd; ///< The VQV transform type.
		using vqm = vqmd; ///< The VQM transform type.

		using qv RTM_DEPRECATED("Use 'vq' instead. To be removed in 2.5.") = vqd; ///< The previous name of vq.
		using qvs RTM_DEPRECATED("Use 'vqs' instead. To be removed in 2.5.") = vqsd; ///< The previous name of vqs.
		using qvv RTM_DEPRECATED("Use 'vqv' instead. To be removed in 2.5.") = vqvd; ///< The previous name of vqv.

		using matrix3x3 = matrix3x3d; ///< The 3x3 matrix type.
		using matrix3x4 = matrix3x4d; ///< The 3x4 affine matrix type.
		using matrix4x4 = matrix4x4d; ///< The 4x4 matrix type.

		using float1 = double; ///< The scalar type for memory storage.
		using float2 = float2d; ///< The 2D vector type for memory storage.
		using float3 = float3d; ///< The 3D vector type for memory storage.
		using float4 = float4d; ///< The 4D vector type for memory storage.

		using int1 = uint64_t; ///< The unsigned integer type with the same size as the float type.
	};

	// Alias all related float types
	template<> struct related_types<vector4f> : related_types<float> {};
	template<> struct related_types<vqf> : related_types<float> {};
	template<> struct related_types<vqsf> : related_types<float> {};
	template<> struct related_types<vqvf> : related_types<float> {};
	template<> struct related_types<vqmf> : related_types<float> {};
	template<> struct related_types<matrix3x3f> : related_types<float> {};
	template<> struct related_types<matrix3x4f> : related_types<float> {};
	template<> struct related_types<matrix4x4f> : related_types<float> {};
	template<> struct related_types<float2f> : related_types<float> {};
	template<> struct related_types<float3f> : related_types<float> {};
	template<> struct related_types<float4f> : related_types<float> {};

#if defined(RTM_SSE2_INTRINSICS)
	// mask4f is an alias of vector4f
	// quatf is an alias of vector4f
	template<> struct related_types<scalarf> : related_types<float> {};
#elif defined(RTM_NEON_INTRINSICS)
	// scalarf is an alias of float
	// quatf is an alias of vector4f
#if !defined(RTM_COMPILER_MSVC)
	// MSVC aliases uint32x4_t and float32x4_t
	template<> struct related_types<mask4f> : related_types<float> {};
#endif
#elif defined(RTM_WASM128_INTRINSICS)
	// scalarf is an alias of float
	// mask4f is an alias of vector4f
	// quatf is an alias of vector4f
#else
	// scalarf is an alias of float
	template<> struct related_types<mask4f> : related_types<float> {};
	template<> struct related_types<quatf> : related_types<float> {};
#endif

	// Alias all related double types
	template<> struct related_types<mask4d> : related_types<double> {};
	template<> struct related_types<vector4d> : related_types<double> {};
	template<> struct related_types<quatd> : related_types<double> {};
	template<> struct related_types<vqd> : related_types<double> {};
	template<> struct related_types<vqsd> : related_types<double> {};
	template<> struct related_types<vqvd> : related_types<double> {};
	template<> struct related_types<vqmd> : related_types<double> {};
	template<> struct related_types<matrix3x3d> : related_types<double> {};
	template<> struct related_types<matrix3x4d> : related_types<double> {};
	template<> struct related_types<matrix4x4d> : related_types<double> {};
	template<> struct related_types<float2d> : related_types<double> {};
	template<> struct related_types<float3d> : related_types<double> {};
	template<> struct related_types<float4d> : related_types<double> {};

#if defined(RTM_SSE2_INTRINSICS)
	template<> struct related_types<scalard> : related_types<double> {};
#elif defined(RTM_NEON_INTRINSICS)
	// scalard is an alias of double
#else
	// scalard is an alias of double
#endif

#if defined(RTM_COMPILER_GCC)
	#pragma GCC diagnostic pop
#endif

	RTM_IMPL_VERSION_NAMESPACE_END
}
