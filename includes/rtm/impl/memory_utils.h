#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
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

#include "rtm/error.h"

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <limits>
#include <memory>
#include <algorithm>

//////////////////////////////////////////////////////////////////////////
// This file contains various memory related utility functions and other misc helpers.
// Everything is hidden in an impl namespace even though they could useful and used
// by anyone only to avoid polluting the rtm namespace. If these become useful enough,
// they might be moved into their own external dependency.
//////////////////////////////////////////////////////////////////////////

namespace rtm
{
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// Allows static branching without any warnings

		template<bool expression_result>
		struct static_condition { static constexpr bool test() { return true; } };

		template<>
		struct static_condition<false> { static constexpr bool test() { return false; } };

		//////////////////////////////////////////////////////////////////////////
		// Various miscellaneous utilities related to alignment

		constexpr bool is_power_of_two(size_t input)
		{
			return input != 0 && (input & (input - 1)) == 0;
		}

		template<typename Type>
		constexpr bool is_alignment_valid(size_t alignment)
		{
			return is_power_of_two(alignment) && alignment >= alignof(Type);
		}

		template<typename PtrType>
		inline bool is_aligned_to(PtrType* value, size_t alignment)
		{
			RTM_ASSERT(is_power_of_two(alignment), "Alignment value must be a power of two");
			return (reinterpret_cast<intptr_t>(value) & (alignment - 1)) == 0;
		}

		template<typename IntegralType>
		inline bool is_aligned_to(IntegralType value, size_t alignment)
		{
			RTM_ASSERT(is_power_of_two(alignment), "Alignment value must be a power of two");
			return (static_cast<size_t>(value) & (alignment - 1)) == 0;
		}

		template<typename PtrType>
		constexpr bool is_aligned(PtrType* value)
		{
			return is_aligned_to(value, alignof(PtrType));
		}

		template<typename PtrType>
		inline PtrType* align_to(PtrType* value, size_t alignment)
		{
			RTM_ASSERT(is_power_of_two(alignment), "Alignment value must be a power of two");
			return reinterpret_cast<PtrType*>((reinterpret_cast<intptr_t>(value) + (alignment - 1)) & ~(alignment - 1));
		}

		template<typename IntegralType>
		inline IntegralType align_to(IntegralType value, size_t alignment)
		{
			RTM_ASSERT(is_power_of_two(alignment), "Alignment value must be a power of two");
			return static_cast<IntegralType>((static_cast<size_t>(value) + (alignment - 1)) & ~(alignment - 1));
		}

		template<typename ElementType, size_t num_elements>
		constexpr size_t get_array_size(ElementType const (&)[num_elements]) { return num_elements; }

		//////////////////////////////////////////////////////////////////////////
		// Type safe casting

		template<typename DestPtrType, typename SrcType>
		struct safe_ptr_to_ptr_cast_impl
		{
			inline static DestPtrType* cast(SrcType* input)
			{
				RTM_ASSERT(is_aligned_to(input, alignof(DestPtrType)), "reinterpret_cast would result in an unaligned pointer");
				return reinterpret_cast<DestPtrType*>(input);
			}
		};

		template<typename SrcType>
		struct safe_ptr_to_ptr_cast_impl<void, SrcType>
		{
			static constexpr void* cast(SrcType* input) { return input; }
		};

		template<typename DestPtrType, typename SrcType>
		struct safe_int_to_ptr_cast_impl
		{
			inline static DestPtrType* cast(SrcType input)
			{
				RTM_ASSERT(is_aligned_to(input, alignof(DestPtrType)), "reinterpret_cast would result in an unaligned pointer");
				return reinterpret_cast<DestPtrType*>(input);
			}
		};

		template<typename SrcType>
		struct safe_int_to_ptr_cast_impl<void, SrcType>
		{
			static constexpr void* cast(SrcType input) { return reinterpret_cast<void*>(input); }
		};

		template<typename DestPtrType, typename SrcType>
		inline DestPtrType* safe_ptr_cast(SrcType* input)
		{
			return safe_ptr_to_ptr_cast_impl<DestPtrType, SrcType>::cast(input);
		}

		template<typename DestPtrType, typename SrcType>
		inline DestPtrType* safe_ptr_cast(SrcType input)
		{
			return safe_int_to_ptr_cast_impl<DestPtrType, SrcType>::cast(input);
		}

		template<typename Type, bool is_enum = true>
		struct safe_underlying_type { using type = typename std::underlying_type<Type>::type; };

		template<typename Type>
		struct safe_underlying_type<Type, false> { using type = Type; };

		template<typename DstType, typename SrcType, bool is_floating_point = false>
		struct is_static_cast_safe_s
		{
			static bool test(SrcType input)
			{
				using SrcRealType = typename safe_underlying_type<SrcType, std::is_enum<SrcType>::value>::type;

				if (static_condition<(std::is_signed<DstType>::value == std::is_signed<SrcRealType>::value)>::test())
					return SrcType(DstType(input)) == input;
				else if (static_condition<(std::is_signed<SrcRealType>::value)>::test())
					return int64_t(input) >= 0 && SrcType(DstType(input)) == input;
				else
					return uint64_t(input) <= uint64_t(std::numeric_limits<DstType>::max());
			};
		};

		template<typename DstType, typename SrcType>
		struct is_static_cast_safe_s<DstType, SrcType, true>
		{
			static bool test(SrcType input)
			{
				return SrcType(DstType(input)) == input;
			}
		};

		template<typename DstType, typename SrcType>
		inline bool is_static_cast_safe(SrcType input)
		{
			// TODO: In C++17 this should be folded to constexpr if
			return is_static_cast_safe_s<DstType, SrcType, static_condition<(std::is_floating_point<SrcType>::value || std::is_floating_point<DstType>::value)>::test()>::test(input);
		}

		template<typename DstType, typename SrcType>
		inline DstType safe_static_cast(SrcType input)
		{
#if defined(RTM_HAS_ASSERT_CHECKS)
			const bool is_safe = is_static_cast_safe<DstType, SrcType>(input);
			RTM_ASSERT(is_safe, "Unsafe static cast resulted in data loss");
#endif

			return static_cast<DstType>(input);
		}

		//////////////////////////////////////////////////////////////////////////
		// Raw memory read/write support

		template<typename DataType>
		inline DataType unaligned_read(const void* input)
		{
			DataType result;
			memcpy(&result, input, sizeof(DataType));
			return result;
		}

		template<typename DataType>
		inline DataType aligned_read(const void* input)
		{
			return *safe_ptr_cast<const DataType, const void*>(input);
		}

		template<typename DataType>
		inline void unaligned_write(DataType input, void* output)
		{
			memcpy(output, &input, sizeof(DataType));
		}
	}
}
