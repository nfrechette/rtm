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

#include "rtm/math.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/error.h"

//////////////////////////////////////////////////////////////////////////
// Specifies the default coordinate system used by RTM.
// To override it with your own, specify the macro RTM_DEFAULT_COORDINATE_SYSTEM
// in all compilation units (e.g. through CMake) or before you include any
// RTM headers (e.g. with a force include or shim include).
// You can conveniently specify it using rtm::coordinate_system_ext<...>
//////////////////////////////////////////////////////////////////////////
#if !defined(RTM_DEFAULT_COORDINATE_SYSTEM)
	#define RTM_DEFAULT_COORDINATE_SYSTEM RTM_IMPL_NAMESPACE::coordinate_system
#endif

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Specifies whether winding is left or right handed.
	//////////////////////////////////////////////////////////////////////////
	enum class handed
	{
		left,
		right,
	};

	//////////////////////////////////////////////////////////////////////////
	// Represents a statically defined coordinate system.
	//////////////////////////////////////////////////////////////////////////
	struct coordinate_system
	{
		//////////////////////////////////////////////////////////////////////////
		// The handed-ness of the coordinate system (default is left handed).
		//////////////////////////////////////////////////////////////////////////
		static constexpr handed handedness = handed::left;

		// TODO: We need to allow axes to be pos/neg, see FBX SDK

		//////////////////////////////////////////////////////////////////////////
		// The forward axis (default is Z+ axis).
		//////////////////////////////////////////////////////////////////////////
		static constexpr component3 forward_axis = component3::z;

		//////////////////////////////////////////////////////////////////////////
		// The cross/left/right axis (default is X+ axis).
		//////////////////////////////////////////////////////////////////////////
		static constexpr component3 cross_axis = component3::x;

		//////////////////////////////////////////////////////////////////////////
		// The up axis (default is Y+ axis).
		//////////////////////////////////////////////////////////////////////////
		static constexpr component3 up_axis = component3::y;

		//////////////////////////////////////////////////////////////////////////
		// Returns the X axis based on the 3 input axes.
		//////////////////////////////////////////////////////////////////////////
		static RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL get_x_axis(vector4f_arg0 forward, vector4f_arg1 cross, vector4f_arg2 up)
		{
			return
				forward_axis == component3::x ? forward :
				cross_axis == component3::x ? cross :
				up;
		}

		//////////////////////////////////////////////////////////////////////////
		// Returns the Y axis based on the 3 input axes.
		//////////////////////////////////////////////////////////////////////////
		static RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL get_y_axis(vector4f_arg0 forward, vector4f_arg1 cross, vector4f_arg2 up)
		{
			return
				forward_axis == component3::y ? forward :
				cross_axis == component3::y ? cross :
				up;
		}

		//////////////////////////////////////////////////////////////////////////
		// Returns the Z axis based on the 3 input axes.
		//////////////////////////////////////////////////////////////////////////
		static RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL get_z_axis(vector4f_arg0 forward, vector4f_arg1 cross, vector4f_arg2 up)
		{
			return
				forward_axis == component3::z ? forward :
				cross_axis == component3::z ? cross :
				up;
		}
	};

	//////////////////////////////////////////////////////////////////////////
	// Specifies a coordinate system through template arguments.
	//////////////////////////////////////////////////////////////////////////
	template<
		handed handedness_,
		component3 forward_axis_,
		component3 cross_axis_,
		component3 up_axis_
		>
	struct coordinate_system_ext : coordinate_system
	{
		static constexpr handed handedness = handedness_;

		static constexpr component3 forward_axis = forward_axis_;
		static constexpr component3 cross_axis = cross_axis_;
		static constexpr component3 up_axis = up_axis_;

		static_assert(
			forward_axis_ != cross_axis_ &&
			forward_axis_ != up_axis_ &&
			cross_axis_ != up_axis_,
			"Coordinate system axes must be unique");
	};

	//////////////////////////////////////////////////////////////////////////
	// The default coordinate system used by all functions.
	//////////////////////////////////////////////////////////////////////////
	using default_coordinate_system = RTM_DEFAULT_COORDINATE_SYSTEM;

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
