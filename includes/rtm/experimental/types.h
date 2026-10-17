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

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	struct vqmd
	{
		// The internal format is meant to be opaque, use vqm_* accessors
		// The scale/shear matrix M has the rows x_axis, y_axis, and z_axis. Each row has 3 components.
		// The [w] lanes hold the third row: m20 = z_axis.x, m21 = z_axis.y, and m22 = z_axis.z.
		// Thus the transform uses 4 registers and 16 components.
		quatd		rotation;
		vector4d	translation_m22;	// [xyz] = translation, [w] = m22
		vector4d	x_axis_m20;			// [xyz] = x_axis, [w] = m20
		vector4d	y_axis_m21;			// [xyz] = y_axis, [w] = m21
	};

	struct vqmf
	{
		// The internal format is meant to be opaque, use vqm_* accessors
		// The scale/shear matrix M has the rows x_axis, y_axis, and z_axis. Each row has 3 components.
		// The [w] lanes hold the third row: m20 = z_axis.x, m21 = z_axis.y, and m22 = z_axis.z.
		// Thus the transform uses 4 registers and 16 components.
		quatf		rotation;
		vector4f	translation_m22;	// [xyz] = translation, [w] = m22
		vector4f	x_axis_m20;			// [xyz] = x_axis, [w] = m20
		vector4f	y_axis_m21;			// [xyz] = y_axis, [w] = m21
	};

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
