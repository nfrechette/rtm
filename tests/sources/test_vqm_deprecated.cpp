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

// Deprecated in 2.4, to be removed in 2.5
// This file tests the deprecated experimental VQM headers. Remove it with these headers.

#include "catch2.impl.h"

// The new headers define the compiler macros that the pragmas below use
#include <rtm/vqmf.h>
#include <rtm/vqmd.h>

// This test includes the deprecated headers on purpose. Disable the deprecation warning.
// Do not define RTM_NO_DEPRECATION: the test must compile the deprecation attributes.
#if defined(RTM_COMPILER_MSVC)
	#pragma warning(disable : 4996)
#elif defined(RTM_COMPILER_CLANG)
	#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(RTM_COMPILER_GCC)
	#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <rtm/experimental/types.h>
#include <rtm/experimental/vqmf.h>
#include <rtm/experimental/vqmd.h>

using namespace rtm;

TEST_CASE("vqm deprecated experimental headers", "[math][vqm]")
{
	// The deprecated headers include the new headers: the types and the functions stay available
	const vqmf transformf = vqm_set(quat_identity(), vector_set(1.0F, 2.0F, 3.0F), vector_set(2.0F));
	CHECK(vector_all_near_equal3(vqm_get_scale(transformf), vector_set(2.0F), 1.0E-6F));

	const vqmd transformd = vqm_cast(transformf);
	CHECK(vector_all_near_equal3(vqm_get_translation(transformd), vector_set(1.0, 2.0, 3.0), 1.0E-9));
}
