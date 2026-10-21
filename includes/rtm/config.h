#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2025 Nicholas Frechette & Realtime Math contributors
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

// This file gathers all defines that can be used to control RTM's behavior
// along with context.

// This library uses a simple system to handle asserts. Asserts are fatal and must terminate
// otherwise the behavior is undefined if execution continues.
//
// A total of 4 behaviors are supported:
//    - We can print to stderr and abort
//    - We can throw and exception
//    - We can call a custom function
//    - Do nothing and strip the check at compile time (default behavior)
//
// Aborting:
//    In order to enable the aborting behavior, simply define the macro RTM_ON_ASSERT_ABORT:
//    #define RTM_ON_ASSERT_ABORT
//
// Throwing:
//    In order to enable the throwing behavior, simply define the macro RTM_ON_ASSERT_THROW:
//    #define RTM_ON_ASSERT_THROW
//    Note that the type of the exception thrown is rtm::runtime_assert.
//
// Custom function:
//    In order to enable the custom function calling behavior, define the macro RTM_ON_ASSERT_CUSTOM
//    with the name of the function to call:
//    #define RTM_ON_ASSERT_CUSTOM on_custom_assert_impl
//    Note that the function signature is as follow:
//    void on_custom_assert_impl(const char* expression, int line, const char* file, const char* format, ...) {}
//
//    You can also define your own assert implementation by defining the RTM_ASSERT macro as well:
//    #define RTM_ON_ASSERT_CUSTOM
//    #define RTM_ASSERT(expression, format, ...) checkf(expression, ANSI_TO_TCHAR(format), #__VA_ARGS__)
//
//    [Custom String Format Specifier]
//    Note that if you use a custom function, you may need to override the RTM_ASSERT_STRING_FORMAT_SPECIFIER
//    to properly handle ANSI/Unicode support. The C++11 standard does not support a way to say that '%s'
//    always means an ANSI string (with 'const char*' as type). MSVC does support '%hs' but other compilers
//    do not.
//
// No checks:
//    By default if no macro mentioned above is defined, all asserts will be stripped
//    at compile time.

// You can uncomment one of these or specify it on the compilation command line
//#define RTM_ON_ASSERT_ABORT
//#define RTM_ON_ASSERT_THROW
//#define RTM_ON_ASSERT_CUSTOM(expression, line, file, format, ...) (void)expression

// See [Custom String Format Specifier] for details
#if !defined(RTM_ASSERT_STRING_FORMAT_SPECIFIER)
	#define RTM_ASSERT_STRING_FORMAT_SPECIFIER "%s"
#endif

// You can disable deprecation warnings by defining RTM_NO_DEPRECATION
//#define RTM_NO_DEPRECATION

// By default, RTM uses the intrinsics of the target platform as specified on the
// compiler command line. You can disable SIMD intrinsic usage by defining RTM_NO_INTRINSICS
//#define RTM_NO_INTRINSICS

// If you use C++20 or greater, you can disable std::bit_cast usage by defining RTM_NO_BIT_CAST
// This is sometimes necessary if you compile with a modern compiler but use an older stdlib
// that does not contain the bit_cast header
//#define RTM_NO_BIT_CAST
