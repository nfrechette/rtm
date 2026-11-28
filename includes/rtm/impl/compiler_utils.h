#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Realtime Math contributors
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

#include "rtm/impl/detect_compiler.h"
#include "rtm/impl/detect_cpp_version.h"

// Because this library is made entirely of headers, we have no control over the
// compilation flags used. However, in some cases, certain options must be forced.
// To do this, every header is wrapped in two macros to push and pop the necessary
// pragmas.
//
// Options we use:
//    - Disable fast math, it can hurt precision for little to no performance gain due to the high level of hand tuned optimizations.
//    - Disable floating point contraction (Clang only), the compiler must not merge a multiply and an add into one FMA instruction.
//      See docs/api_conventions.md for the flags that give the same results on all the targets.
#if defined(RTM_COMPILER_MSVC)
	#define RTM_IMPL_FILE_PRAGMA_PUSH \
		__pragma(float_control(precise, on, push))

	#define RTM_IMPL_FILE_PRAGMA_POP \
		__pragma(float_control(pop))
#elif defined(RTM_COMPILER_CLANG)
	// Clang supports this pragma only on some targets: x86 and x64, AArch64 (clang 16+), and RISC-V (clang 18+).
	// On other targets, clang ignores the pragma and gives a warning. We disable that warning.
	// Clang 10 and earlier do not know the pragma and give a different warning. We also disable that warning.
	// See: https://github.com/llvm/llvm-project/issues/55392
	//
	// Limits of the pragma under clang:
	//    - Clang defines most SSE and NEON intrinsics with operators in system headers.
	//      These headers are outside of our pragma region and fast math stays on for them.
	//    - The pragma does not disable -ffinite-math-only. Clang can still assume that values are not NaN or infinity.
	//    - The pragma does not disable floating point contraction. By default (-ffp-contract=on), Clang merges a multiply
	//      and an add in one expression into one FMA instruction when the target has FMA (for example ARM64).
	//      The FMA does not round the product, thus the result can change with the compiler flags and the target.
	//
	// We also set "clang fp contract(off)" to stop the contraction. float_control(pop) restores the previous mode.
	// On the targets where clang ignores float_control, the pop does nothing and the mode stays off in the code
	// that includes our headers. Thus we set the mode only on the targets that support float_control.
	// Apple clang 15 uses LLVM 16.
	//
	// With -ffp-contract=fast, LLVM clang ignores the pragma and merges the operations of our functions after
	// they are inlined. Use -ffp-contract=fast-honor-pragmas instead. Apple clang honors the pragma in both modes.
	#if defined(__x86_64__) || defined(__i386__) \
		|| (defined(__aarch64__) && defined(__apple_build_version__) && __clang_major__ >= 15) \
		|| (defined(__aarch64__) && !defined(__apple_build_version__) && __clang_major__ >= 16) \
		|| (defined(__riscv) && !defined(__apple_build_version__) && __clang_major__ >= 18)
		#define RTM_IMPL_CLANG_FP_CONTRACT_OFF _Pragma("clang fp contract(off)")
	#else
		#define RTM_IMPL_CLANG_FP_CONTRACT_OFF
	#endif

	#define RTM_IMPL_FILE_PRAGMA_PUSH \
		_Pragma("clang diagnostic push") \
		_Pragma("clang diagnostic ignored \"-Wignored-pragmas\"") \
		_Pragma("clang diagnostic ignored \"-Wunknown-pragmas\"") \
		_Pragma("float_control(precise, on, push)") \
		RTM_IMPL_CLANG_FP_CONTRACT_OFF \
		_Pragma("clang diagnostic pop")

	#define RTM_IMPL_FILE_PRAGMA_POP \
		_Pragma("clang diagnostic push") \
		_Pragma("clang diagnostic ignored \"-Wignored-pragmas\"") \
		_Pragma("clang diagnostic ignored \"-Wunknown-pragmas\"") \
		_Pragma("float_control(pop)") \
		_Pragma("clang diagnostic pop")
#elif defined(RTM_COMPILER_GCC)
	// GCC defaults to -ffp-contract=fast in C++. With that option, GCC can merge a multiply and an add into one
	// FMA instruction. The FMA does not round the product, thus the result can change with the flags and the target.
	//
	// We do not set fp-contract=off here. With -ffp-contract=fast, GCC merges the operations of our functions
	// after it inlines them in the code of the caller: the pragma does not apply there. Also, GCC does not inline
	// a function when its optimize options are not the same as those of the caller ("optimization level attribute
	// mismatch"). Thus the pragma makes the code slower and does not stop the contraction.
	// To get the same results on all the targets, compile with -ffp-contract=off.
	#define RTM_IMPL_FILE_PRAGMA_PUSH \
		_Pragma("GCC push_options") \
		_Pragma("GCC optimize (\"no-fast-math\")")

	#define RTM_IMPL_FILE_PRAGMA_POP \
		_Pragma("GCC pop_options")
#else
	#define RTM_IMPL_FILE_PRAGMA_PUSH
	#define RTM_IMPL_FILE_PRAGMA_POP
#endif

// In some cases, for performance reasons, we wish to disable stack security
// check cookies. This macro serves this purpose.
#if defined(RTM_COMPILER_MSVC)
	#define RTM_DISABLE_SECURITY_COOKIE_CHECK __declspec(safebuffers)
#else
	#define RTM_DISABLE_SECURITY_COOKIE_CHECK
#endif

// Force inline macros for when it is necessary.
#if defined(RTM_COMPILER_MSVC)
	#define RTM_FORCE_INLINE __forceinline
#elif defined(RTM_COMPILER_GCC) || defined(RTM_COMPILER_CLANG)
	#define RTM_FORCE_INLINE __attribute__((always_inline)) inline
#else
	#define RTM_FORCE_INLINE inline
#endif

// Force no-inline macros for when it is necessary.
#if defined(RTM_COMPILER_MSVC)
	#define RTM_FORCE_NOINLINE __declspec(noinline)
#elif defined(RTM_COMPILER_GCC) || defined(RTM_COMPILER_CLANG)
	#define RTM_FORCE_NOINLINE __attribute__((noinline))
#else
	#define RTM_FORCE_NOINLINE
#endif

// Marks a function whose return value must be used.
// With C++17 and later, the compiler gives a warning when the caller does not use the return value.
// With an earlier version, the macro is empty.
// To use a different attribute, or none, define RTM_NO_DISCARD before you include RTM.
#if !defined(RTM_NO_DISCARD)
	#if RTM_CPP_VERSION >= RTM_CPP_VERSION_17
		#define RTM_NO_DISCARD [[nodiscard]]
	#else
		#define RTM_NO_DISCARD
	#endif
#endif

// Joins two pre-processor tokens: RTM_JOIN_TOKENS(foo, bar) yields 'foobar'
#define RTM_JOIN_TOKENS(a, b) a ## b

// Wraps the __has_builtin pre-processor macro to handle non-clang and early
// GCC compilers
#if defined(__has_builtin)
	#define RTM_HAS_BUILTIN(x) __has_builtin(x)
#else
	#define RTM_HAS_BUILTIN(x) 0
#endif
