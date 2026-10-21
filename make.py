import argparse
import json
import multiprocessing
import multiprocessing.pool
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys

def is_arm64_host():
	return platform.machine() == 'arm64' or platform.machine() == 'aarch64'

def is_armv7_host():
	# armv8l is a 32-bit ARM user space on a 64-bit ARM processor
	return platform.machine() == 'armv7l' or platform.machine() == 'armv8l'

def is_riscv64_host():
	return platform.machine() == 'riscv64'

def parse_argv():
	parser = argparse.ArgumentParser(add_help=False)

	actions = parser.add_argument_group(title='Actions', description='If no action is specified, on Windows, OS X, and Linux the solution/make files are generated.  Multiple actions can be used simultaneously.')
	actions.add_argument('-build', action='store_true')
	actions.add_argument('-clean', action='store_true')
	actions.add_argument('-clean_only', action='store_true')
	actions.add_argument('-unit_test', action='store_true')
	actions.add_argument('-bench', action='store_true')
	actions.add_argument('-run_bench', action='store_true')
	actions.add_argument('-pull_bench', action='store_true')	# Android only
	actions.add_argument('-analyze', action='store_true', help='Run the clang static analyzer on the public headers')

	target = parser.add_argument_group(title='Target')
	target.add_argument('-compiler', choices=['vs2015', 'vs2017', 'vs2019', 'vs2019-clang', 'vs2022', 'vs2022-clang', 'vs2026', 'vs2026-clang', 'android', 'clang4', 'clang5', 'clang6', 'clang7', 'clang8', 'clang9', 'clang10', 'clang11', 'clang12', 'clang13', 'clang14', 'clang15', 'clang16', 'clang17', 'clang18', 'clang19', 'clang20', 'clang21', 'clang22', 'clang23', 'gcc5', 'gcc6', 'gcc7', 'gcc8', 'gcc9', 'gcc10', 'gcc11', 'gcc12', 'gcc13', 'gcc14', 'gcc15', 'osx', 'ios', 'emscripten'], help='Defaults to the host system\'s default compiler')
	target.add_argument('-config', choices=['Debug', 'Release'], type=str.capitalize)
	target.add_argument('-cpu', choices=['x86', 'x64', 'armv7', 'arm64', 'arm64ec', 'riscv64', 'wasm'], help='Defaults to the host system\'s architecture')
	target.add_argument('-cpp_version', choices=['11', '14', '17', '20'], help='Defaults to C++11')

	misc = parser.add_argument_group(title='Miscellaneous')
	misc.add_argument('-avx', dest='use_avx', action='store_true', help='Compile using AVX instructions on Windows, OS X, and Linux')
	misc.add_argument('-avx2', dest='use_avx2', action='store_true', help='Compile using AVX2 instructions on Windows, OS X, and Linux')
	misc.add_argument('-nosimd', dest='use_simd', action='store_false', help='Compile without SIMD instructions')
	misc.add_argument('-simd', dest='use_simd', action='store_true', help='Compile with default SIMD instructions')
	misc.add_argument('-num_threads', help='No. to use while compiling and regressing')
	misc.add_argument('-tests_matching', help='Only run tests whose names match this regex')
	misc.add_argument('-vector_mix_test', action='store_true', help='Enable the vector_mix unit tests')
	misc.add_argument('-fast_math', action='store_true', help='Compile the include validation with fast math')
	misc.add_argument('-ci', action='store_true', help='Whether or not this is a Continuous Integration build')
	misc.add_argument('-help', action='help', help='Display this usage information')

	num_threads = multiprocessing.cpu_count()
	if platform.system() == 'Linux' and sys.version_info >= (3, 4):
		num_threads = len(os.sched_getaffinity(0))
	if not num_threads or num_threads == 0:
		num_threads = 4

	parser.set_defaults(build=False, clean=False, clean_only=False, unit_test=False,
		compiler=None, config='Release', cpu=None, cpp_version='11', use_avx=False, use_avx2=False, use_simd=True, num_threads=num_threads, tests_matching='', vector_mix_test=False, fast_math=False,
		bench=False, run_bench=False, pull_bench=False, analyze=False)

	args = parser.parse_args()

	is_arm64_cpu = is_arm64_host()
	is_armv7_cpu = is_armv7_host()
	is_riscv64_cpu = is_riscv64_host()

	# Sanitize and validate our options
	if (args.use_avx or args.use_avx2) and not args.use_simd:
		print('SIMD is explicitly disabled, AVX and AVX2 will not be used')
		args.use_avx = False
		args.use_avx2 = False

	if args.compiler == 'android':
		if not args.cpu:
			args.cpu = 'arm64'

		if not platform.system() == 'Windows':
			print('Android is only supported on Windows')
			sys.exit(1)

		if args.use_avx or args.use_avx2:
			print('AVX and AVX2 are not supported on Android')
			sys.exit(1)

		if not args.cpu in ['armv7', 'arm64']:
			print('{} cpu architecture not in supported list [armv7, arm64] for Android'.format(args.cpu))
			sys.exit(1)
	elif args.compiler == 'ios':
		if not args.cpu:
			args.cpu = 'arm64'

		if not platform.system() == 'Darwin':
			print('iOS is only supported on OS X')
			sys.exit(1)

		if args.use_avx or args.use_avx2:
			print('AVX and AVX2 are not supported on iOS')
			sys.exit(1)

		if args.unit_test:
			print('Unit tests cannot run from the command line on iOS')
			sys.exit(1)

		if not args.cpu in ['arm64']:
			print('{} cpu architecture not in supported list [arm64] for iOS'.format(args.cpu))
			sys.exit(1)
	elif args.compiler == 'emscripten':
		if not args.cpu:
			args.cpu = 'wasm'

		if not platform.system() == 'Darwin' and not platform.system() == 'Linux':
			print('Emscripten is only supported on OS X and Linux')
			sys.exit(1)

		if args.use_avx or args.use_avx2:
			print('AVX and AVX2 are not supported on Emscripten')
			sys.exit(1)

		if not args.cpu in ['wasm']:
			print('{} cpu architecture not in supported list [wasm] for Emscripten'.format(args.cpu))
			sys.exit(1)
	else:
		if not args.cpu:
			if is_arm64_cpu:
				args.cpu = 'arm64'
			elif is_armv7_cpu:
				args.cpu = 'armv7'
			elif is_riscv64_cpu:
				args.cpu = 'riscv64'
			else:
				args.cpu = 'x64'

	if args.cpu == 'arm64':
		is_arm_supported = False

		# Cross compilation
		if args.compiler in ['vs2017', 'vs2019', 'vs2022', 'vs2026', 'ios', 'android']:
			is_arm_supported = True

		# Native compilation
		if platform.system() == 'Darwin' and is_arm64_cpu:
			is_arm_supported = True
		elif platform.system() == 'Linux' and is_arm64_cpu:
			is_arm_supported = True

		if not is_arm_supported:
			print('arm64 is only supported with VS2017, VS2019, VS2022, VS2026, OS X (M* processors), Linux, Android, and iOS')
			sys.exit(1)
	elif args.cpu == 'arm64ec':
		if not args.compiler in ['vs2019', 'vs2022', 'vs2026']:
			print('arm64ec is only supported with VS2019, VS2022, and VS2026')
			sys.exit(1)
	elif args.cpu == 'armv7':
		is_armv7_supported = False

		# Cross compilation
		if args.compiler == 'android':
			is_armv7_supported = True

		# Native compilation
		if platform.system() == 'Linux' and is_armv7_cpu:
			is_armv7_supported = True

		if not is_armv7_supported:
			print('armv7 is only supported with Linux and Android')
			sys.exit(1)
	elif args.cpu == 'riscv64':
		if not platform.system() == 'Linux' or not is_riscv64_cpu:
			print('riscv64 is only supported natively on Linux hosts with a riscv64 compiler target; use CMake directly with a proper toolchain file for cross-compilation')
			sys.exit(1)
	elif args.cpu == 'wasm':
		if not args.compiler == 'emscripten':
			print('wasm is only supported with Emscripten')
			sys.exit(1)

	if args.analyze:
		# The static analyzer reads compile_commands.json, which only the Makefile and Ninja generators write
		if args.compiler in ['vs2015', 'vs2017', 'vs2019', 'vs2019-clang', 'vs2022', 'vs2022-clang', 'vs2026', 'vs2026-clang', 'android', 'osx', 'ios', 'emscripten'] or (args.compiler and args.compiler.startswith('gcc')):
			print('The static analyzer is only supported with clang on Linux and with the default compiler on OS X')
			sys.exit(1)

	if platform.system() == 'Darwin' and args.cpu == 'x86':
		result = subprocess.check_output(['xcodebuild', '-version']).decode("utf-8")
		if 'Xcode 11' in result:
			print('Versions of Xcode 11 and up no longer support x86')
			sys.exit(1)

	return args

def get_generator(compiler, cpu):
	if compiler == None:
		return None

	if platform.system() == 'Windows':
		if compiler == 'vs2015':
			if cpu == 'x86':
				return 'Visual Studio 14'
			elif cpu == 'x64':
				return 'Visual Studio 14 Win64'
		elif compiler == 'vs2017':
			if cpu == 'x86':
				return 'Visual Studio 15'
			elif cpu == 'x64':
				return 'Visual Studio 15 Win64'
			elif cpu == 'arm64':
				# VS2017 ARM/ARM64 support only works with cmake 3.13 and up and the architecture must be specified with
				# the -A cmake switch
				return 'Visual Studio 15 2017'
		elif compiler == 'vs2019' or compiler == 'vs2019-clang':
			return 'Visual Studio 16 2019'
		elif compiler == 'vs2022' or compiler == 'vs2022-clang':
			return 'Visual Studio 17 2022'
		elif compiler == 'vs2026' or compiler == 'vs2026-clang':
			# VS2026 support needs cmake 4.2 and up
			return 'Visual Studio 18 2026'
		elif compiler == 'android':
			# For Android, we use the default generator since we don't build with CMake
			return None
	elif platform.system() == 'Darwin':
		if compiler == 'osx' or compiler == 'ios':
			return 'Xcode'
		elif compiler == 'emscripten':
			# Emscripten uses the default generator
			return None
	elif platform.system() == 'Linux':
		if compiler == 'emscripten':
			# Emscripten uses the default generator
			return None

		return 'Unix Makefiles'

	print('Unknown compiler: {}'.format(compiler))
	print('See help with: python make.py -help')
	sys.exit(1)

def get_architecture(compiler, cpu):
	if compiler == None:
		return None

	if platform.system() == 'Windows':
		if compiler == 'vs2017':
			if cpu == 'arm64':
				return 'ARM64'

		is_modern_vs = False
		if compiler == 'vs2019' or compiler == 'vs2019-clang':
			is_modern_vs = True
		elif compiler == 'vs2022' or compiler == 'vs2022-clang':
			is_modern_vs = True
		elif compiler == 'vs2026' or compiler == 'vs2026-clang':
			is_modern_vs = True

		if is_modern_vs:
			if cpu == 'x86':
				return 'Win32'
			else:
				return cpu

	# This compiler/cpu pair does not need the architecture switch
	return None

def get_toolchain(compiler, cmake_script_dir):
	if platform.system() == 'Windows' and compiler == 'android':
		return os.path.join(cmake_script_dir, 'Toolchain-Android.cmake')
	elif platform.system() == 'Darwin' and compiler == 'ios':
		return os.path.join(cmake_script_dir, 'Toolchain-iOS.cmake')

	# No toolchain
	return None

def set_compiler_env(compiler, args):
	if platform.system() == 'Linux':
		if compiler == 'clang4':
			os.environ['CC'] = 'clang-4.0'
			os.environ['CXX'] = 'clang++-4.0'
		elif compiler == 'clang5':
			os.environ['CC'] = 'clang-5.0'
			os.environ['CXX'] = 'clang++-5.0'
		elif compiler == 'clang6':
			os.environ['CC'] = 'clang-6.0'
			os.environ['CXX'] = 'clang++-6.0'
		elif compiler == 'clang7':
			os.environ['CC'] = 'clang-7'
			os.environ['CXX'] = 'clang++-7'
		elif compiler == 'clang8':
			os.environ['CC'] = 'clang-8'
			os.environ['CXX'] = 'clang++-8'
		elif compiler == 'clang9':
			os.environ['CC'] = 'clang-9'
			os.environ['CXX'] = 'clang++-9'
		elif compiler == 'clang10':
			os.environ['CC'] = 'clang-10'
			os.environ['CXX'] = 'clang++-10'
		elif compiler == 'clang11':
			os.environ['CC'] = 'clang-11'
			os.environ['CXX'] = 'clang++-11'
		elif compiler == 'clang12':
			os.environ['CC'] = 'clang-12'
			os.environ['CXX'] = 'clang++-12'
		elif compiler == 'clang13':
			os.environ['CC'] = 'clang-13'
			os.environ['CXX'] = 'clang++-13'
		elif compiler == 'clang14':
			os.environ['CC'] = 'clang-14'
			os.environ['CXX'] = 'clang++-14'
		elif compiler == 'clang15':
			os.environ['CC'] = 'clang-15'
			os.environ['CXX'] = 'clang++-15'
		elif compiler == 'clang16':
			os.environ['CC'] = 'clang-16'
			os.environ['CXX'] = 'clang++-16'
		elif compiler == 'clang17':
			os.environ['CC'] = 'clang-17'
			os.environ['CXX'] = 'clang++-17'
		elif compiler == 'clang18':
			os.environ['CC'] = 'clang-18'
			os.environ['CXX'] = 'clang++-18'
		elif compiler == 'clang19':
			os.environ['CC'] = 'clang-19'
			os.environ['CXX'] = 'clang++-19'
		elif compiler == 'clang20':
			os.environ['CC'] = 'clang-20'
			os.environ['CXX'] = 'clang++-20'
		elif compiler == 'clang21':
			os.environ['CC'] = 'clang-21'
			os.environ['CXX'] = 'clang++-21'
		elif compiler == 'clang22':
			os.environ['CC'] = 'clang-22'
			os.environ['CXX'] = 'clang++-22'
		elif compiler == 'clang23':
			os.environ['CC'] = 'clang-23'
			os.environ['CXX'] = 'clang++-23'
		elif compiler == 'gcc5':
			os.environ['CC'] = 'gcc-5'
			os.environ['CXX'] = 'g++-5'
		elif compiler == 'gcc6':
			os.environ['CC'] = 'gcc-6'
			os.environ['CXX'] = 'g++-6'
		elif compiler == 'gcc7':
			os.environ['CC'] = 'gcc-7'
			os.environ['CXX'] = 'g++-7'
		elif compiler == 'gcc8':
			os.environ['CC'] = 'gcc-8'
			os.environ['CXX'] = 'g++-8'
		elif compiler == 'gcc9':
			os.environ['CC'] = 'gcc-9'
			os.environ['CXX'] = 'g++-9'
		elif compiler == 'gcc10':
			os.environ['CC'] = 'gcc-10'
			os.environ['CXX'] = 'g++-10'
		elif compiler == 'gcc11':
			os.environ['CC'] = 'gcc-11'
			os.environ['CXX'] = 'g++-11'
		elif compiler == 'gcc12':
			os.environ['CC'] = 'gcc-12'
			os.environ['CXX'] = 'g++-12'
		elif compiler == 'gcc13':
			os.environ['CC'] = 'gcc-13'
			os.environ['CXX'] = 'g++-13'
		elif compiler == 'gcc14':
			os.environ['CC'] = 'gcc-14'
			os.environ['CXX'] = 'g++-14'
		elif compiler == 'gcc15':
			os.environ['CC'] = 'gcc-15'
			os.environ['CXX'] = 'g++-15'
		elif compiler == 'emscripten':
			# Nothing to do for Emscripten
			return
		else:
			print('Unknown compiler: {}'.format(compiler))
			print('See help with: python make.py -help')
			sys.exit(1)

def do_generate_solution(build_dir, cmake_script_dir, args):
	compiler = args.compiler
	cpu = args.cpu
	config = args.config

	is_arm64_cpu = is_arm64_host()

	if compiler:
		set_compiler_env(compiler, args)

	extra_switches = ['--no-warn-unused-cli']
	extra_switches.append('-DCPU_INSTRUCTION_SET:STRING={}'.format(cpu))
	extra_switches.append('-DCMAKE_CXX_STANDARD:STRING={}'.format(args.cpp_version))

	if platform.system() == 'Windows' and not is_arm64_cpu:
		if cpu == 'arm64' or cpu == 'arm64ec':
			extra_switches.append('-DIS_CROSS_COMPILING:BOOL=true')

	if args.use_avx:
		print('Enabling AVX usage')
		extra_switches.append('-DUSE_AVX_INSTRUCTIONS:BOOL=true')

	if args.use_avx2:
		print('Enabling AVX2 usage')
		extra_switches.append('-DUSE_AVX2_INSTRUCTIONS:BOOL=true')

	if not args.use_simd:
		print('Disabling SIMD instruction usage')
		extra_switches.append('-DUSE_SIMD_INSTRUCTIONS:BOOL=false')

	if args.vector_mix_test:
		print('Enabling vector_mix unit tests')
		extra_switches.append('-DWITH_VECTOR_MIX_TESTS:BOOL=true')

	if args.fast_math:
		print('Enabling fast math for the include validation')
		extra_switches.append('-DUSE_FAST_MATH:BOOL=true')

	if args.bench:
		extra_switches.append('-DBUILD_BENCHMARK_EXE:BOOL=true')

	if args.analyze:
		extra_switches.append('-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=true')

	if platform.system() == 'Windows':
		if os.path.sep == '\\':
			# Native Windows
			extra_switches
		else:
			# MSYS2 or Cygwin
			extra_switches.append('-DCMAKE_BUILD_TYPE={}'.format(config.upper()))
	else:
		extra_switches.append('-DCMAKE_BUILD_TYPE={}'.format(config.upper()))

	if platform.system() == 'Darwin' and compiler == 'ios' and args.ci:
		# Disable code signing for CI iOS builds since we just test compilation
		extra_switches.append('-DCMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED=NO')
		extra_switches.append('-DCMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED=NO')

	toolchain = get_toolchain(compiler, cmake_script_dir)
	if toolchain:
		extra_switches.append('-DCMAKE_TOOLCHAIN_FILE={}'.format(toolchain))

	# Generate IDE solution
	print('Generating build files ...')
	if compiler == 'emscripten':
		cmake_cmd = 'emcmake cmake .. -DCMAKE_INSTALL_PREFIX="{}" {}'.format(build_dir, ' '.join(extra_switches))
	else:
		cmake_generator = get_generator(compiler, cpu)
		if not cmake_generator:
			print('Using default generator')
		else:
			generator_suffix = ''
			if compiler == 'vs2019-clang' or compiler == 'vs2022-clang' or compiler == 'vs2026-clang':
				extra_switches.append('-T ClangCL')
				generator_suffix = 'Clang CL'

			print('Using generator: {} {}'.format(cmake_generator, generator_suffix))
			extra_switches.append('-G "{}"'.format(cmake_generator))

		cmake_arch = get_architecture(compiler, cpu)
		if cmake_arch:
			print('Using architecture: {}'.format(cmake_arch))
			extra_switches.append('-A {}'.format(cmake_arch))

		cmake_cmd = 'cmake .. -DCMAKE_INSTALL_PREFIX="{}" {}'.format(build_dir, ' '.join(extra_switches))

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_build(args):
	config = args.config

	print('Building ...')
	cmake_cmd = 'cmake --build .'
	if platform.system() == 'Windows':
		if args.compiler == 'android':
			cmake_cmd += ' --config {}'.format(config)
		else:
			if os.path.sep == '\\':
				# Native Windows
				cmake_cmd += ' --config {} --target INSTALL'.format(config)
			else:
				# MSYS2 or Cygwin
				cmake_cmd += ' --config {} --target install'.format(config)
	elif platform.system() == 'Darwin':
		if args.compiler == 'ios':
			cmake_cmd += ' --config {}'.format(config)
		else:
			cmake_cmd += ' --config {} --target install'.format(config)
	else:
		cmake_cmd += ' --target install'

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_tests_android(build_dir, args):
	# Switch our working directory to where we built everything
	working_dir = os.path.join(build_dir, 'tests', 'main_android')
	os.chdir(working_dir)

	gradlew_exe = os.path.join(working_dir, 'gradlew.bat')

	# We uninstall first and then install
	if args.config == 'Debug':
		install_cmd = 'uninstallAll installDebug'
	elif args.config == 'Release':
		install_cmd = 'uninstallAll installRelease'

	# Install our app
	test_cmd = '"{}" {}'.format(gradlew_exe, install_cmd)
	result = subprocess.call(test_cmd, shell=True)
	if result != 0:
		sys.exit(result)

	# Execute through ADB
	run_cmd = 'adb shell am start -n "com.rtm.unit_tests/com.rtm.unit_tests.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER'
	result = subprocess.call(run_cmd, shell=True)
	if result != 0:
		sys.exit(result)

	# Restore working directory
	os.chdir(build_dir)

def do_tests_cmake(args):
	ctest_cmd = 'ctest --output-on-failure --parallel {}'.format(args.num_threads)

	if platform.system() == 'Windows' or platform.system() == 'Darwin':
		ctest_cmd += ' -C {}'.format(args.config)
	if args.tests_matching:
		ctest_cmd += ' --tests-regex {}'.format(args.tests_matching)

	result = subprocess.call(ctest_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_tests(build_dir, args):
	print('Running unit tests ...')

	if args.compiler == 'android':
		do_tests_android(build_dir, args)
	else:
		do_tests_cmake(args)

def do_run_bench_android(build_dir, args):
	# Switch our working directory to where we built everything
	working_dir = os.path.join(build_dir, 'tools', 'bench', 'main_android')
	os.chdir(working_dir)

	gradlew_exe = os.path.join(working_dir, 'gradlew.bat')

	# We uninstall first and then install
	if args.config == 'Debug':
		install_cmd = 'uninstallAll installDebug'
	elif args.config == 'Release':
		install_cmd = 'uninstallAll installRelease'

	# Install our app
	test_cmd = '"{}" {}'.format(gradlew_exe, install_cmd)
	result = subprocess.call(test_cmd, shell=True)
	if result != 0:
		sys.exit(result)

	# Execute through ADB
	run_cmd = 'adb shell am start -n "com.rtm.benchmark/com.rtm.benchmark.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER'
	result = subprocess.call(run_cmd, shell=True)
	if result != 0:
		sys.exit(result)

	# Restore working directory
	os.chdir(build_dir)

def do_pull_bench_android(build_dir):
	# Grab the android directory we wrote the results to
	output = str(subprocess.check_output('adb logcat -s acl -e "Benchmark results will be written to:" -m 1 -d'))
	matches = re.search(r'Benchmark results will be written to: ([/\.\w]+)', output)
	if matches == None:
		print('Failed to find Android source directory from ADB')
		android_src_dir = '/storage/emulated/0/Android/data/com.rtm.benchmark/files'
		print('{} will be used instead'.format(android_src_dir))
	else:
		android_src_dir = matches.group(1)

	# Grab the benchmark results from the android device
	dst_filename = os.path.join(build_dir, 'benchmark_results.json')
	src_filename = '{}/benchmark_results.json'.format(android_src_dir)
	cmd = 'adb pull "{}" "{}"'.format(src_filename, dst_filename)
	os.system(cmd)

def do_run_bench_native(build_dir):
	if platform.system() == 'Windows':
		bench_exe = os.path.join(os.getcwd(), 'bin/rtm_bench.exe')
	else:
		bench_exe = os.path.join(os.getcwd(), 'bin/rtm_bench')

	benchmark_output_filename = os.path.join(build_dir, 'benchmark_results.json')
	bench_cmd = '{} --benchmark_out={} --benchmark_out_format=json'.format(bench_exe, benchmark_output_filename)

	result = subprocess.call(bench_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_run_bench(build_dir, args):
	if args.compiler == 'ios':
		return	# Not supported on iOS

	print('Running benchmark ...')

	if args.compiler == 'android':
		do_run_bench_android(build_dir, args)
	else:
		do_run_bench_native(build_dir)

def get_analyzer_array_bound_checker(compiler_exe):
	# The array bound checker is not enabled by default
	# Clang 21 and later name it security.ArrayBound, older versions name it alpha.security.ArrayBoundV2
	checkers = [('-analyzer-checker-help', 'security.ArrayBound'), ('-analyzer-checker-help-alpha', 'alpha.security.ArrayBoundV2')]
	for help_switch, checker in checkers:
		try:
			output = subprocess.check_output([compiler_exe, '-cc1', help_switch], stderr=subprocess.STDOUT).decode('utf-8', 'replace')
		except (OSError, subprocess.CalledProcessError):
			continue

		if re.search(r'^\s*{}\s'.format(re.escape(checker)), output, re.MULTILINE):
			return checker

	return None

def get_analyzer_command(entry, analyzer_switches):
	if 'arguments' in entry:
		compile_args = entry['arguments']
	else:
		compile_args = shlex.split(entry['command'], posix=(os.name != 'nt'))

	# We remove the object file, the dependency file, and '-c' (an unused argument with -Werror is an error)
	analyzer_cmd = []
	skip_next_arg = False
	for arg in compile_args:
		if skip_next_arg:
			skip_next_arg = False
		elif arg in ['-o', '-MF', '-MT', '-MQ']:
			skip_next_arg = True
		elif arg not in ['-c', '-MD', '-MMD']:
			analyzer_cmd.append(arg)

	analyzer_cmd.extend(analyzer_switches)
	analyzer_cmd.extend(['-o', os.devnull])
	return analyzer_cmd

def run_analyzer(entry, analyzer_switches):
	analyzer_cmd = get_analyzer_command(entry, analyzer_switches)
	process = subprocess.Popen(analyzer_cmd, cwd=entry['directory'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
	output = process.communicate()[0].decode('utf-8', 'replace')
	return (process.returncode, output)

def normalize_analyzer_line(line, root_dir):
	# Print the paths relative to the root directory, for example 'includes/rtm/vqf.h:171:33: warning: ...'
	match = re.match(r'^(.+?):(\d+:\d+: .*)$', line)
	if not match or not os.path.isabs(match.group(1)):
		return line

	filename = os.path.realpath(match.group(1))
	if filename.startswith(root_dir + os.sep):
		filename = os.path.relpath(filename, root_dir)
	return '{}:{}'.format(filename, match.group(2))

def parse_analyzer_findings(output, root_dir):
	# A finding is a warning line and the source and note lines that follow it
	findings = []
	finding = None
	for line in output.splitlines():
		if re.match(r'^.+?:\d+:\d+: warning: ', line):
			finding = [line]
			findings.append(finding)
		elif line.startswith('In file included from') or re.match(r'^\d+ warnings? generated\.$', line):
			finding = None
		elif finding is not None:
			finding.append(line)

	return [[normalize_analyzer_line(line, root_dir) for line in finding] for finding in findings]

def escape_github_command_data(value):
	return value.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')

def escape_github_command_property(value):
	return escape_github_command_data(value).replace(':', '%3A').replace(',', '%2C')

def print_github_annotation(finding):
	# GitHub Actions shows this line as an annotation on the line of the finding
	# The message is the warning and its notes, without the source lines
	match = re.match(r'^(.+?):(\d+):(\d+): warning: (.*?)(?: \[([\w.-]+)\])?$', finding[0])
	filename = match.group(1).replace(os.sep, '/')
	checker = match.group(5)
	title = 'Static analyzer ({})'.format(checker) if checker else 'Static analyzer'
	notes = [line for line in finding[1:] if re.match(r'^.+?:\d+:\d+: note: ', line)]
	message = '\n'.join([match.group(4)] + notes)

	print('::error file={},line={},col={},title={}::{}'.format(
		escape_github_command_property(filename), match.group(2), match.group(3),
		escape_github_command_property(title), escape_github_command_data(message)))

def do_analyze(build_dir, args):
	print('Running the static analyzer ...')

	compile_commands_filename = os.path.join(build_dir, 'compile_commands.json')
	if not os.path.exists(compile_commands_filename):
		print('{} does not exist. The static analyzer needs a Makefile or Ninja generator.'.format(compile_commands_filename))
		sys.exit(1)

	with open(compile_commands_filename, 'r') as compile_commands_file:
		entries = json.load(compile_commands_file)

	# The include validation has one cpp file for each public header
	# We analyze the functions in the headers too, because a header-only library has no other functions to analyze
	entries = [entry for entry in entries if 'validate_includes' in entry['file'].replace('\\', '/').split('/')]
	if not entries:
		print('No include validation file is in {}'.format(compile_commands_filename))
		sys.exit(1)

	compiler_exe = get_analyzer_command(entries[0], [])[0]
	compiler_version = subprocess.check_output([compiler_exe, '--version']).decode('utf-8', 'replace')
	if 'clang' not in compiler_version.lower():
		print('The static analyzer needs clang. The compiler is: {}'.format(compiler_version.splitlines()[0]))
		sys.exit(1)

	analyzer_switches = ['--analyze', '--analyzer-output', 'text', '-Xclang', '-analyzer-opt-analyze-headers']

	array_bound_checker = get_analyzer_array_bound_checker(compiler_exe)
	if array_bound_checker:
		print('Enabling checker: {}'.format(array_bound_checker))
		analyzer_switches.extend(['-Xclang', '-analyzer-checker={}'.format(array_bound_checker)])
	else:
		print('This compiler has no array bound checker. Only the default checkers are used.')

	thread_pool = multiprocessing.pool.ThreadPool(int(args.num_threads))
	results = thread_pool.map(lambda entry: run_analyzer(entry, analyzer_switches), entries)
	thread_pool.close()
	thread_pool.join()

	def get_finding_location(warning_line):
		match = re.match(r'^(.+?):(\d+):(\d+): ', warning_line)
		return (match.group(1), int(match.group(2)), int(match.group(3)))

	# Each header is analyzed once for each cpp file that includes it, we print each finding once
	# The headers of the compiler are analyzed too (e.g. the intrinsic headers), we ignore the findings in them
	root_dir = os.path.realpath(os.path.dirname(build_dir))
	is_failure = False
	unique_findings = {}
	ignored_findings = set()
	for entry, (result, output) in zip(entries, results):
		if result != 0:
			print('The static analyzer failed on {}:'.format(entry['file']))
			print(output)
			is_failure = True
			continue

		for finding in parse_analyzer_findings(output, root_dir):
			# A finding in the repository has a relative path
			if os.path.isabs(get_finding_location(finding[0])[0]):
				ignored_findings.add(finding[0])
			else:
				unique_findings.setdefault(finding[0], finding)

	# Sort the findings by file, line, and column
	# On GitHub Actions, we also print each finding as an annotation
	is_github_actions = os.environ.get('GITHUB_ACTIONS') == 'true'
	for warning_line in sorted(unique_findings.keys(), key=get_finding_location):
		print('\n'.join(unique_findings[warning_line]))
		if is_github_actions:
			print_github_annotation(unique_findings[warning_line])

	if ignored_findings:
		print('The static analyzer ignored {} issues in files outside of the repository'.format(len(ignored_findings)))

	print('The static analyzer found {} issues in {} files'.format(len(unique_findings), len(entries)))
	if is_failure or unique_findings:
		sys.exit(1)

if __name__ == "__main__":
	args = parse_argv()

	build_dir = os.path.join(os.getcwd(), 'build')
	cmake_script_dir = os.path.join(os.getcwd(), 'cmake')

	is_clean_requested = args.clean or args.clean_only
	if is_clean_requested and os.path.exists(build_dir):
		print('Cleaning previous build ...')
		shutil.rmtree(build_dir)

	if args.clean_only:
		sys.exit(0)

	if not os.path.exists(build_dir):
		os.makedirs(build_dir)

	os.chdir(build_dir)

	print('Using config: {}'.format(args.config))
	print('Using cpu: {}'.format(args.cpu))
	if args.compiler:
		print('Using compiler: {}'.format(args.compiler))
	print('Using C++-{}'.format(args.cpp_version))
	print('Using {} threads'.format(args.num_threads))

	# Make sure 'make' runs with all available cores
	os.environ['MAKEFLAGS'] = '-j{}'.format(args.num_threads)

	do_generate_solution(build_dir, cmake_script_dir, args)

	if args.build:
		do_build(args)

	if args.unit_test:
		do_tests(build_dir, args)

	if args.analyze:
		do_analyze(build_dir, args)

	if args.run_bench:
		do_run_bench(build_dir, args)

	if args.pull_bench:
		do_pull_bench_android(build_dir)

	sys.exit(0)
