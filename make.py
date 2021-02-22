import argparse
import multiprocessing
import os
import platform
import shutil
import subprocess
import sys

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

	target = parser.add_argument_group(title='Target')
	target.add_argument('-compiler', choices=['vs2015', 'vs2017', 'vs2019', 'vs2019-clang', 'android', 'clang4', 'clang5', 'clang6', 'clang7', 'clang8', 'clang9', 'clang10', 'clang11', 'gcc5', 'gcc6', 'gcc7', 'gcc8', 'gcc9', 'gcc10', 'osx', 'ios', 'emscripten'], help='Defaults to the host system\'s default compiler')
	target.add_argument('-config', choices=['Debug', 'Release'], type=str.capitalize)
	target.add_argument('-cpu', choices=['x86', 'x64', 'armv7', 'arm64', 'wasm'], help='Defaults to the host system\'s architecture')

	misc = parser.add_argument_group(title='Miscellaneous')
	misc.add_argument('-avx', dest='use_avx', action='store_true', help='Compile using AVX instructions on Windows, OS X, and Linux')
	misc.add_argument('-avx2', dest='use_avx2', action='store_true', help='Compile using AVX2 instructions on Windows, OS X, and Linux')
	misc.add_argument('-nosimd', dest='use_simd', action='store_false', help='Compile without SIMD instructions')
	misc.add_argument('-num_threads', help='No. to use while compiling and regressing')
	misc.add_argument('-tests_matching', help='Only run tests whose names match this regex')
	misc.add_argument('-vector_mix_test', action='store_true', help='Enable the vector_mix unit tests')
	misc.add_argument('-help', action='help', help='Display this usage information')

	num_threads = multiprocessing.cpu_count()
	if platform.system() == 'Linux' and sys.version_info >= (3, 4):
		num_threads = len(os.sched_getaffinity(0))
	if not num_threads or num_threads == 0:
		num_threads = 4

	parser.set_defaults(build=False, clean=False, clean_only=False, unit_test=False,
		compiler=None, config='Release', cpu=None, use_avx=False, use_avx2=False, use_simd=True, num_threads=num_threads, tests_matching='', vector_mix_test=False,
		bench=False, run_bench=False, pull_bench=False)

	args = parser.parse_args()

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
			args.cpu = 'x64'

	if args.cpu == 'arm64':
		if not args.compiler in ['vs2017', 'vs2019', 'ios', 'android']:
			print('arm64 is only supported with VS2017, VS2019, Android, and iOS')
			sys.exit(1)
	elif args.cpu == 'armv7':
		if not args.compiler == 'android':
			print('armv7 is only supported with Android')
			sys.exit(1)
	elif args.cpu == 'wasm':
		if not args.compiler == 'emscripten':
			print('wasm is only supported with Emscripten')
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
		elif compiler == 'vs2019' or compiler == 'vs2019-clang':
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
		os.environ['MAKEFLAGS'] = '-j{}'.format(args.num_threads)
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

	if compiler:
		set_compiler_env(compiler, args)

	extra_switches = ['--no-warn-unused-cli']
	extra_switches.append('-DCPU_INSTRUCTION_SET:STRING={}'.format(cpu))

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

	if args.bench:
		extra_switches.append('-DBUILD_BENCHMARK_EXE:BOOL=true')

	if not platform.system() == 'Windows':
		extra_switches.append('-DCMAKE_BUILD_TYPE={}'.format(config.upper()))

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
			if compiler == 'vs2019-clang':
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
			cmake_cmd += ' --config {} --target INSTALL'.format(config)
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
	matches = re.search('Benchmark results will be written to: ([/\.\w]+)', output)
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
	print('Using {} threads'.format(args.num_threads))

	do_generate_solution(build_dir, cmake_script_dir, args)

	if args.build:
		do_build(args)

	if args.unit_test:
		do_tests(build_dir, args)

	if args.run_bench:
		do_run_bench(build_dir, args)

	if args.pull_bench:
		do_pull_bench_android(build_dir)

	sys.exit(0)
