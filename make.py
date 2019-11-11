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
	actions.add_argument('-unit_test', action='store_true')
	actions.add_argument('-bench', action='store_true')

	target = parser.add_argument_group(title='Target')
	target.add_argument('-compiler', choices=['vs2015', 'vs2017', 'vs2019', 'android', 'clang4', 'clang5', 'clang6', 'clang7', 'gcc5', 'gcc6', 'gcc7', 'gcc8', 'gcc9', 'osx', 'ios'], help='Defaults to the host system\'s default compiler')
	target.add_argument('-config', choices=['Debug', 'Release'], type=str.capitalize)
	target.add_argument('-cpu', choices=['x86', 'x64', 'arm64'], help='Only supported for Windows, OS X, and Linux; defaults to the host system\'s architecture')

	misc = parser.add_argument_group(title='Miscellaneous')
	misc.add_argument('-avx', dest='use_avx', action='store_true', help='Compile using AVX instructions on Windows, OS X, and Linux')
	misc.add_argument('-avx2', dest='use_avx2', action='store_true', help='Compile using AVX2 instructions on Windows, OS X, and Linux')
	misc.add_argument('-nosimd', dest='use_simd', action='store_false', help='Compile without SIMD instructions')
	misc.add_argument('-num_threads', help='No. to use while compiling and regressing')
	misc.add_argument('-tests_matching', help='Only run tests whose names match this regex')
	misc.add_argument('-help', action='help', help='Display this usage information')

	num_threads = multiprocessing.cpu_count()
	if platform.system() == 'Linux' and sys.version_info >= (3, 4):
		num_threads = len(os.sched_getaffinity(0))
	if not num_threads or num_threads == 0:
		num_threads = 4

	parser.set_defaults(build=False, clean=False, unit_test=False, compiler=None, config='Release', cpu='x64', use_avx=False, use_avx2=False, use_simd=True, num_threads=num_threads, tests_matching='')

	args = parser.parse_args()

	# Sanitize and validate our options
	if (args.use_avx or args.use_avx2) and not args.use_simd:
		print('SIMD is explicitly disabled, AVX and AVX2 will not be used')
		args.use_avx = False
		args.use_avx2 = False

	if args.compiler == 'android':
		args.cpu = 'armv7-a'

		if not platform.system() == 'Windows':
			print('Android is only supported on Windows')
			sys.exit(1)

		if args.use_avx or args.use_avx2:
			print('AVX and AVX2 are not supported on Android')
			sys.exit(1)

		if args.unit_test:
			print('Unit tests cannot run from the command line on Android')
			sys.exit(1)

	if args.compiler == 'ios':
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

	if args.cpu == 'arm64':
		if not args.compiler in ['vs2017', 'vs2019', 'ios']:
			print('ARM64 is only supported with VS2017, VS2019, and iOS')
			sys.exit(1)

	if platform.system() == 'Darwin' and args.cpu == 'x86':
		result = subprocess.check_output(['xcodebuild', '-version']).decode("utf-8")
		if 'Xcode 11' in result:
			print('Versions of Xcode 11 and up no longer support x86')
			sys.exit(1)

	return args

def get_cmake_exes():
	if platform.system() == 'Windows':
		return ('cmake.exe', 'ctest.exe')
	else:
		return ('cmake', 'ctest')

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
		elif compiler == 'vs2019':
			return 'Visual Studio 16 2019'
		elif compiler == 'android':
			return 'Visual Studio 14'
	elif platform.system() == 'Darwin':
		if compiler == 'osx' or compiler == 'ios':
			return 'Xcode'
	else:
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
		elif compiler == 'vs2019':
			if cpu == 'x86':
				return 'Win32'
			else:
				return cpu

	# This compiler/cpu pair does not need the architecture switch
	return None

def get_toolchain(compiler):
	if platform.system() == 'Windows' and compiler == 'android':
		return 'Toolchain-Android.cmake'
	elif platform.system() == 'Darwin' and compiler == 'ios':
		return 'Toolchain-iOS.cmake'

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
		else:
			print('Unknown compiler: {}'.format(compiler))
			print('See help with: python make.py -help')
			sys.exit(1)

def do_generate_solution(cmake_exe, build_dir, cmake_script_dir, args):
	compiler = args.compiler
	cpu = args.cpu
	config = args.config

	if not compiler == None:
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

	if args.bench:
		extra_switches.append('-DBUILD_BENCHMARK_EXE:BOOL=true')

	if not platform.system() == 'Windows':
		extra_switches.append('-DCMAKE_BUILD_TYPE={}'.format(config.upper()))

	toolchain = get_toolchain(compiler)
	if not toolchain == None:
		extra_switches.append('-DCMAKE_TOOLCHAIN_FILE={}'.format(os.path.join(cmake_script_dir, toolchain)))

	# Generate IDE solution
	print('Generating build files ...')
	cmake_cmd = '"{}" .. -DCMAKE_INSTALL_PREFIX="{}" {}'.format(cmake_exe, build_dir, ' '.join(extra_switches))
	cmake_generator = get_generator(compiler, cpu)
	if cmake_generator == None:
		print('Using default generator')
	else:
		print('Using generator: {}'.format(cmake_generator))
		cmake_cmd += ' -G "{}"'.format(cmake_generator)

	cmake_arch = get_architecture(compiler, cpu)
	if cmake_arch:
		print('Using architecture: {}'.format(cmake_arch))
		cmake_cmd += ' -A {}'.format(cmake_arch)

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_build(cmake_exe, args):
	config = args.config

	print('Building ...')
	cmake_cmd = '"{}" --build .'.format(cmake_exe)
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

def do_tests(ctest_exe, args):
	config = args.config

	print('Running unit tests ...')
	ctest_cmd = '"{}" --output-on-failure --parallel {}'.format(ctest_exe, args.num_threads)

	if platform.system() == 'Windows' or platform.system() == 'Darwin':
		ctest_cmd += ' -C {}'.format(config)

	if args.tests_matching:
		ctest_cmd += ' --tests-regex {}'.format(args.tests_matching)

	result = subprocess.call(ctest_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_bench():
	if args.compiler == 'ios' or args.compiler == 'android':
		return	# Not supported on iOS or Android

	print('Running benchmark ...')

	if platform.system() == 'Windows':
		bench_exe = os.path.join(os.getcwd(), 'bin/rtm_bench.exe')
	else:
		bench_exe = os.path.join(os.getcwd(), 'bin/rtm_bench')

	bench_cmd = '{}'.format(bench_exe)

	result = subprocess.call(bench_cmd, shell=True)
	if result != 0:
		sys.exit(result)

if __name__ == "__main__":
	args = parse_argv()

	cmake_exe, ctest_exe = get_cmake_exes()
	compiler = args.compiler
	cpu = args.cpu
	config = args.config

	# Set the RTM_CMAKE_HOME environment variable to point to CMake
	# otherwise we assume it is already in the user PATH
	if 'RTM_CMAKE_HOME' in os.environ:
		cmake_home = os.environ['RTM_CMAKE_HOME']
		cmake_exe = os.path.join(cmake_home, 'bin', cmake_exe)
		ctest_exe = os.path.join(cmake_home, 'bin', ctest_exe)

	build_dir = os.path.join(os.getcwd(), 'build')
	cmake_script_dir = os.path.join(os.getcwd(), 'cmake')

	if args.clean and os.path.exists(build_dir):
		print('Cleaning previous build ...')
		shutil.rmtree(build_dir)

	if not os.path.exists(build_dir):
		os.makedirs(build_dir)

	os.chdir(build_dir)

	print('Using config: {}'.format(config))
	print('Using cpu: {}'.format(cpu))
	if not compiler == None:
		print('Using compiler: {}'.format(compiler))
	print('Using {} threads'.format(args.num_threads))

	do_generate_solution(cmake_exe, build_dir, cmake_script_dir, args)

	if args.build:
		do_build(cmake_exe, args)

	if args.unit_test:
		do_tests(ctest_exe, args)

	if args.bench:
		do_bench()

	sys.exit(0)
