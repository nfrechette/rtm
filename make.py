import os
import platform
import shutil
import subprocess
import sys

def parse_argv():
	options = {}
	options['build'] = False
	options['clean'] = False
	options['unit_test'] = False
	options['use_avx'] = False
	options['use_simd'] = True
	options['compiler'] = None
	options['config'] = 'Release'
	options['cpu'] = 'x64'
	options['num_threads'] = 4
	options['print_help'] = False

	for i in range(1, len(sys.argv)):
		value = sys.argv[i]
		value_upper = value.upper()

		if value == '-build':
			options['build'] = True

		if value == '-clean':
			options['clean'] = True

		if value == '-unit_test':
			options['unit_test'] = True

		if value == '-help':
			options['print_help'] = True

		if value == '-avx':
			options['use_avx'] = True

		if value == '-nosimd':
			options['use_simd'] = False

		# TODO: Refactor to use the form: -compiler=vs2015
		if value == '-vs2015':
			options['compiler'] = 'vs2015'

		if value == '-vs2017':
			options['compiler'] = 'vs2017'

		if value == '-android':
			options['compiler'] = 'android'

		if value == '-clang4':
			options['compiler'] = 'clang4'

		if value == '-clang5':
			options['compiler'] = 'clang5'

		if value == '-clang6':
			options['compiler'] = 'clang6'

		if value == '-gcc5':
			options['compiler'] = 'gcc5'

		if value == '-gcc6':
			options['compiler'] = 'gcc6'

		if value == '-gcc7':
			options['compiler'] = 'gcc7'

		if value == '-gcc8':
			options['compiler'] = 'gcc8'

		if value == '-osx':
			options['compiler'] = 'osx'

		if value == '-ios':
			options['compiler'] = 'ios'

		# TODO: Refactor to use the form: -config=Release
		if value_upper == '-DEBUG':
			options['config'] = 'Debug'

		if value_upper == '-RELEASE':
			options['config'] = 'Release'

		# TODO: Refactor to use the form: -cpu=x86
		if value == '-x86':
			options['cpu'] = 'x86'

		if value == '-x64':
			options['cpu'] = 'x64'

	# Sanitize and validate our options
	if options['use_avx'] and not options['use_simd']:
		print('SIMD is explicitly disabled, AVX will not be used')
		options['use_avx'] = False

	if options['compiler'] == 'android':
		options['cpu'] = 'armv7-a'

		if not platform.system() == 'Windows':
			print('Android is only supported on Windows')
			sys.exit(1)

		if options['use_avx']:
			print('AVX is not supported on Android')
			sys.exit(1)

		if options['unit_test']:
			print('Unit tests cannot run from the command line on Android')
			sys.exit(1)

	if options['compiler'] == 'ios':
		options['cpu'] = 'arm64'

		if not platform.system() == 'Darwin':
			print('iOS is only supported on OS X')
			sys.exit(1)

		if options['use_avx']:
			print('AVX is not supported on iOS')
			sys.exit(1)

		if options['unit_test']:
			print('Unit tests cannot run from the command line on iOS')
			sys.exit(1)

	return options

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
			else:
				return 'Visual Studio 14 Win64'
		elif compiler == 'vs2017':
			if cpu == 'x86':
				return 'Visual Studio 15'
			else:
				return 'Visual Studio 15 Win64'
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

def get_toolchain(compiler):
	if platform.system() == 'Windows' and compiler == 'android':
		return 'Toolchain-Android.cmake'
	elif platform.system() == 'Darwin' and compiler == 'ios':
		return 'Toolchain-iOS.cmake'

	# No toolchain
	return None

def set_compiler_env(compiler, options):
	if platform.system() == 'Linux':
		os.environ['MAKEFLAGS'] = '-j{}'.format(options['num_threads'])
		if compiler == 'clang4':
			os.environ['CC'] = 'clang-4.0'
			os.environ['CXX'] = 'clang++-4.0'
		elif compiler == 'clang5':
			os.environ['CC'] = 'clang-5.0'
			os.environ['CXX'] = 'clang++-5.0'
		elif compiler == 'clang6':
			os.environ['CC'] = 'clang-6.0'
			os.environ['CXX'] = 'clang++-6.0'
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
		else:
			print('Unknown compiler: {}'.format(compiler))
			print('See help with: python make.py -help')
			sys.exit(1)

def do_generate_solution(cmake_exe, build_dir, cmake_script_dir, options):
	compiler = options['compiler']
	cpu = options['cpu']
	config = options['config']

	if not compiler == None:
		set_compiler_env(compiler, options)

	extra_switches = ['--no-warn-unused-cli']
	if not platform.system() == 'Windows':
		extra_switches.append('-DCPU_INSTRUCTION_SET:STRING={}'.format(cpu))

	if options['use_avx']:
		print('Enabling AVX usage')
		extra_switches.append('-DUSE_AVX_INSTRUCTIONS:BOOL=true')

	if not options['use_simd']:
		print('Disabling SIMD instruction usage')
		extra_switches.append('-DUSE_SIMD_INSTRUCTIONS:BOOL=false')

	if not platform.system() == 'Windows' and not platform.system() == 'Darwin':
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

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_build(cmake_exe, options):
	config = options['config']

	print('Building ...')
	cmake_cmd = '"{}" --build .'.format(cmake_exe)
	if platform.system() == 'Windows':
		if options['compiler'] == 'android':
			cmake_cmd += ' --config {}'.format(config)
		else:
			cmake_cmd += ' --config {} --target INSTALL'.format(config)
	elif platform.system() == 'Darwin':
		if options['compiler'] == 'ios':
			cmake_cmd += ' --config {}'.format(config)
		else:
			cmake_cmd += ' --config {} --target install'.format(config)
	else:
		cmake_cmd += ' --target install'

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_tests(ctest_exe, options):
	config = options['config']

	print('Running unit tests ...')
	ctest_cmd = '"{}" --output-on-failure'.format(ctest_exe)
	if platform.system() == 'Windows' or platform.system() == 'Darwin':
		ctest_cmd += ' -C {}'.format(config)

	result = subprocess.call(ctest_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def print_help():
	print('Usage: python make.py [actions] [cpu arch] [compiler] [config] [misc]')
	print()
	print('Actions:')
	print('  If no action is specified, on Windows, OS X, and Linux the solution/make files are generated.')
	print('  Multiple actions can be used simultaneously.')
	print('  -build: Builds the solution.')
	print('  -clean: Cleans the build directory.')
	print('  -unit_test: Runs the unit tests.')
	print()
	print('CPU Architecture:')
	print('  Only supported for Windows, OS X, and Linux. Defaults to the host system architecture.')
	print('  Only a single architecture argument must be used.')
	print('  -x86: Builds an x86 executable.')
	print('  -x64: Builds an x64 executable.')
	print()
	print('Compiler:')
	print('  Defaults to the host system\'s default compiler.')
	print('  Only a single compiler argument must be used.')
	print('  -vs2015: Uses Visual Studio 2015.')
	print('  -vs2017: Uses Visual Studio 2017.')
	print('  -osx: Uses X Code for OS X.')
	print('  -gcc5: Uses GCC 5.')
	print('  -gcc6: Uses GCC 6.')
	print('  -gcc7: Uses GCC 7.')
	print('  -gcc8: Uses GCC 8.')
	print('  -clang4: Uses clang 4.')
	print('  -clang5: Uses clang 5.')
	print('  -clang6: Uses clang 6.')
	print('  -android: Uses NVIDIA CodeWorks.')
	print('  -ios: Uses X Code for iOS.')
	print()
	print('Config:')
	print('  Defaults to Release.')
	print('  Only a single config argument mus tbe used.')
	print('  -debug: Uses the Debug configuration to build and test.')
	print('  -release: Uses the Release configuration to build and test.')
	print()
	print('Miscelanous:')
	print('  -avx: On Windows, OS X, and Linux AVX support will be enabled.')
	print('  -help: Prints this help message.')

if __name__ == "__main__":
	options = parse_argv()
	if options['print_help']:
		print_help()
		sys.exit(1)

	cmake_exe, ctest_exe = get_cmake_exes()
	compiler = options['compiler']
	cpu = options['cpu']
	config = options['config']

	# Set the RTM_CMAKE_HOME environment variable to point to CMake
	# otherwise we assume it is already in the user PATH
	if 'RTM_CMAKE_HOME' in os.environ:
		cmake_home = os.environ['RTM_CMAKE_HOME']
		cmake_exe = os.path.join(cmake_home, 'bin', cmake_exe)
		ctest_exe = os.path.join(cmake_home, 'bin', ctest_exe)

	build_dir = os.path.join(os.getcwd(), 'build')
	cmake_script_dir = os.path.join(os.getcwd(), 'cmake')

	if options['clean'] and os.path.exists(build_dir):
		print('Cleaning previous build ...')
		shutil.rmtree(build_dir)

	if not os.path.exists(build_dir):
		os.makedirs(build_dir)

	os.chdir(build_dir)

	print('Using config: {}'.format(config))
	print('Using cpu: {}'.format(cpu))
	if not compiler == None:
		print('Using compiler: {}'.format(compiler))

	do_generate_solution(cmake_exe, build_dir, cmake_script_dir, options)

	if options['build']:
		do_build(cmake_exe, options)

	if options['unit_test']:
		do_tests(ctest_exe, options)

	sys.exit(0)
