import os
import platform
import shutil
import subprocess
import sys

def get_platform_compilers():
	if platform.system() == 'Windows':
		return [ 'vs2015', 'vs2017', 'vs2019', 'vs2019-clang' ]
	elif platform.system() == 'Linux':
		compilers = []
		if shutil.which('g++-5'):
			compilers.append('gcc5')
		if shutil.which('g++-6'):
			compilers.append('gcc6')
		if shutil.which('g++-7'):
			compilers.append('gcc7')
		if shutil.which('g++-8'):
			compilers.append('gcc8')
		if shutil.which('g++-9'):
			compilers.append('gcc9')
		if shutil.which('g++-10'):
			compilers.append('gcc10')

		if shutil.which('clang++-4.0'):
			compilers.append('clang4')
		if shutil.which('clang++-5.0'):
			compilers.append('clang5')
		if shutil.which('clang++-6.0'):
			compilers.append('clang6')
		if shutil.which('clang++-7'):
			compilers.append('clang7')
		if shutil.which('clang++-8'):
			compilers.append('clang8')
		if shutil.which('clang++-9'):
			compilers.append('clang9')
		if shutil.which('clang++-10'):
			compilers.append('clang10')
		if shutil.which('clang++-11'):
			compilers.append('clang11')

		return compilers
	elif platform.system() == 'Darwin':
		return [ 'osx' ]
	else:
		print('Unknown platform!')
		sys.exit(1)

def get_python_exe_name():
	if platform.system() == 'Windows':
		return 'python'
	else:
		return 'python3'

if __name__ == "__main__":
	os.environ['PYTHONIOENCODING'] = 'utf_8'

	configs = [ 'debug', 'release' ]
	archs = [ 'x86', 'x64' ]
	compilers = get_platform_compilers()
	simd_opts = [ '', '-avx', '-avx2', '-nosimd' ]
	python_exe = get_python_exe_name()

	if platform.system() == 'Darwin':
		result = subprocess.check_output(['xcodebuild', '-version']).decode("utf-8")
		if 'Xcode 11' in result:
			archs.remove('x86')

	cmd_args = []
	for config in configs:
		for arch in archs:
			for compiler in compilers:
				for simd in simd_opts:
					if compiler == 'clang7' and simd == '-nosimd':
						continue	# Hack to avoid compiler issue
					if compiler == 'clang8' and simd == '-nosimd':
						continue	# Hack to avoid compiler issue
					args = [python_exe, 'make.py', '-compiler', compiler, '-cpu', arch, '-config', config, simd, '-build', '-unit_test', '-vector_mix_test', '-clean']
					cmd_args.append([x for x in args if x])

	if platform.system() == 'Windows':
		for config in configs:
			# Windows ARM
			args = [python_exe, 'make.py', '-compiler', 'vs2017', '-cpu', 'arm64', '-config', config, '-build', '-vector_mix_test', '-clean']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'vs2017', '-cpu', 'arm64', '-config', config, '-build', '-vector_mix_test', '-clean', '-nosimd']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'vs2019', '-cpu', 'arm64', '-config', config, '-build', '-vector_mix_test', '-clean']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'vs2019', '-cpu', 'arm64', '-config', config, '-build', '-vector_mix_test', '-clean', '-nosimd']
			cmd_args.append([x for x in args if x])

			# Android
			args = [python_exe, 'make.py', '-compiler', 'android', '-cpu', 'armv7', '-config', config, '-build', '-vector_mix_test', '-clean']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'android', '-cpu', 'armv7', '-config', config, '-build', '-vector_mix_test', '-clean', '-nosimd']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'android', '-cpu', 'arm64', '-config', config, '-build', '-vector_mix_test', '-clean']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'android', '-cpu', 'arm64', '-config', config, '-build', '-vector_mix_test', '-clean', '-nosimd']
			cmd_args.append([x for x in args if x])
	elif platform.system() == 'Darwin':
		for config in configs:
			# iOS
			args = [python_exe, 'make.py', '-compiler', 'ios', '-config', config, '-build', '-vector_mix_test', '-clean']
			cmd_args.append([x for x in args if x])
			args = [python_exe, 'make.py', '-compiler', 'ios', '-config', config, '-build', '-vector_mix_test', '-clean', '-nosimd']
			cmd_args.append([x for x in args if x])

	if platform.system() == 'Darwin' or platform.system() == 'Linux':
		# Emscripten
		args = [python_exe, 'make.py', '-compiler', 'emscripten', '-config', 'debug', '-build', '-vector_mix_test', '-clean']
		cmd_args.append([x for x in args if x])
		args = [python_exe, 'make.py', '-compiler', 'emscripten', '-config', 'release', '-build', '-unit_test', '-vector_mix_test', '-clean']
		cmd_args.append([x for x in args if x])

	root_dir = os.path.join(os.getcwd(), '../..')
	os.chdir(root_dir)

	for args in cmd_args:
		cmd = " ".join(args)
		print('Running command: "{}" ...'.format(cmd))
		try:
			if 'android' in args:
				subprocess.check_call(args)
			else:
				subprocess.check_output(args)
		except subprocess.CalledProcessError as e:
			print('Failed command: {}'.format(cmd))
			print(e.output.decode(sys.stdout.encoding))
			sys.exit(1)

	print('Done!')
	sys.exit(0)
