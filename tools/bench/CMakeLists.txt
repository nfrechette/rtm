cmake_minimum_required (VERSION 3.2)
project(rtm_bench_root CXX)

if(PLATFORM_ANDROID)
	add_subdirectory("${PROJECT_SOURCE_DIR}/main_android")
elseif(PLATFORM_IOS)
	# TODO: not yet supported on iOS, see https://github.com/google/benchmark/issues/891
	#add_subdirectory("${PROJECT_SOURCE_DIR}/main_ios")
else()
	add_subdirectory("${PROJECT_SOURCE_DIR}/main_generic")
endif()
