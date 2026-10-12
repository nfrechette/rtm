project(rtm_unit_tests CXX)

include_directories("${PROJECT_SOURCE_DIR}/../../includes")
include_directories("${PROJECT_SOURCE_DIR}/../../external/catch2/single_include")
include_directories("${PROJECT_SOURCE_DIR}/../sources")

# Grab all of our test source files
file(GLOB_RECURSE ALL_TEST_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/../sources/*.h
	${PROJECT_SOURCE_DIR}/../sources/*.cpp)

if(NOT WITH_VECTOR_MIX_TESTS)
	# The vector_mix unit tests are slow to compile, do not compile them when they are disabled
	file(GLOB VECTOR_MIX_SOURCE_FILES LIST_DIRECTORIES false
		${PROJECT_SOURCE_DIR}/../sources/vector_mix/*.cpp)
	list(REMOVE_ITEM ALL_TEST_SOURCE_FILES ${VECTOR_MIX_SOURCE_FILES})
endif()

create_source_groups("${ALL_TEST_SOURCE_FILES}" ${PROJECT_SOURCE_DIR}/..)

# Grab all of our main source files
file(GLOB_RECURSE ALL_MAIN_SOURCE_FILES LIST_DIRECTORIES false
	${PROJECT_SOURCE_DIR}/*.cpp)

create_source_groups("${ALL_MAIN_SOURCE_FILES}" ${PROJECT_SOURCE_DIR})

add_executable(${PROJECT_NAME} ${ALL_TEST_SOURCE_FILES} ${ALL_MAIN_SOURCE_FILES})

if(NOT IS_CROSS_COMPILING)
	# Don't attempt to detect unit tests if we are cross-compiling
	list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../external/catch2/contrib")
	include(CTest)
	include(Catch)
	catch_discover_tests(${PROJECT_NAME})
endif()

setup_default_compiler_flags(${PROJECT_NAME})

if(MSVC)
	if(CPU_INSTRUCTION_SET MATCHES "arm64" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		# Exceptions are not enabled by default for ARM targets and clang-cl, enable them
		target_compile_options(${PROJECT_NAME} PRIVATE /EHsc)
	endif()
endif()

# Throw on failure to allow us to catch them and recover
add_definitions(-DRTM_ON_ASSERT_THROW)

if(WITH_VECTOR_MIX_TESTS)
	# Enable vector_mix unit tests
	add_definitions(-DRTM_IMPL_WITH_VECTOR_MIX_TESTS)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
