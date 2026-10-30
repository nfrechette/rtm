project(rtm_unit_tests_root NONE)

if(PLATFORM_ANDROID_GRADLE)
	# Gradle builds an Android app. With the NDK CMake toolchain, Android uses main_generic.
	add_subdirectory("${PROJECT_SOURCE_DIR}/main_android")
elseif(PLATFORM_IOS)
	add_subdirectory("${PROJECT_SOURCE_DIR}/main_ios")
elseif(PLATFORM_EMSCRIPTEN)
	add_subdirectory("${PROJECT_SOURCE_DIR}/main_emscripten")
else()
	add_subdirectory("${PROJECT_SOURCE_DIR}/main_generic")
	add_subdirectory("${PROJECT_SOURCE_DIR}/validate_includes")
endif()
