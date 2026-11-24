cmake_minimum_required(VERSION 3.22)
project(can_actuator_hil VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SBW_SANITIZE "Build everything with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(SBW_SANITIZE)
  # Set before GoogleTest is added so the test framework is instrumented too.
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

include(cmake/CompilerWarnings.cmake)

# A custom target has no output file, so it runs on every build; BYPRODUCTS tells Ninja the header is
# generated, and the script only replaces it when the identity changed.
set(SBW_GENERATED_INCLUDE_DIR "${CMAKE_BINARY_DIR}/generated")
set(SBW_BUILD_IDENTITY_HEADER "${SBW_GENERATED_INCLUDE_DIR}/sbw/build_identity.hpp")
add_custom_target(sbw_build_identity ALL
  COMMAND "${CMAKE_COMMAND}" -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DOUTPUT_FILE=${SBW_BUILD_IDENTITY_HEADER}
          -DBUILD_TYPE=${CMAKE_BUILD_TYPE} -P "${PROJECT_SOURCE_DIR}/cmake/WriteBuildIdentity.cmake"
  BYPRODUCTS "${SBW_BUILD_IDENTITY_HEADER}"
  COMMENT "Writing build identity")

include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
  URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_subdirectory(core)
add_subdirectory(sim)
add_subdirectory(tools)
add_subdirectory(tests)
