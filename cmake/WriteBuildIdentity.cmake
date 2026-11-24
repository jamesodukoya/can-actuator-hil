# Runs at build time (cmake -P) so the identity follows every commit without re-running CMake configure.
# Inputs: SOURCE_DIR, OUTPUT_FILE, BUILD_TYPE.

# The full hash cut to 8 digits: `git rev-parse --short=8` returns more digits when 8 are ambiguous.
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${SOURCE_DIR}"
  OUTPUT_VARIABLE fullCommitHash
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE revParseResult)
if(NOT revParseResult EQUAL 0)
  message(FATAL_ERROR "git rev-parse HEAD failed in ${SOURCE_DIR}: the build identity needs a git checkout")
endif()
string(SUBSTRING "${fullCommitHash}" 0 8 commitHash)

# Untracked files are ignored: only tracked files are compiled.
execute_process(
  COMMAND git status --porcelain --untracked-files=no
  WORKING_DIRECTORY "${SOURCE_DIR}"
  OUTPUT_VARIABLE trackedChanges
  RESULT_VARIABLE statusResult)
if(NOT statusResult EQUAL 0)
  message(FATAL_ERROR "git status failed in ${SOURCE_DIR}")
endif()
if(trackedChanges STREQUAL "")
  set(hasUncommittedChanges false)
else()
  set(hasUncommittedChanges true)
endif()

if(BUILD_TYPE STREQUAL "Release")
  set(buildType Release)
else()
  set(buildType Debug)
endif()

file(WRITE "${OUTPUT_FILE}.tmp" "#pragma once
// Generated at build time by cmake/WriteBuildIdentity.cmake. Do not edit.
#include \"sbw/can_protocol.hpp\"

namespace sbw {
inline constexpr BuildMessage kBuildIdentity{.commitHash = 0x${commitHash}u,
                                             .buildType = BuildType::${buildType},
                                             .hasUncommittedChanges = ${hasUncommittedChanges}};
}  // namespace sbw
")
# Replace the header only when its content changed, so an unchanged identity recompiles nothing.
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${OUTPUT_FILE}.tmp" "${OUTPUT_FILE}")
file(REMOVE "${OUTPUT_FILE}.tmp")
