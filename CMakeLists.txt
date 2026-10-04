cmake_minimum_required(VERSION 3.20)
project(bs_v11 VERSION 1.0 LANGUAGES CXX)

# Single-configuration generators build the portable v11 binary by default.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build configuration" FORCE)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BS_BUILD_BASELINE "Build baseline_v9 with the same compiler options" OFF)
option(BS_NATIVE "Optimize for this CPU; resulting binaries are NOT portable" OFF)
option(BS_LTO "Enable checked interprocedural/link-time optimization" OFF)
set(BS_SANITIZERS "none" CACHE STRING "none, address, undefined, address,undefined, or thread")
set_property(CACHE BS_SANITIZERS PROPERTY STRINGS none address undefined "address,undefined" thread)

include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
#if !defined(__x86_64__) && !defined(_M_X64)
#error An x64 compiler target is required
#endif
static_assert(sizeof(void*) == 8);
int main() { return 0; }
" BS_TARGET_X64)
if(NOT BS_TARGET_X64)
    message(FATAL_ERROR "Use an x64 toolchain (Visual Studio: cmake -A x64).")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
add_library(bs_build_options INTERFACE)
target_compile_features(bs_build_options INTERFACE cxx_std_20)
target_include_directories(bs_build_options INTERFACE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(bs_build_options INTERFACE Threads::Threads)

# These rules are shared by the application, baseline and assertion-enabled tests.
# Never enable fast-math or FP contraction: either can change EPS-boundary branches.
if(MSVC)
    target_compile_options(bs_build_options INTERFACE
        /W4 /utf-8 /permissive- /fp:strict
        "$<$<CONFIG:Debug>:/Od>" "$<$<NOT:$<CONFIG:Debug>>:/O2>")
    if(BS_NATIVE)
        message(FATAL_ERROR "BS_NATIVE is only supported by GCC/Clang GNU-style drivers; MSVC stays x64-portable.")
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
    target_compile_options(bs_build_options INTERFACE
        -Wall -Wextra -Wpedantic -pthread -fno-fast-math -ffp-contract=off
        "$<$<CONFIG:Debug>:-O0>" "$<$<NOT:$<CONFIG:Debug>>:-O3>")
    target_link_options(bs_build_options INTERFACE -pthread -fno-fast-math -ffp-contract=off)
    if(BS_NATIVE)
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-march=native" BS_HAS_MARCH_NATIVE)
        if(NOT BS_HAS_MARCH_NATIVE)
            message(FATAL_ERROR "The selected compiler does not support -march=native.")
        endif()
        target_compile_options(bs_build_options INTERFACE -march=native)
        message(STATUS "BS_NATIVE=ON: binaries may not run on other x64 CPUs; FP contraction remains OFF.")
    else()
        target_compile_options(bs_build_options INTERFACE -march=x86-64 -mtune=generic)
    endif()
else()
    message(FATAL_ERROR "Supported compilers: GCC, Clang, or MSVC.")
endif()

if(BS_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT BS_IPO_SUPPORTED OUTPUT BS_IPO_ERROR LANGUAGES CXX)
    if(NOT BS_IPO_SUPPORTED)
        message(FATAL_ERROR "BS_LTO was requested but is unavailable: ${BS_IPO_ERROR}")
    endif()
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

if(NOT BS_SANITIZERS MATCHES "^(none|address|undefined|address,undefined|thread)$")
    message(FATAL_ERROR "Invalid BS_SANITIZERS; choose none, address, undefined, address,undefined, or thread.")
endif()
if(NOT BS_SANITIZERS STREQUAL "none")
    if(MSVC)
        message(FATAL_ERROR "BS_SANITIZERS requires a GCC/Clang GNU-style driver with sanitizer runtimes.")
    endif()
    set(_bs_sanitizer_flags "-fsanitize=${BS_SANITIZERS}" -fno-omit-frame-pointer -fno-sanitize-recover=all)
    # Check both compilation and linking, not just whether a flag is recognized.
    include(CMakePushCheckState)
    cmake_push_check_state(RESET)
    set(CMAKE_REQUIRED_FLAGS "-fsanitize=${BS_SANITIZERS} -fno-omit-frame-pointer -fno-sanitize-recover=all -Werror")
    set(CMAKE_REQUIRED_LINK_OPTIONS "-fsanitize=${BS_SANITIZERS}")
    unset(BS_SANITIZERS_SUPPORTED CACHE)
    check_cxx_source_compiles("int main() { return 0; }" BS_SANITIZERS_SUPPORTED)
    cmake_pop_check_state()
    if(NOT BS_SANITIZERS_SUPPORTED)
        message(FATAL_ERROR "Requested sanitizer compiler/linker support is unavailable. Check CMake's configure log.")
    endif()
    target_compile_options(bs_build_options INTERFACE ${_bs_sanitizer_flags})
    target_link_options(bs_build_options INTERFACE ${_bs_sanitizer_flags})
endif()

function(bs_add_program target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} PRIVATE bs_build_options)
    target_compile_definitions(${target} PRIVATE "$<$<NOT:$<CONFIG:Debug>>:NDEBUG>")
endfunction()

# heuristic.hpp is header-only and included by main.cpp; no extra TU is needed.
bs_add_program(bs_v11 src/main.cpp src/reporting.cpp)
if(BS_BUILD_BASELINE)
    bs_add_program(baseline_v9 benchmarks/baseline_v9.cpp)
endif()

include(CTest)
if(BUILD_TESTING)
    foreach(test_name IN ITEMS geometry_tests solver_tests tail_cache_tests progress_tests heuristic_tests r2_tests r3_tests coverage_tests equal_radius_tests novelty_tests goal_finish_tests prerequisite_tests)
        if(EXISTS "${PROJECT_SOURCE_DIR}/tests/${test_name}.cpp")
            add_executable(${test_name} "tests/${test_name}.cpp" src/reporting.cpp)
            target_link_libraries(${test_name} PRIVATE bs_build_options)
            # CMake's Release flags define NDEBUG globally. Undefine it AFTER
            # those flags so assert-based tests really run in every configuration.
            if(MSVC)
                target_compile_options(${test_name} PRIVATE /UNDEBUG)
            else()
                target_compile_options(${test_name} PRIVATE -UNDEBUG)
            endif()
            add_test(NAME ${test_name} COMMAND ${test_name})
            set_tests_properties(${test_name} PROPERTIES TIMEOUT 120 WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
        else()
            message(STATUS "Skipping ${test_name}: tests/${test_name}.cpp is not present; reconfigure after adding it.")
        endif()
    endforeach()
endif()
