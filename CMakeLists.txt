cmake_minimum_required(VERSION 3.24)
project(fsoc_mvp VERSION 0.1.0 LANGUAGES CXX)

option(FSOC_BUILD_TESTS "Build unit tests" ON)
option(FSOC_ENABLE_SANITIZERS "Enable Address/Undefined sanitizers in Debug" OFF)
option(FSOC_ENABLE_OPENCV "Enable OpenCV-dependent modules when they are added" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)

add_library(fsoc_core
    src/geometry.cpp
    src/camera.cpp
    src/environment.cpp
    src/trajectory.cpp
)
add_library(fsoc::core ALIAS fsoc_core)

target_include_directories(fsoc_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(fsoc_core PUBLIC cxx_std_20)
fsoc_set_project_warnings(fsoc_core)
fsoc_enable_sanitizers(fsoc_core)

if(FSOC_ENABLE_OPENCV)
    find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)
    target_link_libraries(fsoc_core PUBLIC ${OpenCV_LIBS})
    target_compile_definitions(fsoc_core PUBLIC FSOC_HAS_OPENCV=1)
endif()

add_executable(step1_math_smoke apps/step1_math_smoke.cpp)
target_link_libraries(step1_math_smoke PRIVATE fsoc::core)
fsoc_set_project_warnings(step1_math_smoke)
fsoc_enable_sanitizers(step1_math_smoke)

add_executable(step2_trajectory_smoke apps/step2_trajectory_smoke.cpp)
target_link_libraries(step2_trajectory_smoke PRIVATE fsoc::core)
fsoc_set_project_warnings(step2_trajectory_smoke)
fsoc_enable_sanitizers(step2_trajectory_smoke)

if(FSOC_BUILD_TESTS)
    enable_testing()

    add_executable(fsoc_step1_tests tests/step1_tests.cpp)
    target_link_libraries(fsoc_step1_tests PRIVATE fsoc::core)
    fsoc_set_project_warnings(fsoc_step1_tests)
    fsoc_enable_sanitizers(fsoc_step1_tests)
    add_test(NAME fsoc_step1_tests COMMAND fsoc_step1_tests)

    add_executable(fsoc_step2_tests tests/step2_tests.cpp)
    target_link_libraries(fsoc_step2_tests PRIVATE fsoc::core)
    fsoc_set_project_warnings(fsoc_step2_tests)
    fsoc_enable_sanitizers(fsoc_step2_tests)
    add_test(NAME fsoc_step2_tests COMMAND fsoc_step2_tests)
endif()
