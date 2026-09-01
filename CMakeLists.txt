cmake_minimum_required(VERSION 3.24)
project(fsoc_mvp VERSION 0.1.0 LANGUAGES CXX)

option(FSOC_BUILD_TESTS "Build unit tests" ON)
option(FSOC_ENABLE_SANITIZERS "Enable Address/Undefined sanitizers in Debug" OFF)

# OpenCV powers only the image/perception boundary (Step 4+). Pure math/core stays
# OpenCV-free. AUTO builds those targets when OpenCV is discoverable and silently
# skips them otherwise; ON makes OpenCV mandatory; OFF never looks for it.
set(FSOC_ENABLE_OPENCV "AUTO" CACHE STRING "OpenCV renderer: AUTO | ON | OFF")
set_property(CACHE FSOC_ENABLE_OPENCV PROPERTY STRINGS AUTO ON OFF)

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
    src/observation.cpp
    src/tracking_error.cpp
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

# ---------------------------------------------------------------------------
# fsoc_control — pan/tilt PID controller (Step 6). OpenCV-FREE.
# Depends on fsoc::core only (fsoc/tracking_error.hpp contract). No image,
# perception, or OpenCV dependency.
# ---------------------------------------------------------------------------
add_library(fsoc_control src/pid_controller.cpp)
add_library(fsoc::control ALIAS fsoc_control)
target_include_directories(fsoc_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(fsoc_control PUBLIC cxx_std_20)
target_link_libraries(fsoc_control PUBLIC fsoc::core)
fsoc_set_project_warnings(fsoc_control)
fsoc_enable_sanitizers(fsoc_control)

# ---------------------------------------------------------------------------
# OpenCV discovery (image/perception boundary only — never linked into fsoc_core)
# ---------------------------------------------------------------------------
set(FSOC_OPENCV_AVAILABLE OFF)
if(NOT FSOC_ENABLE_OPENCV STREQUAL "OFF")
    if(FSOC_ENABLE_OPENCV STREQUAL "ON")
        find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
    else()
        find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
    endif()
    if(OpenCV_FOUND)
        set(FSOC_OPENCV_AVAILABLE ON)
        message(STATUS "FSOC: OpenCV ${OpenCV_VERSION} found (${OpenCV_DIR}) - building Step 4 renderer + Step 5 detector")
    else()
        message(STATUS "FSOC: OpenCV not found - Step 4/5 image modules disabled. Install it with: brew install opencv")
    endif()
endif()

add_executable(step1_math_smoke apps/step1_math_smoke.cpp)
target_link_libraries(step1_math_smoke PRIVATE fsoc::core)
fsoc_set_project_warnings(step1_math_smoke)
fsoc_enable_sanitizers(step1_math_smoke)

add_executable(step2_trajectory_smoke apps/step2_trajectory_smoke.cpp)
target_link_libraries(step2_trajectory_smoke PRIVATE fsoc::core)
fsoc_set_project_warnings(step2_trajectory_smoke)
fsoc_enable_sanitizers(step2_trajectory_smoke)

add_executable(step3_observation_smoke apps/step3_observation_smoke.cpp)
target_link_libraries(step3_observation_smoke PRIVATE fsoc::core)
fsoc_set_project_warnings(step3_observation_smoke)
fsoc_enable_sanitizers(step3_observation_smoke)

add_executable(step6_pid_smoke apps/step6_pid_smoke.cpp)
target_link_libraries(step6_pid_smoke PRIVATE fsoc::control)
fsoc_set_project_warnings(step6_pid_smoke)
fsoc_enable_sanitizers(step6_pid_smoke)

# ---------------------------------------------------------------------------
# fsoc_render — synthetic image renderer (OpenCV-dependent; Step 4)
# ---------------------------------------------------------------------------
if(FSOC_OPENCV_AVAILABLE)
    add_library(fsoc_render src/renderer.cpp)
    add_library(fsoc::render ALIAS fsoc_render)
    target_include_directories(fsoc_render
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_render PUBLIC cxx_std_20)
    target_link_libraries(fsoc_render PUBLIC fsoc::core ${OpenCV_LIBS})
    fsoc_set_project_warnings(fsoc_render)
    fsoc_enable_sanitizers(fsoc_render)

    add_executable(step4_renderer_smoke apps/step4_renderer_smoke.cpp)
    target_link_libraries(step4_renderer_smoke PRIVATE fsoc::render)
    fsoc_set_project_warnings(step4_renderer_smoke)
    fsoc_enable_sanitizers(step4_renderer_smoke)

    # -----------------------------------------------------------------------
    # fsoc_perception — baseline beacon detector (Step 5)
    # Depends on fsoc::core + OpenCV (core, imgproc). MUST NOT depend on fsoc_render.
    # -----------------------------------------------------------------------
    add_library(fsoc_perception src/detector.cpp)
    add_library(fsoc::perception ALIAS fsoc_perception)
    target_include_directories(fsoc_perception
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_perception PUBLIC cxx_std_20)
    target_link_libraries(fsoc_perception PUBLIC fsoc::core opencv_core opencv_imgproc)
    fsoc_set_project_warnings(fsoc_perception)
    fsoc_enable_sanitizers(fsoc_perception)

    add_executable(step5_detector_smoke apps/step5_detector_smoke.cpp)
    target_link_libraries(step5_detector_smoke PRIVATE fsoc::perception fsoc::render)
    fsoc_set_project_warnings(step5_detector_smoke)
    fsoc_enable_sanitizers(step5_detector_smoke)

    # -----------------------------------------------------------------------
    # fsoc_simulation — the deterministic closed-loop integration layer (Step 7).
    # This is the ONE place the tested modules come together.
    # -----------------------------------------------------------------------
    add_library(fsoc_simulation src/simulation_runner.cpp)
    add_library(fsoc::simulation ALIAS fsoc_simulation)
    target_include_directories(fsoc_simulation
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_simulation PUBLIC cxx_std_20)
    target_link_libraries(fsoc_simulation
        PUBLIC fsoc::core fsoc::render fsoc::perception fsoc::control)
    fsoc_set_project_warnings(fsoc_simulation)
    fsoc_enable_sanitizers(fsoc_simulation)

    add_executable(step7_closed_loop_smoke apps/step7_closed_loop_smoke.cpp)
    target_link_libraries(step7_closed_loop_smoke PRIVATE fsoc::simulation)
    fsoc_set_project_warnings(step7_closed_loop_smoke)
    fsoc_enable_sanitizers(step7_closed_loop_smoke)

    # -----------------------------------------------------------------------
    # fsoc_telemetry — observer-only telemetry + benchmarking (Step 8).
    # Consumes SimulationStepResult; never reaches back into the loop.
    # -----------------------------------------------------------------------
    add_library(fsoc_telemetry src/telemetry.cpp)
    add_library(fsoc::telemetry ALIAS fsoc_telemetry)
    target_include_directories(fsoc_telemetry
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_telemetry PUBLIC cxx_std_20)
    target_link_libraries(fsoc_telemetry PUBLIC fsoc::simulation)
    fsoc_set_project_warnings(fsoc_telemetry)
    fsoc_enable_sanitizers(fsoc_telemetry)

    add_executable(step8_telemetry_smoke apps/step8_telemetry_smoke.cpp)
    target_link_libraries(step8_telemetry_smoke PRIVATE fsoc::telemetry)
    fsoc_set_project_warnings(step8_telemetry_smoke)
    fsoc_enable_sanitizers(step8_telemetry_smoke)

    # -----------------------------------------------------------------------
    # fsoc_visualization — engineering camera-view debug overlay (Step 9).
    # Observer-only: annotates a COPY; the perception frame is never touched.
    # -----------------------------------------------------------------------
    add_library(fsoc_visualization src/visualization.cpp)
    add_library(fsoc::visualization ALIAS fsoc_visualization)
    target_include_directories(fsoc_visualization
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_visualization PUBLIC cxx_std_20)
    target_link_libraries(fsoc_visualization
        PUBLIC fsoc::simulation fsoc::telemetry opencv_core opencv_imgproc opencv_imgcodecs)
    # Optional MP4 export — only if this OpenCV build has videoio.
    if(TARGET opencv_videoio)
        target_link_libraries(fsoc_visualization PUBLIC opencv_videoio)
        target_compile_definitions(fsoc_visualization PUBLIC FSOC_HAS_VIDEOIO=1)
        message(STATUS "FSOC: OpenCV videoio present - Step 9 optional MP4 export enabled")
    else()
        message(STATUS "FSOC: OpenCV videoio absent - Step 9 uses the PNG path only")
    endif()
    fsoc_set_project_warnings(fsoc_visualization)
    fsoc_enable_sanitizers(fsoc_visualization)

    add_executable(step9_visualization_smoke apps/step9_visualization_smoke.cpp)
    target_link_libraries(step9_visualization_smoke PRIVATE fsoc::visualization)
    fsoc_set_project_warnings(step9_visualization_smoke)
    fsoc_enable_sanitizers(step9_visualization_smoke)

    # -----------------------------------------------------------------------
    # fsoc_validation — baseline acceptance / validation suite (Step 10).
    # Evaluation layer: runs the existing system, checks documented gates.
    # -----------------------------------------------------------------------
    add_library(fsoc_validation src/validation.cpp)
    add_library(fsoc::validation ALIAS fsoc_validation)
    target_include_directories(fsoc_validation
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_validation PUBLIC cxx_std_20)
    target_link_libraries(fsoc_validation
        PUBLIC fsoc::simulation fsoc::telemetry fsoc::visualization)
    fsoc_set_project_warnings(fsoc_validation)
    fsoc_enable_sanitizers(fsoc_validation)

    add_executable(step10_validation_smoke apps/step10_validation_smoke.cpp)
    target_link_libraries(step10_validation_smoke PRIVATE fsoc::validation)
    fsoc_set_project_warnings(step10_validation_smoke)
    fsoc_enable_sanitizers(step10_validation_smoke)

    # -----------------------------------------------------------------------
    # fsoc_demo_support — demo/frontend packaging layer (Step 11). ADDITIVE.
    # Pure observer: copies SimulationStepResult + TelemetryRecord into a
    # frontend-oriented DemoSnapshot; owns DemoScenario presets + DemoSession.
    # No physics, no control, no networking, no JSON.
    # -----------------------------------------------------------------------
    add_library(fsoc_demo_support src/demo.cpp)
    add_library(fsoc::demo_support ALIAS fsoc_demo_support)
    target_include_directories(fsoc_demo_support
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(fsoc_demo_support PUBLIC cxx_std_20)
    target_link_libraries(fsoc_demo_support PUBLIC fsoc::simulation fsoc::telemetry)
    fsoc_set_project_warnings(fsoc_demo_support)
    fsoc_enable_sanitizers(fsoc_demo_support)

    add_executable(fsoc_demo apps/fsoc_demo.cpp)
    target_link_libraries(fsoc_demo PRIVATE fsoc::demo_support)
    fsoc_set_project_warnings(fsoc_demo)
    fsoc_enable_sanitizers(fsoc_demo)
endif()

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

    add_executable(fsoc_step3_tests tests/step3_tests.cpp)
    target_link_libraries(fsoc_step3_tests PRIVATE fsoc::core)
    fsoc_set_project_warnings(fsoc_step3_tests)
    fsoc_enable_sanitizers(fsoc_step3_tests)
    add_test(NAME fsoc_step3_tests COMMAND fsoc_step3_tests)

    add_executable(fsoc_step6_tests tests/step6_tests.cpp)
    target_link_libraries(fsoc_step6_tests PRIVATE fsoc::control)
    fsoc_set_project_warnings(fsoc_step6_tests)
    fsoc_enable_sanitizers(fsoc_step6_tests)
    add_test(NAME fsoc_step6_tests COMMAND fsoc_step6_tests)

    if(FSOC_OPENCV_AVAILABLE)
        add_executable(fsoc_step4_tests tests/step4_tests.cpp)
        target_link_libraries(fsoc_step4_tests PRIVATE fsoc::render)
        fsoc_set_project_warnings(fsoc_step4_tests)
        fsoc_enable_sanitizers(fsoc_step4_tests)
        add_test(NAME fsoc_step4_tests COMMAND fsoc_step4_tests)

        add_executable(fsoc_step5_tests tests/step5_tests.cpp)
        target_link_libraries(fsoc_step5_tests PRIVATE fsoc::perception fsoc::render)
        fsoc_set_project_warnings(fsoc_step5_tests)
        fsoc_enable_sanitizers(fsoc_step5_tests)
        add_test(NAME fsoc_step5_tests COMMAND fsoc_step5_tests)

        add_executable(fsoc_step7_tests tests/step7_tests.cpp)
        target_link_libraries(fsoc_step7_tests PRIVATE fsoc::simulation)
        fsoc_set_project_warnings(fsoc_step7_tests)
        fsoc_enable_sanitizers(fsoc_step7_tests)
        add_test(NAME fsoc_step7_tests COMMAND fsoc_step7_tests)

        add_executable(fsoc_step8_tests tests/step8_tests.cpp)
        target_link_libraries(fsoc_step8_tests PRIVATE fsoc::telemetry)
        fsoc_set_project_warnings(fsoc_step8_tests)
        fsoc_enable_sanitizers(fsoc_step8_tests)
        add_test(NAME fsoc_step8_tests COMMAND fsoc_step8_tests)

        add_executable(fsoc_step9_tests tests/step9_tests.cpp)
        target_link_libraries(fsoc_step9_tests PRIVATE fsoc::visualization)
        fsoc_set_project_warnings(fsoc_step9_tests)
        fsoc_enable_sanitizers(fsoc_step9_tests)
        add_test(NAME fsoc_step9_tests COMMAND fsoc_step9_tests)

        add_executable(fsoc_step10_tests tests/step10_tests.cpp)
        target_link_libraries(fsoc_step10_tests PRIVATE fsoc::validation)
        fsoc_set_project_warnings(fsoc_step10_tests)
        fsoc_enable_sanitizers(fsoc_step10_tests)
        add_test(NAME fsoc_step10_tests COMMAND fsoc_step10_tests)

        add_executable(fsoc_step11_tests tests/step11_tests.cpp)
        target_link_libraries(fsoc_step11_tests PRIVATE fsoc::demo_support fsoc::validation)
        fsoc_set_project_warnings(fsoc_step11_tests)
        fsoc_enable_sanitizers(fsoc_step11_tests)
        add_test(NAME fsoc_step11_tests COMMAND fsoc_step11_tests)
    endif()
endif()
