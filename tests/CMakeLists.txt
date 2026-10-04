cmake_minimum_required(VERSION 3.20)
project(NEXUS LANGUAGES CXX)
enable_testing()

add_library(nexus_core STATIC
    ../src/nexus/estimator.cpp
    ../src/nexus/controller.cpp
    ../src/nexus/calibration.cpp
    ../src/nexus/automatic_calibration.cpp)
target_compile_features(nexus_core PUBLIC cxx_std_23)
target_include_directories(nexus_core PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../include")
if(MSVC)
    # Calibration uses portable C stdio shared with the PROS firmware.
    target_compile_definitions(nexus_core PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(nexus_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(nexus_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME log_preparation COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/prepare_dynamics_test.py" --calibrate "$<TARGET_FILE:nexus_calibrate>")
    add_test(NAME odometry_trace_log COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/odometry_trace_log_test.py")
endif()

foreach(name estimator controller calibration system autonomous_units driver_input automatic_calibration calibration_routine pod_offset mechanism arm_motion startup)
    add_executable(${name}_test ${name}_test.cpp)
    target_link_libraries(${name}_test PRIVATE nexus_core)
    add_test(NAME ${name} COMMAND ${name}_test)
endforeach()
find_package(Threads REQUIRED)
add_executable(chassis_adapter_test chassis_adapter_test.cpp ../src/nexus/chassis.cpp)
target_include_directories(chassis_adapter_test BEFORE PRIVATE "${CMAKE_CURRENT_LIST_DIR}/fake_pros")
target_link_libraries(chassis_adapter_test PRIVATE nexus_core Threads::Threads)
add_test(NAME chassis_adapter COMMAND chassis_adapter_test)
add_executable(nexus_calibrate ../tools/nexus_calibrate.cpp)
target_link_libraries(nexus_calibrate PRIVATE nexus_core)
