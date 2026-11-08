cmake_minimum_required(VERSION 3.20)
project(can_dbc_multiplexing LANGUAGES CXX)

find_package(GTest REQUIRED)
enable_testing()
include(GoogleTest)

foreach(suite IN ITEMS mux_select frame_bounds)
    add_executable(${suite}_test tests/${suite}_test.cpp)
    target_compile_features(${suite}_test PRIVATE cxx_std_20)
    target_include_directories(${suite}_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${suite}_test PRIVATE GTest::gtest_main)
    gtest_discover_tests(${suite}_test)
endforeach()
