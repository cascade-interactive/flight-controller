cmake_minimum_required(VERSION 3.14)
project(LYNN_FC_Tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(test_ekf
    test_ekf.cpp
    ../Core/Src/Estimation/AttitudeEstimator.cpp
)

target_include_directories(test_ekf PRIVATE
    ../Core/Inc
    ../Core/Inc/Estimation
    ../Core/Inc/Math
)

target_link_libraries(test_ekf PRIVATE
    GTest::gtest_main
)