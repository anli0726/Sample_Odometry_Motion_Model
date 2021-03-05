# Odometry_motion_model/CmakeLists.txt

cmake_minimum_required(VERSION 3.10)
add_compile_options(-std=c++11)
project(sample_odometry_motion_estimate)

include_directories("lib/include")
add_library(predictor "lib/src/SampleOdemetryMotionPredictor.cpp")

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(main "src/main.cpp")
target_link_libraries(main predictor ${OpenCV_LIBS})