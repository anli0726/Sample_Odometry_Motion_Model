#  Odometry_motion_model/lib/CmakeLists.txt

cmake_minimum_required(VERSION 3.10)
add_compile_options(-std=c++11)
project(predictor
	VERSION 1.0.0
	DESCRIPTION "Library target for main to use")

add_library(predictor "src/SampleOdemetryMotionPredictor.cpp")
target_include_directories(predictor 
	PUBLIC include 
	PRIVATE src)


