cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(CV_Project)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(billiardAnalyzer SHARED src/billiardAnalyzer.cpp)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} billiardAnalyzer)