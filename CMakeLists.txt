cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(CV_Project)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

set(SOURCE_FILES
    src/PoolTable.cpp
    src/Ball.cpp
    src/Utility.cpp
    src/Detection.cpp
    src/EightBallPoolGame.cpp
)

add_library(billiardAnalyzer SHARED ${SOURCE_FILES})

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} billiardAnalyzer)