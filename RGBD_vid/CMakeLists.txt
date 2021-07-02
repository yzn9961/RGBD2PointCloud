cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

# opencv
set(OpenCV_DIR "/home/telemoro/openCV/opencv-4.5.2/build/OpenCVConfig.cmake")# 选择4.5.2版本opencv
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
# 打印消息
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(DepthCapture main.cpp)
target_link_libraries(DepthCapture ${OpenCV_LIBS} -lpthread -lm)
