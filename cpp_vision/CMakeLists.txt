cmake_minimum_required(VERSION 3.10)
project(cpp_vision)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)

# Create the executable
add_executable(main src/main.cpp src/image_processing.cpp)
target_link_libraries(main ${OpenCV_LIBS})
