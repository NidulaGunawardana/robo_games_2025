cmake_minimum_required(VERSION 3.10)
project(ObjectDetection)

# Set OpenCV directory
set(OpenCV_DIR C:/opencv/build)  # Replace with the path to your OpenCV installation

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(${PROJECT_NAME} src/Main.cpp)

# Link OpenCV libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
