# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(project_name)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Set include directory
include_directories(include)

# Set source directory
file(GLOB main_file "${CMAKE_SOURCE_DIR}/main.cpp")
file(GLOB src_files "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Declare the executable target built from your sources
add_executable(output ${src_files} ${main_file})

# Link your application with OpenCV libraries
target_link_libraries(output ${OpenCV_LIBS})