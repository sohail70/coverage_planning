cmake_minimum_required(VERSION 3.10)
project(Coverage)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #to use it for autocomplete in the vs code json system!

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# For find_package(Foo), CMake searches for either of these (config mode):
# FooConfig.cmake
# foo-config.cmake
# FooConfigVersion.cmake
# foo-config-version.cmake

#find /usr -iname "*opencv*config*.cmake" 2>/dev/null
#find /usr -iname "*config.cmake" 2>/dev/null | sort



find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}") # pkg-config --libs opencv4
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")

include_directories(include)


add_library(map src/rasterized_image.cpp src/map_geometry.cpp)
target_link_libraries(map ${OpenCV_LIBS})
target_include_directories(map PRIVATE ${OpenCV_INCLUDE_DIRS})


add_executable(test tests/test.cpp)
add_executable(test_computational_geometry tests/test_computational_geometry.cpp)

add_executable(test_image tests/test_image.cpp)
add_executable(test_map_geometry tests/test_map_geometry.cpp)


#find /usr -name "libopencv_*.so*" 2>/dev/null | sort

# CMake's target_link_libraries accepts several forms. From libopencv_core.so:
# Form	Example
# Full path	/usr/lib/x86_64-linux-gnu/libopencv_core.so
# -l short name	opencv_core
# CMake target (if the package defines it)	opencv_core or OpenCV::core
# The rule for the -l form: strip lib prefix and .so suffix. So libopencv_imgproc.so → opencv_imgproc.
target_link_libraries(test_image PRIVATE map)
target_include_directories(test_image PRIVATE map)

target_link_libraries(test_map_geometry PRIVATE map)
target_include_directories(test_map_geometry PRIVATE map)


