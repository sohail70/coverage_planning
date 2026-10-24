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

# include_directories(include) is global! and now its redundant because coverage_core handles it for test files that use coverage_core!
# effectively says:
# test
# test_bcd
# test_image
# test_computational_geometry
# coverage_core
# future executables
# future libraries

# all get include/, whether they need it or not.
# By contrast:
# target_include_directories(
#     coverage_core
#     PUBLIC
#         ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# says:
# coverage_core needs include/

# and because it is PUBLIC:
# anything linking coverage_core also gets include/

# That gives you a dependency structure like:
# test_bcd
#    |
#    v
# coverage_core
#    |
#    v
# include/

# instead of:
# include/
#  | | | | |
#  v v v v v
# everything

# include_directories(include)



# Main coverage library
add_library(coverage_core
    src/rasterized_image.cpp
    src/map_geometry.cpp
    src/boustrophedon_decomposer.cpp
    src/depth_first_traversal.cpp
    src/boustrophedon_coverage_generator.cpp
    src/visibility_graph.cpp
    src/holonomic_polygon_transtion_planner.cpp
)
# first install sudo apt install coinor-libipopt-dev for ipopt and then build casadi from source with ipopt support : https://github.com/zehuilu/Tutorial-on-CasADi-with-CPP
# cmake .. \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DCMAKE_INSTALL_PREFIX=/usr/local \
#     -DWITH_PYTHON=OFF \
#     -DWITH_MATLAB=OFF \
#     -DWITH_OCTAVE=OFF
#     -DWITH_IPOPT=ON
#
# cmake --build . -j"$(nproc)"
# sudo cmake --install .
# sudo ldconfig

find_package(casadi CONFIG REQUIRED)

add_library(coverage_control
    src/control/mpc.cpp
)

# The nice part is that because this is PUBLIC:
# target_include_directories(coverage_core
#     PUBLIC
#         ${CMAKE_CURRENT_SOURCE_DIR}/include
#         ${OpenCV_INCLUDE_DIRS}
# )
# a test only needs:
# target_link_libraries(test_bcd PRIVATE coverage_core)
# and it automatically gets access to headers such as:
# #include "cleaner/map_geometry.hpp"
# #include "cleaner/boustrophedon_decomposer.hpp"
# You don't need to repeat include directories for every executable. 

target_include_directories(coverage_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(coverage_core
    PUBLIC
        ${OpenCV_LIBS}
)

# TO SEE IF THE INCLUDE_DIRS AND _LIBS VARIABLE ARE DEFINED:
# grep -nE 'add_library|INTERFACE_INCLUDE_DIRECTORIES|casadi_INCLUDE|casadi_LIB' \
#     /usr/local/lib/cmake/casadi/casadi-config.cmake \
#     /usr/local/lib/cmake/casadi/casadi-targets.cmake
target_include_directories(coverage_control
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(coverage_control 
    PRIVATE 
        casadi::casadi
)

#######################

add_executable(test tests/test.cpp)
add_executable(test_computational_geometry tests/test_computational_geometry.cpp)
add_executable(test_image tests/test_image.cpp)
add_executable(test_bcd tests/test_bcd.cpp)
add_executable(test_mpc tests/test_mpc.cpp)


#find /usr -name "libopencv_*.so*" 2>/dev/null | sort

# CMake's target_link_libraries accepts several forms. From libopencv_core.so:
# Form	Example
# Full path	/usr/lib/x86_64-linux-gnu/libopencv_core.so
# -l short name	opencv_core
# CMake target (if the package defines it)	opencv_core or OpenCV::core
# The rule for the -l form: strip lib prefix and .so suffix. So libopencv_imgproc.so → opencv_imgproc.
target_link_libraries(test_image PRIVATE coverage_core)
target_link_libraries(test_bcd PRIVATE coverage_core)
target_link_libraries(test_mpc PRIVATE coverage_control)


