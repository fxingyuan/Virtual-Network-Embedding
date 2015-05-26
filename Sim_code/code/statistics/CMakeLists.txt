cmake_minimum_required (VERSION 2.8)
project(Static)
# Matlab Runtime Library
set(MATLAB_2013a_PATH "C:/Program Files/MATLAB/R2013a")
include_directories(${MATLAB_2013a_PATH}/extern/include/win64)
include_directories(${MATLAB_2013a_PATH}/extern/include)
link_directories(${MATLAB_2013a_PATH}/extern/lib/win64/microsoft)
#main
set(static_srcs static2xls.cpp)
source_group("src" FILES ${static_srcs})
add_executable(static2xls ${static_srcs})
target_link_libraries(static2xls libeng libmx)