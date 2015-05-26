cmake_minimum_required (VERSION 2.8)
project(MYSIM)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)
add_subdirectory(VNE)
add_subdirectory(analysis)
add_subdirectory(statistics)

